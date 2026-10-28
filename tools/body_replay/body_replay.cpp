// Replays a SteamVR session recording (engine/src/recorder.h) through the body
// solver, at the recorded engine frame times, with the inputs the engine had
// (config, tracker rotations and accelerations, HMD pose), and writes a CSV next
// to the recorded outputs. Lets the solver be changed and compared on real
// sessions without SteamVR (tools/plot_replay.py plots the CSV).
//
// Usage: smt_body_replay session.jsonl out.csv [--from S] [--to S] [--floor Y] [--model foot_model.json|none]
//   --from/--to: seconds from the start of the recording
//   --floor: floor height in raw tracking space (default: from the recording)
//   --dump inputs.bin: also write the per-frame inputs as float64 rows (names in
//     inputs.bin.json): HMD and controller poses, calibrated segment orientations,
//     world accelerations (tools/pose_model)
//   SMT_NO_WAIST=1: ignore the waist tracker (ankles-only mode on a three-tracker recording)
//   SMT_MAG_REPLAY=1: rerun the mag heading filter (current code, SMT_TUNE mag constants)
//     instead of the recorded headings. The replayed heading is aligned to the
//     recorded one at each body calibration, which the mountings were measured with.
//
// Parsing the JSON lines dominates the run time (a 5 h session is 3.4 GB), so the
// parsed events are cached in a binary file next to the session (session.jsonl.evc,
// or in SMT_EVENT_CACHE_DIR), rebuilt when the session file changes.
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "smt/body_solver.h"
#include "smt/config.h"
#include "smt/foot_model.h"
#include "smt/mag.h"
#include "smt/tilt_fix.h"
#include "smt/pose_model.h"
#include "smt/protocol.h"

using namespace smt;
using nlohmann::json;
namespace fs = std::filesystem;

namespace {

struct TrackerState {
	bool hasRotation = false;
	Quat qWorld, prevRotation;
	double prevTime = 0;
	double angularSpeed = 0;
	Vec3 accelLocal;
	// Same firmware glitch recovery as the tracker hub
	TiltFix tiltFix;
	Vec3 rawLinear;
	bool hasHeading = false;
	double heading = 0;
	// SMT_MAG_REPLAY
	HeadingFilter filter;
	double magResidual = -1, lastMag = 0, temperature = std::nan("");
	double recHeading = 0, offset = 0;
	bool recHas = false, aligned = false;
	// Alignment: circular mean of (recorded - replayed) over kAlignTime after each
	// trigger, then frozen (one instant would bake a transient disagreement in)
	double alignSin = 0, alignCos = 0, alignStart = -1;
	int boot = -1;
};

// ---- parsed events and their binary cache ----

enum class Kind : uint8_t { MacDef, Cfg, Ev, Rot, Acc, Trk, Dev, Out, Mag, Tmp };

struct Event {
	double t = 0;
	Kind kind = Kind::Cfg;
	uint8_t mac = 0;
	uint8_t nf = 0;
	float f[18] = {};
	std::string str;  // MacDef: mac, Cfg: config JSON, Ev: what '\0' detail
};

constexpr uint32_t kCacheMagic = 0x31435645;  // "EVC1"

class EventWriter {
public:
	explicit EventWriter(const std::string& path) : m_file(std::fopen(path.c_str(), "wb")) {}
	~EventWriter() {
		if (m_file) std::fclose(m_file);
	}
	bool ok() const { return m_file != nullptr; }
	void header(uint64_t size, int64_t mtime) {
		std::fwrite(&kCacheMagic, 4, 1, m_file);
		std::fwrite(&size, 8, 1, m_file);
		std::fwrite(&mtime, 8, 1, m_file);
	}
	void write(const Event& e) {
		std::fwrite(&e.t, 8, 1, m_file);
		uint8_t h[3] = {uint8_t(e.kind), e.mac, e.nf};
		std::fwrite(h, 1, 3, m_file);
		if (e.nf) std::fwrite(e.f, 4, e.nf, m_file);
		if (e.kind == Kind::MacDef || e.kind == Kind::Cfg || e.kind == Kind::Ev) {
			uint32_t n = uint32_t(e.str.size());
			std::fwrite(&n, 4, 1, m_file);
			std::fwrite(e.str.data(), 1, n, m_file);
		}
	}

private:
	FILE* m_file;
};

class EventReader {
public:
	explicit EventReader(const std::string& path) : m_file(std::fopen(path.c_str(), "rb")) {
		if (m_file) std::setvbuf(m_file, nullptr, _IOFBF, 1 << 20);
	}
	~EventReader() {
		if (m_file) std::fclose(m_file);
	}
	bool header(uint64_t& size, int64_t& mtime) {
		uint32_t magic = 0;
		return m_file && std::fread(&magic, 4, 1, m_file) == 1 && magic == kCacheMagic &&
			   std::fread(&size, 8, 1, m_file) == 1 && std::fread(&mtime, 8, 1, m_file) == 1;
	}
	bool next(Event& e) {
		uint8_t h[3];
		if (std::fread(&e.t, 8, 1, m_file) != 1 || std::fread(h, 1, 3, m_file) != 3) return false;
		e.kind = Kind(h[0]);
		e.mac = h[1];
		e.nf = h[2];
		if (e.nf > 18 || (e.nf && std::fread(e.f, 4, e.nf, m_file) != e.nf)) return false;
		if (e.kind == Kind::MacDef || e.kind == Kind::Cfg || e.kind == Kind::Ev) {
			uint32_t n = 0;
			if (std::fread(&n, 4, 1, m_file) != 1) return false;
			e.str.resize(n);
			if (n && std::fread(e.str.data(), 1, n, m_file) != n) return false;
		}
		return true;
	}

private:
	FILE* m_file;
};

// Parses the JSON lines into the cache file. The replay then always reads the cache
bool buildCache(const std::string& session, const std::string& cache, uint64_t size, int64_t mtime) {
	std::ifstream in(session);
	if (!in) return false;
	// Parallel replays may build the same cache: each writes its own file
	std::string tmp = cache + "." + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".tmp";
	{
		EventWriter w(tmp);
		if (!w.ok()) return false;
		w.header(size, mtime);
		std::map<std::string, uint8_t> macs;
		auto macIndex = [&](const std::string& mac, double t) {
			auto it = macs.find(mac);
			if (it != macs.end()) return it->second;
			uint8_t idx = uint8_t(macs.size());
			macs[mac] = idx;
			Event d;
			d.t = t;
			d.kind = Kind::MacDef;
			d.mac = idx;
			d.str = mac;
			w.write(d);
			return idx;
		};
		std::string line;
		while (std::getline(in, line)) {
			if (line.find("\"k\":\"bat\"") != std::string::npos || line.find("\"k\":\"acc\"") == std::string::npos &&
				line.find("\"k\":\"dev\"") != std::string::npos && line.find("\"cls\":1,") == std::string::npos &&
				line.find("\"cls\":2,") == std::string::npos)
				continue;
			json j = json::parse(line, nullptr, false);
			if (j.is_discarded() || !j.contains("k")) continue;
			Event e;
			e.t = j.value("t", 0.0);
			const std::string k = j["k"].get<std::string>();
			auto put = [&](const json& a, size_t n, size_t offset = 0) {
				for (size_t i = 0; i < n; i++) e.f[e.nf++] = a.at(offset + i).get<float>();
			};
			if (k == "cfg") {
				e.kind = Kind::Cfg;
				e.str = j["config"].dump();
			} else if (k == "ev") {
				e.kind = Kind::Ev;
				e.str = j.value("what", "") + std::string(1, '\0') + j.value("detail", "");
			} else if (k == "rot" || k == "acc" || k == "mag") {
				if (j.value("s", 0) != 0) continue;
				e.kind = k == "rot" ? Kind::Rot : k == "acc" ? Kind::Acc : Kind::Mag;
				e.mac = macIndex(j["mac"].get<std::string>(), e.t);
				if (k == "rot") put(j["q"], 4);
				else if (k == "acc") put(j["a"], 3);
				else put(j["m"], 3);
			} else if (k == "trk") {
				e.kind = Kind::Trk;
				e.mac = macIndex(j["mac"].get<std::string>(), e.t);
				e.f[0] = float(j.value("hh", 0));
				e.f[1] = float(j.value("hdg", 0.0));
				e.f[2] = float(j.value("boot", 0));
				e.nf = 3;
			} else if (k == "tmp") {
				e.kind = Kind::Tmp;
				e.mac = macIndex(j["mac"].get<std::string>(), e.t);
				e.f[0] = float(j.value("c", 0.0));
				e.nf = 1;
			} else if (k == "dev") {
				int cls = j.value("cls", 0), role = j.value("role", 0);
				if (cls != 1 && !(cls == 2 && (role == 1 || role == 2))) continue;
				e.kind = Kind::Dev;
				e.f[0] = float(cls);
				e.f[1] = float(role);
				e.f[2] = float(j.value("ok", 0));
				e.nf = 3;
				put(j["p"], 3);
				put(j["q"], 4);
			} else if (k == "out") {
				e.kind = Kind::Out;
				const json &wj = j["waist"], &lf = j["lf"], &rf = j["rf"];
				put(wj, 8);
				put(lf, 4);
				put(rf, 4);
				e.f[e.nf++] = j["kn"][0].get<float>();
				e.f[e.nf++] = j["kn"][1].get<float>();
				// Planted flags: lf[0]/rf[0] (valid) are only used for the floor estimate
				e.f[8] = lf[0].get<float>() * (j["pl"][0].get<int>() ? 1.0f : -1.0f);
				e.f[12] = rf[0].get<float>() * (j["pl"][1].get<int>() ? 1.0f : -1.0f);
			} else {
				continue;
			}
			w.write(e);
		}
	}
	std::error_code ec;
	fs::rename(tmp, cache, ec);
	return !ec;
}

std::string cachePath(const std::string& session) {
	if (const char* dir = std::getenv("SMT_EVENT_CACHE_DIR"))
		return (fs::path(dir) / fs::path(session).filename()).string() + ".evc";
	return session + ".evc";
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: %s session.jsonl out.csv [--from S] [--to S] [--floor Y]\n", argv[0]);
		return 1;
	}
	double from = 0, to = 1e18;
	std::optional<double> floorArg;
	std::string dumpPath;
	for (int i = 3; i + 1 < argc; i += 2) {
		std::string k = argv[i];
		double v = std::atof(argv[i + 1]);
		if (k == "--from") from = v;
		else if (k == "--to") to = v;
		else if (k == "--floor") floorArg = v;
		else if (k == "--dump") dumpPath = argv[i + 1];
	}

	const std::string session = argv[1];
	std::error_code ec;
	uint64_t size = fs::file_size(session, ec);
	if (ec) {
		std::printf("cannot open %s\n", argv[1]);
		return 1;
	}
	int64_t mtime = int64_t(fs::last_write_time(session, ec).time_since_epoch().count());
	const std::string cache = cachePath(session);
	{
		EventReader r(cache);
		uint64_t cs = 0;
		int64_t cm = 0;
		bool fresh = r.header(cs, cm) && cs == size && cm == mtime;
		if (!fresh && !buildCache(session, cache, size, mtime)) {
			std::printf("cannot build the event cache %s\n", cache.c_str());
			return 1;
		}
	}

	FILE* csv = std::fopen(argv[2], "w");
	if (!csv) {
		std::printf("cannot write %s\n", argv[2]);
		return 1;
	}
	std::fprintf(csv,
				 "t,hmd_x,hmd_y,hmd_z,hmd_pitch,"
				 "rec_valid,rec_wx,rec_wy,rec_wz,rec_lx,rec_ly,rec_lz,rec_rx,rec_ry,rec_rz,rec_kl,rec_kr,rec_pl,rec_pr,"
				 "valid,wx,wy,wz,lx,ly,lz,rx,ry,rz,kl,kr,pl,pr,"
				 "pelvis_up_y,lshin_up_y,rshin_up_y,l_accel_y,r_accel_y,l_accel,r_accel,l_gyro,r_gyro,"
				 "standing,hip0_y,l_ins_y,r_ins_y,l_ins_sigma,r_ins_sigma,l_anchor_y,r_anchor_y,floor,"
				 "ankle_floor,eye_h,lc_x,lc_y,lc_z,rc_x,rc_y,rc_z,"
				 "hmd_yaw,rec_w_yaw,rec_w_tilt,w_yaw,w_tilt,l_shin_yaw,r_shin_yaw,hip0_x,hip0_z,"
				 "l_td_swing,r_td_swing,l_td_h,r_td_h,"
				 "lkx,lky,lkz,rkx,rky,rkz,l_flex,r_flex,l_abd,r_abd,l_ins_x,l_ins_z,r_ins_x,r_ins_z,pelvis_fwd_y,pelvis_up_x,pelvis_up_z,l_toe_x,l_toe_y,l_toe_z,r_toe_x,r_toe_y,r_toe_z,l_hinge,r_hinge,l_hold,r_hold,l_latch,r_latch,l_lower,r_lower,l_cross,r_cross,l_xrest,r_xrest\n");

	Config config;
	bool haveConfig = false;
	std::vector<std::string> macNames;
	std::map<std::string, TrackerState> trackers;
	Pose hmd;
	bool hmdValid = false;
	std::array<Vec3, 2> controllers;  // left, right hand
	std::array<Quat, 2> controllerRot;
	std::array<bool, 2> controllerValid{};
	static const char* kDumpNames[] = {
		"t", "hmd_valid", "hx", "hy", "hz", "hqw", "hqx", "hqy", "hqz", "floor", "eye_h", "height",
		"w_valid", "wqw", "wqx", "wqy", "wqz", "wax", "way", "waz",
		"l_valid", "lqw", "lqx", "lqy", "lqz", "lax", "lay", "laz",
		"r_valid", "rqw", "rqx", "rqy", "rqz", "rax", "ray", "raz",
		"c0_valid", "c0x", "c0y", "c0z", "c0qw", "c0qx", "c0qy", "c0qz",
		"c1_valid", "c1x", "c1y", "c1z", "c1qw", "c1qx", "c1qy", "c1qz",
		"out_valid", "o_wx", "o_wy", "o_wz", "o_lx", "o_ly", "o_lz", "o_rx", "o_ry", "o_rz"};
	constexpr size_t kDumpColumns = sizeof(kDumpNames) / sizeof(kDumpNames[0]);
	FILE* dump = nullptr;
	if (!dumpPath.empty()) {
		dump = std::fopen(dumpPath.c_str(), "wb");
		json names = json::array();
		for (const char* n : kDumpNames) names.push_back(n);
		std::ofstream(dumpPath + ".json") << json{{"columns", names}}.dump();
	}
	double floorY = floorArg.value_or(0);
	bool floorKnown = floorArg.has_value();
	// Old recordings have no floor line: floor = HMD height at the stand capture -
	// the eye height the next config reports
	std::optional<double> standHmdY;
	// Recordings started mid-session (before the engine wrote the floor at the start)
	// have neither: estimate it from the recorded planted feet
	std::optional<double> plantedAnkleY;
	if (!floorArg) {
		EventReader pre(cache);
		uint64_t cs;
		int64_t cm;
		pre.header(cs, cm);
		Event e;
		bool hasFloor = false;
		std::vector<double> ys;
		while (pre.next(e)) {
			if (e.kind == Kind::Ev) {
				std::string what = e.str.c_str(), detail = e.str.substr(std::min(e.str.size(), what.size() + 1));
				if (what == "floor" || detail.rfind("ski:", 0) == 0 || detail.rfind("done: Reset", 0) == 0) {
					hasFloor = true;
					break;
				}
			} else if (e.kind == Kind::Out && ys.size() < 200000) {
				if (e.f[8] > 0) ys.push_back(e.f[10]);
				if (e.f[12] > 0) ys.push_back(e.f[14]);
			}
		}
		if (!hasFloor && ys.size() > 100) {
			// Low percentile: feet also rest on the couch or the bed
			size_t k = ys.size() / 10;
			std::nth_element(ys.begin(), ys.begin() + k, ys.end());
			plantedAnkleY = ys[k];
		}
	}
	BodySolver solver;
	const bool noWaist = std::getenv("SMT_NO_WAIST") && std::atoi(std::getenv("SMT_NO_WAIST"));
	const bool magReplay = std::getenv("SMT_MAG_REPLAY") && std::atoi(std::getenv("SMT_MAG_REPLAY"));
	HeadingFilterSettings magSettings;
	magSettings.magSigma = deg2rad(tunable("magSigmaDeg", rad2deg(magSettings.magSigma)));
	magSettings.magCorrTime = tunable("magCorrTime", magSettings.magCorrTime);
	magSettings.headingWalk = deg2rad(tunable("magHeadingWalkDeg", rad2deg(magSettings.headingWalk)));
	magSettings.rateWalk = deg2rad(tunable("magRateWalkDeg", rad2deg(magSettings.rateWalk)));
	magSettings.normSoft = tunable("magNormSoft", magSettings.normSoft);
	magSettings.dipSoft = deg2rad(tunable("magDipSoftDeg", rad2deg(magSettings.dipSoft)));
	magSettings.stillGrowth = tunable("magStillGrowth", magSettings.stillGrowth);
	magSettings.sensitivity = tunable("magSensitivity", magSettings.sensitivity);
	auto realign = [&] {
		for (auto& [mac, ts] : trackers) ts.aligned = false;
	};
	// Learned foot model: --model path, or SMT_FOOT_MODEL, by default the installed one
	// as the engine runs it (replays without it did not reproduce the live sessions);
	// "--model none" leaves it out
	FootModel footModel;
	std::string modelPath = std::getenv("SMT_FOOT_MODEL") ? std::getenv("SMT_FOOT_MODEL") : "";
	for (int i = 3; i + 1 < argc; i += 2)
		if (std::string(argv[i]) == "--model") modelPath = argv[i + 1];
	if (modelPath.empty() && std::getenv("LOCALAPPDATA")) {
		const std::string installed = std::string(std::getenv("LOCALAPPDATA")) +
									  "/Slime Motion Tracker/driver/slime_motion_tracker/bin/win64/models/foot_model.json";
		if (std::ifstream(installed)) modelPath = installed;
	}
	if (modelPath == "none") modelPath.clear();
	if (!modelPath.empty() && !footModel.load(modelPath)) std::printf("cannot load the foot model %s\n", modelPath.c_str());
	// Fusion sigma of the model (defaults of footMeasurementSigma), tunable offline
	const double modelFloor = tunable("modelFloor", 0.03), modelScale = tunable("modelScale", 0.6);
	// Learned pose model (optional): --pose-model path, or SMT_POSE_MODEL; fusion
	// weights from SMT_POSE_CROUCH_W / SMT_POSE_LYING_W
	PoseModel poseModel;
	PoseFusionSettings fusion;
	PoseFuser fuser;
	std::string poseModelPath = std::getenv("SMT_POSE_MODEL") ? std::getenv("SMT_POSE_MODEL") : "";
	for (int i = 3; i + 1 < argc; i += 2)
		if (std::string(argv[i]) == "--pose-model") poseModelPath = argv[i + 1];
	if (!poseModelPath.empty() && !poseModel.load(poseModelPath))
		std::printf("cannot load the pose model %s: %s\n", poseModelPath.c_str(), poseModel.error().c_str());
	if (const char* v = std::getenv("SMT_POSE_CROUCH_W")) fusion.crouchWeight = std::atof(v);
	if (const char* v = std::getenv("SMT_POSE_LYING_W")) fusion.lyingWeight = std::atof(v);
	double t0 = -1, lastFrame = -1;
	int frames = 0;
	double solveSeconds = 0;

	BodyDimensions dims;
	auto configureSolver = [&] {
		dims = BodyDimensions::fromHeight(config.userHeight, config.measuredEyeHeight);
		solver.configure(dims, config.settings, config.mode);
	};

	EventReader reader(cache);
	{
		uint64_t cs;
		int64_t cm;
		reader.header(cs, cm);
	}
	Event ev;
	while (reader.next(ev)) {
		const double t = ev.t;
		if (t0 < 0) t0 = t;
		const float* f = ev.f;

		if (ev.kind == Kind::MacDef) {
			if (macNames.size() <= ev.mac) macNames.resize(ev.mac + 1);
			macNames[ev.mac] = ev.str;
		} else if (ev.kind == Kind::Cfg) {
			config.fromJson(json::parse(ev.str));
			haveConfig = true;
			configureSolver();
			if (plantedAnkleY && !floorKnown) {
				floorY = *plantedAnkleY - dims.ankleHeight;
				floorKnown = true;
				std::printf("no floor in the recording: %.3f from the recorded planted feet\n", floorY);
			}
			if (!floorArg && standHmdY && config.measuredEyeHeight) {
				floorY = *standHmdY - *config.measuredEyeHeight;
				floorKnown = true;
				standHmdY.reset();
			}
			if (magReplay)
				for (const auto& [mac, tc] : config.trackers) {
					TrackerState& ts = trackers[mac];
					if (!tc.mag.valid || ts.magResidual == tc.mag.residual) continue;
					// A replaced calibration carries the offset the tracker hub gave it
					if (ts.magResidual < 0 || ts.lastMag == 0) ts.filter.setCalibration(tc.mag);
					else ts.filter.updateCalibration(tc.mag);
					ts.magResidual = tc.mag.residual;
				}
		} else if (ev.kind == Kind::Ev) {
			std::string what = ev.str.c_str(), detail = ev.str.substr(std::min(ev.str.size(), what.size() + 1));
			if (what == "floor" && !floorArg) {
				floorY = std::atof(detail.c_str());
				floorKnown = true;
			} else if (what == "calibration") {
				if (detail.rfind("ski:", 0) == 0 || detail.rfind("done: Reset", 0) == 0) {
					standHmdY = hmd.position.y;
					// Same stand pose as the engine calibrates the controllers to the wrists on
					if (poseModel.loaded() && hmdValid && controllerValid[0] && controllerValid[1])
						poseModel.calibrateWrists(hmd, {Pose{controllers[0], controllerRot[0]},
														Pose{controllers[1], controllerRot[1]}});
				}
				// Stand pose capture times, for tools that calibrate other devices on it
				if (detail.rfind("ski:", 0) == 0 && dump) std::printf("stand_capture %.3f\n", t - t0);
				if (detail.rfind("done:", 0) == 0) {
					solver.reset();
					realign();
				}
			}
		} else if (ev.kind == Kind::Rot) {
			Quat qw = proto::firmwareToWorld(Quat{f[0], f[1], f[2], f[3]});
			TrackerState& ts = trackers[macNames[ev.mac]];
			// Same angular speed estimate as the tracker hub
			double rotDt = 0;
			if (ts.prevTime > 0) {
				double dt = t - ts.prevTime;
				if (dt > 1e-4 && dt < 0.5) {
					rotDt = dt;
					double ang = (ts.prevRotation.conj() * qw).angle(), w = ang / dt;
					if (w > 40.0 && ang > deg2rad(20)) {
						if (magReplay) ts.filter.gyroGlitch();
						ts.tiltFix.glitch(t);
					}
					ts.angularSpeed += (w - ts.angularSpeed) * std::min(1.0, dt / 0.05);
				}
			}
			ts.prevRotation = qw;
			ts.prevTime = t;
			ts.tiltFix.update(qw, ts.rawLinear, ts.angularSpeed, t, rotDt);
			if (magReplay && ts.tiltFix.still(ts.angularSpeed) && !ts.filter.relocking()) ts.filter.gyroGlitch();
			ts.qWorld = ts.tiltFix.apply(qw);
			ts.accelLocal = ts.tiltFix.linear(qw, ts.rawLinear);
			ts.hasRotation = true;
		} else if (ev.kind == Kind::Acc) {
			TrackerState& ts = trackers[macNames[ev.mac]];
			ts.rawLinear = {f[0], f[1], f[2]};
			ts.accelLocal = ts.tiltFix.linear(ts.prevRotation, ts.rawLinear);
		} else if (ev.kind == Kind::Trk) {
			TrackerState& ts = trackers[macNames[ev.mac]];
			if (magReplay) {
				// Same rule as the tracker hub: a reconnect restarts the device fusion
				int boot = int(f[2]);
				// The offset to the recorded heading stays: both filters re-acquire the
				// absolute mag heading, and realigning on a transient disagreement would
				// leave a permanent error
				if (ts.boot >= 0 && boot != ts.boot) ts.filter.reset();
				ts.boot = boot;
				// The live filter lost its heading (reset): mirror it, and realign once both
				// re-acquired the absolute mag heading (their calibrations' heading offsets
				// may differ)
				bool has = f[0] != 0;
				if (ts.recHas && !has) ts.filter.reset();
				if (!ts.recHas && has) ts.aligned = false;
				ts.recHas = has;
				ts.recHeading = f[1];
				if (ts.recHas && ts.filter.hasHeading()) {
					if (!ts.aligned) {
						ts.aligned = true;
						ts.alignStart = t;
						ts.alignSin = ts.alignCos = 0;
					}
					if (t - ts.alignStart < 120.0) {
						double d = wrapAngle(ts.recHeading - ts.filter.heading());
						ts.alignSin += std::sin(d);
						ts.alignCos += std::cos(d);
						ts.offset = std::atan2(ts.alignSin, ts.alignCos);
					}
				}
				ts.hasHeading = ts.aligned && ts.filter.hasHeading();
				ts.heading = wrapAngle(ts.filter.heading() + ts.offset);
			} else {
				ts.hasHeading = f[0] != 0;
				ts.heading = f[1];
			}
		} else if (ev.kind == Kind::Tmp) {
			trackers[macNames[ev.mac]].temperature = f[0];
		} else if (ev.kind == Kind::Mag) {
			if (!magReplay) continue;
			TrackerState& ts = trackers[macNames[ev.mac]];
			if (!ts.hasRotation) continue;
			std::array<int16_t, 3> raw{int16_t(f[0]), int16_t(f[1]), int16_t(f[2])};
			double dt = ts.lastMag > 0 ? std::clamp(t - ts.lastMag, 0.0, 0.2) : 0.02;
			ts.lastMag = t;
			ts.filter.update(ts.qWorld, raw, ts.angularSpeed, dt, magSettings, ts.temperature);
			if (ts.aligned) ts.heading = wrapAngle(ts.filter.heading() + ts.offset);
		} else if (ev.kind == Kind::Dev) {
			int cls = int(f[0]), role = int(f[1]);
			if (cls == 2 && (role == 1 || role == 2)) {
				controllers[role - 1] = {f[3], f[4], f[5]};
				controllerRot[role - 1] = Quat{f[6], f[7], f[8], f[9]};
				controllerValid[role - 1] = f[2] != 0;
			}
			if (cls != 1) continue;
			hmdValid = f[2] != 0;
			hmd.position = {f[3], f[4], f[5]};
			hmd.rotation = Quat{f[6], f[7], f[8], f[9]};
		} else if (ev.kind == Kind::Out) {
			double dt = lastFrame > 0 ? t - lastFrame : 1.0 / 90;
			lastFrame = t;
			if (!haveConfig) continue;

			BodyInput bin;
			bin.dt = dt;
			bin.hmdValid = hmdValid && floorKnown;
			bin.hmd = hmd;
			bin.floorY = floorY;
			std::array<Vec3, 2> accel;
			std::array<double, 2> gyro{};
			Vec3 waistAccel;
			for (const auto& [mac, tc] : config.trackers) {
				auto it = trackers.find(mac);
				if (it == trackers.end() || !it->second.hasRotation || !tc.mountingValid) continue;
				const TrackerState& ts = it->second;
				Quat q = ts.qWorld;
				if (tc.yawInMagFrame) {
					if (!ts.hasHeading) continue;
					q = Quat::yaw(-ts.heading) * q;
				}
				Quat seg = Quat::yaw(tc.yawAlign) * q * tc.mounting;
				if (tc.role == Role::Waist) {
					if (noWaist) continue;
					bin.hasWaist = true;
					bin.waist = seg;
					waistAccel = (seg * tc.mounting.conj()).rotate(ts.accelLocal);
					bin.waistAccel = waistAccel;
				} else if (tc.role == Role::LeftAnkle || tc.role == Role::RightAnkle) {
					int i = tc.role == Role::LeftAnkle ? 0 : 1;
					auto& leg = bin.legs[i];
					leg.valid = true;
					leg.shin = seg;
					leg.accelWorld = (seg * tc.mounting.conj()).rotate(ts.accelLocal);
					leg.angularSpeed = ts.angularSpeed;
					accel[i] = leg.accelWorld;
					gyro[i] = ts.angularSpeed;
				}
			}
			auto solveStart = std::chrono::steady_clock::now();
			if (footModel.loaded()) {
				FootEstimate fe = footModel.update(bin, t);
				for (int i = 0; i < 2 && fe.valid; i++) {
					auto& leg = bin.legs[i];
					leg.hasFootMeasurement = true;
					leg.footMeasurement = fe.ankle[i];
					leg.footMeasurementSigma = footMeasurementSigma(fe.sigma[i], modelFloor, modelScale);
				}
			}
			BodyOutput o = solver.update(bin);
			if (poseModel.loaded()) {
				if (!poseModel.wristsCalibrated() && config.wristCalibration)
					poseModel.setWristCalibration(*config.wristCalibration);
				PoseModelInput pin;
				pin.floorY = floorY;
				pin.hmdValid = bin.hmdValid;
				pin.hmd = hmd;
				for (int i = 0; i < 2; i++) {
					pin.handValid[i] = controllerValid[i];
					pin.hands[i] = {controllers[i], controllerRot[i]};
					pin.legValid[i] = bin.legs[i].valid;
					pin.shin[i] = bin.legs[i].shin;
					pin.legAccel[i] = bin.legs[i].accelWorld;
				}
				pin.hasWaist = bin.hasWaist;
				pin.waist = bin.waist;
				pin.waistAccel = waistAccel;
				poseModel.setHeight(dims.height);
				fuser.update(poseModel.update(pin, t), hmd, floorY, dims.eyeHeight, dims.thigh + dims.shin + 0.06, dt, o,
							 fusion);
			}
			solveSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - solveStart).count();

			double rel = t - t0;
			if (rel < from || rel > to) continue;
			frames++;
			if (dump) {
				double row[kDumpColumns] = {};
				size_t c = 0;
				auto put = [&](double v) { row[c++] = v; };
				auto putQ = [&](const Quat& q) { put(q.w); put(q.x); put(q.y); put(q.z); };
				auto putV = [&](const Vec3& v) { put(v.x); put(v.y); put(v.z); };
				put(rel);
				put(bin.hmdValid);
				putV(hmd.position);
				putQ(hmd.rotation);
				put(floorY);
				put(dims.eyeHeight);
				put(dims.height);
				put(bin.hasWaist);
				putQ(bin.waist);
				putV(waistAccel);
				for (int i = 0; i < 2; i++) {
					put(bin.legs[i].valid);
					putQ(bin.legs[i].shin);
					putV(bin.legs[i].accelWorld);
				}
				for (int i = 0; i < 2; i++) {
					put(controllerValid[i]);
					putV(controllers[i]);
					putQ(controllerRot[i]);
				}
				put(o.valid);
				putV(o.waist.position);
				putV(o.feet[0].position);
				putV(o.feet[1].position);
				std::fwrite(row, sizeof(double), kDumpColumns, dump);
			}
			Vec3 hf = hmd.rotation.rotate({0, 0, -1});
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.1f,", rel, hmd.position.x, hmd.position.y, hmd.position.z,
						 rad2deg(std::asin(std::clamp(hf.y, -1.0, 1.0))));
			// Recorded outputs: waist [valid, x, y, z, qw, qx, qy, qz], feet [+-valid, x, y, z]
			// with the sign of the foot's valid flag carrying "planted", knee angles
			std::fprintf(csv, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%d,%d,", int(f[0]), f[1],
						 f[2], f[3], f[9], f[10], f[11], f[13], f[14], f[15], f[16], f[17], f[8] > 0 ? 1 : 0,
						 f[12] > 0 ? 1 : 0);
			std::fprintf(csv, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%d,%d,", o.valid ? 1 : 0,
						 o.waist.position.x, o.waist.position.y, o.waist.position.z, o.feet[0].position.x,
						 o.feet[0].position.y, o.feet[0].position.z, o.feet[1].position.x, o.feet[1].position.y,
						 o.feet[1].position.z, o.kneeAngles[0], o.kneeAngles[1], o.planted[0] ? 1 : 0,
						 o.planted[1] ? 1 : 0);
			std::fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,", bin.waist.rotate({0, 1, 0}).y,
						 bin.legs[0].shin.rotate({0, 1, 0}).y, bin.legs[1].shin.rotate({0, 1, 0}).y, accel[0].y,
						 accel[1].y, accel[0].norm(), accel[1].norm(), gyro[0], gyro[1]);
			std::fprintf(csv, "%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,", o.debug.standing, o.debug.hip0.y,
						 o.debug.ins[0].y, o.debug.ins[1].y, o.debug.insSigma[0], o.debug.insSigma[1],
						 o.debug.anchors[0].y, o.debug.anchors[1].y, floorY);
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,", floorY + dims.ankleHeight, dims.eyeHeight,
						 controllers[0].x, controllers[0].y, controllers[0].z, controllers[1].x, controllers[1].y,
						 controllers[1].z);
			// Orientations as yaw and tilt of the up axis (deg)
			auto tilt = [](const Quat& q) { return rad2deg(std::acos(std::clamp(q.rotate({0, 1, 0}).y, -1.0, 1.0))); };
			Quat recWaist{f[4], f[5], f[6], f[7]};
			std::fprintf(csv, "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.4f,%.4f,", rad2deg(hmd.rotation.yawAngle()),
						 rad2deg(recWaist.yawAngle()), tilt(recWaist), rad2deg(o.waist.rotation.yawAngle()),
						 tilt(o.waist.rotation), rad2deg(bin.legs[0].shin.yawAngle()),
						 rad2deg(bin.legs[1].shin.yawAngle()), o.debug.hip0.x, o.debug.hip0.z);
			const auto& dbg = o.debug;
			std::fprintf(csv, "%.3f,%.3f,%.4f,%.4f,", dbg.touchdownSwing[0], dbg.touchdownSwing[1],
						 dbg.touchdownHeight[0], dbg.touchdownHeight[1]);
			// Thigh direction in the pelvis frame: flexion (sagittal) and abduction
			// (outward > 0), deg
			std::array<double, 2> flex{}, abd{};
			for (int i = 0; i < 2; i++) {
				Vec3 th = o.waist.rotation.conj().rotate(o.knees[i] - dbg.hipJoints[i]);
				flex[i] = rad2deg(std::atan2(-th.z, -th.y));
				abd[i] = rad2deg(std::atan2(th.x * (i == 0 ? -1 : 1), std::hypot(th.y, th.z)));
			}
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f,", o.knees[0].x, o.knees[0].y,
						 o.knees[0].z, o.knees[1].x, o.knees[1].y, o.knees[1].z, flex[0], flex[1], abd[0], abd[1]);
			const Vec3 pelvisUp = o.waist.rotation.rotate({0, 1, 0});
			// SteamVR foot point, as the engine places it (instep, ahead of the ankle)
			Vec3 toe[2];
			for (int i = 0; i < 2; i++)
				toe[i] = o.feet[i].position + o.feet[i].rotation.rotate(Vec3{0, 0, -0.045 * dims.height});
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.3f,%.3f,%d,%d,%.3f,%.3f,%.2f,%.2f,%d,%d\n", dbg.ins[0].x,
						 dbg.ins[0].z, dbg.ins[1].x, dbg.ins[1].z, bin.hasWaist ? bin.waist.rotate({0, 0, 1}).y : 0.0,
						 pelvisUp.x, pelvisUp.z, toe[0].x, toe[0].y, toe[0].z, toe[1].x, toe[1].y, toe[1].z,
						 rad2deg(dbg.hingeYaw[0]), rad2deg(dbg.hingeYaw[1]), dbg.hold[0], dbg.hold[1],
						 int(dbg.holdLatched[0]), int(dbg.holdLatched[1]), dbg.lowerDisp[0], dbg.lowerDisp[1],
						 dbg.crossed[0], dbg.crossed[1], int(dbg.crossRest[0]), int(dbg.crossRest[1]));
		}
	}
	std::fclose(csv);
	if (dump) std::fclose(dump);
	std::printf("%d frames written, floor %.3f%s, solver %.2f s\n", frames, floorY, floorKnown ? "" : " (unknown)",
				solveSeconds);
	return 0;
}
