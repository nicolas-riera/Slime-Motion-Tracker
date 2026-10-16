// Replays a SteamVR session recording (engine/src/recorder.h) through the body
// solver, at the recorded engine frame times, with the inputs the engine had
// (config, tracker rotations and accelerations, HMD pose), and writes a CSV next
// to the recorded outputs. Lets the solver be changed and compared on real
// sessions without SteamVR (tools/plot_replay.py plots the CSV).
//
// Usage: slimelite_body_replay session.jsonl out.csv [--from S] [--to S] [--floor Y]
//   --from/--to: seconds from the start of the recording
//   --floor: floor height in raw tracking space (default: from the recording)
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "slimelite/body_solver.h"
#include "slimelite/config.h"
#include "slimelite/protocol.h"

using namespace slimelite;
using nlohmann::json;

namespace {

struct TrackerState {
	bool hasRotation = false;
	Quat qWorld, prevRotation;
	double prevTime = 0;
	double angularSpeed = 0;
	Vec3 accelLocal;
	bool hasHeading = false;
	double heading = 0;
};

Vec3 vec(const json& a, size_t offset = 0) {
	return {a.at(offset).get<double>(), a.at(offset + 1).get<double>(), a.at(offset + 2).get<double>()};
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: %s session.jsonl out.csv [--from S] [--to S] [--floor Y]\n", argv[0]);
		return 1;
	}
	double from = 0, to = 1e18;
	std::optional<double> floorArg;
	for (int i = 3; i + 1 < argc; i += 2) {
		std::string k = argv[i];
		double v = std::atof(argv[i + 1]);
		if (k == "--from") from = v;
		else if (k == "--to") to = v;
		else if (k == "--floor") floorArg = v;
	}

	std::ifstream in(argv[1]);
	if (!in) {
		std::printf("cannot open %s\n", argv[1]);
		return 1;
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
				 "lkx,lky,lkz,rkx,rky,rkz,l_flex,r_flex,l_abd,r_abd\n");

	Config config;
	bool haveConfig = false;
	std::map<std::string, TrackerState> trackers;
	Pose hmd;
	bool hmdValid = false;
	std::array<Vec3, 2> controllers;  // left, right hand (for diagnosis only)
	double floorY = floorArg.value_or(0);
	bool floorKnown = floorArg.has_value();
	// Old recordings have no floor line: floor = HMD height at the stand capture -
	// the eye height the next config reports
	std::optional<double> standHmdY;
	BodySolver solver;
	double t0 = -1, lastFrame = -1;
	int frames = 0;
	double solveSeconds = 0;

	BodyDimensions dims;
	auto configureSolver = [&] {
		dims = BodyDimensions::fromHeight(config.userHeight, config.measuredEyeHeight);
		solver.configure(dims, config.settings, config.mode);
	};

	std::string line;
	while (std::getline(in, line)) {
		// Skip the kinds the replay does not use without parsing them (speed)
		if (line.find("\"k\":\"mag\"") != std::string::npos || line.find("\"k\":\"bat\"") != std::string::npos)
			continue;
		json j = json::parse(line, nullptr, false);
		if (j.is_discarded() || !j.contains("k")) continue;
		double t = j.value("t", 0.0);
		if (t0 < 0) t0 = t;
		const std::string k = j["k"].get<std::string>();

		if (k == "cfg") {
			config.fromJson(j["config"]);
			haveConfig = true;
			configureSolver();
			if (!floorArg && standHmdY && config.measuredEyeHeight) {
				floorY = *standHmdY - *config.measuredEyeHeight;
				floorKnown = true;
				standHmdY.reset();
			}
		} else if (k == "ev") {
			std::string what = j.value("what", ""), detail = j.value("detail", "");
			if (what == "floor" && !floorArg) {
				floorY = std::atof(detail.c_str());
				floorKnown = true;
			} else if (what == "calibration") {
				if (detail.rfind("ski:", 0) == 0 || detail.rfind("done: Reset", 0) == 0) standHmdY = hmd.position.y;
				if (detail.rfind("done:", 0) == 0) solver.reset();
			}
		} else if (k == "rot") {
			if (j.value("s", 0) != 0) continue;
			auto q = j["q"];
			Quat qw = proto::firmwareToWorld(
				Quat{q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()});
			TrackerState& ts = trackers[j["mac"].get<std::string>()];
			// Same angular speed estimate as the tracker hub
			if (ts.prevTime > 0) {
				double dt = t - ts.prevTime;
				if (dt > 1e-4 && dt < 0.5) {
					double w = (ts.prevRotation.conj() * qw).angle() / dt;
					ts.angularSpeed += (w - ts.angularSpeed) * std::min(1.0, dt / 0.05);
				}
			}
			ts.prevRotation = qw;
			ts.prevTime = t;
			ts.qWorld = qw;
			ts.hasRotation = true;
		} else if (k == "acc") {
			if (j.value("s", 0) != 0) continue;
			trackers[j["mac"].get<std::string>()].accelLocal = vec(j["a"]);
		} else if (k == "trk") {
			TrackerState& ts = trackers[j["mac"].get<std::string>()];
			ts.hasHeading = j.value("hh", 0) != 0;
			ts.heading = j.value("hdg", 0.0);
		} else if (k == "dev") {
			int cls = j.value("cls", 0), role = j.value("role", 0);
			if (cls == 2 && (role == 1 || role == 2)) controllers[role - 1] = vec(j["p"]);
			if (cls != 1) continue;
			hmdValid = j.value("ok", 0) != 0;
			auto q = j["q"];
			hmd.position = vec(j["p"]);
			hmd.rotation = Quat{q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()};
		} else if (k == "out") {
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
					bin.hasWaist = true;
					bin.waist = seg;
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
			BodyOutput o = solver.update(bin);
			solveSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - solveStart).count();
			if (std::getenv("REPLAY_DEBUG") && frames % 5000 == 0) {
				std::printf("t=%.1f cfg trackers %zu, states %zu, legs %d %d hmd %d floor %d\n", t - t0,
							config.trackers.size(), trackers.size(), bin.legs[0].valid, bin.legs[1].valid,
							bin.hmdValid, floorKnown);
				for (const auto& [mac, tc] : config.trackers)
					std::printf("  %s role %d mounting %d rot %d\n", mac.c_str(), int(tc.role), tc.mountingValid,
								trackers.count(mac) ? trackers[mac].hasRotation : -1);
			}

			double rel = t - t0;
			if (rel < from || rel > to) continue;
			frames++;
			Vec3 hf = hmd.rotation.rotate({0, 0, -1});
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.1f,", rel, hmd.position.x, hmd.position.y, hmd.position.z,
						 rad2deg(std::asin(std::clamp(hf.y, -1.0, 1.0))));
			const json &w = j["waist"], &lf = j["lf"], &rf = j["rf"];
			std::fprintf(csv, "%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%d,%d,",
						 w[0].get<int>(), w[1].get<double>(), w[2].get<double>(), w[3].get<double>(),
						 lf[1].get<double>(), lf[2].get<double>(), lf[3].get<double>(), rf[1].get<double>(),
						 rf[2].get<double>(), rf[3].get<double>(), j["kn"][0].get<double>(),
						 j["kn"][1].get<double>(), j["pl"][0].get<int>(), j["pl"][1].get<int>());
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
			Quat recWaist{w[4].get<double>(), w[5].get<double>(), w[6].get<double>(), w[7].get<double>()};
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
			std::fprintf(csv, "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%.1f,%.1f,%.1f\n", o.knees[0].x, o.knees[0].y,
						 o.knees[0].z, o.knees[1].x, o.knees[1].y, o.knees[1].z, flex[0], flex[1], abd[0], abd[1]);
		}
	}
	std::fclose(csv);
	std::printf("%d frames written, floor %.3f%s, solver %.2f s\n", frames, floorY, floorKnown ? "" : " (unknown)",
				solveSeconds);
	return 0;
}
