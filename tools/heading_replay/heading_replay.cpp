// Replays the mag heading filter on a SteamVR session recording (engine/src/recorder.h)
// with the current code, next to the headings the engine recorded live, to tune the
// anti-mag on real sessions. The calibrations come from the recorded config lines.
//
// Usage: smt_heading_replay session.jsonl out.csv [--sensitivity S]
// CSV: t (relative), mac, role, heading_deg (replayed), state, rec_heading_deg, rate_deg_min, sigma_deg,
//      norm (field / calibration norm), dip_dev_deg (dip - calibration dip), speed (rad/s),
//      dev_yaw_deg (device yaw, uncorrected), hmd_yaw_deg
// Options: --sensitivity S, --adaptive 0|1 (local field reference), --ref-time S,
//          --mag-sigma DEG, --corr-time S, --heading-walk DEG_PER_SQRT_MIN, --consistent 0|1,
//          --max-rate DEG_PER_MIN, --fast 0|1 (fast realignment), --restore 1 (start from the
//          recorded live headings instead of acquiring), --autocal S (the tracker hub's automatic
//          calibration, also fed while disturbed for more than S s: prints each replacement)
// mag_deg: the heading the mag measures (the filter's target)
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "smt/config.h"
#include "smt/mag.h"
#include "smt/protocol.h"

using namespace smt;
using nlohmann::json;

namespace {

struct Track {
	HeadingFilter filter;
	MagCalibration cal;
	Quat qWorld, prevRotation;
	bool hasRotation = false;
	double prevRotTime = 0, angularSpeed = 0, lastMag = 0, recHeading = 0;
	double temperature = std::nan("");
	bool restored = false;
	int lastOutput = -1;
	// --autocal: the tracker hub's automatic calibration
	MagCalibrator calibrator;
	double lastSolve = 0, disturbedSince = -1;
};

const char* stateName(HeadingFilter::State s) {
	switch (s) {
		case HeadingFilter::State::Uncalibrated: return "uncalibrated";
		case HeadingFilter::State::Acquiring: return "acquiring";
		case HeadingFilter::State::Ok: return "ok";
		case HeadingFilter::State::Disturbed: return "disturbed";
	}
	return "?";
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: %s session.jsonl out.csv [--sensitivity S]\n", argv[0]);
		return 1;
	}
	HeadingFilterSettings settings;
	bool restore = false;
	double autocal = -1;  // >= 0: automatic calibration, samples kept after that many s disturbed
	for (int i = 3; i + 1 < argc; i += 2)
		if (std::string(argv[i]) == "--sensitivity") settings.sensitivity = std::atof(argv[i + 1]);
		else if (std::string(argv[i]) == "--adaptive") settings.adaptiveRef = std::atoi(argv[i + 1]) != 0;
		else if (std::string(argv[i]) == "--ref-time") settings.refTime = std::atof(argv[i + 1]);
		else if (std::string(argv[i]) == "--mag-sigma") settings.magSigma = deg2rad(std::atof(argv[i + 1]));
		else if (std::string(argv[i]) == "--corr-time") settings.magCorrTime = std::atof(argv[i + 1]);
		else if (std::string(argv[i]) == "--consistent") settings.consistentRelock = std::atoi(argv[i + 1]) != 0;
		else if (std::string(argv[i]) == "--fast") settings.fastRealign = std::atoi(argv[i + 1]) != 0;
		else if (std::string(argv[i]) == "--restore") restore = std::atoi(argv[i + 1]) != 0;
		else if (std::string(argv[i]) == "--autocal") autocal = std::atof(argv[i + 1]);
		else if (std::string(argv[i]) == "--max-rate") settings.maxRate = deg2rad(std::atof(argv[i + 1])) / 60.0;
		else if (std::string(argv[i]) == "--heading-walk")
			settings.headingWalk = deg2rad(std::atof(argv[i + 1])) / std::sqrt(60.0);
	std::ifstream in(argv[1]);
	FILE* out = std::fopen(argv[2], "w");
	if (!in || !out) {
		std::printf("cannot open the files\n");
		return 1;
	}
	std::fprintf(out, "t,mac,role,heading_deg,state,rec_heading_deg,rate_deg_min,sigma_deg,norm,dip_dev_deg,speed,dev_yaw_deg,hmd_yaw_deg,mag_deg\n");

	Config config;
	std::map<std::string, Track> tracks;
	double t0 = -1;
	double hmdYaw = 0;
	bool hasHmd = false;
	std::string line;
	while (std::getline(in, line)) {
		json j = json::parse(line, nullptr, false);
		if (j.is_discarded() || !j.contains("k")) continue;
		double t = j.value("t", 0.0);
		if (t0 < 0) t0 = t;
		const std::string k = j["k"].get<std::string>();
		if (k == "cfg") {
			config.fromJson(j["config"]);
			for (const auto& [mac, tc] : config.trackers) {
				Track& tr = tracks[mac];
				if (!tc.mag.valid) continue;
				bool changed = !tr.cal.valid || tr.cal.residual != tc.mag.residual;
				if (changed) tr.filter.setCalibration(tc.mag);
				tr.cal = tc.mag;
			}
		} else if (k == "dev" && j.value("cls", 0) == 1 && j.contains("q")) {
			auto q = j["q"];
			hmdYaw = 2.0 * std::atan2(q[2].get<double>(), q[0].get<double>());
			hasHmd = true;
		} else if (k == "tmp") {
			tracks[j["mac"].get<std::string>()].temperature = j.value("c", 0.0);
		} else if (k == "trk") {
			Track& tr = tracks[j["mac"].get<std::string>()];
			tr.recHeading = rad2deg(j.value("hdg", 0.0));
			if (restore && !tr.restored && tr.cal.valid) {
				tr.filter.restore(j.value("hdg", 0.0));
				tr.restored = true;
			}
		} else if (k == "rot" && j.value("s", 0) == 0) {
			auto q = j["q"];
			Quat qw = proto::firmwareToWorld(
				Quat{q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()});
			Track& tr = tracks[j["mac"].get<std::string>()];
			if (tr.prevRotTime > 0) {
				double dt = t - tr.prevRotTime;
				if (dt > 1e-4 && dt < 0.5) {
					double ang = (tr.prevRotation.conj() * qw).angle();
					tr.angularSpeed += (ang / dt - tr.angularSpeed) * std::min(1.0, dt / 0.05);
					// Same glitch rule as the tracker hub
					if (ang / dt > 40.0 && ang > deg2rad(20)) tr.filter.gyroGlitch();
				}
			}
			tr.prevRotation = tr.qWorld = qw;
			tr.prevRotTime = t;
			tr.hasRotation = true;
		} else if (k == "mag" && j.value("s", 0) == 0) {
			const std::string mac = j["mac"].get<std::string>();
			Track& tr = tracks[mac];
			if (!tr.hasRotation) continue;
			auto m = j["m"];
			std::array<int16_t, 3> raw{int16_t(m[0].get<int>()), int16_t(m[1].get<int>()), int16_t(m[2].get<int>())};
			double dt = tr.lastMag > 0 ? std::clamp(t - tr.lastMag, 0.0, 0.2) : 0.02;
			tr.lastMag = t;
			if (autocal >= 0 && tr.cal.valid) {
				// Same rules as TrackerHub (core/src/tracker_hub.cpp)
				const bool disturbed = tr.filter.state() == HeadingFilter::State::Disturbed;
				if (!disturbed) tr.disturbedSince = -1;
				else if (tr.disturbedSince < 0) tr.disturbedSince = t;
				if (!disturbed || t - tr.disturbedSince > autocal) tr.calibrator.addSample(tr.qWorld, raw, tr.angularSpeed, t);
				if (t - tr.lastSolve > 20) {
					tr.lastSolve = t;
					if (auto cal = tr.calibrator.solve()) {
						const MagCalibration& cur = tr.filter.calibration();
						double curResidual = cur.valid ? tr.calibrator.residualOf(cur).value_or(cur.residual) : 1e9;
						if (cal->residual < 0.065 && cal->residual < 0.8 * curResidual) {
							tr.filter.updateCalibration(*cal, tr.qWorld, raw);
							std::printf("%.1f %s: calibration replaced, residual %.4f (current %.4f), c %.2f %.2f %.2f\n", t - t0,
										mac.c_str(), cal->residual, curResidual, cal->c.x, cal->c.y, cal->c.z);
						}
					}
				}
			}
			tr.filter.update(tr.qWorld, raw, tr.angularSpeed, dt, settings, tr.temperature);
			int second = int((t - t0) * 2);  // 2 Hz output
			if (second != tr.lastOutput) {
				tr.lastOutput = second;
				auto it = config.trackers.find(mac);
				const char* role = it != config.trackers.end() ? roleName(it->second.role) : "none";
				// Yaws (twist about the vertical): device (uncorrected) and headset
				const double devYaw = 2.0 * std::atan2(tr.qWorld.y, tr.qWorld.w);
				std::fprintf(out, "%.2f,%s,%s,%.2f,%s,%.2f,%.3f,%.2f,%.4f,%.2f,%.3f,%.2f,%s,%.2f\n", t - t0, mac.c_str(), role,
							 rad2deg(tr.filter.heading()), stateName(tr.filter.state()), tr.recHeading,
							 rad2deg(tr.filter.rate()) * 60, rad2deg(tr.filter.headingSigma()), tr.filter.lastNorm(),
							 rad2deg(tr.filter.lastDip() - tr.filter.calibration().dip), tr.angularSpeed,
							 rad2deg(devYaw), hasHmd ? std::to_string(rad2deg(hmdYaw)).c_str() : "",
							 rad2deg(tr.filter.lastMagHeading()));
			}
		}
	}
	std::fclose(out);
	return 0;
}
