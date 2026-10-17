// Replays the mag heading filter on a SteamVR session recording (engine/src/recorder.h)
// with the current code, next to the headings the engine recorded live, to tune the
// anti-mag on real sessions. The calibrations come from the recorded config lines.
//
// Usage: slimelite_heading_replay session.jsonl out.csv [--sensitivity S]
// CSV: t (relative), mac, role, heading_deg (replayed), state, rec_heading_deg
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "slimelite/config.h"
#include "slimelite/mag.h"
#include "slimelite/protocol.h"

using namespace slimelite;
using nlohmann::json;

namespace {

struct Track {
	HeadingFilter filter;
	MagCalibration cal;
	Quat qWorld, prevRotation;
	bool hasRotation = false;
	double prevRotTime = 0, angularSpeed = 0, lastMag = 0, recHeading = 0;
	int lastOutput = -1;
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
	for (int i = 3; i + 1 < argc; i += 2)
		if (std::string(argv[i]) == "--sensitivity") settings.sensitivity = std::atof(argv[i + 1]);
	std::ifstream in(argv[1]);
	FILE* out = std::fopen(argv[2], "w");
	if (!in || !out) {
		std::printf("cannot open the files\n");
		return 1;
	}
	std::fprintf(out, "t,mac,role,heading_deg,state,rec_heading_deg\n");

	Config config;
	std::map<std::string, Track> tracks;
	double t0 = -1;
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
		} else if (k == "trk") {
			tracks[j["mac"].get<std::string>()].recHeading = rad2deg(j.value("hdg", 0.0));
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
			tr.filter.update(tr.qWorld, raw, tr.angularSpeed, dt, settings);
			int second = int((t - t0) * 2);  // 2 Hz output
			if (second != tr.lastOutput) {
				tr.lastOutput = second;
				auto it = config.trackers.find(mac);
				const char* role = it != config.trackers.end() ? roleName(it->second.role) : "none";
				std::fprintf(out, "%.2f,%s,%s,%.2f,%s,%.2f\n", t - t0, mac.c_str(), role,
							 rad2deg(tr.filter.heading()), stateName(tr.filter.state()), tr.recHeading);
			}
		}
	}
	std::fclose(out);
	return 0;
}
