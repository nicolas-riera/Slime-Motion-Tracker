// Replays a JSONL recording (tools/slime_server.py --record) through the core:
// mag calibration on the whole recording, then the heading filter in real time
// order, printing one line per second.
//
// Usage: slimelite_replay recording.jsonl [--sensitivity 0.5] [--fit-from T --fit-to T]
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>

#include "slimelite/mag.h"
#include "slimelite/protocol.h"

using namespace slimelite;
using nlohmann::json;

int main(int argc, char** argv) {
	if (argc < 2) {
		std::printf("usage: %s recording.jsonl [--sensitivity S] [--fit-from T] [--fit-to T]\n", argv[0]);
		return 1;
	}
	double sensitivity = 0.5, fitFrom = 0, fitTo = 1e9;
	for (int i = 2; i + 1 < argc; i += 2) {
		std::string k = argv[i];
		double v = std::atof(argv[i + 1]);
		if (k == "--sensitivity") sensitivity = v;
		if (k == "--fit-from") fitFrom = v;
		if (k == "--fit-to") fitTo = v;
	}

	struct Row {
		double t;
		std::string kind;
		Quat q;
		std::array<int16_t, 3> m{};
	};
	std::vector<Row> rows;
	std::ifstream in(argv[1]);
	std::string line;
	double t0 = -1;
	while (std::getline(in, line)) {
		json j = json::parse(line, nullptr, false);
		if (j.is_discarded()) continue;
		Row r;
		r.t = j["t"].get<double>();
		if (t0 < 0) t0 = r.t;
		r.t -= t0;
		r.kind = j["kind"].get<std::string>();
		if (r.kind == "rot") {
			auto q = j["q"];
			r.q = proto::firmwareToWorld(Quat{q[3].get<double>(), q[0].get<double>(), q[1].get<double>(), q[2].get<double>()});
		} else if (r.kind == "mag") {
			auto m = j["m"];
			r.m = {int16_t(m[0].get<int>()), int16_t(m[1].get<int>()), int16_t(m[2].get<int>())};
		} else {
			continue;
		}
		rows.push_back(r);
	}
	std::printf("%zu rows\n", rows.size());

	// Pass 1: calibration on the selected window, as the guided calibration would
	MagCalibrator cal;
	Quat q;
	bool hasQ = false;
	double prevT = 0, w = 0;
	Quat prevQ;
	for (const auto& r : rows) {
		if (r.kind == "rot") {
			if (hasQ && r.t - prevT > 1e-4) w = (prevQ.conj() * r.q).angle() / (r.t - prevT);
			prevQ = r.q;
			prevT = r.t;
			q = r.q;
			hasQ = true;
		} else if (hasQ && r.t >= fitFrom && r.t <= fitTo) {
			cal.addSample(q, r.m, w, r.t);
		}
	}
	std::printf("coverage %.2f, %zu samples\n", cal.coverage(), cal.sampleCount());
	auto c = cal.solve();
	if (!c) {
		std::printf("calibration FAILED (degenerate or implausible)\n");
		return 2;
	}
	std::printf("calibration: dip %.1f deg, residual %.4f\n", rad2deg(c->dip), c->residual);

	// Pass 2: heading filter
	HeadingFilter hf;
	hf.setCalibration(*c);
	HeadingFilterSettings s;
	s.sensitivity = sensitivity;
	hasQ = false;
	double lastMag = -1, lastPrint = -1;
	std::map<std::string, int> stateTime;
	const char* names[] = {"uncalibrated", "acquiring", "ok", "DISTURBED"};
	for (const auto& r : rows) {
		if (r.kind == "rot") {
			if (hasQ && r.t - prevT > 1e-4) w = (prevQ.conj() * r.q).angle() / (r.t - prevT);
			prevQ = r.q;
			prevT = r.t;
			q = r.q;
			hasQ = true;
			continue;
		}
		if (!hasQ) continue;
		double dt = lastMag < 0 ? 0.02 : std::min(0.2, r.t - lastMag);
		lastMag = r.t;
		hf.update(q, r.m, w, dt, s);
		stateTime[names[int(hf.state())]]++;
		if (r.t - lastPrint >= 1.0) {
			lastPrint = r.t;
			Quat corrected = Quat::yaw(-hf.heading()) * q;
			std::printf("t=%6.1f  %-12s heading %7.2f  corrected yaw %7.2f  norm %.3f  dip %5.1f  w %.1f\n",
						r.t, names[int(hf.state())], rad2deg(hf.heading()),
						rad2deg(corrected.yawAngle()), hf.lastNorm(), rad2deg(hf.lastDip()), w);
		}
	}
	for (auto& [k, v] : stateTime) std::printf("%s: %d samples\n", k.c_str(), v);
	return 0;
}
