// Dumps, for every engine frame of a session recording (engine/src/recorder.h),
// the body solver inputs exactly as the engine builds them (HMD pose, floor,
// pelvis and shin segment orientations, shin world accelerations and angular
// speeds) plus the outputs recorded live, as a raw float64 matrix. Used to train
// and check the learned foot model (tools/ml, core/src/foot_model.cpp).
//
// Usage: smt_ml_dump session.jsonl out.bin
//   out.bin: rows of kColumns float64 (column names in out.bin.json)
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "smt/config.h"
#include "smt/linalg.h"
#include "smt/protocol.h"

using namespace smt;
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

const char* kNames[] = {
	"t", "hmd_valid", "hx", "hy", "hz", "hqw", "hqx", "hqy", "hqz", "floor", "has_waist", "wqw", "wqx", "wqy", "wqz",
	"l_valid", "l_qw", "l_qx", "l_qy", "l_qz", "l_ax", "l_ay", "l_az", "l_gyro",
	"r_valid", "r_qw", "r_qx", "r_qy", "r_qz", "r_ax", "r_ay", "r_az", "r_gyro",
	"rec_valid", "rec_wx", "rec_wy", "rec_wz", "rec_lx", "rec_ly", "rec_lz", "rec_rx", "rec_ry", "rec_rz",
	"rec_pl", "rec_pr", "c0x", "c0y", "c0z", "c1x", "c1y", "c1z"};
constexpr size_t kColumns = sizeof(kNames) / sizeof(kNames[0]);

}  // namespace

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: %s session.jsonl out.bin\n", argv[0]);
		return 1;
	}
	std::ifstream in(argv[1]);
	if (!in) {
		std::printf("cannot open %s\n", argv[1]);
		return 1;
	}
	FILE* out = std::fopen(argv[2], "wb");
	if (!out) {
		std::printf("cannot write %s\n", argv[2]);
		return 1;
	}

	Config config;
	bool haveConfig = false;
	std::map<std::string, TrackerState> trackers;
	Vec3 hmdPos;
	Quat hmdRot;
	bool hmdValid = false;
	std::array<Vec3, 2> controllers;
	double floorY = 0;
	bool floorKnown = false;
	std::optional<double> standHmdY;
	size_t frames = 0;

	std::string line;
	std::vector<double> row(kColumns);
	while (std::getline(in, line)) {
		if (line.find("\"k\":\"mag\"") != std::string::npos || line.find("\"k\":\"bat\"") != std::string::npos)
			continue;
		json j = json::parse(line, nullptr, false);
		if (j.is_discarded() || !j.contains("k")) continue;
		double t = j.value("t", 0.0);
		const std::string k = j["k"].get<std::string>();

		if (k == "cfg") {
			config.fromJson(j["config"]);
			haveConfig = true;
			if (standHmdY && config.measuredEyeHeight) {
				floorY = *standHmdY - *config.measuredEyeHeight;
				floorKnown = true;
				standHmdY.reset();
			}
		} else if (k == "ev") {
			std::string what = j.value("what", ""), detail = j.value("detail", "");
			if (what == "floor") {
				floorY = std::atof(detail.c_str());
				floorKnown = true;
			} else if (what == "calibration") {
				if (detail.rfind("ski:", 0) == 0 || detail.rfind("done: Reset", 0) == 0) standHmdY = hmdPos.y;
			}
		} else if (k == "rot") {
			if (j.value("s", 0) != 0) continue;
			auto q = j["q"];
			Quat qw = proto::firmwareToWorld(
				Quat{q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()});
			TrackerState& ts = trackers[j["mac"].get<std::string>()];
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
			hmdPos = vec(j["p"]);
			hmdRot = Quat{q[0].get<double>(), q[1].get<double>(), q[2].get<double>(), q[3].get<double>()};
		} else if (k == "out") {
			if (!haveConfig) continue;
			std::fill(row.begin(), row.end(), 0.0);
			row[0] = t;
			row[1] = hmdValid && floorKnown ? 1 : 0;
			row[2] = hmdPos.x, row[3] = hmdPos.y, row[4] = hmdPos.z;
			row[5] = hmdRot.w, row[6] = hmdRot.x, row[7] = hmdRot.y, row[8] = hmdRot.z;
			row[9] = floorY;
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
					row[10] = 1;
					row[11] = seg.w, row[12] = seg.x, row[13] = seg.y, row[14] = seg.z;
				} else if (tc.role == Role::LeftAnkle || tc.role == Role::RightAnkle) {
					size_t b = tc.role == Role::LeftAnkle ? 15 : 24;
					Vec3 a = (seg * tc.mounting.conj()).rotate(ts.accelLocal);
					row[b] = 1;
					row[b + 1] = seg.w, row[b + 2] = seg.x, row[b + 3] = seg.y, row[b + 4] = seg.z;
					row[b + 5] = a.x, row[b + 6] = a.y, row[b + 7] = a.z;
					row[b + 8] = ts.angularSpeed;
				}
			}
			const json &w = j["waist"], &lf = j["lf"], &rf = j["rf"];
			row[33] = w[0].get<double>();
			for (int i = 0; i < 3; i++) {
				row[34 + i] = w[1 + i].get<double>();
				row[37 + i] = lf[1 + i].get<double>();
				row[40 + i] = rf[1 + i].get<double>();
			}
			row[43] = j["pl"][0].get<double>();
			row[44] = j["pl"][1].get<double>();
			for (int i = 0; i < 3; i++) {
				row[45 + i] = controllers[0][i];
				row[48 + i] = controllers[1][i];
			}
			std::fwrite(row.data(), sizeof(double), kColumns, out);
			frames++;
		}
	}
	std::fclose(out);
	json meta;
	meta["columns"] = std::vector<std::string>(kNames, kNames + kColumns);
	meta["frames"] = frames;
	meta["userHeight"] = config.userHeight;
	if (config.measuredEyeHeight) meta["eyeHeight"] = *config.measuredEyeHeight;
	std::ofstream(std::string(argv[2]) + ".json") << meta.dump(1);
	std::printf("%zu frames, floor %.3f%s\n", frames, floorY, floorKnown ? "" : " (unknown)");
	return 0;
}
