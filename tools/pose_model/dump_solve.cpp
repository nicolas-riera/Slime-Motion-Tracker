// Runs the body solver (and optionally the pose model fusion) on per-frame inputs
// in the body_replay --dump layout: a recorded session's dump, or motion capture
// converted to our sensors (tools/pose_model/amass_bench.py), to score the
// solver against exact ground truth.
//
// Usage: smt_dump_solve inputs.bin out.bin [--ankles] [--pose-model model.bin]
//        [--wrist-identity] [--stand T]
// out.bin: float64 rows t, valid, waist xyz, left ankle xyz, right ankle xyz, pose
// model weight, knees (columns in out.bin.json). SMT_TUNE applies to the solver as usual.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "smt/body_solver.h"
#include "smt/pose_model.h"

using namespace smt;

int main(int argc, char** argv) {
	if (argc < 3) {
		std::fprintf(stderr, "usage: %s inputs.bin out.bin [--ankles] [--pose-model m.bin] [--wrist-identity] [--stand T]\n",
					 argv[0]);
		return 1;
	}
	bool ankles = false, wristIdentity = false;
	std::string posePath;
	double stand = -1;
	for (int i = 3; i < argc; i++) {
		std::string a = argv[i];
		if (a == "--ankles") ankles = true;
		else if (a == "--wrist-identity") wristIdentity = true;
		else if (a == "--pose-model" && i + 1 < argc) posePath = argv[++i];
		else if (a == "--stand" && i + 1 < argc) stand = std::atof(argv[++i]);
	}
	std::ifstream jf(std::string(argv[1]) + ".json");
	if (!jf) {
		std::fprintf(stderr, "no %s.json\n", argv[1]);
		return 1;
	}
	auto cols = nlohmann::json::parse(jf).at("columns").get<std::vector<std::string>>();
	std::map<std::string, size_t> c;
	for (size_t i = 0; i < cols.size(); i++) c[cols[i]] = i;
	std::ifstream bf(argv[1], std::ios::binary);
	bf.seekg(0, std::ios::end);
	size_t bytes = static_cast<size_t>(bf.tellg());
	bf.seekg(0);
	std::vector<double> data(bytes / sizeof(double));
	bf.read(reinterpret_cast<char*>(data.data()), bytes);
	const size_t nc = cols.size(), n = data.size() / nc;
	if (n == 0) return 1;
	// Older dumps lack some columns (waist acceleration): read them as 0
	auto at = [&](size_t row, const std::string& name) {
		auto it = c.find(name);
		return it == c.end() ? 0.0 : data[row * nc + it->second];
	};
	auto vec = [&](size_t row, const std::string& p) { return Vec3{at(row, p + "x"), at(row, p + "y"), at(row, p + "z")}; };
	auto quat = [&](size_t row, const std::string& p) {
		return Quat{at(row, p + "qw"), at(row, p + "qx"), at(row, p + "qy"), at(row, p + "qz")};
	};

	BodySolver solver;
	Settings settings;
	solver.configure(BodyDimensions::fromHeight(at(0, "height"), at(0, "eye_h")), settings,
					 ankles ? Mode::Ankles : Mode::Three);
	const BodyDimensions dims = BodyDimensions::fromHeight(at(0, "height"), at(0, "eye_h"));
	PoseModel pose;
	if (!posePath.empty()) {
		if (!pose.load(posePath)) {
			std::fprintf(stderr, "pose model: %s\n", pose.error().c_str());
			return 1;
		}
		pose.setHeight(dims.height);
		if (wristIdentity) pose.setWristCalibration({Quat{}, Quat{}});
	}

	std::FILE* out = std::fopen(argv[2], "wb");
	std::ofstream(std::string(argv[2]) + ".json")
		<< nlohmann::json{{"columns", {"t", "valid", "wx", "wy", "wz", "lx", "ly", "lz", "rx", "ry", "rz", "pose_w", "lkx", "lky", "lkz", "rkx", "rky", "rkz"}}}.dump();
	std::array<Quat, 2> prevShin;
	std::array<double, 2> gyro{};
	double prevT = -1;
	bool calibrated = wristIdentity;
	for (size_t r = 0; r < n; r++) {
		const double t = at(r, "t");
		const double dt = prevT >= 0 ? t - prevT : 1.0 / 60;
		BodyInput in;
		in.dt = dt;
		in.hmdValid = at(r, "hmd_valid") > 0;
		in.hmd = {vec(r, "h"), quat(r, "h")};
		in.floorY = at(r, "floor");
		in.hasWaist = !ankles && at(r, "w_valid") > 0;
		in.waist = quat(r, "w");
		in.waistAccel = vec(r, "wa");
		for (int i = 0; i < 2; i++) {
			const std::string p = i == 0 ? "l" : "r";
			auto& leg = in.legs[i];
			leg.valid = at(r, p + "_valid") > 0;
			leg.shin = quat(r, p);
			leg.accelWorld = vec(r, p + "a");
			// Angular speed from the orientation change, low-passed like the replay
			if (prevT >= 0) {
				double w = (prevShin[i].conj() * leg.shin).normalized().angle() / std::max(dt, 1e-4);
				gyro[i] += (w - gyro[i]) * std::min(1.0, dt / 0.05);
			}
			prevShin[i] = leg.shin;
			leg.angularSpeed = gyro[i];
		}
		prevT = t;
		BodyOutput o = solver.update(in);
		double w = 0;
		if (pose.loaded()) {
			PoseModelInput pin;
			pin.floorY = in.floorY;
			pin.hmdValid = in.hmdValid;
			pin.hmd = in.hmd;
			for (int i = 0; i < 2; i++) {
				const std::string p = i == 0 ? "c0" : "c1";
				pin.handValid[i] = at(r, p + "_valid") > 0;
				pin.hands[i] = {vec(r, p), quat(r, p)};
				pin.legValid[i] = in.legs[i].valid;
				pin.shin[i] = in.legs[i].shin;
				pin.legAccel[i] = in.legs[i].accelWorld;
			}
			pin.hasWaist = in.hasWaist;
			pin.waist = in.waist;
			pin.waistAccel = vec(r, "wa");
			if (!calibrated && stand >= 0 && t >= stand) {
				pose.calibrateWrists(pin.hmd, pin.hands);
				calibrated = true;
			}
			w = fusePose(pose.update(pin, t), in.hmd, in.floorY, dims.eyeHeight, o, PoseFusionSettings::forWaist(in.hasWaist));
		}
		double row[18] = {t, o.valid ? 1.0 : 0.0, o.waist.position.x, o.waist.position.y, o.waist.position.z,
						  o.feet[0].position.x, o.feet[0].position.y, o.feet[0].position.z,
						  o.feet[1].position.x, o.feet[1].position.y, o.feet[1].position.z, w,
						  o.knees[0].x, o.knees[0].y, o.knees[0].z, o.knees[1].x, o.knees[1].y, o.knees[1].z};
		std::fwrite(row, sizeof(double), 18, out);
	}
	std::fclose(out);
	return 0;
}
