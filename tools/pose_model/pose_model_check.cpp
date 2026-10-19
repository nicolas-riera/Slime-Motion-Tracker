// Runs the C++ pose model on a body_replay --dump (resampled at 60 Hz like
// tools/pose_model/features.py) and writes both ankles per step, for comparison
// with the Python inference (run_hmdposer.py), and the time per step.
//
// Usage: smt_pose_model_check model.bin inputs.bin stand_time out.csv [max_steps]
#include <chrono>
#include <cstdio>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "smt/pose_model.h"

using namespace smt;

int main(int argc, char** argv) {
	if (argc < 5) {
		std::fprintf(stderr, "usage: %s model.bin inputs.bin stand_time out.csv [max_steps]\n", argv[0]);
		return 1;
	}
	PoseModel model;
	if (!model.load(argv[1])) {
		std::fprintf(stderr, "load: %s\n", model.error().c_str());
		return 1;
	}
	std::ifstream jf(std::string(argv[2]) + ".json");
	auto cols = nlohmann::json::parse(jf).at("columns").get<std::vector<std::string>>();
	std::map<std::string, size_t> c;
	for (size_t i = 0; i < cols.size(); i++) c[cols[i]] = i;
	std::ifstream bf(argv[2], std::ios::binary);
	bf.seekg(0, std::ios::end);
	size_t bytes = static_cast<size_t>(bf.tellg());
	bf.seekg(0);
	std::vector<double> data(bytes / sizeof(double));
	bf.read(reinterpret_cast<char*>(data.data()), bytes);
	const size_t nc = cols.size(), n = data.size() / nc;
	auto at = [&](size_t row, const char* name) { return data[row * nc + c.at(name)]; };
	auto vec = [&](size_t row, const std::string& p) {
		return Vec3{at(row, (p + "x").c_str()), at(row, (p + "y").c_str()), at(row, (p + "z").c_str())};
	};
	auto quat = [&](size_t row, const std::string& p) {
		return Quat{at(row, (p + "qw").c_str()), at(row, (p + "qx").c_str()), at(row, (p + "qy").c_str()),
					at(row, (p + "qz").c_str())};
	};
	auto input = [&](size_t row) {
		PoseModelInput in;
		in.floorY = at(row, "floor");
		in.hmdValid = at(row, "hmd_valid") > 0;
		in.hmd = {vec(row, "h"), quat(row, "h")};
		for (int s = 0; s < 2; s++) {
			std::string p = s == 0 ? "c0" : "c1";
			in.handValid[s] = at(row, (p + "_valid").c_str()) > 0;
			in.hands[s] = {vec(row, p), quat(row, p)};
			std::string l = s == 0 ? "l" : "r";
			in.legValid[s] = at(row, (l + "_valid").c_str()) > 0;
			in.shin[s] = quat(row, l);
			in.legAccel[s] = vec(row, l + "a");
		}
		in.hasWaist = at(row, "w_valid") > 0;
		in.waist = quat(row, "w");
		in.waistAccel = vec(row, "wa");
		return in;
	};
	const double t0 = at(0, "t"), stand = std::atof(argv[3]);
	size_t maxSteps = argc > 5 ? std::stoul(argv[5]) : SIZE_MAX;
	// Stand capture row: nearest at the 60 Hz grid (features.wrist_calibration)
	size_t row = 0, standRow = 0;
	double best = 1e9;
	for (size_t r = 0; r < n; r++) {
		double d = std::fabs(at(r, "t") - stand);
		if (d < best) best = d, standRow = r;
	}
	{
		PoseModelInput s = input(standRow);
		model.calibrateWrists(s.hmd, s.hands);
	}
	std::vector<double> heights;
	for (size_t r = 0; r < n; r += 100) heights.push_back(at(r, "height"));
	std::nth_element(heights.begin(), heights.begin() + heights.size() / 2, heights.end());
	model.setHeight(heights[heights.size() / 2]);
	std::FILE* out = std::fopen(argv[4], "w");
	std::fprintf(out, "t,valid,lx,ly,lz,rx,ry,rz\n");
	double busy = 0;
	size_t steps = 0;
	for (double tq = t0; steps < maxSteps; tq = t0 + double(steps) / 60.0) {
		while (row + 1 < n && at(row + 1, "t") <= tq) row++;
		if (row + 1 >= n) break;
		auto a = std::chrono::steady_clock::now();
		PoseEstimate e = model.step(input(row));
		busy += std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count();
		std::fprintf(out, "%.4f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n", tq, e.valid ? 1 : 0, e.ankle[0].x, e.ankle[0].y,
					 e.ankle[0].z, e.ankle[1].x, e.ankle[1].y, e.ankle[1].z);
		steps++;
	}
	std::fclose(out);
	std::printf("%zu steps, %.3f ms/step\n", steps, busy / steps * 1000);
	return 0;
}
