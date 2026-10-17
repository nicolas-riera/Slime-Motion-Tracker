// Checks the C++ foot model (core/src/foot_model.cpp) against the Python
// reference (tools/ml/export.py --ref): runs FootModel over every frame of a
// slimelite_ml_dump file and compares its outputs at the reference frames.
//
// Usage: slimelite_foot_model_check dump.bin model.json ref.bin [--bench]
//   ref.bin: float64 rows [frame index, 6 world ankle coordinates, 6 sigmas]
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "slimelite/foot_model.h"

using namespace slimelite;

int main(int argc, char** argv) {
	if (argc < 4) {
		std::printf("usage: %s dump.bin model.json ref.bin\n", argv[0]);
		return 1;
	}
	nlohmann::json meta = nlohmann::json::parse(std::ifstream(std::string(argv[1]) + ".json"));
	auto names = meta["columns"].get<std::vector<std::string>>();
	auto col = [&](const char* n) {
		for (size_t i = 0; i < names.size(); i++)
			if (names[i] == n) return static_cast<int>(i);
		std::printf("missing column %s\n", n);
		std::exit(1);
	};
	const size_t nc = names.size();
	std::vector<double> dump;
	{
		std::ifstream f(argv[1], std::ios::binary | std::ios::ate);
		size_t bytes = static_cast<size_t>(f.tellg());
		dump.resize(bytes / sizeof(double));
		f.seekg(0);
		f.read(reinterpret_cast<char*>(dump.data()), static_cast<std::streamsize>(dump.size() * sizeof(double)));
	}
	std::vector<double> ref;
	{
		std::ifstream f(argv[3], std::ios::binary | std::ios::ate);
		size_t bytes = static_cast<size_t>(f.tellg());
		ref.resize(bytes / sizeof(double));
		f.seekg(0);
		f.read(reinterpret_cast<char*>(ref.data()), static_cast<std::streamsize>(ref.size() * sizeof(double)));
	}
	const size_t nRef = ref.size() / 13;

	FootModel model;
	if (!model.load(argv[2])) {
		std::printf("load failed: %s\n", model.error().c_str());
		return 1;
	}
	int cT = col("t"), cHv = col("hmd_valid"), cHx = col("hx"), cHq = col("hqw"), cFl = col("floor"),
		cWv = col("has_waist"), cWq = col("wqw");
	int cLeg[2] = {col("l_valid"), col("r_valid")};

	size_t frames = dump.size() / nc, next = 0, compared = 0, missing = 0;
	double maxErr = 0, maxSigErr = 0, inferSeconds = 0;
	size_t inferences = 0;
	for (size_t i = 0; i < frames && next < nRef; i++) {
		const double* r = &dump[i * nc];
		BodyInput in;
		in.hmdValid = r[cHv] > 0;
		in.hmd.position = {r[cHx], r[cHx + 1], r[cHx + 2]};
		in.hmd.rotation = {r[cHq], r[cHq + 1], r[cHq + 2], r[cHq + 3]};
		in.floorY = r[cFl];
		in.hasWaist = r[cWv] > 0;
		in.waist = {r[cWq], r[cWq + 1], r[cWq + 2], r[cWq + 3]};
		for (int k = 0; k < 2; k++) {
			const double* l = r + cLeg[k];
			auto& leg = in.legs[k];
			leg.valid = l[0] > 0;
			leg.shin = {l[1], l[2], l[3], l[4]};
			leg.accelWorld = {l[5], l[6], l[7]};
			leg.angularSpeed = l[8];
		}
		auto t0 = std::chrono::steady_clock::now();
		FootEstimate e = model.update(in, r[cT]);
		inferSeconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		inferences++;
		if (static_cast<size_t>(ref[next * 13]) != i) continue;
		const double* q = &ref[next * 13 + 1];
		next++;
		if (!e.valid) {
			missing++;
			continue;
		}
		for (int k = 0; k < 2; k++)
			for (int a = 0; a < 3; a++) {
				maxErr = std::max(maxErr, std::fabs(e.ankle[k][a] - q[3 * k + a]));
				maxSigErr = std::max(maxSigErr, std::fabs(e.sigma[k][a] - q[6 + 3 * k + a]));
			}
		compared++;
	}
	std::printf("%zu reference frames, %zu compared, %zu invalid in C++; max |C++ - Python| ankle %.2e m, sigma %.2e m\n",
				nRef, compared, missing, maxErr, maxSigErr);
	std::printf("update(): %.4f ms per frame on average (%zu frames)\n", 1000 * inferSeconds / inferences, inferences);
	bool ok = compared == nRef && maxErr < 1e-4 && maxSigErr < 1e-4;
	std::printf("%s\n", ok ? "PASS" : "FAIL");
	return ok ? 0 : 2;
}
