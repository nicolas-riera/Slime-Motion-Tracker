// Runs the engine like the SteamVR driver does, without SteamVR: a fake standing
// HMD at 90 Hz, hot reload when the engine DLL changes. Lets the reload path and
// the recorder be tested from the command line.
//
// Usage: slimelite_engine_host <path\to\slimelite_engine.dll> [--seconds N]
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

#include "../../driver/src/engine_loader.h"

using namespace slimelite;

int main(int argc, char** argv) {
	if (argc < 2) {
		std::printf("usage: %s slimelite_engine.dll [--seconds N]\n", argv[0]);
		return 1;
	}
	double seconds = 1e9;
	for (int i = 2; i + 1 < argc; i++)
		if (!std::strcmp(argv[i], "--seconds")) seconds = std::atof(argv[i + 1]);

	std::filesystem::path engine = std::filesystem::absolute(argv[1]);
	std::string dir = engine.parent_path().string();
	EngineLoader loader(engine, dir, [](const std::string& msg) { std::printf("[engine] %s\n", msg.c_str()); });
	if (!loader.loaded()) return 2;

	using clock = std::chrono::steady_clock;
	auto now = [] { return std::chrono::duration<double>(clock::now().time_since_epoch()).count(); };
	double start = now(), lastPrint = 0;
	static SlFrameInput in;
	SlFrameOutput out{};
	while (now() - start < seconds) {
		double t = now();
		loader.poll(t);
		in = SlFrameInput{};
		in.now = t;
		in.hmdValid = 1;
		// Identity rotation, eyes 1.65 m above a floor at y = 0
		in.hmd[0][0] = in.hmd[1][1] = in.hmd[2][2] = 1;
		in.hmd[1][3] = 1.65;
		in.deviceCount = 1;
		in.devices[0].connected = in.devices[0].valid = 1;
		in.devices[0].deviceClass = 1;
		std::memcpy(in.devices[0].pose, in.hmd, sizeof(in.hmd));
		std::strcpy(in.devices[0].serial, "FAKE-HMD");
		loader.frame(in, out);
		if (t - lastPrint > 2.0) {
			lastPrint = t;
			std::printf("t=%5.1f outputs valid: waist %d, left %d, right %d\n", t - start,
						out.devices[0].valid, out.devices[1].valid, out.devices[2].valid);
			std::fflush(stdout);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(11));
	}
	return 0;
}
