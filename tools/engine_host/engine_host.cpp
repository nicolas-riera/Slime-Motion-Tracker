// Runs the engine like the SteamVR driver does, without SteamVR: a fake standing
// HMD at 90 Hz, hot reload when the engine DLL changes. Lets the reload path and
// the recorder be tested from the command line. The overlay also starts it when
// SteamVR is not running (standalone mode: tracker status without a headset),
// with --parent (exit with the overlay) and --until-steamvr (exit when SteamVR
// starts, so the driver gets the tracker port and the dongle).
//
// Usage: slimelite_engine_host <path\to\slimelite_engine.dll> [--seconds N]
//        [--parent PID] [--until-steamvr]
#include <windows.h>
#include <tlhelp32.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

#include "../../driver/src/engine_loader.h"

using namespace slimelite;

namespace {

bool processRunning(const wchar_t* exeName) {
	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return false;
	PROCESSENTRY32W e{};
	e.dwSize = sizeof(e);
	bool found = false;
	for (BOOL ok = Process32FirstW(snap, &e); ok && !found; ok = Process32NextW(snap, &e))
		found = _wcsicmp(e.szExeFile, exeName) == 0;
	CloseHandle(snap);
	return found;
}

}  // namespace

int main(int argc, char** argv) {
	if (argc < 2) {
		std::printf("usage: %s slimelite_engine.dll [--seconds N]\n", argv[0]);
		return 1;
	}
	double seconds = 1e9;
	HANDLE parent = nullptr;
	bool untilSteamVr = false;
	for (int i = 2; i < argc; i++) {
		if (!std::strcmp(argv[i], "--seconds") && i + 1 < argc) seconds = std::atof(argv[++i]);
		else if (!std::strcmp(argv[i], "--parent") && i + 1 < argc)
			parent = OpenProcess(SYNCHRONIZE, FALSE, DWORD(std::atoi(argv[++i])));
		else if (!std::strcmp(argv[i], "--until-steamvr")) untilSteamVr = true;
	}

	std::filesystem::path engine = std::filesystem::absolute(argv[1]);
	std::string dir = engine.parent_path().string();
	EngineLoader loader(engine, dir, [](const std::string& msg) { std::printf("[engine] %s\n", msg.c_str()); });
	if (!loader.loaded()) return 2;

	using clock = std::chrono::steady_clock;
	auto now = [] { return std::chrono::duration<double>(clock::now().time_since_epoch()).count(); };
	double start = now(), lastPrint = 0, lastCheck = 0;
	static SlFrameInput in;
	SlFrameOutput out{};
	while (now() - start < seconds) {
		double t = now();
		if (t - lastCheck > 0.5) {
			lastCheck = t;
			if (parent && WaitForSingleObject(parent, 0) == WAIT_OBJECT_0) break;
			if (untilSteamVr && processRunning(L"vrserver.exe")) break;
		}
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
