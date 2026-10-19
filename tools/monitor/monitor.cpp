// Live diagnostics without SteamVR: runs the tracker hub and prints each tracker
// once per second. Uses and updates the mag calibrations of the driver config.
//
// Usage: smt_monitor [--magcal MAC] [--seconds N]
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include "smt/config.h"
#include "smt/tracker_hub.h"

using namespace smt;

int main(int argc, char** argv) {
	std::string magcal;
	double seconds = 1e9;
	for (int i = 1; i + 1 < argc; i += 2) {
		if (!std::strcmp(argv[i], "--magcal")) magcal = argv[i + 1];
		if (!std::strcmp(argv[i], "--seconds")) seconds = std::atof(argv[i + 1]);
	}

	Config config;
	config.load(Config::defaultPath());
	TrackerHub hub;
	for (const auto& [mac, tc] : config.trackers)
		if (tc.mag.valid) hub.setStoredMagCalibration(mac, tc.mag);
	hub.onMagCalibration([&](const std::string& mac, const MagCalibration& cal) {
		std::printf(">>> new mag calibration for %s: dip %.1f, residual %.3f\n", mac.c_str(),
					rad2deg(cal.dip), cal.residual);
		config.trackers[mac].mag = cal;
		config.save(Config::defaultPath());
	});
	HeadingFilterSettings hs;
	hs.sensitivity = config.settings.magSensitivity;
	hub.setMagSettings(true, hs);
	if (!hub.start()) {
		std::printf("%s\n", hub.portError().c_str());
		return 1;
	}

	bool guidedStarted = false;
	double start = TrackerHub::now();
	while (TrackerHub::now() - start < seconds) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
		for (const auto& t : hub.snapshot()) {
			if (!magcal.empty() && t.mac == magcal && !guidedStarted && t.connected) {
				hub.startGuidedMagCalibration(magcal);
				guidedStarted = true;
				std::printf(">>> guided mag calibration started: rotate %s in every direction\n", magcal.c_str());
			}
			Quat c = t.corrected();
			std::printf("%s %s rot %3.0fHz mag %2.0fHz | mag %-12s %3.0f%% norm %.2f dip %5.1f | yaw raw %7.1f corrected %7.1f | |a| %.2f w %.2f\n",
						t.mac.c_str(), t.connected ? "up  " : "DOWN", t.rotationHz, t.magHz,
						magStateName(t.magState), t.magProgress * 100, t.magNorm, t.magDip,
						rad2deg(t.qWorld.yawAngle()), rad2deg(c.yawAngle()), t.accelLocal.norm(),
						t.angularSpeed);
		}
		std::fflush(stdout);
	}
	hub.stop();
	return 0;
}
