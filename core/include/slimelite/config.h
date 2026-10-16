// Persistent configuration, stored as JSON in %LOCALAPPDATA%\SlimeVR-lite\config.json
#pragma once

#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "slimelite/linalg.h"
#include "slimelite/mag.h"

namespace slimelite {

enum class Role { None, Waist, LeftAnkle, RightAnkle };
enum class Mode { Three, Ankles };

const char* roleName(Role r);
Role roleFromName(const std::string& s);
const char* modeName(Mode m);
Mode modeFromName(const std::string& s);

struct Settings {
	bool magEnabled = true;
	double magSensitivity = 0.5;
	double smoothing = 0.3;
	double footPlantStrength = 0.7;
	double inertialWeight = 0.5;
	bool velocityPrediction = false;
	bool recording = true;  // record sessions for offline tuning
};

struct TrackerConfig {
	Role role = Role::None;
	// Segment orientation = Ry(yawAlign) * q_corrected * mounting
	bool mountingValid = false;
	Quat mounting;
	double yawAlign = 0;
	// yawAlign was solved in the magnetic frame (absolute, survives tracker reboots)
	// rather than in the device's arbitrary boot frame
	bool yawInMagFrame = false;
	MagCalibration mag;
};

struct Config {
	Mode mode = Mode::Three;
	double userHeight = 1.75;
	std::optional<double> measuredEyeHeight;
	Settings settings;
	std::map<std::string, TrackerConfig> trackers;  // by MAC

	static std::filesystem::path defaultPath();
	nlohmann::json toJson() const;
	bool fromJson(const nlohmann::json& j);
	bool load(const std::filesystem::path& path);
	bool save(const std::filesystem::path& path) const;
};

}  // namespace slimelite
