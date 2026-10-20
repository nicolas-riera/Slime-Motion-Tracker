// Persistent configuration, stored as JSON in %LOCALAPPDATA%\Slime Motion Tracker\config.json
#pragma once

#include <array>
#include <filesystem>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "smt/linalg.h"
#include "smt/mag.h"

namespace smt {

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
	bool recording = false;  // record sessions for offline tuning
	// Fuse the learned pose model (models/pose_model.bin) when present. Off: in VR the v4
	// model moved a foot 70 cm while nothing moved and broke the leg lengths (session
	// 2026-10-19 23:09); the solver alone until that is fixed. Saved as "poseModelFusion"
	// (the old "poseModel" key, true everywhere, is ignored).
	bool poseModel = false;
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
	// Hip center to the right of the head chain (m, negative: left), measured with the
	// controllers on both sides of the pelvis (a torso leaning sideways)
	double hipSideOffset = 0;
	std::optional<double> measuredEyeHeight;
	// Wall clock (Unix seconds) of the last successful calibrations, 0 = never
	double lastFullCalibration = 0;
	double lastQuickReset = 0;
	// Controller-to-wrist rotations of the pose model (left, right), measured at the
	// calibration stand pose
	std::optional<std::array<Quat, 2>> wristCalibration;
	Settings settings;
	std::map<std::string, TrackerConfig> trackers;  // by MAC

	static std::filesystem::path defaultPath();
	nlohmann::json toJson() const;
	bool fromJson(const nlohmann::json& j);
	bool load(const std::filesystem::path& path);
	bool save(const std::filesystem::path& path) const;
};

}  // namespace smt
