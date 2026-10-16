#include "slimelite/config.h"

#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp>

namespace slimelite {

using nlohmann::json;

const char* roleName(Role r) {
	switch (r) {
		case Role::Waist: return "waist";
		case Role::LeftAnkle: return "left_ankle";
		case Role::RightAnkle: return "right_ankle";
		default: return "none";
	}
}

Role roleFromName(const std::string& s) {
	if (s == "waist") return Role::Waist;
	if (s == "left_ankle") return Role::LeftAnkle;
	if (s == "right_ankle") return Role::RightAnkle;
	return Role::None;
}

const char* modeName(Mode m) { return m == Mode::Ankles ? "ankles" : "three"; }
Mode modeFromName(const std::string& s) { return s == "ankles" ? Mode::Ankles : Mode::Three; }

namespace {

json quatToJson(const Quat& q) { return json::array({q.w, q.x, q.y, q.z}); }
Quat quatFromJson(const json& j) {
	return Quat{j.at(0).get<double>(), j.at(1).get<double>(), j.at(2).get<double>(),
				j.at(3).get<double>()}
		.normalized();
}

}  // namespace

std::filesystem::path Config::defaultPath() {
	const char* base = std::getenv("LOCALAPPDATA");
	std::filesystem::path dir = base ? base : ".";
	return dir / "SlimeVR-lite" / "config.json";
}

bool Config::load(const std::filesystem::path& path) {
	std::ifstream in(path);
	if (!in) return false;
	try {
		return fromJson(json::parse(in));
	} catch (const std::exception&) {
		return false;
	}
}

bool Config::fromJson(const json& j) {
	try {
		mode = modeFromName(j.value("mode", "three"));
		userHeight = j.value("userHeight", 1.75);
		measuredEyeHeight.reset();
		lastFullCalibration = j.value("lastFullCalibration", 0.0);
		lastQuickReset = j.value("lastQuickReset", 0.0);
		if (j.contains("measuredEyeHeight") && j["measuredEyeHeight"].is_number())
			measuredEyeHeight = j["measuredEyeHeight"].get<double>();
		if (j.contains("settings")) {
			const json& s = j["settings"];
			settings.magEnabled = s.value("magEnabled", settings.magEnabled);
			settings.magSensitivity = s.value("magSensitivity", settings.magSensitivity);
			settings.smoothing = s.value("smoothing", settings.smoothing);
			settings.footPlantStrength = s.value("footPlantStrength", settings.footPlantStrength);
			settings.inertialWeight = s.value("inertialWeight", settings.inertialWeight);
			settings.velocityPrediction = s.value("velocityPrediction", settings.velocityPrediction);
			settings.recording = s.value("recording", settings.recording);
		}
		trackers.clear();
		const json trackersJson = j.value("trackers", json::object());
		for (auto& [mac, t] : trackersJson.items()) {
			TrackerConfig tc;
			tc.role = roleFromName(t.value("role", "none"));
			tc.mountingValid = t.value("mountingValid", false);
			if (t.contains("mounting")) tc.mounting = quatFromJson(t["mounting"]);
			tc.yawAlign = t.value("yawAlign", 0.0);
			tc.yawInMagFrame = t.value("yawInMagFrame", false);
			if (t.contains("mag") && t["mag"].value("valid", false)) {
				const json& m = t["mag"];
				for (int i = 0; i < 9; i++) tc.mag.K.m[i] = m["K"].at(i).get<double>();
				tc.mag.c = {m["c"].at(0).get<double>(), m["c"].at(1).get<double>(),
							m["c"].at(2).get<double>()};
				tc.mag.dip = m.value("dip", 0.0);
				tc.mag.residual = m.value("residual", 1.0);
				tc.mag.headingOffset = m.value("headingOffset", 0.0);
				tc.mag.valid = true;
			}
			trackers[mac] = tc;
		}
		return true;
	} catch (const std::exception&) {
		return false;
	}
}

json Config::toJson() const {
	json j;
	j["mode"] = modeName(mode);
	j["userHeight"] = userHeight;
	j["measuredEyeHeight"] = measuredEyeHeight ? json(*measuredEyeHeight) : json(nullptr);
	j["lastFullCalibration"] = lastFullCalibration;
	j["lastQuickReset"] = lastQuickReset;
	j["settings"] = {{"magEnabled", settings.magEnabled},
					 {"magSensitivity", settings.magSensitivity},
					 {"smoothing", settings.smoothing},
					 {"footPlantStrength", settings.footPlantStrength},
					 {"inertialWeight", settings.inertialWeight},
					 {"velocityPrediction", settings.velocityPrediction},
					 {"recording", settings.recording}};
	json ts = json::object();
	for (const auto& [mac, t] : trackers) {
		json tj = {{"role", roleName(t.role)},
				   {"mountingValid", t.mountingValid},
				   {"mounting", quatToJson(t.mounting)},
				   {"yawAlign", t.yawAlign},
				   {"yawInMagFrame", t.yawInMagFrame}};
		if (t.mag.valid) {
			tj["mag"] = {{"valid", true},
						 {"K", t.mag.K.m},
						 {"c", {t.mag.c.x, t.mag.c.y, t.mag.c.z}},
						 {"dip", t.mag.dip},
						 {"residual", t.mag.residual},
						 {"headingOffset", t.mag.headingOffset}};
		}
		ts[mac] = tj;
	}
	j["trackers"] = ts;
	return j;
}

bool Config::save(const std::filesystem::path& path) const {
	json j = toJson();
	std::error_code ec;
	std::filesystem::create_directories(path.parent_path(), ec);
	auto tmp = path;
	tmp += ".tmp";
	{
		std::ofstream out(tmp);
		if (!out) return false;
		out << j.dump(2);
	}
	std::filesystem::rename(tmp, path, ec);
	return !ec;
}

}  // namespace slimelite
