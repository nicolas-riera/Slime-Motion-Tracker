#include "device_provider.h"

#include <windows.h>

#include <cstdarg>
#include <cstdio>
#include <filesystem>

#include "slimelite/mounting_calibration.h"

namespace slimelite {

using nlohmann::json;

namespace {

constexpr const char* kVersion = "0.1.0";
constexpr double kStandSeconds = 5.0;
constexpr double kSkiSeconds = 5.0;

const char* kSerials[3] = {"SLV-WAIST", "SLV-LFOOT", "SLV-RFOOT"};
const char* kRoles[3] = {"TrackerRole_Waist", "TrackerRole_LeftFoot", "TrackerRole_RightFoot"};

Pose poseFromMatrix(const vr::HmdMatrix34_t& m) {
	Pose p;
	p.position = {m.m[0][3], m.m[1][3], m.m[2][3]};
	Mat3 r;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) r(i, j) = m.m[i][j];
	p.rotation = Quat::fromMatrix(r);
	return p;
}

std::filesystem::path moduleDirectory() {
	HMODULE module = nullptr;
	GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
					   reinterpret_cast<LPCWSTR>(&moduleDirectory), &module);
	wchar_t path[MAX_PATH];
	GetModuleFileNameW(module, path, MAX_PATH);
	return std::filesystem::path(path).parent_path();
}

}  // namespace

void DeviceProvider::log(const char* fmt, ...) {
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (auto* l = vr::VRDriverLog()) {
		std::string s = std::string("[slimevr_lite] ") + buf + "\n";
		l->Log(s.c_str());
	}
}

vr::EVRInitError DeviceProvider::Init(vr::IVRDriverContext* ctx) {
	VR_INIT_SERVER_DRIVER_CONTEXT(ctx);
	log("SlimeVR-lite %s starting, config %s", kVersion, Config::defaultPath().string().c_str());

	m_config.load(Config::defaultPath());
	for (const auto& [mac, tc] : m_config.trackers)
		if (tc.mag.valid) m_hub.setStoredMagCalibration(mac, tc.mag);
	m_hub.onMagCalibration([this](const std::string& mac, const MagCalibration& cal) {
		std::lock_guard lock(m_pendingMutex);
		m_pendingMag.emplace_back(mac, cal);
	});
	applySettings();
	if (!m_hub.start()) log("%s", m_hub.portError().c_str());
	if (!m_overlay.start()) log("cannot bind the overlay port %d", OverlayLink::kPort);

	for (int i = 0; i < 3; i++) {
		m_devices[i] = std::make_unique<VirtualTracker>(kSerials[i], kRoles[i]);
		// Assign the SteamVR tracker role so games pick them up without manual setup
		std::string key = std::string("/devices/slimevr_lite/") + kSerials[i];
		vr::VRSettings()->SetString("trackers", key.c_str(), kRoles[i]);
		vr::VRServerDriverHost()->TrackedDeviceAdded(kSerials[i], vr::TrackedDeviceClass_GenericTracker,
													  m_devices[i].get());
	}
	launchOverlay();
	return vr::VRInitError_None;
}

void DeviceProvider::Cleanup() {
	if (m_dirty) m_config.save(Config::defaultPath());
	m_hub.stop();
	m_overlay.stop();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void DeviceProvider::launchOverlay() {
	auto exe = moduleDirectory() / "slimevr_lite_overlay.exe";
	if (!std::filesystem::exists(exe)) {
		log("overlay not found: %s", exe.string().c_str());
		return;
	}
	STARTUPINFOW si{};
	si.cb = sizeof(si);
	PROCESS_INFORMATION pi{};
	std::wstring cmd = L"\"" + exe.wstring() + L"\"";
	if (CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, FALSE, DETACHED_PROCESS, nullptr,
					   exe.parent_path().c_str(), &si, &pi)) {
		CloseHandle(pi.hThread);
		CloseHandle(pi.hProcess);
	} else {
		log("cannot start the overlay (error %lu)", GetLastError());
	}
}

void DeviceProvider::applySettings() {
	const Settings& s = m_config.settings;
	HeadingFilterSettings hs;
	hs.sensitivity = s.magSensitivity;
	m_hub.setMagSettings(s.magEnabled, hs);
	m_solver.configure(BodyDimensions::fromHeight(m_config.userHeight, m_config.measuredEyeHeight),
					   s, m_config.mode);
}

bool DeviceProvider::trackerOrientation(const TrackerSnapshot& t, const TrackerConfig& tc, Quat& out) const {
	if (!t.connected || !t.hasRotation || !tc.mountingValid) return false;
	Quat q;
	if (tc.yawInMagFrame) {
		// Calibrated in the magnetic frame: wait for the heading after a reboot
		if (!t.hasHeading) return false;
		q = t.corrected();
	} else {
		auto it = m_bootAtCalibration.find(t.mac);
		if (it != m_bootAtCalibration.end() && it->second != t.bootCount) return false;
		q = t.qWorld;
	}
	out = Quat::yaw(tc.yawAlign) * q * tc.mounting;
	return true;
}

void DeviceProvider::RunFrame() {
	vr::VREvent_t ev;
	while (vr::VRServerDriverHost()->PollNextEvent(&ev, sizeof(ev))) {}

	double now = TrackerHub::now();
	double dt = m_lastFrame > 0 ? now - m_lastFrame : 1.0 / 90;
	m_lastFrame = now;

	// Mag calibrations found by the network thread
	{
		std::lock_guard lock(m_pendingMutex);
		for (auto& [mac, cal] : m_pendingMag) {
			m_config.trackers[mac].mag = cal;
			markDirty();
		}
		m_pendingMag.clear();
	}
	handleOverlay();

	vr::TrackedDevicePose_t hmdPoses[1];
	vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmdPoses, 1);
	bool hmdValid = hmdPoses[0].bPoseIsValid;
	Pose hmd = poseFromMatrix(hmdPoses[0].mDeviceToAbsoluteTracking);

	auto trackers = m_hub.snapshot();
	updateCalibration(now, hmd, hmdValid, trackers);

	// Gather segment orientations by role
	BodyInput in;
	in.dt = dt;
	in.hmdValid = hmdValid && m_floorKnown;
	in.hmd = hmd;
	in.floorY = m_floorY;
	for (const auto& t : trackers) {
		auto it = m_config.trackers.find(t.mac);
		if (it == m_config.trackers.end()) continue;
		const TrackerConfig& tc = it->second;
		Quat seg;
		if (!trackerOrientation(t, tc, seg)) continue;
		if (tc.role == Role::Waist) {
			in.hasWaist = true;
			in.waist = seg;
		} else if (tc.role == Role::LeftAnkle || tc.role == Role::RightAnkle) {
			auto& leg = in.legs[tc.role == Role::LeftAnkle ? 0 : 1];
			leg.valid = true;
			leg.shin = seg;
			// Same world frame as seg, without the mounting
			Quat world = seg * tc.mounting.conj();
			leg.accelWorld = world.rotate(t.accelLocal);
			leg.angularSpeed = t.angularSpeed;
		}
	}

	BodyOutput out = m_solver.update(in);
	m_lastOutput = out;
	bool vel = m_config.settings.velocityPrediction;
	bool waistOk = out.valid && (m_config.mode == Mode::Ankles || in.hasWaist);
	m_devices[0]->updatePose(waistOk, out.waist, out.waistVelocity, vel);
	m_devices[1]->updatePose(out.valid, out.feet[0], out.footVelocity[0], vel);
	m_devices[2]->updatePose(out.valid, out.feet[1], out.footVelocity[1], vel);

	if (now - m_lastState > 0.1) {
		m_lastState = now;
		publishState(trackers, out);
	}
	if (m_dirty && now - m_lastSave > 1.0) {
		m_lastSave = now;
		m_dirty = false;
		m_config.save(Config::defaultPath());
	}
}

void DeviceProvider::handleOverlay() {
	for (const auto& msg : m_overlay.poll()) handleCommand(msg);
}

void DeviceProvider::handleCommand(const json& cmd) {
	std::string type = cmd.value("type", "");
	if (type == "hello") return;
	if (type == "floor") {
		if (cmd.value("valid", false) && cmd.contains("y")) {
			m_floorY = cmd["y"].get<double>();
			m_floorKnown = m_floorFromOverlay = true;
		}
		return;
	}
	if (type == "setMode") {
		m_config.mode = modeFromName(cmd.value("mode", "three"));
		m_solver.reset();
	} else if (type == "setHeight") {
		double h = cmd.value("meters", m_config.userHeight);
		if (h > 1.0 && h < 2.5) m_config.userHeight = h;
		m_solver.reset();
	} else if (type == "setRole") {
		std::string mac = cmd.value("mac", "");
		Role role = roleFromName(cmd.value("role", "none"));
		// One tracker per role
		if (role != Role::None)
			for (auto& [m, tc] : m_config.trackers)
				if (tc.role == role) tc.role = Role::None;
		m_config.trackers[mac].role = role;
		m_solver.reset();
	} else if (type == "setSetting") {
		std::string key = cmd.value("key", "");
		const json& v = cmd["value"];
		Settings& s = m_config.settings;
		if (key == "magEnabled") s.magEnabled = v.get<bool>();
		else if (key == "magSensitivity") s.magSensitivity = v.get<double>();
		else if (key == "smoothing") s.smoothing = v.get<double>();
		else if (key == "footPlantStrength") s.footPlantStrength = v.get<double>();
		else if (key == "inertialWeight") s.inertialWeight = v.get<double>();
		else if (key == "velocityPrediction") s.velocityPrediction = v.get<bool>();
	} else if (type == "calibrate" || type == "quickReset") {
		m_calQuick = type == "quickReset";
		m_calPhase = CalPhase::Stand;
		m_calDeadline = TrackerHub::now() + kStandSeconds;
		m_calMessage = "Stand straight, feet parallel, look straight ahead";
		return;
	} else if (type == "cancelCalibration") {
		m_calPhase = CalPhase::Idle;
		m_calMessage.clear();
		return;
	} else if (type == "magCalibrate") {
		m_hub.startGuidedMagCalibration(cmd.value("mac", ""));
		return;
	} else if (type == "magCancel") {
		m_hub.cancelGuidedMagCalibration(cmd.value("mac", ""));
		return;
	} else if (type == "magReset") {
		std::string mac = cmd.value("mac", "");
		m_hub.resetMagCalibration(mac);
		m_config.trackers[mac].mag = MagCalibration{};
	} else {
		return;
	}
	applySettings();
	markDirty();
}

void DeviceProvider::updateCalibration(double now, const Pose& hmd, bool hmdValid,
									   const std::vector<TrackerSnapshot>& trackers) {
	if (m_calPhase != CalPhase::Stand && m_calPhase != CalPhase::Ski) return;
	if (now < m_calDeadline) return;
	if (!hmdValid) {
		m_calPhase = CalPhase::Error;
		m_calMessage = "Headset not tracked";
		return;
	}

	// Capture every assigned, connected tracker
	std::map<std::string, Capture> captures;
	for (const auto& t : trackers) {
		auto it = m_config.trackers.find(t.mac);
		if (it == m_config.trackers.end() || it->second.role == Role::None) continue;
		if (!t.connected || !t.hasRotation) continue;
		Capture c;
		c.ok = true;
		c.inMagFrame = t.hasHeading;
		c.q = t.corrected();
		captures[t.mac] = c;
	}

	if (m_calPhase == CalPhase::Stand) {
		m_standHmd = hmd;
		m_standCaptures = captures;
		if (!m_floorFromOverlay) {
			m_floorY = hmd.position.y - 0.936 * m_config.userHeight;
			m_floorKnown = true;
		}
		if (m_calQuick) {
			finishCalibration(hmd);
			return;
		}
		m_calPhase = CalPhase::Ski;
		m_calDeadline = now + kSkiSeconds;
		m_calMessage = "Ski pose: bend your knees and lean forward, arms down, look ahead";
		return;
	}

	// Ski capture: solve the mounting of every tracker
	double bodyYaw = m_standHmd.rotation.yawAngle();
	std::string errors;
	int solved = 0;
	for (auto& [mac, stand] : m_standCaptures) {
		auto ski = captures.find(mac);
		TrackerConfig& tc = m_config.trackers[mac];
		if (ski == captures.end()) {
			errors += mac.substr(9) + ": disconnected. ";
			continue;
		}
		double minBend = tc.role == Role::Waist ? 8.0 : 15.0;
		if (stand.inMagFrame != ski->second.inMagFrame) {
			errors += mac.substr(9) + ": mag heading changed during calibration. ";
			continue;
		}
		MountingResult r = solveMounting(stand.q, ski->second.q, bodyYaw, minBend);
		if (!r.ok) {
			errors += std::string(roleName(tc.role)) + ": " + r.error + ". ";
			continue;
		}
		tc.mounting = r.mounting;
		tc.yawAlign = r.yawAlign;
		tc.mountingValid = true;
		tc.yawInMagFrame = stand.inMagFrame;
		solved++;
		for (const auto& t : trackers)
			if (t.mac == mac) m_bootAtCalibration[mac] = t.bootCount;
	}
	finishCalibration(m_standHmd);
	if (!errors.empty()) {
		m_calPhase = CalPhase::Error;
		m_calMessage = errors;
	} else if (solved == 0) {
		m_calPhase = CalPhase::Error;
		m_calMessage = "No assigned tracker: set the tracker roles first";
	}
}

void DeviceProvider::finishCalibration(const Pose& hmd) {
	// Eye height above the floor tunes the spine length
	if (m_floorKnown) m_config.measuredEyeHeight = hmd.position.y - m_floorY;
	if (m_calQuick) {
		double bodyYaw = hmd.rotation.yawAngle();
		std::string errors;
		for (auto& [mac, stand] : m_standCaptures) {
			TrackerConfig& tc = m_config.trackers[mac];
			if (!tc.mountingValid) {
				errors += std::string(roleName(tc.role)) + ": needs a full calibration first. ";
				continue;
			}
			MountingResult r = solveQuickReset(stand.q, tc.mounting, bodyYaw);
			tc.mounting = r.mounting;
			tc.yawAlign = r.yawAlign;
			tc.yawInMagFrame = stand.inMagFrame;
		}
		m_calPhase = errors.empty() ? CalPhase::Done : CalPhase::Error;
		m_calMessage = errors.empty() ? "Reset done" : errors;
	} else {
		m_calPhase = CalPhase::Done;
		m_calMessage = "Calibration done";
	}
	auto trackers = m_hub.snapshot();
	for (const auto& t : trackers) m_bootAtCalibration[t.mac] = t.bootCount;
	applySettings();
	m_solver.reset();
	markDirty();
}

void DeviceProvider::publishState(const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out) {
	static const char* phases[] = {"idle", "stand", "ski", "done", "error"};
	const Settings& s = m_config.settings;
	double now = TrackerHub::now();

	bool calibrated = false;
	json ts = json::array();
	for (const auto& t : trackers) {
		const TrackerConfig* tc = nullptr;
		auto it = m_config.trackers.find(t.mac);
		if (it != m_config.trackers.end()) tc = &it->second;
		if (tc && tc->role != Role::None && tc->mountingValid) calibrated = true;
		bool planted = false;
		if (tc && (tc->role == Role::LeftAnkle || tc->role == Role::RightAnkle))
			planted = out.planted[tc->role == Role::LeftAnkle ? 0 : 1];
		ts.push_back({{"mac", t.mac},
					  {"ip", t.ip},
					  {"role", roleName(tc ? tc->role : Role::None)},
					  {"connected", t.connected},
					  {"rotationHz", t.rotationHz},
					  {"magHz", t.magHz},
					  {"battery", t.batteryLevel >= 0 ? t.batteryLevel : -1.0},
					  {"batteryVoltage", t.batteryVoltage},
					  {"rssi", t.rssi},
					  {"temperature", t.temperature},
					  {"activity", t.activity},
					  {"planted", planted},
					  {"mag",
					   {{"state", magStateName(t.magState)},
						{"progress", t.magProgress},
						{"residual", t.magCalibration.valid ? t.magCalibration.residual : 1.0},
						{"dip", rad2deg(t.magCalibration.dip)}}}});
	}

	double countdown = (m_calPhase == CalPhase::Stand || m_calPhase == CalPhase::Ski)
						 ? std::max(0.0, m_calDeadline - now)
						 : 0.0;
	json state = {
		{"type", "state"},
		{"version", 1},
		{"driver", {{"version", kVersion}, {"trackerPort", 6969}, {"portError", m_hub.portError()}}},
		{"mode", modeName(m_config.mode)},
		{"userHeight", m_config.userHeight},
		{"measuredEyeHeight", m_config.measuredEyeHeight ? json(*m_config.measuredEyeHeight) : json(nullptr)},
		{"floorKnown", m_floorKnown},
		{"calibrated", calibrated},
		{"calibration", {{"phase", phases[int(m_calPhase)]}, {"countdown", countdown}, {"message", m_calMessage}}},
		{"settings",
		 {{"magEnabled", s.magEnabled},
		  {"magSensitivity", s.magSensitivity},
		  {"smoothing", s.smoothing},
		  {"footPlantStrength", s.footPlantStrength},
		  {"inertialWeight", s.inertialWeight},
		  {"velocityPrediction", s.velocityPrediction}}},
		{"trackers", ts},
		{"outputs", {{"waist", out.valid}, {"leftFoot", out.valid}, {"rightFoot", out.valid}}},
	};
	m_overlay.send(state);
}

}  // namespace slimelite
