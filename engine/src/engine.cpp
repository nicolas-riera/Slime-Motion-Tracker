#include "engine.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <filesystem>

#include "smt/mounting_calibration.h"

namespace smt {

using nlohmann::json;

namespace {

constexpr const char* kVersion = "0.2.0";
constexpr double kStandSeconds = 3.0;
constexpr double kQuickResetSeconds = 3.0;
constexpr double kSkiSeconds = 3.0;

Pose poseFromMatrix(const double m[3][4]) {
	Pose p;
	p.position = {m[0][3], m[1][3], m[2][3]};
	Mat3 r;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++) r(i, j) = m[i][j];
	p.rotation = Quat::fromMatrix(r);
	return p;
}

void toSlPose(bool valid, const Pose& p, const Vec3& v, SmtPose& out) {
	out.valid = valid ? 1 : 0;
	out.position[0] = p.position.x;
	out.position[1] = p.position.y;
	out.position[2] = p.position.z;
	out.rotation[0] = p.rotation.w;
	out.rotation[1] = p.rotation.x;
	out.rotation[2] = p.rotation.y;
	out.rotation[3] = p.rotation.z;
	out.velocity[0] = v.x;
	out.velocity[1] = v.y;
	out.velocity[2] = v.z;
}

}  // namespace

void Engine::log(const char* fmt, ...) {
	char buf[1024];
	va_list args;
	va_start(args, fmt);
	std::vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);
	if (m_host.log) m_host.log(buf);
}

Engine::Engine(const SmtEngineHost& host, const char* handover)
	: m_host(host), m_moduleDirectory(host.moduleDirectory ? host.moduleDirectory : "") {
	m_host.moduleDirectory = nullptr;  // not owned, use our copy
	log("engine %s starting, config %s", kVersion, Config::defaultPath().string().c_str());

	m_config.load(Config::defaultPath());
	{
		std::string path = (std::filesystem::path(m_moduleDirectory) / "models" / "foot_model.json").string();
		if (m_footModel.load(path)) log("foot model loaded: %s", path.c_str());
		else log("no foot model (%s): solver only", path.c_str());
		std::string posePath = (std::filesystem::path(m_moduleDirectory) / "models" / "pose_model.bin").string();
		if (!m_config.settings.poseModel) log("pose model disabled in the settings");
		else if (m_pose.start(posePath)) log("pose model loaded: %s (%s)", posePath.c_str(), m_pose.name().c_str());
		else log("no pose model (%s): %s", posePath.c_str(), m_pose.error().c_str());
		if (m_config.wristCalibration) m_pose.setWristCalibration(*m_config.wristCalibration);
	}
	for (const auto& [mac, tc] : m_config.trackers)
		if (tc.mag.valid) m_hub.setStoredMagCalibration(mac, tc.mag);
	m_hub.onMagCalibration([this](const std::string& mac, const MagCalibration& cal) {
		std::lock_guard lock(m_pendingMutex);
		m_pendingMag.emplace_back(mac, cal);
	});

	json h = handover ? json::parse(handover, nullptr, false) : json();
	if (h.is_object()) {
		m_floorKnown = h.value("floorKnown", false);
		m_floorFromOverlay = h.value("floorFromOverlay", false);
		m_floorY = h.value("floorY", 0.0);
		const json boots = h.value("bootAtCalibration", json::object());
		for (auto& [mac, boot] : boots.items())
			if (boot.is_number_integer()) m_bootAtCalibration[mac] = boot.get<int>();
		if (h.contains("hub")) m_hub.importHandover(h["hub"], TrackerHub::now());
		m_handoverRecording = h.value("recordingPath", "");
		log("hot reload: state restored");
	}

	m_hub.setEventSink([this](const std::string& mac, const proto::Event& e, double t) {
		m_recorder.trackerEvent(mac, e, t);
	});
	updateRecordingState();
	recordEvent(h.is_object() ? "engine_reloaded" : "engine_started", kVersion);
	applySettings();
	m_hub.setLog([this](const std::string& msg) { log("%s", msg.c_str()); });
	m_dongle.setLog([this](const std::string& msg) { log("%s", msg.c_str()); });
	if (!m_hub.start()) log("%s", m_hub.portError().c_str());
	m_dongle.start(proto::kTrackerPort);
	if (!m_overlay.start()) log("cannot bind the overlay port %d", OverlayLink::kPort);
	// The overlay is not started with SteamVR: the user launches it (desktop shortcut)
}

Engine::~Engine() {
	if (m_dirty) m_config.save(Config::defaultPath());
	m_dongle.stop();
	m_hub.stop();
	m_overlay.stop();
	m_recorder.close();
}

void Engine::updateRecordingState() {
	if (m_config.settings.recording && !m_recorder.isOpen()) {
		if (m_recorder.open(m_handoverRecording)) {
			log("recording to %s", m_recorder.path().c_str());
			m_recorder.json(TrackerHub::now(), "cfg", {{"config", m_config.toJson()}});
			// A recording started mid-session must still be replayable
			if (m_floorKnown) recordEvent("floor", std::to_string(m_floorY));
		} else {
			log("cannot open the recording file");
		}
		m_handoverRecording.clear();
	} else if (!m_config.settings.recording && m_recorder.isOpen()) {
		m_recorder.close();
	}
}

void Engine::recordEvent(const char* what, const std::string& detail) {
	m_recorder.json(TrackerHub::now(), "ev", {{"what", what}, {"detail", detail}});
}

void Engine::recordFrame(const SmtFrameInput& in, const BodyOutput& out,
						 const std::vector<TrackerSnapshot>& trackers) {
	if (!m_recorder.isOpen()) return;
	for (int i = 0; i < in.deviceCount && i < SMT_MAX_DEVICES; i++) {
		const SmtDevicePose& d = in.devices[i];
		if (!d.connected) continue;
		// Our own outputs, see "out" ("SLV-" before the rename)
		if (std::strncmp(d.serial, "SMT-", 4) == 0 || std::strncmp(d.serial, "SLV-", 4) == 0) continue;
		Pose p = poseFromMatrix(d.pose);
		m_recorder.line(
			R"({"t":%.6f,"k":"dev","i":%d,"cls":%d,"role":%d,"ser":"%s","ok":%d,"p":[%.5f,%.5f,%.5f],"q":[%.6f,%.6f,%.6f,%.6f],"v":[%.4f,%.4f,%.4f],"w":[%.4f,%.4f,%.4f]})",
			in.now, i, d.deviceClass, d.controllerRole, d.serial, d.valid, p.position.x, p.position.y,
			p.position.z, p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z, d.velocity[0],
			d.velocity[1], d.velocity[2], d.angularVelocity[0], d.angularVelocity[1], d.angularVelocity[2]);
	}
	auto pose = [](bool valid, const Pose& p) {
		char b[160];
		std::snprintf(b, sizeof(b), "[%d,%.5f,%.5f,%.5f,%.6f,%.6f,%.6f,%.6f]", valid ? 1 : 0, p.position.x,
					  p.position.y, p.position.z, p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z);
		return std::string(b);
	};
	m_recorder.line(R"({"t":%.6f,"k":"out","waist":%s,"lf":%s,"rf":%s,"pl":[%d,%d],"kn":[%.2f,%.2f]})",
					in.now, pose(out.valid, out.waist).c_str(), pose(out.valid, out.feet[0]).c_str(),
					pose(out.valid, out.feet[1]).c_str(), out.planted[0] ? 1 : 0, out.planted[1] ? 1 : 0,
					out.kneeAngles[0], out.kneeAngles[1]);
	if (in.now - m_lastTrackerRecord >= 0.1) {
		m_lastTrackerRecord = in.now;
		for (const auto& t : trackers) {
			auto it = m_config.trackers.find(t.mac);
			const char* role = it != m_config.trackers.end() ? roleName(it->second.role) : "none";
			m_recorder.line(R"({"t":%.6f,"k":"trk","mac":"%s","role":"%s","up":%d,"hh":%d,"hdg":%.6f,"ms":"%s","boot":%d})",
							in.now, t.mac.c_str(), role, t.connected ? 1 : 0, t.hasHeading ? 1 : 0,
							t.heading, magStateName(t.magState), t.bootCount);
		}
	}
}

json Engine::handover() const {
	json boots = json::object();
	for (const auto& [mac, b] : m_bootAtCalibration) boots[mac] = b;
	return {{"floorKnown", m_floorKnown},
			{"floorFromOverlay", m_floorFromOverlay},
			{"floorY", m_floorY},
			{"bootAtCalibration", boots},
			{"recordingPath", m_recorder.isOpen() ? m_recorder.path() : std::string()},
			{"hub", m_hub.exportHandover()}};
}

void Engine::applySettings() {
	const Settings& s = m_config.settings;
	HeadingFilterSettings hs;
	hs.sensitivity = s.magSensitivity;
	m_hub.setMagSettings(s.magEnabled, hs);
	BodyDimensions dims = BodyDimensions::fromHeight(m_config.userHeight, m_config.measuredEyeHeight);
	m_eyeHeight = dims.eyeHeight;
	m_solver.configure(dims, s, m_config.mode);
}

bool Engine::trackerOrientation(const TrackerSnapshot& t, const TrackerConfig& tc, Quat& out) const {
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

void Engine::frame(const SmtFrameInput& fin, SmtFrameOutput& fout) {
	double now = fin.now;
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
	for (const auto& msg : m_overlay.poll()) handleCommand(msg);

	bool hmdValid = fin.hmdValid != 0;
	Pose hmd = poseFromMatrix(fin.hmd);
	auto trackers = m_hub.snapshot();
	m_handValid = {};
	for (int i = 0; i < fin.deviceCount && i < SMT_MAX_DEVICES; i++) {
		const SmtDevicePose& d = fin.devices[i];
		// vr::TrackedDeviceClass_Controller, TrackedControllerRole_LeftHand / RightHand
		if (!d.connected || !d.valid || d.deviceClass != 2 || (d.controllerRole != 1 && d.controllerRole != 2)) continue;
		m_handValid[d.controllerRole - 1] = true;
		m_hands[d.controllerRole - 1] = poseFromMatrix(d.pose);
	}
	updateCalibration(now, hmd, hmdValid, trackers);

	// Gather segment orientations by role
	BodyInput in;
	in.dt = dt;
	in.hmdValid = hmdValid && m_floorKnown;
	in.hmd = hmd;
	in.floorY = m_floorY;
	Vec3 waistAccel;
	for (const auto& t : trackers) {
		auto it = m_config.trackers.find(t.mac);
		if (it == m_config.trackers.end()) continue;
		const TrackerConfig& tc = it->second;
		Quat seg;
		if (!trackerOrientation(t, tc, seg)) continue;
		if (tc.role == Role::Waist) {
			in.hasWaist = true;
			in.waist = seg;
			waistAccel = (seg * tc.mounting.conj()).rotate(t.accelLocal);
			in.waistAccel = waistAccel;
		} else if (tc.role == Role::LeftAnkle || tc.role == Role::RightAnkle) {
			auto& leg = in.legs[tc.role == Role::LeftAnkle ? 0 : 1];
			leg.valid = true;
			leg.shin = seg;
			// Tracker frame in the same world frame as seg, without the mounting
			Quat world = seg * tc.mounting.conj();
			leg.accelWorld = world.rotate(t.accelLocal);
			leg.angularSpeed = t.angularSpeed;
		}
	}

	updateMode(now, in.hasWaist);

	// The foot model sees every frame (its history), valid or not
	if (m_footModel.loaded()) {
		FootEstimate fe = m_footModel.update(in, now);
		for (int i = 0; i < 2 && fe.valid; i++) {
			auto& leg = in.legs[i];
			leg.hasFootMeasurement = true;
			leg.footMeasurement = fe.ankle[i];
			leg.footMeasurementSigma = footMeasurementSigma(fe.sigma[i]);
		}
	}
	BodyOutput out = m_solver.update(in);
	if (m_pose.running() && m_config.settings.poseModel) {
		PoseModelInput pin;
		pin.floorY = m_floorY;
		pin.hmdValid = in.hmdValid;
		pin.hmd = hmd;
		pin.handValid = m_handValid;
		pin.hands = m_hands;
		for (int i = 0; i < 2; i++) {
			pin.legValid[i] = in.legs[i].valid;
			pin.shin[i] = in.legs[i].shin;
			pin.legAccel[i] = in.legs[i].accelWorld;
		}
		pin.hasWaist = in.hasWaist;
		pin.waist = in.waist;
		pin.waistAccel = waistAccel;
		m_pose.submit(pin, now, m_config.userHeight);
		fusePose(m_pose.latest(), hmd, m_floorY, m_eyeHeight, out, PoseFusionSettings::forWaist(in.hasWaist));
	}
	recordFrame(fin, out, trackers);
	if (m_calPhase != m_recordedPhase) {
		static const char* names[] = {"idle", "stand", "ski", "done", "error"};
		m_recordedPhase = m_calPhase;
		recordEvent("calibration", std::string(names[int(m_calPhase)]) + ": " + m_calMessage);
		if (m_calPhase == CalPhase::Done) {
			double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
			(m_calQuick ? m_config.lastQuickReset : m_config.lastFullCalibration) = wall;
			markDirty();
		}
	}
	bool waistOk = out.valid && (m_config.mode == Mode::Ankles || in.hasWaist);
	toSlPose(waistOk, out.waist, out.waistVelocity, fout.devices[SMT_DEVICE_WAIST]);
	toSlPose(out.valid, out.feet[0], out.footVelocity[0], fout.devices[SMT_DEVICE_LEFT_FOOT]);
	toSlPose(out.valid, out.feet[1], out.footVelocity[1], fout.devices[SMT_DEVICE_RIGHT_FOOT]);
	fout.sendVelocity = m_config.settings.velocityPrediction ? 1 : 0;

	if (now - m_lastState > 0.1) {
		m_lastState = now;
		publishState(trackers, out);
	}
	publishViews(now, trackers, out, hmd, hmdValid);
	if (m_dirty && now - m_lastSave > 1.0) {
		m_lastSave = now;
		m_dirty = false;
		m_config.save(Config::defaultPath());
		if (m_recorder.isOpen()) m_recorder.json(now, "cfg", {{"config", m_config.toJson()}});
	}
}

void Engine::onFloor(double y, double now) {
	// The overlay reports the floor of the SteamVR play space in raw tracking
	// space. Play space tools (OVR Advanced Settings space drag, climbing...) move
	// the play space, not the physical floor: keep the first value, then accept
	// only small corrections (floor fix, room setup tweak) once they have settled
	constexpr double kMaxCorrection = 0.10;  // m
	constexpr double kSettleTime = 2.0;  // s
	if (!m_floorFromOverlay || !m_floorKnown) {
		m_floorY = y;
		m_floorKnown = m_floorFromOverlay = true;
		recordEvent("floor", std::to_string(y));
		return;
	}
	double change = y - m_floorY;
	if (std::fabs(change) < 0.002) {
		m_floorCandidateSince = -1;
		return;
	}
	if (std::fabs(change) > kMaxCorrection) {
		m_floorCandidateSince = -1;
		if (std::fabs(y - m_floorIgnoredLogged) > 0.05) {
			m_floorIgnoredLogged = y;
			recordEvent("floor_ignored", std::to_string(y));
			log("play space moved by %.2f m (space drag?): keeping the floor at %.3f", change, m_floorY);
		}
		return;
	}
	if (m_floorCandidateSince < 0 || std::fabs(y - m_floorCandidate) > 0.005) {
		m_floorCandidate = y;
		m_floorCandidateSince = now;
		return;
	}
	if (now - m_floorCandidateSince >= kSettleTime) {
		m_floorY = y;
		m_floorCandidateSince = -1;
		recordEvent("floor", std::to_string(y));
	}
}

void Engine::handleCommand(const json& cmd) {
	std::string type = cmd.value("type", "");
	if (type == "hello") return;
	if (type == "floor") {
		if (cmd.value("valid", false) && cmd.contains("y")) onFloor(cmd["y"].get<double>(), TrackerHub::now());
		return;
	}
	if (type == "view") {
		// The overlay shows a 3D view: stream the poses it needs while it lasts
		if (cmd.value("visible", false)) {
			m_viewUntil = TrackerHub::now() + 2.0;
			m_viewTracker = cmd.value("tracker", "");
			m_viewMag = cmd.value("magView", false);
		} else {
			m_viewUntil = -1;
		}
		return;
	}
	if (type == "setHeight") {
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
		if (!cmd.contains("value")) return;
		const json& v = cmd["value"];
		Settings& s = m_config.settings;
		if (key == "magEnabled") s.magEnabled = v.get<bool>();
		else if (key == "magSensitivity") s.magSensitivity = v.get<double>();
		else if (key == "smoothing") s.smoothing = v.get<double>();
		else if (key == "footPlantStrength") s.footPlantStrength = v.get<double>();
		else if (key == "inertialWeight") s.inertialWeight = v.get<double>();
		else if (key == "velocityPrediction") s.velocityPrediction = v.get<bool>();
		else if (key == "recording") s.recording = v.get<bool>();
		recordEvent("setting", key + "=" + v.dump());
		updateRecordingState();
	} else if (type == "resetSettings") {
		m_config.settings = Settings{};
		recordEvent("setting", "reset");
		updateRecordingState();
	} else if (type == "calibrate" || type == "quickReset") {
		m_calQuick = type == "quickReset";
		m_calPhase = CalPhase::Stand;
		m_calDeadline = TrackerHub::now() + (m_calQuick ? kQuickResetSeconds : kStandSeconds);
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

void Engine::updateCalibration(double now, const Pose& hmd, bool hmdValid,
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
		captures[t.mac] = Capture{t.corrected(), t.hasHeading};
	}

	if (m_calPhase == CalPhase::Stand) {
		m_standHmd = hmd;
		// The stand pose (arms hanging, palms in) also calibrates the controllers
		// to the wrists for the pose model
		if (m_handValid[0] && m_handValid[1]) {
			m_config.wristCalibration = PoseModel::wristCalibrationFor(hmd, m_hands);
			m_pose.setWristCalibration(*m_config.wristCalibration);
		}
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
		std::string name = roleName(tc.role);
		if (ski == captures.end()) {
			errors += name + ": disconnected. ";
			continue;
		}
		if (stand.inMagFrame != ski->second.inMagFrame) {
			errors += name + ": mag heading changed during calibration. ";
			continue;
		}
		double minBend = tc.role == Role::Waist ? 8.0 : 15.0;
		MountingResult r = solveMounting(stand.q, ski->second.q, bodyYaw, minBend);
		if (!r.ok) {
			errors += name + ": " + r.error + ". ";
			continue;
		}
		tc.mounting = r.mounting;
		tc.yawAlign = r.yawAlign;
		tc.mountingValid = true;
		tc.yawInMagFrame = stand.inMagFrame;
		solved++;
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

void Engine::batteries(SmtBattery out[SMT_DEVICE_COUNT]) const {
	for (int i = 0; i < SMT_DEVICE_COUNT; i++) out[i] = SmtBattery{};
	for (const auto& t : m_hub.snapshot()) {
		auto it = m_config.trackers.find(t.mac);
		if (it == m_config.trackers.end() || !t.connected || t.batteryLevel < 0) continue;
		int slot = -1;
		switch (it->second.role) {
			case Role::Waist: slot = SMT_DEVICE_WAIST; break;
			case Role::LeftAnkle: slot = SMT_DEVICE_LEFT_FOOT; break;
			case Role::RightAnkle: slot = SMT_DEVICE_RIGHT_FOOT; break;
			default: break;
		}
		if (slot < 0) continue;
		// The trackers do not report charging
		out[slot] = SmtBattery{1, std::clamp(t.batteryLevel, 0.0f, 1.0f), 0};
	}
}

void Engine::finishCalibration(const Pose& hmd) {
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
	for (const auto& t : m_hub.snapshot()) m_bootAtCalibration[t.mac] = t.bootCount;
	applySettings();
	m_solver.reset();
	markDirty();
}

void Engine::publishState(const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out) {
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
					  // Trackers on the USB dongle reach the hub through the local bridge
					  {"transport", t.ip == "127.0.0.1" ? "espnow" : "wifi"},
					  {"role", roleName(tc ? tc->role : Role::None)},
					  {"connected", t.connected},
					  {"rotationHz", t.rotationHz},
					  {"magHz", t.magHz},
					  {"battery", t.batteryLevel >= 0 ? t.batteryLevel : -1.0},
					  {"batteryVoltage", t.batteryVoltage},
					  {"rssi", t.rssi},
					  {"latencyMs", t.latencyMs},
					  {"temperature", t.temperature},
					  {"activity", t.activity},
					  {"planted", planted},
					  {"mag",
					   {{"state", magStateName(t.magState)},
						{"progress", t.magProgress},
						{"residual", t.magCalibration.valid ? t.magCalibration.residual : 1.0},
						{"dip", rad2deg(t.magCalibration.dip)}}}});
	}
	// Assigned trackers the hub has not heard since the start (the Body page lists them)
	json mappedOffline = json::array();
	for (const auto& [mac, tc] : m_config.trackers) {
		if (tc.role == Role::None) continue;
		if (std::any_of(trackers.begin(), trackers.end(), [&](const TrackerSnapshot& t) { return t.mac == mac; }))
			continue;
		mappedOffline.push_back({{"mac", mac}, {"role", roleName(tc.role)}});
	}

	double countdown = (m_calPhase == CalPhase::Stand || m_calPhase == CalPhase::Ski)
						 ? std::max(0.0, m_calDeadline - now)
						 : 0.0;
	json state = {
		{"type", "state"},
		{"version", 1},
		{"driver", {{"version", kVersion}, {"trackerPort", 6969}, {"portError", m_hub.portError()}}},
		{"dongle", [&] {
			 DongleBridge::Status d = m_dongle.status();
			 return json{{"connected", d.connected}, {"port", d.port},     {"mac", d.mac},
						 {"channel", d.channel},     {"version", d.version}, {"trackers", d.trackers}};
		 }()},
		{"mode", modeName(m_config.mode)},
		{"userHeight", m_config.userHeight},
		{"measuredEyeHeight", m_config.measuredEyeHeight ? json(*m_config.measuredEyeHeight) : json(nullptr)},
		{"floorKnown", m_floorKnown},
		{"calibrated", calibrated},
		{"ready", ready(trackers)},
		{"calibration",
		 {{"phase", phases[int(m_calPhase)]}, {"quick", m_calQuick}, {"countdown", countdown}, {"message", m_calMessage}}},
		{"lastCalibration",
		 {{"full", m_config.lastFullCalibration > 0 ? json(m_config.lastFullCalibration) : json(nullptr)},
		  {"reset", m_config.lastQuickReset > 0 ? json(m_config.lastQuickReset) : json(nullptr)}}},
		{"settings",
		 {{"magEnabled", s.magEnabled},
		  {"magSensitivity", s.magSensitivity},
		  {"smoothing", s.smoothing},
		  {"footPlantStrength", s.footPlantStrength},
		  {"inertialWeight", s.inertialWeight},
		  {"velocityPrediction", s.velocityPrediction},
		  {"recording", s.recording}}},
		{"recording",
		 {{"active", m_recorder.isOpen()},
		  {"file", m_recorder.isOpen() ? m_recorder.path() : std::string()},
		  {"megabytes", double(m_recorder.bytes()) / 1e6}}},
		{"trackers", ts},
		{"mappedOffline", mappedOffline},
		{"outputs", {{"waist", out.valid}, {"leftFoot", out.valid}, {"rightFoot", out.valid}}},
	};
	m_overlay.send(state);
}

bool Engine::ready(const std::vector<TrackerSnapshot>& trackers) const {
	bool left = false, right = false;
	for (const auto& [mac, tc] : m_config.trackers) {
		if (tc.role == Role::None) continue;
		auto t = std::find_if(trackers.begin(), trackers.end(), [&](const TrackerSnapshot& s) { return s.mac == mac; });
		Quat q;
		if (t == trackers.end() || !trackerOrientation(*t, tc, q)) return false;
		left = left || tc.role == Role::LeftAnkle;
		right = right || tc.role == Role::RightAnkle;
	}
	return left && right;
}

void Engine::updateMode(double now, bool waistUsable) {
	// A tracker blinking out for a moment must not reset the solver twice
	constexpr double kToThree = 0.5, kToAnkles = 2.0;  // s
	if (waistUsable) {
		m_waistLostSince = -1;
		if (m_waistSeenSince < 0) m_waistSeenSince = now;
	} else {
		m_waistSeenSince = -1;
		if (m_waistLostSince < 0) m_waistLostSince = now;
	}
	Mode want = m_config.mode;
	if (m_config.mode == Mode::Ankles && waistUsable && now - m_waistSeenSince >= kToThree) want = Mode::Three;
	if (m_config.mode == Mode::Three && !waistUsable && now - m_waistLostSince >= kToAnkles) want = Mode::Ankles;
	if (want == m_config.mode) return;
	m_config.mode = want;
	log("mode: %s (waist tracker %s)", modeName(want), waistUsable ? "usable" : "missing");
	recordEvent("mode", modeName(want));
	applySettings();
	m_solver.reset();
	markDirty();
}

namespace {

// Compact JSON numbers for the view messages (0.1 mm / 1e-4)
double r4(double v) { return std::round(v * 1e4) / 1e4 + 0.0; }
json v3(const Vec3& v) { return json::array({r4(v.x), r4(v.y), r4(v.z)}); }
json q4(const Quat& q) { return json::array({r4(q.w), r4(q.x), r4(q.y), r4(q.z)}); }

}  // namespace

void Engine::publishViews(double now, const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out,
						  const Pose& hmd, bool hmdValid) {
	if (m_viewUntil < 0 || TrackerHub::now() > m_viewUntil || !m_overlay.hasPeer()) return;

	if (now - m_lastPose >= 1.0 / 30 || now < m_lastPose) {
		m_lastPose = now;
		json hands = json::array();
		for (int i = 0; i < 2; i++)
			hands.push_back({{"valid", m_handValid[i]}, {"p", v3(m_hands[i].position)}, {"q", q4(m_hands[i].rotation)}});
		json pose = {
			{"type", "pose"},
			{"valid", out.valid},
			{"ready", ready(trackers)},
			{"floor", r4(m_floorY)},
			{"floorKnown", m_floorKnown},
			{"mode", modeName(m_config.mode)},
			{"head", {{"valid", hmdValid}, {"p", v3(hmd.position)}, {"q", q4(hmd.rotation)}}},
			{"joints",
			 {{"neck", v3(out.debug.neck)},
			  {"pelvis", v3(out.waist.position)},
			  {"hip0", v3(out.debug.hip0)},
			  {"hipL", v3(out.debug.hipJoints[0])},
			  {"hipR", v3(out.debug.hipJoints[1])},
			  {"kneeL", v3(out.knees[0])},
			  {"kneeR", v3(out.knees[1])},
			  {"ankleL", v3(out.feet[0].position)},
			  {"ankleR", v3(out.feet[1].position)}}},
			{"pelvisQ", q4(out.waist.rotation)},
			{"feetQ", json::array({q4(out.feet[0].rotation), q4(out.feet[1].rotation)})},
			{"planted", json::array({out.planted[0], out.planted[1]})},
			{"hands", hands},
		};
		m_overlay.send(pose);
	}

	if (!m_viewTracker.empty() && (now - m_lastDetail >= 1.0 / 20 || now < m_lastDetail)) {
		m_lastDetail = now;
		auto t = std::find_if(trackers.begin(), trackers.end(),
							  [&](const TrackerSnapshot& s) { return s.mac == m_viewTracker; });
		if (t != trackers.end()) publishDetail(*t, m_viewMag || t->magState == MagState::Calibrating);
	}
}

void Engine::publishDetail(const TrackerSnapshot& t, bool withMagView) {
	const MagCalibration& cal = t.magCalibration;
	json d = {
		{"type", "detail"},
		{"mac", t.mac},
		{"connected", t.connected},
		{"firmware", t.firmware},
		{"board", t.board},
		{"imuType", t.imuType},
		{"mcu", t.mcu},
		{"protocolVersion", t.protocolVersion},
		{"bootCount", t.bootCount},
		{"hasRotation", t.hasRotation},
		{"q", q4(t.corrected())},
		{"qRaw", q4(t.qWorld)},
		{"gyro", v3(t.gyroLocal)},
		{"accel", v3(t.accelLocal)},
		{"hasMag", t.hasMag},
		{"magRaw", json::array({t.magRaw[0], t.magRaw[1], t.magRaw[2]})},
		{"mag",
		 {{"state", magStateName(t.magState)},
		  {"progress", r4(t.magProgress)},
		  {"valid", cal.valid},
		  {"residual", cal.valid ? r4(cal.residual) : 1.0},
		  {"dip", r4(rad2deg(cal.dip))},
		  {"norm", r4(t.magNorm)},
		  {"fieldDip", r4(t.magDip)},
		  {"hasHeading", t.hasHeading},
		  {"heading", r4(rad2deg(t.heading))}}},
	};
	// Calibrated field in the sensor frame (units of the calibration's field norm)
	if (cal.valid && t.hasMag) d["magField"] = v3(cal.apply(t.magRaw));
	if (withMagView) {
		if (auto v = m_hub.magView(t.mac)) {
			// Keep the datagram small (well under 64 KB): at most ~600 points
			const size_t n = v->samples.size(), step = std::max<size_t>(1, (n + 599) / 600);
			json pts = json::array();
			for (size_t i = 0; i < n; i += step) {
				const Vec3& p = v->samples[i];
				pts.push_back({std::lround(p.x), std::lround(p.y), std::lround(p.z)});
			}
			json dirs = json::array();
			for (int i = 0; i < MagCalibrator::kBins; i++) dirs.push_back(v3(MagCalibrator::binDirection(i)));
			d["magView"] = {{"samples", pts},
							{"bins", v->bins},
							{"binDirs", dirs},
							{"target", MagCalibrator::binTarget()},
							{"guided", v->guided},
							{"progress", r4(v->progress)}};
		}
	}
	m_overlay.send(d);
}

}  // namespace smt

// ---- C ABI ----

extern "C" __declspec(dllexport) int sl_engine_api_version() { return SMT_ENGINE_API_VERSION; }

extern "C" __declspec(dllexport) void* sl_engine_create(const SmtEngineHost* host, const char* handover) {
	try {
		return new smt::Engine(*host, handover);
	} catch (const std::exception& e) {
		if (host && host->log) host->log((std::string("engine failed to start: ") + e.what()).c_str());
		return nullptr;
	} catch (...) {
		if (host && host->log) host->log("engine failed to start: unknown exception");
		return nullptr;
	}
}

extern "C" __declspec(dllexport) void sl_engine_batteries(void* engine, SmtBattery* out) {
	static_cast<const smt::Engine*>(engine)->batteries(out);
}

extern "C" __declspec(dllexport) void sl_engine_frame(void* engine, const SmtFrameInput* in, SmtFrameOutput* out) {
	std::memset(out, 0, sizeof(*out));
	static_cast<smt::Engine*>(engine)->frame(*in, *out);
}

extern "C" __declspec(dllexport) size_t sl_engine_destroy(void* engine, char* handover, size_t capacity) {
	auto* e = static_cast<smt::Engine*>(engine);
	std::string s = e->handover().dump();
	delete e;
	if (handover && capacity) {
		size_t n = std::min(s.size(), capacity - 1);
		std::memcpy(handover, s.data(), n);
		handover[n] = 0;
	}
	return s.size();
}
