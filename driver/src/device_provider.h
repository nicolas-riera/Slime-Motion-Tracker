// SteamVR server driver: owns the tracker hub, the body solver, the calibration
// state machine, the virtual trackers and the overlay link.
#pragma once

#include <openvr_driver.h>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include "overlay_link.h"
#include "slimelite/body_solver.h"
#include "slimelite/config.h"
#include "slimelite/tracker_hub.h"
#include "virtual_tracker.h"

namespace slimelite {

class DeviceProvider : public vr::IServerTrackedDeviceProvider {
public:
	vr::EVRInitError Init(vr::IVRDriverContext* ctx) override;
	void Cleanup() override;
	const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
	void RunFrame() override;
	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}

private:
	enum class CalPhase { Idle, Stand, Ski, Done, Error };
	struct Capture {
		bool ok = false;
		Quat q;  // corrected tracker orientation
		bool inMagFrame = false;
	};

	void handleOverlay();
	void handleCommand(const nlohmann::json& cmd);
	void updateCalibration(double now, const Pose& hmd, bool hmdValid,
						   const std::vector<TrackerSnapshot>& trackers);
	void finishCalibration(const Pose& hmd);
	void publishState(const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out);
	void applySettings();
	void markDirty() { m_dirty = true; }
	bool trackerOrientation(const TrackerSnapshot& t, const TrackerConfig& tc, Quat& out) const;
	void launchOverlay();
	void log(const char* fmt, ...);

	Config m_config;
	TrackerHub m_hub;
	OverlayLink m_overlay;
	BodySolver m_solver;
	std::array<std::unique_ptr<VirtualTracker>, 3> m_devices;  // waist, left foot, right foot

	// Floor height in raw tracking space, from the overlay or the calibration
	bool m_floorFromOverlay = false;
	double m_floorY = 0;
	bool m_floorKnown = false;

	CalPhase m_calPhase = CalPhase::Idle;
	bool m_calQuick = false;
	double m_calDeadline = 0;
	std::string m_calMessage;
	Pose m_standHmd;
	std::map<std::string, Capture> m_standCaptures;
	// Trackers whose boot frame yaw alignment is stale (rebooted, no mag)
	std::map<std::string, int> m_bootAtCalibration;

	std::mutex m_pendingMutex;
	std::vector<std::pair<std::string, MagCalibration>> m_pendingMag;

	bool m_dirty = false;
	double m_lastSave = 0;
	double m_lastFrame = 0;
	double m_lastState = 0;
	BodyOutput m_lastOutput;
};

}  // namespace slimelite
