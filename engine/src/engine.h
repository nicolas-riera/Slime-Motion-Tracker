// The hot-reloadable engine: tracker hub, calibration state machine, body solver,
// overlay link and config. Driven by the host through engine_api.h.
#pragma once

#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "overlay_link.h"
#include "pose_worker.h"
#include "recorder.h"
#include "slimelite/body_solver.h"
#include "slimelite/config.h"
#include "slimelite/dongle_bridge.h"
#include "slimelite/engine_api.h"
#include "slimelite/foot_model.h"
#include "slimelite/tracker_hub.h"

namespace slimelite {

class Engine {
public:
	Engine(const SlEngineHost& host, const char* handover);
	~Engine();

	void frame(const SlFrameInput& in, SlFrameOutput& out);
	// Battery of the tracker behind each virtual tracker slot
	void batteries(SlBattery out[SL_DEVICE_COUNT]) const;
	// State passed to the next engine on hot reload
	nlohmann::json handover() const;

private:
	enum class CalPhase { Idle, Stand, Ski, Done, Error };
	struct Capture {
		Quat q;  // corrected tracker orientation
		bool inMagFrame = false;
	};

	void handleCommand(const nlohmann::json& cmd);

	void onFloor(double y, double now);
	void updateCalibration(double now, const Pose& hmd, bool hmdValid,
						   const std::vector<TrackerSnapshot>& trackers);
	void finishCalibration(const Pose& hmd);
	void publishState(const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out);
	void applySettings();
	void markDirty() { m_dirty = true; }
	bool trackerOrientation(const TrackerSnapshot& t, const TrackerConfig& tc, Quat& out) const;
	void recordFrame(const SlFrameInput& in, const BodyOutput& out,
					 const std::vector<TrackerSnapshot>& trackers);
	void recordEvent(const char* what, const std::string& detail = {});
	void updateRecordingState();
	void log(const char* fmt, ...);

	SlEngineHost m_host;
	std::string m_moduleDirectory;
	Config m_config;
	TrackerHub m_hub;
	DongleBridge m_dongle;  // after m_hub: stops first
	OverlayLink m_overlay;
	Recorder m_recorder;
	std::string m_handoverRecording;  // file to keep appending to after a hot reload
	double m_lastTrackerRecord = 0;
	BodySolver m_solver;
	// Learned foot model (models/foot_model.json next to the engine DLL), fused by
	// the solver as a horizontal ankle measurement; skipped when it cannot load
	FootModel m_footModel;
	// Learned pose model (models/pose_model.bin), fused into the solver's feet
	// (fusePose); skipped when it cannot load or the setting is off
	PoseWorker m_pose;
	double m_eyeHeight = 0;  // standing eye height, for the fusion
	// Controllers this frame (0 left, 1 right), for the pose model
	std::array<bool, 2> m_handValid{};
	std::array<Pose, 2> m_hands;

	// Floor height in raw tracking space, from the overlay or the calibration
	bool m_floorFromOverlay = false;
	double m_floorY = 0;
	// Floor update waiting to settle (see onFloor)
	double m_floorCandidate = 0, m_floorCandidateSince = -1, m_floorIgnoredLogged = 1e9;
	bool m_floorKnown = false;

	CalPhase m_calPhase = CalPhase::Idle;
	CalPhase m_recordedPhase = CalPhase::Idle;
	bool m_calQuick = false;
	double m_calDeadline = 0;
	std::string m_calMessage;
	Pose m_standHmd;
	std::map<std::string, Capture> m_standCaptures;
	// Tracker boot counts at the last calibration: without the mag, the yaw
	// alignment is lost when a tracker reboots
	std::map<std::string, int> m_bootAtCalibration;

	std::mutex m_pendingMutex;
	std::vector<std::pair<std::string, MagCalibration>> m_pendingMag;

	bool m_dirty = false;
	double m_lastSave = 0;
	double m_lastFrame = 0;
	double m_lastState = 0;
};

}  // namespace slimelite
