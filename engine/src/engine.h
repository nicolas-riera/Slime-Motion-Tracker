// The hot-reloadable engine: tracker hub, calibration state machine, body solver,
// overlay link and config. Driven by the host through engine_api.h.
#pragma once

#include <map>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "overlay_link.h"
#include "recorder.h"
#include "slimelite/body_solver.h"
#include "slimelite/config.h"
#include "slimelite/engine_api.h"
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
	void updateCalibration(double now, const Pose& hmd, bool hmdValid,
						   const std::vector<TrackerSnapshot>& trackers);
	void finishCalibration(const Pose& hmd);
	void publishState(const std::vector<TrackerSnapshot>& trackers, const BodyOutput& out);
	void applySettings();
	void markDirty() { m_dirty = true; }
	bool trackerOrientation(const TrackerSnapshot& t, const TrackerConfig& tc, Quat& out) const;
	void launchOverlay();
	void recordFrame(const SlFrameInput& in, const BodyOutput& out,
					 const std::vector<TrackerSnapshot>& trackers);
	void recordEvent(const char* what, const std::string& detail = {});
	void updateRecordingState();
	void log(const char* fmt, ...);

	SlEngineHost m_host;
	std::string m_moduleDirectory;
	Config m_config;
	TrackerHub m_hub;
	OverlayLink m_overlay;
	Recorder m_recorder;
	std::string m_handoverRecording;  // file to keep appending to after a hot reload
	double m_lastTrackerRecord = 0;
	BodySolver m_solver;

	// Floor height in raw tracking space, from the overlay or the calibration
	bool m_floorFromOverlay = false;
	double m_floorY = 0;
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
