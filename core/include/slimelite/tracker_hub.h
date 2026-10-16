// UDP server speaking the SlimeVR tracker protocol, replacing the SlimeVR server.
// Runs its own network thread; per tracker it keeps the latest orientation and
// acceleration and runs the magnetometer calibration + heading filter.
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "slimelite/linalg.h"
#include "slimelite/mag.h"
#include "slimelite/protocol.h"

namespace slimelite {

enum class MagState { Absent, Disabled, Uncalibrated, Calibrating, Ok, Disturbed };
const char* magStateName(MagState s);

struct TrackerSnapshot {
	std::string mac;
	std::string ip;
	bool connected = false;
	int bootCount = 0;  // increments when the tracker (re)connects with a handshake

	bool hasRotation = false;
	Quat qWorld;  // device fusion output in the SteamVR world frame (yaw drifts)
	Vec3 accelLocal;  // linear acceleration, tracker frame, m/s^2
	double angularSpeed = 0;  // rad/s
	double activity = 0;  // 0..1, smoothed angular speed for identification
	double lastRotationTime = 0;

	// Mag heading: q_corrected = Ry(-heading) * qWorld
	bool hasHeading = false;
	double heading = 0;
	MagState magState = MagState::Absent;
	double magProgress = 0;
	MagCalibration magCalibration;
	double magNorm = 0, magDip = 0;

	double rotationHz = 0, magHz = 0;
	float batteryVoltage = 0, batteryLevel = -1;
	float temperature = 0;
	int rssi = 0;

	Quat corrected() const { return hasHeading ? Quat::yaw(-heading) * qWorld : qWorld; }
};

class TrackerHub {
public:
	using MagCalibrationCallback = std::function<void(const std::string& mac, const MagCalibration&)>;

	TrackerHub();
	~TrackerHub();

	// Network thread. Returns false (and sets portError) if the port is in use.
	bool start(uint16_t port = proto::kTrackerPort);
	void stop();
	std::string portError() const;

	std::vector<TrackerSnapshot> snapshot() const;

	void setMagSettings(bool enabled, const HeadingFilterSettings& s);
	// Called from the network thread when a calibration is found (guided or online)
	void onMagCalibration(MagCalibrationCallback cb);
	// Known calibration for a tracker (from the config), applied when it connects
	void setStoredMagCalibration(const std::string& mac, const MagCalibration& cal);
	void startGuidedMagCalibration(const std::string& mac);
	void cancelGuidedMagCalibration(const std::string& mac);
	void resetMagCalibration(const std::string& mac);

	// Entry points used by the network thread, public for the replay tool
	void processEvent(const std::string& mac, const proto::Event& e, double now);
	static double now();

private:
	struct Tracker;
	void networkLoop();
	Tracker& trackerFor(const std::string& mac);

	mutable std::mutex m_mutex;
	std::map<std::string, std::unique_ptr<Tracker>> m_trackers;
	std::map<std::string, MagCalibration> m_storedMag;
	MagCalibrationCallback m_magCallback;
	bool m_magEnabled = true;
	HeadingFilterSettings m_headingSettings;

	std::thread m_thread;
	std::atomic<bool> m_running{false};
	uintptr_t m_socket = ~uintptr_t(0);
	std::string m_portError;
	uint64_t m_packetNumber = 0;
};

}  // namespace slimelite
