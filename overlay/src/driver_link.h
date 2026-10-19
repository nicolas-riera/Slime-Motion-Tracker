// UDP link to the SteamVR driver (see docs/IPC.md).
//
// The overlay binds an ephemeral localhost port, sends `hello` once per second to
// 127.0.0.1:6971 and receives `state` messages streamed back by the driver.
#pragma once

#include <winsock2.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace smt {

inline constexpr uint16_t kDriverPort = 6971;

struct MagInfo {
    std::string state = "absent";  // absent|disabled|uncalibrated|calibrating|ok|disturbed
    float progress = 0.0f;
    float residual = 0.0f;
    float dip = 0.0f;
};

struct TrackerInfo {
    std::string mac;
    std::string ip;
    std::string role = "none";  // waist|left_ankle|right_ankle|none
    bool connected = false;
    float rotationHz = 0.0f;
    float magHz = 0.0f;
    float battery = -1.0f;  // 0..1, -1 = unknown
    float batteryVoltage = 0.0f;
    int rssi = 0;  // dBm, 0 = unknown
    float latencyMs = -1.0f;  // ping round trip, -1 = unknown
    float temperature = 0.0f;
    float activity = 0.0f;  // 0..1
    bool planted = false;
    MagInfo mag;
};

struct CalibrationInfo {
    std::string phase = "idle";  // idle|stand|ski|done|error
    bool quick = false;  // quick reset (stand only) rather than the full calibration
    float countdown = 0.0f;
    std::string message;
    // Unix seconds of the last successful full calibration / quick reset
    std::optional<double> lastFull;
    std::optional<double> lastReset;
};

struct DriverSettings {
    bool magEnabled = true;
    float magSensitivity = 0.5f;
    float smoothing = 0.3f;
    float footPlantStrength = 0.7f;
    float inertialWeight = 0.5f;
    bool velocityPrediction = false;
    bool recording = false;
};

struct RecordingInfo {
    bool active = false;
    std::string file;
    float megabytes = 0.0f;
};

struct OutputsInfo {
    bool waist = false;
    bool leftFoot = false;
    bool rightFoot = false;
};

struct Vec3f {
    float x = 0, y = 0, z = 0;
};
struct Quatf {
    float w = 1, x = 0, y = 0, z = 0;
};

// `pose` stream (docs/IPC.md): the solved body in the SteamVR raw tracking space
// (Y up, meters), sent while the overlay shows the skeleton.
struct PoseInfo {
    bool valid = false;
    bool ready = false;
    float floor = 0.0f;
    bool floorKnown = false;
    bool headValid = false;
    Vec3f head;
    Quatf headQ;
    Vec3f neck, pelvis, hip0, hipL, hipR, kneeL, kneeR, ankleL, ankleR;
    Quatf pelvisQ;
    Quatf footQ[2];
    bool planted[2] = {false, false};
    bool handValid[2] = {false, false};
    Vec3f hand[2];
    Quatf handQ[2];
};

struct MagViewInfo {
    std::vector<Vec3f> samples;  // raw counts, sensor frame
    std::vector<int> bins;  // samples per orientation bin
    std::vector<Vec3f> binDirs;  // where "up" is in the sensor frame for each bin
    int target = 3;  // samples that fill a bin
    bool guided = false;
    float progress = 0.0f;
};

// `detail` stream: everything about the tracker selected in the UI.
struct TrackerDetail {
    std::string mac;
    bool connected = false;
    std::string firmware;
    int board = 0, imuType = 0, mcu = 0, protocolVersion = 0;
    int bootCount = 0;
    bool hasRotation = false;
    Quatf q, qRaw;  // mag-corrected / device fusion, sensor -> world
    Vec3f gyro;  // rad/s, sensor frame
    Vec3f accel;  // linear, m/s^2, sensor frame
    bool hasMag = false;
    int magRaw[3] = {0, 0, 0};
    bool hasMagField = false;
    Vec3f magField;  // calibrated, sensor frame, units of the field norm
    std::string magState = "absent";
    float magProgress = 0.0f;
    bool magValid = false;
    float magResidual = 1.0f, magDip = 0.0f, magNorm = 0.0f, magFieldDip = 0.0f;
    bool hasHeading = false;
    float heading = 0.0f;
    bool hasMagView = false;
    MagViewInfo magView;
};

// Last `state` message received from the driver.
struct DriverState {
    int version = 0;
    std::string driverVersion;
    int trackerPort = 0;
    std::string portError;
    std::string mode = "three";  // three|ankles
    float userHeight = 1.75f;
    std::optional<float> measuredEyeHeight;
    bool floorKnown = false;
    bool calibrated = false;
    bool ready = false;  // all mapped trackers connected and calibrated, both ankles mapped
    CalibrationInfo calibration;
    DriverSettings settings;
    std::vector<TrackerInfo> trackers;
    // Assigned trackers not heard since the engine started (offline, for the Body page)
    std::vector<TrackerInfo> mappedOffline;
    OutputsInfo outputs;
    RecordingInfo recording;
};

class DriverLink {
public:
    DriverLink() = default;
    ~DriverLink();
    DriverLink(const DriverLink&) = delete;
    DriverLink& operator=(const DriverLink&) = delete;

    // Initializes Winsock and opens the socket. Returns false on failure.
    bool start();
    void stop();

    // Call every frame: drains received datagrams and sends `hello` when due.
    void poll();

    // True when a `state` was received within the last 2 seconds.
    [[nodiscard]] bool connected() const;
    // True once at least one valid `state` was received (state() holds real data).
    [[nodiscard]] bool hasState() const { return hasState_; }
    [[nodiscard]] const DriverState& state() const { return state_; }
    // Latest `pose` / `detail` (see sendView) and how old they are, in seconds
    [[nodiscard]] const PoseInfo& pose() const { return pose_; }
    [[nodiscard]] double poseAge() const;
    [[nodiscard]] const TrackerDetail& detail() const { return detail_; }
    [[nodiscard]] double detailAge() const;

    // Commands (docs/IPC.md, "Overlay -> driver").
    void sendHello();
    void setHeight(float meters);
    void setRole(std::string_view mac, std::string_view role);
    void setSetting(std::string_view key, float value);
    void setSetting(std::string_view key, bool value);
    void calibrate();
    void quickReset();
    void cancelCalibration();
    void magCalibrate(std::string_view mac);
    void magCancel(std::string_view mac);
    void magReset(std::string_view mac);
    void sendFloor(bool valid, float y);
    void resetSettings();
    // What the UI shows: the driver streams `pose` (and `detail` for `tracker`) while
    // visible. Re-send about every 0.5 s; it expires after 2 s.
    void sendView(bool visible, std::string_view tracker, bool magView);

private:
    using Clock = std::chrono::steady_clock;

    void send(const nlohmann::json& msg);
    void handleDatagram(std::string_view data);

    SOCKET socket_ = INVALID_SOCKET;
    bool wsaStarted_ = false;
    std::vector<char> buffer_;  // receive buffer, allocated once
    sockaddr_in driverAddr_{};
    Clock::time_point lastHello_{};
    Clock::time_point lastState_{};
    bool hasState_ = false;
    DriverState state_;
    PoseInfo pose_;
    Clock::time_point lastPose_{};
    TrackerDetail detail_;
    Clock::time_point lastDetail_{};
};

}  // namespace smt
