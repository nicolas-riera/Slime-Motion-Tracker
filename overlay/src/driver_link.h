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

namespace slimelite {

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
    std::string transport = "wifi";  // wifi|espnow (through the USB dongle)
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
    bool recording = true;
};

struct RecordingInfo {
    bool active = false;
    std::string file;
    float megabytes = 0.0f;
};

struct DongleInfo {
    bool connected = false;
    std::string port, mac;
    int channel = 0;
    int version = 0;
    int trackers = 0;
};

struct OutputsInfo {
    bool waist = false;
    bool leftFoot = false;
    bool rightFoot = false;
};

// Last `state` message received from the driver.
struct DriverState {
    int version = 0;
    std::string driverVersion;
    int trackerPort = 0;
    std::string portError;
    DongleInfo dongle;
    std::string mode = "three";  // three|ankles
    float userHeight = 1.75f;
    std::optional<float> measuredEyeHeight;
    bool floorKnown = false;
    bool calibrated = false;
    CalibrationInfo calibration;
    DriverSettings settings;
    std::vector<TrackerInfo> trackers;
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

    // Commands (docs/IPC.md, "Overlay -> driver").
    void sendHello();
    void setMode(std::string_view mode);
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

private:
    using Clock = std::chrono::steady_clock;

    void send(const nlohmann::json& msg);
    void handleDatagram(std::string_view data);

    SOCKET socket_ = INVALID_SOCKET;
    bool wsaStarted_ = false;
    sockaddr_in driverAddr_{};
    Clock::time_point lastHello_{};
    Clock::time_point lastState_{};
    bool hasState_ = false;
    DriverState state_;
};

}  // namespace slimelite
