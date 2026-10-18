#include "driver_link.h"

#include <ws2tcpip.h>
#include <mswsock.h>  // SIO_UDP_CONNRESET

#include <array>
#include <cmath>

namespace slimelite {

namespace {

using nlohmann::json;

constexpr auto kHelloInterval = std::chrono::seconds(1);
constexpr auto kStateTimeout = std::chrono::seconds(2);

// Tolerant accessors: a missing key, a null or a wrongly typed value yields the default
// instead of throwing, so a partially filled `state` never breaks the UI.
float getFloat(const json& j, const char* key, float def) {
    auto it = j.find(key);
    return (it != j.end() && it->is_number()) ? it->get<float>() : def;
}

int getInt(const json& j, const char* key, int def) {
    auto it = j.find(key);
    return (it != j.end() && it->is_number()) ? static_cast<int>(it->get<double>()) : def;
}

bool getBool(const json& j, const char* key, bool def) {
    auto it = j.find(key);
    return (it != j.end() && it->is_boolean()) ? it->get<bool>() : def;
}

std::string getString(const json& j, const char* key, const std::string& def) {
    auto it = j.find(key);
    return (it != j.end() && it->is_string()) ? it->get<std::string>() : def;
}

// Floats are sent rounded to 0.1 mm / 1e-4 so the JSON stays readable
// (1.78 instead of 1.7799999713897705).
// Adding 0.0 turns -0.0 into 0.0.
double round4(float v) { return std::round(static_cast<double>(v) * 1e4) / 1e4 + 0.0; }

const json& getObject(const json& j, const char* key) {
    static const json kEmpty = json::object();
    auto it = j.find(key);
    return (it != j.end() && it->is_object()) ? *it : kEmpty;
}

TrackerInfo parseTracker(const json& j) {
    TrackerInfo t;
    t.mac = getString(j, "mac", "");
    t.ip = getString(j, "ip", "");
    t.transport = getString(j, "transport", "wifi");
    t.role = getString(j, "role", "none");
    t.connected = getBool(j, "connected", false);
    t.rotationHz = getFloat(j, "rotationHz", 0.0f);
    t.magHz = getFloat(j, "magHz", 0.0f);
    t.battery = getFloat(j, "battery", -1.0f);
    t.batteryVoltage = getFloat(j, "batteryVoltage", 0.0f);
    t.rssi = getInt(j, "rssi", 0);
    t.temperature = getFloat(j, "temperature", 0.0f);
    t.activity = getFloat(j, "activity", 0.0f);
    t.planted = getBool(j, "planted", false);
    const json& mag = getObject(j, "mag");
    t.mag.state = getString(mag, "state", "absent");
    t.mag.progress = getFloat(mag, "progress", 0.0f);
    t.mag.residual = getFloat(mag, "residual", 0.0f);
    t.mag.dip = getFloat(mag, "dip", 0.0f);
    return t;
}

DriverState parseState(const json& j) {
    DriverState s;
    s.version = getInt(j, "version", 0);

    const json& drv = getObject(j, "driver");
    s.driverVersion = getString(drv, "version", "");
    s.trackerPort = getInt(drv, "trackerPort", 0);
    s.portError = getString(drv, "portError", "");

    const json& dg = getObject(j, "dongle");
    s.dongle.connected = getBool(dg, "connected", false);
    s.dongle.port = getString(dg, "port", "");
    s.dongle.mac = getString(dg, "mac", "");
    s.dongle.channel = getInt(dg, "channel", 0);
    s.dongle.version = getInt(dg, "version", 0);
    s.dongle.trackers = getInt(dg, "trackers", 0);

    s.mode = getString(j, "mode", "three");
    s.userHeight = getFloat(j, "userHeight", 1.75f);
    if (auto it = j.find("measuredEyeHeight"); it != j.end() && it->is_number())
        s.measuredEyeHeight = it->get<float>();
    s.floorKnown = getBool(j, "floorKnown", false);
    s.calibrated = getBool(j, "calibrated", false);

    const json& cal = getObject(j, "calibration");
    s.calibration.phase = getString(cal, "phase", "idle");
    s.calibration.countdown = getFloat(cal, "countdown", 0.0f);
    s.calibration.message = getString(cal, "message", "");
    const json& last = getObject(j, "lastCalibration");
    if (auto it = last.find("full"); it != last.end() && it->is_number()) s.calibration.lastFull = it->get<double>();
    if (auto it = last.find("reset"); it != last.end() && it->is_number()) s.calibration.lastReset = it->get<double>();

    const json& set = getObject(j, "settings");
    const DriverSettings def;
    s.settings.magEnabled = getBool(set, "magEnabled", def.magEnabled);
    s.settings.magSensitivity = getFloat(set, "magSensitivity", def.magSensitivity);
    s.settings.smoothing = getFloat(set, "smoothing", def.smoothing);
    s.settings.footPlantStrength = getFloat(set, "footPlantStrength", def.footPlantStrength);
    s.settings.inertialWeight = getFloat(set, "inertialWeight", def.inertialWeight);
    s.settings.velocityPrediction = getBool(set, "velocityPrediction", def.velocityPrediction);
    s.settings.recording = getBool(set, "recording", def.recording);

    const json& rec = getObject(j, "recording");
    s.recording.active = getBool(rec, "active", false);
    s.recording.file = getString(rec, "file", "");
    s.recording.megabytes = getFloat(rec, "megabytes", 0.0f);

    if (auto it = j.find("trackers"); it != j.end() && it->is_array()) {
        for (const json& t : *it)
            if (t.is_object()) s.trackers.push_back(parseTracker(t));
    }

    const json& out = getObject(j, "outputs");
    s.outputs.waist = getBool(out, "waist", false);
    s.outputs.leftFoot = getBool(out, "leftFoot", false);
    s.outputs.rightFoot = getBool(out, "rightFoot", false);
    return s;
}

}  // namespace

DriverLink::~DriverLink() { stop(); }

bool DriverLink::start() {
    if (socket_ != INVALID_SOCKET) return true;

    if (!wsaStarted_) {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return false;
        wsaStarted_ = true;
    }

    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) return false;

    // Ephemeral port on the loopback interface only.
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = 0;
    inet_pton(AF_INET, "127.0.0.1", &local.sin_addr);
    if (::bind(socket_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        stop();
        return false;
    }

    u_long nonBlocking = 1;
    ioctlsocket(socket_, FIONBIO, &nonBlocking);

    // Without this, an ICMP "port unreachable" (driver not running) makes every
    // following recvfrom() fail with WSAECONNRESET on Windows.
    BOOL reportConnReset = FALSE;
    DWORD bytesReturned = 0;
    WSAIoctl(socket_, SIO_UDP_CONNRESET, &reportConnReset, sizeof(reportConnReset), nullptr, 0,
             &bytesReturned, nullptr, nullptr);

    driverAddr_ = {};
    driverAddr_.sin_family = AF_INET;
    driverAddr_.sin_port = htons(kDriverPort);
    inet_pton(AF_INET, "127.0.0.1", &driverAddr_.sin_addr);

    sendHello();
    return true;
}

void DriverLink::stop() {
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
        socket_ = INVALID_SOCKET;
    }
    if (wsaStarted_) {
        WSACleanup();
        wsaStarted_ = false;
    }
}

void DriverLink::poll() {
    if (socket_ == INVALID_SOCKET) return;

    std::array<char, 65536> buffer{};
    for (;;) {
        sockaddr_in from{};
        int fromLen = sizeof(from);
        const int n = recvfrom(socket_, buffer.data(), static_cast<int>(buffer.size()), 0,
                               reinterpret_cast<sockaddr*>(&from), &fromLen);
        if (n == SOCKET_ERROR) {
            const int err = WSAGetLastError();
            if (err == WSAECONNRESET || err == WSAEMSGSIZE) continue;  // skip this datagram
            break;                                                     // WSAEWOULDBLOCK: drained
        }
        // Only accept datagrams coming from the driver's port.
        if (from.sin_port != driverAddr_.sin_port ||
            from.sin_addr.s_addr != driverAddr_.sin_addr.s_addr)
            continue;
        handleDatagram(std::string_view(buffer.data(), static_cast<size_t>(n)));
    }

    if (Clock::now() - lastHello_ >= kHelloInterval) sendHello();
}

void DriverLink::handleDatagram(std::string_view data) {
    const json msg = json::parse(data, nullptr, /*allow_exceptions=*/false);
    if (!msg.is_object()) return;
    if (getString(msg, "type", "") != "state") return;
    state_ = parseState(msg);
    hasState_ = true;
    lastState_ = Clock::now();
}

bool DriverLink::connected() const {
    return hasState_ && (Clock::now() - lastState_) < kStateTimeout;
}

void DriverLink::send(const json& msg) {
    if (socket_ == INVALID_SOCKET) return;
    const std::string text = msg.dump();
    sendto(socket_, text.data(), static_cast<int>(text.size()), 0,
           reinterpret_cast<const sockaddr*>(&driverAddr_), sizeof(driverAddr_));
}

void DriverLink::sendHello() {
    lastHello_ = Clock::now();
    send({{"type", "hello"}});
}

void DriverLink::setMode(std::string_view mode) { send({{"type", "setMode"}, {"mode", mode}}); }

void DriverLink::setHeight(float meters) { send({{"type", "setHeight"}, {"meters", round4(meters)}}); }

void DriverLink::setRole(std::string_view mac, std::string_view role) {
    send({{"type", "setRole"}, {"mac", mac}, {"role", role}});
}

void DriverLink::setSetting(std::string_view key, float value) {
    send({{"type", "setSetting"}, {"key", key}, {"value", round4(value)}});
}

void DriverLink::setSetting(std::string_view key, bool value) {
    send({{"type", "setSetting"}, {"key", key}, {"value", value}});
}

void DriverLink::calibrate() { send({{"type", "calibrate"}}); }

void DriverLink::quickReset() { send({{"type", "quickReset"}}); }

void DriverLink::cancelCalibration() { send({{"type", "cancelCalibration"}}); }

void DriverLink::magCalibrate(std::string_view mac) {
    send({{"type", "magCalibrate"}, {"mac", mac}});
}

void DriverLink::magCancel(std::string_view mac) { send({{"type", "magCancel"}, {"mac", mac}}); }

void DriverLink::magReset(std::string_view mac) { send({{"type", "magReset"}, {"mac", mac}}); }

void DriverLink::sendFloor(bool valid, float y) {
    send({{"type", "floor"}, {"valid", valid}, {"y", round4(y)}});
}

}  // namespace slimelite
