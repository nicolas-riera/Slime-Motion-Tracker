#include "driver_link.h"

#include <ws2tcpip.h>
#include <mswsock.h>  // SIO_UDP_CONNRESET

#include <algorithm>
#include <cmath>

namespace smt {

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
    t.role = getString(j, "role", "none");
    t.connected = getBool(j, "connected", false);
    t.rotationHz = getFloat(j, "rotationHz", 0.0f);
    t.magHz = getFloat(j, "magHz", 0.0f);
    t.battery = getFloat(j, "battery", -1.0f);
    t.batteryVoltage = getFloat(j, "batteryVoltage", 0.0f);
    t.rssi = getInt(j, "rssi", 0);
    t.latencyMs = getFloat(j, "latencyMs", -1.0f);
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

bool isVec3(const json& a) {
    return a.is_array() && a.size() == 3 && a[0].is_number() && a[1].is_number() && a[2].is_number();
}

Vec3f toVec3(const json& a) {
    if (!isVec3(a)) return {};
    return {a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
}

Vec3f getVec3(const json& j, const char* key) {
    auto it = j.find(key);
    return it != j.end() ? toVec3(*it) : Vec3f{};
}

Quatf toQuat(const json& a) {
    if (!a.is_array() || a.size() != 4 || !a[0].is_number() || !a[1].is_number() || !a[2].is_number() ||
        !a[3].is_number())
        return {};
    return {a[0].get<float>(), a[1].get<float>(), a[2].get<float>(), a[3].get<float>()};
}

Quatf getQuat(const json& j, const char* key) {
    auto it = j.find(key);
    return it != j.end() ? toQuat(*it) : Quatf{};
}

PoseInfo parsePose(const json& j) {
    PoseInfo p;
    p.valid = getBool(j, "valid", false);
    p.ready = getBool(j, "ready", false);
    p.floor = getFloat(j, "floor", 0.0f);
    p.floorKnown = getBool(j, "floorKnown", false);
    const json& head = getObject(j, "head");
    p.headValid = getBool(head, "valid", false);
    p.head = getVec3(head, "p");
    p.headQ = getQuat(head, "q");
    const json& jt = getObject(j, "joints");
    p.neck = getVec3(jt, "neck");
    p.pelvis = getVec3(jt, "pelvis");
    p.hip0 = getVec3(jt, "hip0");
    p.hipL = getVec3(jt, "hipL");
    p.hipR = getVec3(jt, "hipR");
    p.kneeL = getVec3(jt, "kneeL");
    p.kneeR = getVec3(jt, "kneeR");
    p.ankleL = getVec3(jt, "ankleL");
    p.ankleR = getVec3(jt, "ankleR");
    p.pelvisQ = getQuat(j, "pelvisQ");
    if (auto it = j.find("feetQ"); it != j.end() && it->is_array() && it->size() == 2)
        for (size_t i = 0; i < 2; ++i) p.footQ[i] = toQuat((*it)[i]);
    if (auto it = j.find("planted"); it != j.end() && it->is_array() && it->size() == 2)
        for (size_t i = 0; i < 2; ++i) p.planted[i] = (*it)[i].is_boolean() && (*it)[i].get<bool>();
    if (auto it = j.find("hands"); it != j.end() && it->is_array() && it->size() == 2) {
        for (size_t i = 0; i < 2; ++i) {
            const json& h = (*it)[i];
            if (!h.is_object()) continue;
            p.handValid[i] = getBool(h, "valid", false);
            p.hand[i] = getVec3(h, "p");
            p.handQ[i] = getQuat(h, "q");
        }
    }
    return p;
}

TrackerDetail parseDetail(const json& j) {
    TrackerDetail d;
    d.mac = getString(j, "mac", "");
    d.connected = getBool(j, "connected", false);
    d.firmware = getString(j, "firmware", "");
    d.board = getInt(j, "board", 0);
    d.imuType = getInt(j, "imuType", 0);
    d.mcu = getInt(j, "mcu", 0);
    d.protocolVersion = getInt(j, "protocolVersion", 0);
    d.bootCount = getInt(j, "bootCount", 0);
    d.hasRotation = getBool(j, "hasRotation", false);
    d.q = getQuat(j, "q");
    d.qRaw = getQuat(j, "qRaw");
    d.gyro = getVec3(j, "gyro");
    d.accel = getVec3(j, "accel");
    d.hasMag = getBool(j, "hasMag", false);
    const Vec3f raw = getVec3(j, "magRaw");
    d.magRaw[0] = static_cast<int>(raw.x);
    d.magRaw[1] = static_cast<int>(raw.y);
    d.magRaw[2] = static_cast<int>(raw.z);
    if (auto it = j.find("magField"); it != j.end() && isVec3(*it)) {
        d.hasMagField = true;
        d.magField = toVec3(*it);
    }
    const json& mag = getObject(j, "mag");
    d.magState = getString(mag, "state", "absent");
    d.magProgress = getFloat(mag, "progress", 0.0f);
    d.magValid = getBool(mag, "valid", false);
    d.magResidual = getFloat(mag, "residual", 1.0f);
    d.magDip = getFloat(mag, "dip", 0.0f);
    d.magNorm = getFloat(mag, "norm", 0.0f);
    d.magFieldDip = getFloat(mag, "fieldDip", 0.0f);
    d.hasHeading = getBool(mag, "hasHeading", false);
    d.heading = getFloat(mag, "heading", 0.0f);
    if (auto it = j.find("magView"); it != j.end() && it->is_object()) {
        const json& v = *it;
        d.hasMagView = true;
        if (auto s = v.find("samples"); s != v.end() && s->is_array()) {
            d.magView.samples.reserve(s->size());
            for (const json& p : *s)
                if (isVec3(p)) d.magView.samples.push_back(toVec3(p));
        }
        if (auto b = v.find("bins"); b != v.end() && b->is_array())
            for (const json& n : *b) d.magView.bins.push_back(n.is_number() ? static_cast<int>(n.get<double>()) : 0);
        if (auto b = v.find("binDirs"); b != v.end() && b->is_array())
            for (const json& p : *b)
                if (isVec3(p)) d.magView.binDirs.push_back(toVec3(p));
        d.magView.target = std::max(1, getInt(v, "target", 3));
        d.magView.guided = getBool(v, "guided", false);
        d.magView.progress = getFloat(v, "progress", 0.0f);
    }
    return d;
}

DriverState parseState(const json& j) {
    DriverState s;
    s.version = getInt(j, "version", 0);

    const json& drv = getObject(j, "driver");
    s.driverVersion = getString(drv, "version", "");
    s.trackerPort = getInt(drv, "trackerPort", 0);
    s.portError = getString(drv, "portError", "");

    s.mode = getString(j, "mode", "three");
    s.userHeight = getFloat(j, "userHeight", 1.75f);
    if (auto it = j.find("measuredEyeHeight"); it != j.end() && it->is_number())
        s.measuredEyeHeight = it->get<float>();
    s.floorKnown = getBool(j, "floorKnown", false);
    s.calibrated = getBool(j, "calibrated", false);
    s.ready = getBool(j, "ready", false);

    const json& cal = getObject(j, "calibration");
    s.calibration.phase = getString(cal, "phase", "idle");
    s.calibration.quick = getBool(cal, "quick", false);
    s.calibration.countdown = getFloat(cal, "countdown", 0.0f);
    s.calibration.message = getString(cal, "message", "");
    s.hipSideOffset = getFloat(j, "hipSideOffset", 0.0f);
    const json& hip = getObject(j, "hipCalibration");
    s.hipCalibration.phase = getString(hip, "phase", "idle");
    s.hipCalibration.countdown = getFloat(hip, "countdown", 0.0f);
    s.hipCalibration.message = getString(hip, "message", "");
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
    if (auto it = j.find("mappedOffline"); it != j.end() && it->is_array()) {
        for (const json& t : *it)
            if (t.is_object()) s.mappedOffline.push_back(parseTracker(t));
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

    if (buffer_.empty()) buffer_.resize(65536);  // the largest UDP datagram
    for (;;) {
        sockaddr_in from{};
        int fromLen = sizeof(from);
        const int n = recvfrom(socket_, buffer_.data(), static_cast<int>(buffer_.size()), 0,
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
        handleDatagram(std::string_view(buffer_.data(), static_cast<size_t>(n)));
    }

    if (Clock::now() - lastHello_ >= kHelloInterval) sendHello();
}

void DriverLink::handleDatagram(std::string_view data) {
    const json msg = json::parse(data, nullptr, /*allow_exceptions=*/false);
    if (!msg.is_object()) return;
    const std::string type = getString(msg, "type", "");
    if (type == "state") {
        state_ = parseState(msg);
        hasState_ = true;
        lastState_ = Clock::now();
    } else if (type == "pose") {
        pose_ = parsePose(msg);
        lastPose_ = Clock::now();
    } else if (type == "detail") {
        detail_ = parseDetail(msg);
        lastDetail_ = Clock::now();
    }
}

double DriverLink::poseAge() const {
    if (lastPose_ == Clock::time_point{}) return 1e9;
    return std::chrono::duration<double>(Clock::now() - lastPose_).count();
}

double DriverLink::detailAge() const {
    if (lastDetail_ == Clock::time_point{}) return 1e9;
    return std::chrono::duration<double>(Clock::now() - lastDetail_).count();
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

void DriverLink::hipCalibrate() { send({{"type", "hipCalibrate"}}); }

void DriverLink::hipCancel() { send({{"type", "hipCancel"}}); }

void DriverLink::hipReset() { send({{"type", "hipReset"}}); }

void DriverLink::magCalibrate(std::string_view mac) {
    send({{"type", "magCalibrate"}, {"mac", mac}});
}

void DriverLink::magCancel(std::string_view mac) { send({{"type", "magCancel"}, {"mac", mac}}); }

void DriverLink::magReset(std::string_view mac) { send({{"type", "magReset"}, {"mac", mac}}); }

void DriverLink::sendFloor(bool valid, float y) {
    send({{"type", "floor"}, {"valid", valid}, {"y", round4(y)}});
}

void DriverLink::resetSettings() { send({{"type", "resetSettings"}}); }

void DriverLink::sendView(bool visible, std::string_view tracker, bool magView) {
    send({{"type", "view"}, {"visible", visible}, {"tracker", tracker}, {"magView", magView}});
}

}  // namespace smt
