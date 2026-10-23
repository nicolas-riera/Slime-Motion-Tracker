// State and helpers shared by the UI pages (ui.cpp, ui_tracker.cpp, ui_skeleton.cpp).
#pragma once

#include <chrono>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <type_traits>

#include "battery_estimator.h"
#include "driver_link.h"
#include "imgui.h"
#include "ui.h"
#include "view3d.h"
#include "widgets.h"

namespace smt::ui {

using Clock = std::chrono::steady_clock;

// A value the user just sent to the driver. It is displayed instead of the driver's
// value until the driver echoes it back (or a timeout expires), so a widget does not
// jump back to the old value during the round trip.
template <typename T>
class PendingValues {
public:
    void set(const std::string& key, T value) { entries_[key] = {std::move(value), Clock::now()}; }

    T resolve(const std::string& key, const T& remote) {
        auto it = entries_.find(key);
        if (it == entries_.end()) return remote;
        if (matches(it->second.value, remote) || Clock::now() - it->second.since > std::chrono::seconds(2)) {
            entries_.erase(it);
            return remote;
        }
        return it->second.value;
    }

    void clear() { entries_.clear(); }

private:
    static bool matches(const T& a, const T& b) {
        if constexpr (std::is_floating_point_v<T>)
            return std::abs(a - b) < 1e-3;
        else
            return a == b;
    }

    struct Entry {
        T value;
        Clock::time_point since;
    };
    std::map<std::string, Entry> entries_;
};

struct UiState {
    enum class Page { Home, Body, Settings };
    Page page = Page::Home;
    std::string trackerPage;  // Home: MAC of the open tracker page, empty for the list
    std::string magPage;  // MAC of the full-screen mag calibration page, empty when closed
    bool magSawGuided = false;  // the guided calibration ran while the page was open

    PendingValues<double> pendingNumbers;
    PendingValues<std::string> pendingStrings;
    // Slider being dragged: its value is kept locally and only sent on release
    std::string activeSlider;
    float activeSliderValue = 0.0f;
    bool sliderEdited = false;  // the active slider moved since it was grabbed

    BatteryEstimator battery;
    Clock::time_point lastBatterySave = Clock::now();

    // Countdown: local interpolation between the 10 Hz states, sounds
    std::string cdPhase = "idle";
    float cdValue = 0.0f, cdDuration = 3.0f;
    double cdAt = 0.0;
    std::string soundPhase = "idle";
    int soundSecond = 0;
    // Same for the single-tracker heading reset
    std::string trkResetSoundPhase = "idle";
    int trkResetSoundSecond = 0;
    // Last calibration result shown under the buttons
    std::string resultPhase;
    std::string resultMessage;
    double resultAt = -1e9;
    bool resultDismissed = true;

    // Motion glow of each tracker (0..1, fast rise, slow fade)
    std::map<std::string, float> glow;

    // 3D views requested this frame, and what was last sent to the engine
    bool wantSkeleton = false;
    std::string wantTracker;
    bool wantMag = false;
    bool viewSent = false;
    std::string sentTracker;
    bool sentMag = false;
    double lastViewSent = -1e9;

    v3::Orbit skeletonOrbit, trackerOrbit, cloudOrbit, coverageOrbit;
    // Kept between frames so their primitive buffers keep their capacity
    v3::Scene skeletonScene, trackerScene, cloudScene, coverageScene;
    float bodyYaw = 0.0f;
    bool bodyYawValid = false;
    v3::V3 bodyTarget;
    bool bodyTargetValid = false;

    bool connected = false;
    bool confirmReset = false;
};
extern UiState g;

double steadyNow();  // seconds
double wallNow();  // Unix seconds
std::string shortMac(const std::string& mac);
// "12 min ago", "never"
std::string agoText(const std::optional<double>& when);
const TrackerInfo* findTracker(const DriverState& s, const std::string& mac);

// Role shown for a tracker (the pending value while the change travels)
std::string resolvedRole(const TrackerInfo& t);
// Role dropdown for a tracker, sends setRole on change
void roleSelector(DriverLink& link, const TrackerInfo& t, float width);
// Name of a tracker in lists: its role, or "Tracker 36:36:94" when unassigned
std::string trackerName(const TrackerInfo& t);

// WiFi signal: 3 excellent, 2 good, 1 fair, 0 weak, -1 unknown (no reading yet)
int signalLevel(int rssi);
const char* signalQuality(int rssi);  // "excellent" .. "weak", "" when unknown
// WiFi icon of a tracker: one lit arc per signal level, the dot alone in orange
// when weak, slashed when offline
void wifiIcon(ImDrawList* dl, ImVec2 c, float s, const TrackerInfo& t, ImU32 on);
ImU32 magStateColor(const std::string& state);
const char* magStateLabel(const std::string& state);

// Cells of the tracker rows, drawn at p in a cell of height h
void drawConnection(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t);
void drawBattery(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t);
void drawMag(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t);
// Purple outline around a row while its tracker moves
void drawGlow(ImDrawList* dl, ImVec2 min, ImVec2 max, float glow, float rounding);

std::string batteryEstimateText(const std::string& mac, ImU32* color = nullptr);

// Pages
void drawSkeletonCard(const DriverLink& link, const DriverState& s, float height);
void drawTrackerPage(DriverLink& link, const DriverState& s, const TrackerInfo& t);
void drawMagPage(DriverLink& link, const DriverState& s);

// Firmware enums (SlimeVR firmware consts.h)
const char* boardName(int id);
const char* imuName(int id);
const char* mcuName(int id);

}  // namespace smt::ui
