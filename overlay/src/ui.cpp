#include "ui.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <map>
#include <string>
#include <type_traits>

#include "battery_estimator.h"
#include "imgui.h"

namespace smt::ui {

namespace {

using Clock = std::chrono::steady_clock;

const ImVec4 kGreen{0.35f, 0.85f, 0.40f, 1.0f};
const ImVec4 kOrange{1.00f, 0.65f, 0.20f, 1.0f};
const ImVec4 kRed{1.00f, 0.35f, 0.35f, 1.0f};
const ImVec4 kGrey{0.60f, 0.60f, 0.62f, 1.0f};
const ImVec4 kAccent{0.47f, 0.40f, 0.95f, 1.0f};

constexpr int kMinHeightCm = 120;
constexpr int kMaxHeightCm = 220;

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
        if (matches(it->second.value, remote) ||
            Clock::now() - it->second.since > std::chrono::seconds(2)) {
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
    ImFont* bodyFont = nullptr;
    ImFont* mediumFont = nullptr;  // big button labels
    ImFont* largeFont = nullptr;   // countdown, height value

    PendingValues<double> pendingNumbers;
    PendingValues<std::string> pendingStrings;

    // Slider currently dragged: its value is kept locally and only sent on release.
    std::string activeSlider;
    float activeSliderValue = 0.0f;

    std::string prevPhase = "idle";

    BatteryEstimator battery;
    Clock::time_point lastBatterySave = Clock::now();
};

UiState g;

// ---------------------------------------------------------------- small widgets

bool isCalibrationRunning(const std::string& phase) { return phase == "stand" || phase == "ski"; }

// Full-width colored box with wrapped text.
void banner(const char* text, const ImVec4& bg, const ImVec4& fg = ImVec4(1, 1, 1, 1)) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, bg);
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 6.0f);
    ImGui::PushID(text);
    ImGui::BeginChild("##banner", ImVec2(0, 0),
                      ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_AlwaysUseWindowPadding);
    ImGui::PushStyleColor(ImGuiCol_Text, fg);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
    ImGui::EndChild();
    ImGui::PopID();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
}

void helpText(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, kGrey);
    ImGui::TextWrapped("%s", text);
    ImGui::PopStyleColor();
}

void centeredText(const char* text, const ImVec4& color) {
    const float w = ImGui::CalcTextSize(text).x;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (ImGui::GetContentRegionAvail().x - w) * 0.5f));
    ImGui::TextColored(color, "%s", text);
}

// Slider that sends its value only when released (not every frame while dragging).
// Returns true and fills `sent` when a new value must be sent.
bool sliderOnRelease(const char* label, const std::string& key, float remote, float vMin, float vMax,
                     const char* format, float& sent) {
    float v = (g.activeSlider == key) ? g.activeSliderValue
                                      : static_cast<float>(g.pendingNumbers.resolve(key, remote));
    ImGui::SliderFloat(label, &v, vMin, vMax, format, ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::IsItemActive()) {
        g.activeSlider = key;
        g.activeSliderValue = v;
    } else if (g.activeSlider == key) {
        g.activeSlider.clear();
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        g.pendingNumbers.set(key, v);
        sent = v;
        return true;
    }
    return false;
}

// Checkbox bound to a boolean driver setting; sends immediately on click.
void settingCheckbox(DriverLink& link, const char* label, const char* key, bool remote) {
    bool v = g.pendingNumbers.resolve(key, remote ? 1.0 : 0.0) > 0.5;
    if (ImGui::Checkbox(label, &v)) {
        g.pendingNumbers.set(key, v ? 1.0 : 0.0);
        link.setSetting(key, v);
    }
}

void settingSlider(DriverLink& link, const char* label, const char* key, float remote) {
    float sent = 0.0f;
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (sliderOnRelease(label, key, remote, 0.0f, 1.0f, "%.2f", sent)) link.setSetting(key, sent);
}

std::string shortMac(const std::string& mac) {
    // "B4:E6:2D:36:36:94" -> "36:36:94"
    return mac.size() >= 8 ? mac.substr(mac.size() - 8) : mac;
}

// ---------------------------------------------------------------- sections

void drawHeader(const DriverLink& link, const DriverState& s, bool connected) {
    ImGui::TextColored(kAccent, "Slime Motion Tracker");
    ImGui::SameLine();
    if (connected) {
        if (s.driverVersion.empty())
            ImGui::TextColored(kGreen, "Driver connected");
        else
            ImGui::TextColored(kGreen, "Driver connected (v%s)", s.driverVersion.c_str());
        if (s.version != 1) {
            ImGui::SameLine();
            ImGui::TextColored(kOrange, "[protocol v%d, expected v1]", s.version);
        }
    } else {
        // The overlay runs the engine itself without SteamVR and hands over to the
        // driver when SteamVR starts: only the few seconds of a switch end up here
        ImGui::TextColored(kGrey, "Starting the engine...");
    }

    if (connected && !s.portError.empty()) {
        banner(s.portError.c_str(), ImVec4(0.55f, 0.12f, 0.12f, 1.0f));
    }
}

void drawSetup(DriverLink& link, const DriverState& s) {
    ImGui::SeparatorText("Tracker setup");
    const std::string mode = g.pendingStrings.resolve("mode", s.mode);
    struct ModeOption {
        const char* label;
        const char* value;
    };
    for (const ModeOption& opt : {ModeOption{"Ankles + waist (3 trackers)", "three"},
                                  ModeOption{"Ankles only (like Pico)", "ankles"}}) {
        if (ImGui::RadioButton(opt.label, mode == opt.value) && mode != opt.value) {
            g.pendingStrings.set("mode", opt.value);
            link.setMode(opt.value);
        }
    }

    ImGui::SeparatorText("User height");
    const int remoteCm = static_cast<int>(std::lround(s.userHeight * 100.0f));
    int cm = (g.activeSlider == "height")
                 ? static_cast<int>(g.activeSliderValue)
                 : static_cast<int>(std::lround(g.pendingNumbers.resolve("height", remoteCm)));
    const auto sendHeight = [&](int newCm) {
        newCm = std::clamp(newCm, kMinHeightCm, kMaxHeightCm);
        g.pendingNumbers.set("height", newCm);
        link.setHeight(static_cast<float>(newCm) / 100.0f);
    };

    const ImVec2 bigSquare(ImGui::GetFrameHeight() * 1.8f, ImGui::GetFrameHeight() * 1.8f);
    ImGui::PushFont(g.mediumFont);
    if (ImGui::Button("-", bigSquare)) sendHeight(cm - 1);
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::PushFont(g.largeFont);
    ImGui::Text("%d cm", cm);
    ImGui::PopFont();
    ImGui::SameLine();
    ImGui::PushFont(g.mediumFont);
    if (ImGui::Button("+", bigSquare)) sendHeight(cm + 1);
    ImGui::PopFont();

    ImGui::SetNextItemWidth(-FLT_MIN);
    int sliderCm = cm;
    ImGui::SliderInt("##height", &sliderCm, kMinHeightCm, kMaxHeightCm, "%d cm",
                     ImGuiSliderFlags_AlwaysClamp);
    if (ImGui::IsItemActive()) {
        g.activeSlider = "height";
        g.activeSliderValue = static_cast<float>(sliderCm);
    } else if (g.activeSlider == "height") {
        g.activeSlider.clear();
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) sendHeight(sliderCm);
    helpText("Your real standing height (barefoot). Used to size the body model.");

    if (s.measuredEyeHeight)
        ImGui::Text("Measured eye height (last calibration): %.0f cm", *s.measuredEyeHeight * 100.0f);
    else
        ImGui::TextColored(kGrey, "Measured eye height: not measured yet (run a calibration)");

    if (s.floorKnown)
        ImGui::TextColored(kGreen, "Floor height: known");
    else
        ImGui::TextColored(kOrange, "Floor height: unknown (set up the SteamVR play area)");
}

// "Label: 23:41 (38 min ago)", local time
void lastCalibrationLine(const char* label, const std::optional<double>& when) {
    if (!when) {
        ImGui::TextColored(kGrey, "%s: never", label);
        return;
    }
    std::time_t t = static_cast<std::time_t>(*when);
    std::tm local{};
    localtime_s(&local, &t);
    char clock[16];
    std::strftime(clock, sizeof(clock), "%H:%M", &local);
    double now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    int minutes = static_cast<int>(std::max(0.0, now - *when) / 60.0);
    char ago[32];
    if (minutes < 1)
        std::snprintf(ago, sizeof(ago), "just now");
    else if (minutes < 60)
        std::snprintf(ago, sizeof(ago), "%d min ago", minutes);
    else if (minutes < 48 * 60)
        std::snprintf(ago, sizeof(ago), "%dh%02d ago", minutes / 60, minutes % 60);
    else
        std::snprintf(ago, sizeof(ago), "%d days ago", minutes / (24 * 60));
    ImGui::TextColored(kGrey, "%s: %s (%s)", label, clock, ago);
}

void drawCalibration(DriverLink& link, const DriverState& s) {
    const CalibrationInfo& cal = s.calibration;
    const bool running = isCalibrationRunning(cal.phase);

    if (s.calibrated)
        ImGui::TextColored(kGreen, "Status: calibrated");
    else
        ImGui::TextColored(kOrange, "Status: not calibrated - run a full calibration");
    if (!s.floorKnown) ImGui::TextColored(kOrange, "Floor height unknown: heights may be off.");
    lastCalibrationLine("Last full calibration", cal.lastFull);
    lastCalibrationLine("Last quick reset", cal.lastReset);

    const float bigButtonH = ImGui::GetFrameHeight() * 2.2f;
    if (running) {
        ImGui::Spacing();
        centeredText(cal.phase == "stand" ? "Stand straight, look ahead" : "Ski pose", kAccent);
        ImGui::PushFont(g.largeFont);
        char countdown[16];
        std::snprintf(countdown, sizeof(countdown), "%d", static_cast<int>(std::ceil(std::max(0.0f, cal.countdown))));
        centeredText(countdown, ImVec4(1, 1, 1, 1));
        ImGui::PopFont();
        if (!cal.message.empty()) centeredText(cal.message.c_str(), ImVec4(1, 1, 1, 1));
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.15f, 0.15f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.70f, 0.20f, 0.20f, 1.0f));
        if (ImGui::Button("Cancel", ImVec2(-FLT_MIN, bigButtonH))) link.cancelCalibration();
        ImGui::PopStyleColor(2);
    } else {
        ImGui::Spacing();
        if (ImGui::Button("Full calibration (stand + ski pose)", ImVec2(-FLT_MIN, bigButtonH)))
            link.calibrate();
        helpText("Needed once after strapping the trackers: finds how each tracker is mounted.");
        ImGui::Spacing();
        if (ImGui::Button("Quick reset (stand)", ImVec2(-FLT_MIN, bigButtonH))) link.quickReset();
        helpText("Stand straight and look ahead: re-aligns the heading, keeps the mounting.");

        if (cal.phase == "done")
            ImGui::TextColored(kGreen, "%s", cal.message.empty() ? "Calibration done." : cal.message.c_str());
        else if (cal.phase == "error")
            ImGui::TextColored(kRed, "Calibration failed: %s", cal.message.c_str());
        else if (!cal.message.empty())
            ImGui::TextWrapped("%s", cal.message.c_str());
    }

    ImGui::Spacing();
    ImGui::SeparatorText("Ski pose");
    ImGui::TextWrapped("Ski pose: bend your knees and lean forward, arms down, look ahead");
}

void drawMagCell(DriverLink& link, const TrackerInfo& t) {
    const MagInfo& m = t.mag;
    if (m.state == "ok") {
        ImGui::TextColored(kGreen, "OK");
        ImGui::SameLine();
        ImGui::TextColored(m.residual < 0.08f ? kGrey : kOrange, "fit %.3f", m.residual);
    } else if (m.state == "disturbed") {
        ImGui::TextColored(kOrange, "Disturbed (gyro only)");
    } else if (m.state == "calibrating") {
        ImGui::TextColored(kAccent, "Calibrating...");
        ImGui::ProgressBar(std::clamp(m.progress, 0.0f, 1.0f), ImVec2(-FLT_MIN, 0));
    } else if (m.state == "uncalibrated") {
        ImGui::TextColored(kGrey, "Not calibrated");
    } else if (m.state == "disabled") {
        ImGui::TextColored(kGrey, "Disabled in settings");
    } else {
        ImGui::TextColored(kGrey, "No magnetometer");
    }

    const bool hasMag = m.state != "absent" && m.state != "disabled";
    if (m.state == "calibrating") {
        if (ImGui::Button("Cancel")) link.magCancel(t.mac);
    } else {
        ImGui::BeginDisabled(!hasMag || !t.connected);
        if (ImGui::Button("Calibrate mag")) link.magCalibrate(t.mac);
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!hasMag || m.state == "uncalibrated");
    if (ImGui::Button("Reset")) link.magReset(t.mac);
    ImGui::EndDisabled();
}

void drawTrackers(DriverLink& link, const DriverState& s, bool connected) {
    // USB dongle (ESP-NOW trackers)
    if (s.dongle.connected) {
        ImGui::TextColored(kGreen, "USB dongle on %s", s.dongle.port.c_str());
        ImGui::SameLine();
        ImGui::Text("- channel %d, %d tracker%s", s.dongle.channel, s.dongle.trackers, s.dongle.trackers == 1 ? "" : "s");
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s, firmware v%d", s.dongle.mac.c_str(), s.dongle.version);
    } else if (connected) {
        ImGui::TextColored(kGrey, "USB dongle: not detected (trackers use WiFi)");
    }
    if (s.trackers.empty()) {
        ImGui::TextColored(kGrey, connected ? "No tracker found yet. Turn the trackers on and check they "
                                              "are on the same network as this PC."
                                            : "Starting the engine...");
    } else if (ImGui::BeginTable("trackers", 4,
                                 ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH |
                                     ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX)) {
        ImGui::TableSetupColumn("Tracker", ImGuiTableColumnFlags_WidthStretch, 2.2f);
        ImGui::TableSetupColumn("Status", ImGuiTableColumnFlags_WidthStretch, 2.4f);
        ImGui::TableSetupColumn("Activity", ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableSetupColumn("Magnetometer", ImGuiTableColumnFlags_WidthStretch, 3.0f);
        ImGui::TableHeadersRow();

        struct RoleOption {
            const char* value;
            const char* label;
        };
        static constexpr RoleOption kRoles[] = {
            {"none", "None"}, {"waist", "Waist"}, {"left_ankle", "Left ankle"}, {"right_ankle", "Right ankle"}};

        for (const TrackerInfo& t : s.trackers) {
            ImGui::PushID(t.mac.c_str());
            ImGui::TableNextRow();

            // Identity + role
            ImGui::TableNextColumn();
            ImGui::Text("%s", shortMac(t.mac).c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\n%s", t.mac.c_str(), t.ip.c_str());
            const std::string roleKey = "role:" + t.mac;
            const std::string role = g.pendingStrings.resolve(roleKey, t.role);
            const char* roleLabel = role.c_str();
            for (const RoleOption& r : kRoles)
                if (role == r.value) roleLabel = r.label;
            ImGui::SetNextItemWidth(-FLT_MIN);
            if (ImGui::BeginCombo("##role", roleLabel)) {
                for (const RoleOption& r : kRoles) {
                    if (ImGui::Selectable(r.label, role == r.value) && role != r.value) {
                        g.pendingStrings.set(roleKey, r.value);
                        link.setRole(t.mac, r.value);
                    }
                }
                ImGui::EndCombo();
            }

            // Link status
            ImGui::TableNextColumn();
            if (t.connected)
                ImGui::TextColored(kGreen, t.transport == "espnow" ? "Connected (ESP-NOW)" : "Connected (WiFi)");
            else
                ImGui::TextColored(kRed, "Disconnected");
            ImGui::Text("%.0f Hz / mag %.0f Hz", t.rotationHz, t.magHz);
            if (t.battery >= 0.0f) {
                ImGui::TextColored(t.battery < 0.15f ? kOrange : ImVec4(1, 1, 1, 1), "Battery %.0f%%",
                                   t.battery * 100.0f);
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("%.2f V", t.batteryVoltage);
                ImGui::SameLine();
                const BatteryEstimator::Estimate e = g.battery.estimate(t.mac);
                if (e.state == BatteryEstimator::State::Charging)
                    ImGui::TextColored(kGreen, "charging");
                else if (e.state == BatteryEstimator::State::Estimating)
                    ImGui::TextColored(kGrey, "~...");
                else if (e.hours >= 24)
                    ImGui::TextColored(kGrey, ">24h");
                else {
                    const int minutes = int(e.hours * 60 + 0.5);
                    ImGui::TextColored(e.hours < 0.5 ? kOrange : kGrey, "~%dh%02d", minutes / 60, minutes % 60);
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Remaining time, from the discharge rate of the last 40 minutes\n"
                                      "(first estimate after 8 minutes). Restarts after a charge, when\n"
                                      "the tracker was off and when it switches between WiFi and the\n"
                                      "USB dongle.");
            } else
                ImGui::TextColored(kGrey, "Battery --");
            if (t.rssi < 0)
                ImGui::Text("%d dBm", t.rssi);
            else
                ImGui::TextColored(kGrey, "-- dBm");
            ImGui::SameLine();
            if (t.connected && t.latencyMs >= 0)
                ImGui::TextColored(t.latencyMs > 50 ? kOrange : ImVec4(1, 1, 1, 1), " %.0f ms", t.latencyMs);
            else
                ImGui::TextColored(kGrey, " -- ms");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Round trip PC -> tracker -> PC (median of the last 5 pings)");
            ImGui::SameLine();
            ImGui::Text(" %.1f \xC2\xB0""C", t.temperature);

            // Activity + foot plant
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, kAccent);
            ImGui::ProgressBar(std::clamp(t.activity, 0.0f, 1.0f), ImVec2(-FLT_MIN, 0), "");
            ImGui::PopStyleColor();
            if (t.planted)
                ImGui::TextColored(kGreen, "Planted");
            else
                ImGui::TextColored(kGrey, "Moving");

            // Magnetometer
            ImGui::TableNextColumn();
            drawMagCell(link, t);

            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    ImGui::Spacing();
    helpText("Shake a tracker to find it: its activity bar fills up.");
    helpText("Mag calibration: before strapping the tracker, slowly rotate it in every direction "
             "(figure eights, upside down, on each side) until the bar is full.");
}

void drawSettings(DriverLink& link, const DriverState& s) {
    const DriverSettings& st = s.settings;

    ImGui::SeparatorText("Magnetometer");
    settingCheckbox(link, "Use magnetometer (prevents heading drift)", "magEnabled", st.magEnabled);
    ImGui::Text("Anti-mag sensitivity");
    settingSlider(link, "##magSensitivity", "magSensitivity", st.magSensitivity);
    helpText("Higher rejects magnetic disturbances (metal, speakers) more aggressively.");

    ImGui::SeparatorText("Motion");
    ImGui::Text("Smoothing");
    settingSlider(link, "##smoothing", "smoothing", st.smoothing);
    ImGui::Text("Foot plant strength");
    settingSlider(link, "##footPlantStrength", "footPlantStrength", st.footPlantStrength);
    ImGui::Text("Inertial weight");
    settingSlider(link, "##inertialWeight", "inertialWeight", st.inertialWeight);
    settingCheckbox(link, "Velocity prediction", "velocityPrediction", st.velocityPrediction);

    ImGui::SeparatorText("Recording");
    settingCheckbox(link, "Record sessions (trackers, headset, controllers)", "recording", st.recording);
    if (s.recording.active)
        ImGui::TextColored(kGreen, "Recording: %.0f MB", s.recording.megabytes);
    else
        ImGui::TextColored(kGrey, "Not recording");
    if (!s.recording.file.empty()) helpText(s.recording.file.c_str());
    helpText("Used to replay sessions offline and tune the tracking. About 400 MB per hour.");
}

void drawFooter(const DriverState& s, bool connected, const FrameInfo& info) {
    ImGui::Separator();
    ImGui::Text("Outputs:");
    const auto output = [&](const char* name, bool on) {
        ImGui::SameLine();
        ImGui::TextColored(connected && on ? kGreen : kGrey, "%s %s", name, connected && on ? "ON" : "off");
    };
    output("Waist", s.outputs.waist);
    output(" Left foot", s.outputs.leftFoot);
    output(" Right foot", s.outputs.rightFoot);

    char right[96];
    std::snprintf(right, sizeof(right), "SteamVR: %s", info.vrActive ? "connected" : "not running");
    ImGui::SameLine(ImGui::GetContentRegionMax().x - ImGui::CalcTextSize(right).x);
    ImGui::TextColored(info.vrActive ? kGreen : kGrey, "%s", right);
}

std::string defaultFontPath(const char* file) {
    char windir[MAX_PATH] = {};
    const UINT n = GetWindowsDirectoryA(windir, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    return std::string(windir) + "\\Fonts\\" + file;
}

bool fileExists(const std::string& path) {
    const DWORD attr = GetFileAttributesA(path.c_str());
    return !path.empty() && attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

}  // namespace

void setup() {
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // no imgui.ini next to the driver binaries
    io.LogFilename = nullptr;

    if (const char* appData = std::getenv("LOCALAPPDATA"))
        g.battery.load(std::string(appData) + "\\Slime Motion Tracker\\battery_history.json");

    // Large fonts so the UI is readable in the headset. Segoe UI ships with Windows;
    // fall back to the (scaled) built-in font otherwise.
    const std::string regular = defaultFontPath("segoeui.ttf");
    const std::string bold = defaultFontPath("segoeuib.ttf");
    if (fileExists(regular)) {
        g.bodyFont = io.Fonts->AddFontFromFileTTF(regular.c_str(), 26.0f);
        const char* boldPath = fileExists(bold) ? bold.c_str() : regular.c_str();
        g.mediumFont = io.Fonts->AddFontFromFileTTF(boldPath, 44.0f);
        g.largeFont = io.Fonts->AddFontFromFileTTF(boldPath, 72.0f);
    }
    if (!g.bodyFont || !g.mediumFont || !g.largeFont) {
        io.Fonts->Clear();
        ImFontConfig cfg;
        cfg.SizePixels = 26.0f;
        g.bodyFont = io.Fonts->AddFontDefault(&cfg);
        cfg.SizePixels = 39.0f;
        g.mediumFont = io.Fonts->AddFontDefault(&cfg);
        cfg.SizePixels = 65.0f;
        g.largeFont = io.Fonts->AddFontDefault(&cfg);
    }

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.WindowBorderSize = 0.0f;
    style.WindowPadding = ImVec2(16, 12);
    style.FramePadding = ImVec2(14, 8);
    style.ItemSpacing = ImVec2(12, 10);
    style.ItemInnerSpacing = ImVec2(10, 8);
    style.CellPadding = ImVec2(8, 6);
    style.FrameRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.ScrollbarSize = 26.0f;
    style.GrabMinSize = 28.0f;
    style.SeparatorTextBorderSize = 2.0f;

    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.09f, 0.09f, 0.12f, 1.0f);  // opaque: the VR overlay must not be see-through
    c[ImGuiCol_Button] = ImVec4(0.30f, 0.26f, 0.62f, 1.0f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.40f, 0.35f, 0.80f, 1.0f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.48f, 0.42f, 0.95f, 1.0f);
    c[ImGuiCol_Tab] = ImVec4(0.18f, 0.16f, 0.32f, 1.0f);
    c[ImGuiCol_TabHovered] = ImVec4(0.40f, 0.35f, 0.80f, 1.0f);
    c[ImGuiCol_TabSelected] = ImVec4(0.32f, 0.28f, 0.66f, 1.0f);
    c[ImGuiCol_CheckMark] = kAccent;
    c[ImGuiCol_SliderGrab] = kAccent;
    c[ImGuiCol_SliderGrabActive] = ImVec4(0.60f, 0.55f, 1.0f, 1.0f);
    c[ImGuiCol_FrameBg] = ImVec4(0.17f, 0.17f, 0.23f, 1.0f);
    c[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.23f, 0.34f, 1.0f);
    c[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.28f, 0.44f, 1.0f);
    c[ImGuiCol_Header] = ImVec4(0.30f, 0.26f, 0.62f, 0.6f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.40f, 0.35f, 0.80f, 0.8f);
}

void draw(DriverLink& link, const FrameInfo& info) {
    const bool connected = link.connected();
    // While the engine switches between the driver and the standalone host, the last
    // state stays on screen (disabled); defaults before the first one.
    static const DriverState kPlaceholder{};
    const DriverState& s = link.hasState() ? link.state() : kPlaceholder;
    if (!connected) {
        g.pendingNumbers.clear();
        g.pendingStrings.clear();
    }

    if (connected) {
        const double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
        for (const TrackerInfo& t : s.trackers)
            if (t.connected && t.battery >= 0.0f) g.battery.add(t.mac, t.transport, t.battery, wall);
    }
    if (Clock::now() - g.lastBatterySave > std::chrono::seconds(60)) {
        g.lastBatterySave = Clock::now();
        g.battery.save();
    }

    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::Begin("Slime Motion Tracker", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus);

    drawHeader(link, s, connected);

    // Jump to the calibration tab when a calibration starts (it may be triggered elsewhere).
    const bool calibrationStarted = isCalibrationRunning(s.calibration.phase) && !isCalibrationRunning(g.prevPhase);
    g.prevPhase = s.calibration.phase;

    const float footerHeight = ImGui::GetFrameHeightWithSpacing() + ImGui::GetStyle().ItemSpacing.y;
    if (ImGui::BeginTabBar("tabs")) {
        const auto tab = [&](const char* name, bool select, auto&& body) {
            if (ImGui::BeginTabItem(name, nullptr, select ? ImGuiTabItemFlags_SetSelected : 0)) {
                ImGui::BeginChild("##content", ImVec2(0, -footerHeight));
                ImGui::BeginDisabled(!connected);
                body();
                ImGui::EndDisabled();
                ImGui::EndChild();
                ImGui::EndTabItem();
            }
        };
        tab("Setup", false, [&] { drawSetup(link, s); });
        tab("Calibration", calibrationStarted, [&] { drawCalibration(link, s); });
        tab("Trackers", false, [&] { drawTrackers(link, s, connected); });
        tab("Settings", false, [&] { drawSettings(link, s); });
        ImGui::EndTabBar();
    }

    drawFooter(s, connected, info);

    ImGui::End();
}

}  // namespace smt::ui
