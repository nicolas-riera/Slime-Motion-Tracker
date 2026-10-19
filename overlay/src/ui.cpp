#include "ui.h"

#include <windows.h>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "app_prefs.h"
#include "imgui.h"
#include "sounds.h"
#include "ui_internal.h"

namespace smt::ui {

UiState g;

namespace {

constexpr int kMinHeightCm = 120;
constexpr int kMaxHeightCm = 220;
constexpr float kRowHeight = 44.0f;

bool isRunning(const std::string& phase) { return phase == "stand" || phase == "ski"; }

// ---------------------------------------------------------------- generic pieces

// Full-width colored banner: icon, bold title and wrapped text
void banner(IconFn iconFn, ImU32 tint, ImU32 bg, const char* title, const char* body) {
    const float width = ImGui::GetContentRegionAvail().x;
    const float pad = 14.0f, iconBox = 34.0f;
    const float textX = pad + iconBox + 12.0f;
    const float wrap = width - textX - pad;
    const ImVec2 titleSize = textSize(fonts.bold, title);
    const ImVec2 bodySize =
        body && *body ? fonts.small->CalcTextSizeA(fonts.small->FontSize, FLT_MAX, wrap, body) : ImVec2(0, 0);
    const float contentH = titleSize.y + (bodySize.y > 0 ? bodySize.y + 2.0f : 0.0f);
    const float h = std::max(iconBox, contentH) + 2 * pad;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(width, h));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + width, p.y + h), sty(bg), 12.0f);
    dl->AddRect(p, ImVec2(p.x + width, p.y + h), sty(alpha(tint, 0.45f)), 12.0f, 0, 1.2f);
    const ImVec2 ic(p.x + pad + iconBox * 0.5f, p.y + h * 0.5f);
    dl->AddCircleFilled(ic, iconBox * 0.5f, sty(alpha(tint, 0.18f)), 24);
    iconFn(dl, ic, 20.0f, tint);
    const float ty = p.y + (h - contentH) * 0.5f;
    textAt(dl, fonts.bold, ImVec2(p.x + textX, ty), col::text, title);
    if (bodySize.y > 0)
        dl->AddText(fonts.small, fonts.small->FontSize, ImVec2(p.x + textX, ty + titleSize.y + 2.0f), sty(col::textDim),
                    body, nullptr, wrap);
}

// Slider sending its value only when released
bool sliderOnRelease(const char* id, const std::string& key, float remote, float vMin, float vMax, float& sent) {
    float v = (g.activeSlider == key) ? g.activeSliderValue : static_cast<float>(g.pendingNumbers.resolve(key, remote));
    bool released = false;
    const bool changed = slider(id, &v, vMin, vMax, &released);
    if (ImGui::IsItemActive()) {
        g.activeSlider = key;
        g.activeSliderValue = v;
        if (changed) g.sliderEdited = true;
    } else if (g.activeSlider == key) {
        g.activeSlider.clear();
    }
    if (released) {
        const bool edited = g.sliderEdited;
        g.sliderEdited = false;
        if (edited) {
            g.pendingNumbers.set(key, v);
            sent = v;
            return true;
        }
    }
    return false;
}

// Label (+ value on the right), slider, help
void settingSlider(DriverLink& link, const char* label, const char* key, float remote, const char* help,
                   const char* format = "%.2f") {
    ImGui::PushID(key);
    const float shown = g.activeSlider == key ? g.activeSliderValue
                                              : static_cast<float>(g.pendingNumbers.resolve(key, remote));
    const float avail = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    char value[32];
    std::snprintf(value, sizeof(value), format, shown);
    text(fonts.bold, col::text, "%s", label);
    const ImVec2 vs = textSize(fonts.body, value);
    textAt(ImGui::GetWindowDrawList(), fonts.body, ImVec2(p.x + avail - vs.x, p.y + 1), col::accentSoft, value);
    float sent = 0.0f;
    if (sliderOnRelease("##slider", key, remote, 0.0f, 1.0f, sent)) link.setSetting(key, sent);
    if (help) helpText(help);
    ImGui::PopID();
}

void settingToggle(DriverLink& link, const char* label, const char* help, const char* key, bool remote) {
    bool v = g.pendingNumbers.resolve(key, remote ? 1.0 : 0.0) > 0.5;
    if (toggleRow(label, help, &v)) {
        g.pendingNumbers.set(key, v ? 1.0 : 0.0);
        link.setSetting(key, v);
    }
}

void pageTitle(const char* title, const char* subtitle) {
    text(fonts.title, col::text, "%s", title);
    if (subtitle) {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 8.0f);
        text(fonts.small, col::textDim, "%s", subtitle);
    }
    ImGui::Dummy(ImVec2(0, 4));
}

// ---------------------------------------------------------------- sidebar

void drawSidebar(const DriverState& s, const FrameInfo& info, float width, float height) {
    // Compact rail: logo, icon + short label entries, engine status at the bottom
    ImGui::PushStyleColor(ImGuiCol_ChildBg, col::sidebar);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 16));
    ImGui::BeginChild("##sidebar", ImVec2(width, height), ImGuiChildFlags_AlwaysUseWindowPadding,
                      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    const bool collapsed = width < 76.0f;
    const float inner = ImGui::GetContentRegionAvail().x;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float logo = 38.0f;
    icon::logo(dl, ImVec2(p.x + (inner - logo) * 0.5f, p.y), logo);
    ImGui::Dummy(ImVec2(inner, logo));
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Slime Motion Tracker");
    ImGui::Dummy(ImVec2(0, 10));

    struct Item {
        const char* label;
        IconFn icon;
        UiState::Page page;
    };
    const Item items[] = {{"Home", icon::home, UiState::Page::Home},
                          {"Body", icon::body, UiState::Page::Body},
                          {"Settings", icon::gear, UiState::Page::Settings}};
    for (const Item& it : items) {
        if (navItem(it.label, it.icon, g.page == it.page, collapsed, inner)) {
            if (g.page == it.page && it.page == UiState::Page::Home) g.trackerPage.clear();  // back to the list
            g.page = it.page;
        }
        ImGui::Dummy(ImVec2(0, -4));
    }

    // Engine status at the bottom: a dot and a short word, details in the tooltip
    ImU32 dot = col::orange;
    const char* label = "Starting";
    const char* tip = "The engine is starting or switching between SteamVR and the standalone host.";
    if (g.connected && info.standalone) {
        dot = col::cyan;
        label = "Local";
        tip = "Standalone: SteamVR is not running, the app runs the tracking engine itself so the trackers can be "
              "checked without a headset.";
    } else if (g.connected) {
        dot = col::green;
        label = "SteamVR";
        tip = "The tracking engine runs in the SteamVR driver.";
    }
    const float blockH = collapsed ? 30.0f : 46.0f;
    ImGui::SetCursorPosY(std::max(ImGui::GetCursorPosY(), ImGui::GetWindowHeight() - 16.0f - blockH));
    const ImVec2 b = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(inner, blockH));
    if (ImGui::IsItemHovered()) {
        if (!s.driverVersion.empty())
            ImGui::SetTooltip("%s\nEngine v%s%s", tip, s.driverVersion.c_str(),
                              s.version != 1 ? " (unexpected protocol version)" : "");
        else
            ImGui::SetTooltip("%s", tip);
    }
    const ImVec2 dc(b.x + inner * 0.5f, b.y + 12);
    dl->AddCircleFilled(dc, 5.0f, dot, 16);
    dl->AddCircle(dc, 9.0f, alpha(dot, 0.30f), 20, 2.0f);
    if (!collapsed) {
        const ImVec2 ts = textSize(fonts.small, label);
        textAt(dl, fonts.small, ImVec2(b.x + (inner - ts.x) * 0.5f, b.y + 26), col::textDim, label);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

// ---------------------------------------------------------------- home: calibration

float displayedCountdown(const CalibrationInfo& cal) {
    const double now = steadyNow();
    if (cal.phase != g.cdPhase) {
        g.cdPhase = cal.phase;
        g.cdDuration = std::max(3.0f, std::ceil(cal.countdown - 1e-3f));
        g.cdValue = cal.countdown;
        g.cdAt = now;
    } else if (std::fabs(cal.countdown - g.cdValue) > 1e-4f) {
        g.cdValue = cal.countdown;
        g.cdAt = now;
    }
    return std::max(0.0f, g.cdValue - static_cast<float>(now - g.cdAt));
}

bool calibrationButton(const char* id, IconFn iconFn, const char* title, const std::string& sub, ImVec2 size,
                       bool running, float progress, bool split, const char* runTitle, float countdown) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 q(p.x + size.x, p.y + size.y);
    const bool clicked = ImGui::InvisibleButton(id, size);
    const bool hovered = ImGui::IsItemHovered();
    const float hov = animate(ImGui::GetItemID(), hovered ? 1.0f : 0.0f, 16.0f);
    const float run = animate(ImGui::GetItemID() + 1, running ? 1.0f : 0.0f, 10.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float rounding = 14.0f;
    dl->AddRectFilled(p, q, sty(mix(col::card, col::cardHover, hov)), rounding);
    if (running) {
        // The countdown fills the button
        const float w = size.x * std::clamp(progress, 0.0f, 1.0f);
        dl->PushClipRect(p, ImVec2(p.x + w, q.y), true);
        dl->AddRectFilled(p, q, sty(col::accent), rounding);
        dl->AddRectFilled(ImVec2(p.x + w - 3.0f, p.y), ImVec2(p.x + w, q.y), sty(alpha(col::white, 0.25f)));
        dl->PopClipRect();
        if (split)
            dl->AddLine(ImVec2(p.x + size.x * 0.5f, p.y + 12), ImVec2(p.x + size.x * 0.5f, q.y - 12),
                        sty(alpha(col::white, 0.30f)), 1.5f);
    }
    const ImU32 border = running ? alpha(col::accentSoft, 0.9f) : alpha(col::accentSoft, 0.55f * hov);
    if (border & IM_COL32_A_MASK) dl->AddRect(p, q, sty(border), rounding, 0, 1.5f);

    const float box = 46.0f;
    const ImVec2 ic(p.x + 18 + box * 0.5f, p.y + size.y * 0.5f);
    dl->AddRectFilled(ImVec2(ic.x - box * 0.5f, ic.y - box * 0.5f), ImVec2(ic.x + box * 0.5f, ic.y + box * 0.5f),
                      sty(mix(col::accentDeep, alpha(col::white, 0.16f), run)), 12.0f);
    iconFn(dl, ic, 26.0f, mix(col::accentSoft, col::white, run));
    const float tx = p.x + 18 + box + 14;
    const char* t1 = running ? runTitle : title;
    const char* t2 = running ? "Click to cancel" : sub.c_str();
    float right = q.x - 14;
    if (running) {
        char digits[8];
        std::snprintf(digits, sizeof(digits), "%d", static_cast<int>(std::ceil(countdown - 1e-3f)));
        const ImVec2 ds = textSize(fonts.big, digits);
        textAt(dl, fonts.big, ImVec2(q.x - 20 - ds.x, p.y + (size.y - ds.y) * 0.5f - 1), col::white, digits);
        right = q.x - 30 - ds.x;
    }
    dl->PushClipRect(ImVec2(tx, p.y), ImVec2(right, q.y), true);
    const float ty = p.y + size.y * 0.5f - fonts.bold->FontSize;
    textAt(dl, fonts.bold, ImVec2(tx, ty), running ? col::white : col::text, t1);
    textAt(dl, fonts.small, ImVec2(tx, ty + fonts.bold->FontSize + 3), running ? alpha(col::white, 0.8f) : col::textDim,
           t2);
    dl->PopClipRect();
    return clicked;
}

void drawCalibration(DriverLink& link, const DriverState& s) {
    const CalibrationInfo& cal = s.calibration;
    const bool running = isRunning(cal.phase);
    const float countdown = displayedCountdown(cal);
    const float frac = running ? 1.0f - countdown / std::max(0.1f, g.cdDuration) : 0.0f;

    const float avail = ImGui::GetContentRegionAvail().x;
    const float gap = 12.0f;
    const bool stacked = avail < 420.0f;
    const ImVec2 size(stacked ? avail : (avail - gap) * 0.5f, 84.0f);

    // Full calibration
    {
        const bool mine = running && !cal.quick;
        std::string sub = cal.lastFull ? "Last: " + agoText(cal.lastFull) : std::string("Never done");
        if (!s.calibrated && !mine) sub = "Required once after strapping";
        const float progress = cal.phase == "stand" ? 0.5f * frac : 0.5f + 0.5f * frac;
        ImGui::BeginDisabled(running && !mine);
        if (calibrationButton("##full", icon::calibrate, "Full calibration", sub, size, mine, progress, true,
                              cal.phase == "stand" ? "Stand straight" : "Ski pose", countdown)) {
            if (mine) {
                link.cancelCalibration();
            } else {
                link.calibrate();
                g.resultDismissed = true;
            }
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered() && !running)
            ImGui::SetTooltip("Stand straight, then ski pose (3 s each).\nFinds how each tracker is mounted.");
    }
    if (!stacked) ImGui::SameLine(0, gap);
    // Quick reset
    {
        const bool mine = running && cal.quick;
        std::string sub = cal.lastReset ? "Last: " + agoText(cal.lastReset) : std::string("Never done");
        ImGui::BeginDisabled(running && !mine);
        if (calibrationButton("##quick", icon::reset, "Quick reset", sub, size, mine, frac, false, "Stand straight",
                              countdown)) {
            if (mine) {
                link.cancelCalibration();
            } else {
                link.quickReset();
                g.resultDismissed = true;
            }
        }
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered() && !running)
            ImGui::SetTooltip("Stand straight and look ahead (3 s).\nRe-aligns the heading, keeps the mounting.");
    }

    // Instructions while running, then the result
    if (running) {
        banner(icon::info, col::accentSoft, alpha(col::accent, 0.16f), cal.phase == "stand" ? "Stand straight" : "Ski pose",
               cal.message.empty() ? nullptr : cal.message.c_str());
    } else if (!g.resultDismissed) {
        const bool ok = g.resultPhase == "done";
        if (ok && steadyNow() - g.resultAt > 6.0) {
            g.resultDismissed = true;
        } else {
            const ImVec2 before = ImGui::GetCursorScreenPos();
            if (ok)
                banner(icon::check, col::green, alpha(col::green, 0.10f),
                       g.resultMessage.empty() ? "Calibration done" : g.resultMessage.c_str(), nullptr);
            else
                banner(icon::warning, col::red, alpha(col::red, 0.10f), "Calibration failed", g.resultMessage.c_str());
            // Click to dismiss
            const ImVec2 after = ImGui::GetItemRectMax();
            if (ImGui::IsMouseHoveringRect(before, after) && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
                g.resultDismissed = true;
        }
    }
}

// ---------------------------------------------------------------- home: trackers

void drawTrackerTable(const DriverState& s) {
    const float width = ImGui::GetContentRegionAvail().x;
    int online = 0;
    for (const TrackerInfo& t : s.trackers) online += t.connected ? 1 : 0;
    {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        text(fonts.bold, col::text, "Trackers");
        char count[48];
        std::snprintf(count, sizeof(count), "%d / %d online", online, static_cast<int>(s.trackers.size()));
        const ImVec2 cs = textSize(fonts.small, count);
        textAt(ImGui::GetWindowDrawList(), fonts.small, ImVec2(p.x + width - cs.x, p.y + 3), col::textDim, count);
    }
    if (s.trackers.empty()) {
        banner(icon::info, col::textDim, col::card, g.connected ? "No tracker found yet" : "Waiting for the engine",
               g.connected ? "Turn the trackers on. They must be on the same WiFi network as this PC."
                           : "The tracking engine is starting.");
        return;
    }

    // Columns
    const float x0 = 16.0f;
    const float cName = x0 + 18.0f;
    const float cConn = width * 0.38f;
    const float cBat = width * 0.58f;
    const float cMag = width * 0.79f;
    {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImU32 c = col::textFaint;
        textAt(dl, fonts.smallBold, ImVec2(p.x + cName, p.y), c, "POSITION");
        textAt(dl, fonts.smallBold, ImVec2(p.x + cConn, p.y), c, "SIGNAL");
        textAt(dl, fonts.smallBold, ImVec2(p.x + cBat, p.y), c, "BATTERY");
        textAt(dl, fonts.smallBold, ImVec2(p.x + cMag, p.y), c, "MAG");
        ImGui::Dummy(ImVec2(width, fonts.smallBold->FontSize));
    }
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(10, 6));
    for (const TrackerInfo& t : s.trackers) {
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const ImVec2 q(p.x + width, p.y + kRowHeight);
        ImGui::PushID(t.mac.c_str());
        const bool clicked = ImGui::InvisibleButton("##row", ImVec2(width, kRowHeight));
        const bool hovered = ImGui::IsItemHovered();
        const float hov = animate(ImGui::GetItemID(), hovered ? 1.0f : 0.0f, 16.0f);
        ImGui::PopID();
        if (hovered) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            ImGui::SetTooltip("%s\n%s", t.mac.c_str(), t.ip.c_str());
        }
        if (clicked) g.trackerPage = t.mac;

        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p, q, sty(mix(col::card, col::cardHover, hov)), 10.0f);
        drawGlow(dl, p, q, g.glow[t.mac], 10.0f);

        const float cy = p.y + kRowHeight * 0.5f;
        dl->AddCircleFilled(ImVec2(p.x + x0 + 3, cy), 4.5f, sty(t.connected ? col::green : col::textFaint), 12);
        const std::string name = trackerName(t);
        const bool assigned = resolvedRole(t) != "none";
        const ImU32 nameCol = !t.connected ? col::textDim : (assigned ? col::text : col::textDim);
        dl->PushClipRect(ImVec2(p.x, p.y), ImVec2(p.x + cConn - 8, q.y), true);
        textAt(dl, fonts.bold, ImVec2(p.x + cName, cy - fonts.bold->FontSize * 0.5f - 1), nameCol, name.c_str());
        dl->PopClipRect();

        drawConnection(dl, ImVec2(p.x + cConn, p.y), kRowHeight, t);
        drawBattery(dl, ImVec2(p.x + cBat, p.y), kRowHeight, t);
        dl->PushClipRect(ImVec2(p.x + cMag, p.y), ImVec2(q.x - 26, q.y), true);
        drawMag(dl, ImVec2(p.x + cMag, p.y), kRowHeight, t);
        dl->PopClipRect();
        icon::chevronRight(dl, ImVec2(q.x - 16, cy), 16.0f, mix(col::textFaint, col::text, hov));
    }
    ImGui::PopStyleVar();
    ImGui::Dummy(ImVec2(0, 2));
    text(fonts.small, col::textFaint, "Shake a tracker to find it. Click a tracker for its details.");
}

void drawHomeMain(DriverLink& link, const DriverState& s) {
    drawCalibration(link, s);
    if (!s.portError.empty()) banner(icon::warning, col::red, alpha(col::red, 0.10f), "Tracker port", s.portError.c_str());
    ImGui::Dummy(ImVec2(0, 2));
    drawTrackerTable(s);
}

void drawHome(DriverLink& link, const DriverState& s) {
    if (!g.trackerPage.empty()) {
        if (const TrackerInfo* t = findTracker(s, g.trackerPage)) {
            drawTrackerPage(link, s, *t);
            return;
        }
        g.trackerPage.clear();
    }
    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const bool wide = avail.x >= 700.0f;
    if (wide) {
        const float gap = 18.0f;
        const float skelW = std::clamp(avail.x * 0.28f, 240.0f, 380.0f);
        ImGui::BeginChild("##homeLeft", ImVec2(avail.x - skelW - gap, avail.y), 0, ImGuiWindowFlags_NoBackground);
        drawHomeMain(link, s);
        ImGui::EndChild();
        ImGui::SameLine(0, gap);
        ImGui::BeginChild("##homeRight", ImVec2(skelW, avail.y), 0,
                          ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawSkeletonCard(link, s, avail.y);
        ImGui::EndChild();
    } else {
        ImGui::BeginChild("##homeScroll", avail, 0, ImGuiWindowFlags_NoBackground);
        drawHomeMain(link, s);
        ImGui::Dummy(ImVec2(0, 4));
        drawSkeletonCard(link, s, std::clamp(avail.y * 0.8f, 320.0f, 520.0f));
        ImGui::EndChild();
    }
}

// ---------------------------------------------------------------- body

void drawBody(DriverLink& link, const DriverState& s) {
    pageTitle("Body", "Your height and which tracker is where");

    beginCard("##height");
    sectionTitle("HEIGHT");
    const int remoteCm = static_cast<int>(std::lround(s.userHeight * 100.0f));
    int cm = (g.activeSlider == "height") ? static_cast<int>(g.activeSliderValue)
                                          : static_cast<int>(std::lround(g.pendingNumbers.resolve("height", remoteCm)));
    const auto sendHeight = [&](int newCm) {
        newCm = std::clamp(newCm, kMinHeightCm, kMaxHeightCm);
        g.pendingNumbers.set("height", newCm);
        link.setHeight(static_cast<float>(newCm) / 100.0f);
    };
    {
        const ImVec2 bs(44, 44);
        if (button("\xE2\x88\x92##minus", bs)) sendHeight(cm - 1);
        ImGui::SameLine(0, 0);
        char value[16];
        std::snprintf(value, sizeof(value), "%d cm", cm);
        const ImVec2 vs = textSize(fonts.big, value);
        const float mid = 150.0f;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImGui::Dummy(ImVec2(mid, bs.y));
        textAt(ImGui::GetWindowDrawList(), fonts.big, ImVec2(p.x + (mid - vs.x) * 0.5f, p.y + (bs.y - vs.y) * 0.5f - 2),
               col::text, value);
        ImGui::SameLine(0, 0);
        if (button("+##plus", bs)) sendHeight(cm + 1);
    }
    {
        float sliderCm = static_cast<float>(cm);
        bool released = false;
        slider("##height", &sliderCm, static_cast<float>(kMinHeightCm), static_cast<float>(kMaxHeightCm), &released);
        sliderCm = std::round(sliderCm);
        if (ImGui::IsItemActive()) {
            g.activeSlider = "height";
            g.activeSliderValue = sliderCm;
        } else if (g.activeSlider == "height") {
            g.activeSlider.clear();
        }
        if (released && static_cast<int>(sliderCm) != remoteCm) sendHeight(static_cast<int>(sliderCm));
    }
    helpText("Your real standing height, barefoot. Sizes the body model.");
    separator(12.0f);
    if (s.measuredEyeHeight)
        infoRow("Eye height (last calibration)", col::text, "%.0f cm", *s.measuredEyeHeight * 100.0f);
    else
        infoRow("Eye height", col::textDim, "Measured at the next calibration");
    infoRow("Floor height", s.floorKnown ? col::green : col::orange, "%s",
            s.floorKnown ? "Known" : "Unknown (set up the play area)");
    infoRow("Tracking mode", col::text, "%s", s.mode == "ankles" ? "Ankles only (waist estimated)" : "Ankles + waist");
    endCard();
    ImGui::Dummy(ImVec2(0, 4));

    beginCard("##mapping");
    sectionTitle("TRACKER ASSIGNMENT");
    helpText("Shake a tracker: its row lights up. The mode (ankles only, or ankles + waist) follows the trackers "
             "assigned and connected.");
    ImGui::Dummy(ImVec2(0, 4));
    // The trackers heard since the start, then the assigned ones not heard (offline)
    std::vector<const TrackerInfo*> rows;
    for (const TrackerInfo& t : s.trackers) rows.push_back(&t);
    for (const TrackerInfo& t : s.mappedOffline) rows.push_back(&t);
    if (rows.empty()) text(fonts.body, col::textDim, "No tracker yet.");
    for (const TrackerInfo* tp : rows) {
        const TrackerInfo& t = *tp;
        const float width = ImGui::GetContentRegionAvail().x;
        const float h = 50.0f;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const ImVec2 q(p.x + width, p.y + h);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p, q, sty(IM_COL32(32, 32, 43, 255)), 10.0f);
        drawGlow(dl, p, q, g.glow[t.mac], 10.0f);
        const float cy = p.y + h * 0.5f;
        dl->AddCircleFilled(ImVec2(p.x + 16, cy), 4.5f, sty(t.connected ? col::green : col::textFaint), 12);
        wifiIcon(dl, ImVec2(p.x + 40, cy), 18.0f, t, col::textDim);
        textAt(dl, fonts.body, ImVec2(p.x + 62, cy - fonts.body->FontSize * 0.5f - 1),
               t.connected ? col::text : col::textDim, t.mac.c_str());
        const float comboW = std::min(180.0f, width * 0.42f);
        ImGui::SetCursorScreenPos(ImVec2(q.x - comboW - 8, cy - ImGui::GetFrameHeight() * 0.5f));
        ImGui::PushID(t.mac.c_str());
        roleSelector(link, t, comboW);
        ImGui::PopID();
        ImGui::SetCursorScreenPos(ImVec2(p.x, q.y + 6));
        ImGui::Dummy(ImVec2(0, 0));
    }
    endCard();
}

// ---------------------------------------------------------------- settings

void drawSettings(DriverLink& link, const DriverState& s) {
    pageTitle("Settings", nullptr);
    const DriverSettings& st = s.settings;
    AppPrefs& prefs = appPrefs();

    beginCard("##mag");
    sectionTitle("MAGNETOMETER");
    settingToggle(link, "Use the magnetometer", "Keeps the heading from drifting (needs a mag calibration per tracker).",
                  "magEnabled", st.magEnabled);
    ImGui::Dummy(ImVec2(0, 4));
    settingSlider(link, "Anti-disturbance sensitivity", "magSensitivity", st.magSensitivity,
                  "Higher rejects magnetic disturbances (metal furniture, speakers) more aggressively.");
    endCard();
    ImGui::Dummy(ImVec2(0, 4));

    beginCard("##motion");
    sectionTitle("MOTION");
    settingSlider(link, "Smoothing", "smoothing", st.smoothing, nullptr);
    ImGui::Dummy(ImVec2(0, 2));
    settingSlider(link, "Foot plant strength", "footPlantStrength", st.footPlantStrength,
                  "How firmly a resting foot stays planted.");
    ImGui::Dummy(ImVec2(0, 2));
    settingSlider(link, "Inertial weight", "inertialWeight", st.inertialWeight,
                  "Weight of the accelerometers in the foot motion.");
    ImGui::Dummy(ImVec2(0, 4));
    settingToggle(link, "Velocity prediction", "Sends the tracker velocities to SteamVR (smoother, may overshoot).",
                  "velocityPrediction", st.velocityPrediction);
    endCard();
    ImGui::Dummy(ImVec2(0, 4));

    beginCard("##recording");
    sectionTitle("RECORDING");
    settingToggle(link, "Record sessions",
                  "Trackers, headset and controllers, to replay and tune the tracking offline. About 400 MB per hour.",
                  "recording", st.recording);
    if (s.recording.active) {
        infoRow("Recording", col::green, "%.0f MB", s.recording.megabytes);
        if (!s.recording.file.empty()) helpText(s.recording.file.c_str());
    }
    endCard();
    ImGui::Dummy(ImVec2(0, 4));

    beginCard("##app");
    sectionTitle("APPLICATION");
    if (toggleRow("Minimize to tray", "The minimize button hides the window in the notification area.",
                  &prefs.minimizeToTray))
        prefs.save();
    ImGui::Dummy(ImVec2(0, 4));
    if (toggleRow("Start in tray", "Start hidden in the notification area (click its icon to open).", &prefs.startInTray))
        prefs.save();
    endCard();
    ImGui::Dummy(ImVec2(0, 4));

    beginCard("##sounds");
    sectionTitle("SOUNDS");
    bool enabled = !prefs.soundMuted;
    if (toggleRow("Countdown sounds",
                  "A beep every second of a calibration countdown, and a chime when the pose is captured.", &enabled)) {
        prefs.soundMuted = !enabled;
        prefs.save();
    }
    ImGui::Dummy(ImVec2(0, 4));
    ImGui::BeginDisabled(prefs.soundMuted);
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        text(fonts.bold, col::text, "Volume");
        char value[16];
        std::snprintf(value, sizeof(value), "%d %%", static_cast<int>(std::lround(prefs.soundVolume * 100.0f)));
        const ImVec2 vs = textSize(fonts.body, value);
        textAt(ImGui::GetWindowDrawList(), fonts.body, ImVec2(p.x + avail - vs.x, p.y + 1), col::accentSoft, value);
        const float testW = 92.0f;
        const ImVec2 row = ImGui::GetCursorPos();
        ImGui::BeginChild("##volume", ImVec2(avail - testW - 16.0f, 36.0f), 0,
                          ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        ImGui::SetCursorPosY(5.0f);
        float v = prefs.soundVolume;
        bool released = false;
        if (slider("##volumeSlider", &v, 0.0f, 1.0f, &released)) prefs.soundVolume = v;
        if (released) {
            prefs.save();
            sounds::play(sounds::Sound::Tick, prefs.soundVolume);
        }
        ImGui::EndChild();
        ImGui::SetCursorPos(ImVec2(row.x + avail - testW, row.y + 1.0f));
        if (button("Test", ImVec2(testW, 34.0f), ButtonStyle::Secondary, icon::speaker))
            sounds::play(sounds::Sound::Done, prefs.soundVolume);
    }
    ImGui::EndDisabled();
    endCard();
    ImGui::Dummy(ImVec2(0, 8));

    if (button("Reset settings", ImVec2(0, 40), ButtonStyle::Danger)) g.confirmReset = true;
    helpText("Restores the default tracking settings above. Tracker roles, calibrations and the app "
             "preferences are kept.");
    ImGui::Dummy(ImVec2(0, 8));

    if (g.confirmReset) {
        ImGui::OpenPopup("Reset settings?");
        g.confirmReset = false;
    }
    const ImVec2 center = ImGui::GetMainViewport()->GetCenter();
    ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(420, 0));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22, 20));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 14.0f);
    if (ImGui::BeginPopupModal("Reset settings?", nullptr,
                               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove)) {
        text(fonts.title, col::text, "Reset settings?");
        textWrapped(fonts.body, col::textDim,
                    "The tracking settings go back to their defaults. Tracker roles, calibrations and the app "
                    "preferences are kept.");
        ImGui::Dummy(ImVec2(0, 6));
        const float w = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.5f;
        if (button("Cancel", ImVec2(w, 40))) ImGui::CloseCurrentPopup();
        ImGui::SameLine(0, 10.0f);
        if (button("Reset", ImVec2(w, 40), ButtonStyle::Primary)) {
            link.resetSettings();
            g.pendingNumbers.clear();
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    ImGui::PopStyleVar(2);
}

void updateGlow(const DriverState& s) {
    const float dt = ImGui::GetIO().DeltaTime;
    for (const TrackerInfo& t : s.trackers) {
        const float target = t.connected ? std::clamp((t.activity - 0.04f) / 0.22f, 0.0f, 1.0f) : 0.0f;
        float& v = g.glow[t.mac];
        const float rate = target > v ? 18.0f : 2.5f;
        v += (target - v) * (1.0f - std::exp(-rate * dt));
    }
}

}  // namespace

// ---------------------------------------------------------------- shared helpers

double steadyNow() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

double wallNow() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string shortMac(const std::string& mac) { return mac.size() >= 8 ? mac.substr(mac.size() - 8) : mac; }

std::string agoText(const std::optional<double>& when) {
    if (!when) return "never";
    const int minutes = static_cast<int>(std::max(0.0, wallNow() - *when) / 60.0);
    char buf[48];
    if (minutes < 1)
        std::snprintf(buf, sizeof(buf), "just now");
    else if (minutes < 60)
        std::snprintf(buf, sizeof(buf), "%d min ago", minutes);
    else if (minutes < 24 * 60)
        std::snprintf(buf, sizeof(buf), "%d h %02d min ago", minutes / 60, minutes % 60);
    else if (minutes < 48 * 60)
        std::snprintf(buf, sizeof(buf), "1 day ago");
    else
        std::snprintf(buf, sizeof(buf), "%d days ago", minutes / (24 * 60));
    return buf;
}

const TrackerInfo* findTracker(const DriverState& s, const std::string& mac) {
    for (const TrackerInfo& t : s.trackers)
        if (t.mac == mac) return &t;
    return nullptr;
}

std::string resolvedRole(const TrackerInfo& t) { return g.pendingStrings.resolve("role:" + t.mac, t.role); }

void roleSelector(DriverLink& link, const TrackerInfo& t, float width) {
    std::string role = resolvedRole(t);
    if (roleCombo("##role", role, width)) {
        g.pendingStrings.set("role:" + t.mac, role);
        link.setRole(t.mac, role);
    }
}

std::string trackerName(const TrackerInfo& t) {
    const std::string role = resolvedRole(t);
    if (role == "none") return "Tracker " + shortMac(t.mac);
    return roleLabel(role);
}

int signalLevel(int rssi) {
    if (rssi >= 0) return -1;
    if (rssi >= -55) return 3;
    if (rssi >= -67) return 2;
    if (rssi >= -77) return 1;
    return 0;
}

const char* signalQuality(int rssi) {
    static const char* names[] = {"weak", "fair", "good", "excellent"};
    const int level = signalLevel(rssi);
    return level < 0 ? "" : names[level];
}

void wifiIcon(ImDrawList* dl, ImVec2 c, float s, const TrackerInfo& t, ImU32 on) {
    const ImU32 off = alpha(col::textFaint, 0.40f);
    if (!t.connected) {
        icon::wifi(dl, c, s, -1, col::textFaint, off);
        return;
    }
    const int level = signalLevel(t.rssi);
    icon::wifi(dl, c, s, std::max(level, 0), level < 0 ? col::textFaint : (level == 0 ? col::orange : on), off);
}

ImU32 magStateColor(const std::string& state) {
    if (state == "ok") return col::green;
    if (state == "disturbed") return col::orange;
    if (state == "calibrating") return col::accentSoft;
    if (state == "uncalibrated") return col::orange;
    if (state == "acquiring") return col::cyan;
    return col::textFaint;
}

const char* magStateLabel(const std::string& state) {
    if (state == "ok") return "OK";
    if (state == "disturbed") return "Disturbed";
    if (state == "calibrating") return "Calibrating";
    if (state == "uncalibrated") return "Not calibrated";
    if (state == "acquiring") return "Acquiring";
    if (state == "disabled") return "Off";
    return "None";
}

void drawConnection(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t) {
    const float cy = p.y + h * 0.5f;
    const float x = p.x + 26;
    if (!t.connected) {
        wifiIcon(dl, ImVec2(p.x + 10, cy), 18.0f, t, col::text);
        textAt(dl, fonts.small, ImVec2(p.x + 28, cy - fonts.small->FontSize * 0.5f - 1), alpha(col::red, 0.85f), "Offline");
        return;
    }
    wifiIcon(dl, ImVec2(p.x + 10, cy + 1), 19.0f, t, col::text);
    char ms[24];
    if (t.latencyMs >= 0)
        std::snprintf(ms, sizeof(ms), "%.0f ms", t.latencyMs);
    else
        std::snprintf(ms, sizeof(ms), "-- ms");
    textAt(dl, fonts.small, ImVec2(x, cy - fonts.small->FontSize * 0.5f - 1), t.latencyMs > 50 ? col::orange : col::textDim,
           ms);
}

std::string batteryEstimateText(const std::string& mac, ImU32* color) {
    const BatteryEstimator::Estimate e = g.battery.estimate(mac);
    if (color) *color = col::textFaint;
    if (e.state == BatteryEstimator::State::Charging) {
        if (color) *color = col::green;
        return "charging";
    }
    if (e.state == BatteryEstimator::State::Estimating) return "";
    if (e.hours >= 24) return ">24 h";
    const int minutes = static_cast<int>(e.hours * 60 + 0.5);
    if (color && e.hours < 0.5) *color = col::orange;
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%dh%02d", minutes / 60, minutes % 60);
    return buf;
}

void drawBattery(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t) {
    const float cy = p.y + h * 0.5f;
    // ~0 % or < 3 V: USB powered or no battery sense, not a level
    const bool known = t.connected && t.battery >= 0.0f && BatteryEstimator::plausible(t.battery, t.batteryVoltage);
    const float level = known ? t.battery : -1.0f;
    const ImU32 fill = level < 0 ? col::textFaint : level < 0.15f ? col::red : level < 0.35f ? col::orange : col::green;
    const bool charging = known && g.battery.estimate(t.mac).state == BatteryEstimator::State::Charging;
    icon::battery(dl, ImVec2(p.x + 13, cy), 26.0f, level, charging, known ? col::textDim : col::textFaint, fill);
    const float ty = cy - fonts.small->FontSize * 0.5f - 1;
    if (!known) {
        textAt(dl, fonts.small, ImVec2(p.x + 32, ty), col::textFaint, "--");
        return;
    }
    char pct[16];
    std::snprintf(pct, sizeof(pct), "%.0f%%", t.battery * 100.0f);
    textAt(dl, fonts.small, ImVec2(p.x + 32, ty), level < 0.15f ? col::red : col::text, pct);
    ImU32 ec = 0;
    const std::string est = batteryEstimateText(t.mac, &ec);
    if (!est.empty()) {
        const float w = textSize(fonts.small, pct).x;
        textAt(dl, fonts.small, ImVec2(p.x + 32 + w + 6, ty), ec, est.c_str());
    }
}

void drawMag(ImDrawList* dl, ImVec2 p, float h, const TrackerInfo& t) {
    const float cy = p.y + h * 0.5f;
    std::string state = t.mag.state;
    // A valid calibration without a heading yet: the filter is acquiring
    if (state == "uncalibrated" && t.mag.residual < 0.999f) state = "acquiring";
    const ImU32 c = t.connected ? magStateColor(state) : col::textFaint;
    icon::compass(dl, ImVec2(p.x + 9, cy), 18.0f, t.connected ? col::textDim : col::textFaint, c);
    char label[48];
    if (state == "calibrating")
        std::snprintf(label, sizeof(label), "%.0f%%", t.mag.progress * 100.0f);
    else
        std::snprintf(label, sizeof(label), "%s", magStateLabel(state));
    textAt(dl, fonts.small, ImVec2(p.x + 24, cy - fonts.small->FontSize * 0.5f - 1), t.connected ? c : col::textFaint,
           label);
}

void drawGlow(ImDrawList* dl, ImVec2 min, ImVec2 max, float glow, float rounding) {
    if (glow <= 0.01f) return;
    dl->AddRect(ImVec2(min.x - 1.5f, min.y - 1.5f), ImVec2(max.x + 1.5f, max.y + 1.5f),
                sty(alpha(col::accentSoft, 0.25f * glow)), rounding + 1.5f, 0, 4.0f);
    dl->AddRect(min, max, sty(alpha(col::accentSoft, glow)), rounding, 0, 1.8f);
}

const char* boardName(int id) {
    static const char* const kNames[] = {"Unknown",         "SlimeVR (legacy)", "SlimeVR (dev)",     "NodeMCU",
                                         "Custom",          "ESP32 WROOM",      "Wemos D1 mini",     "TTGO T-Base",
                                         "ESP-01",          "SlimeVR v1.0/1.1", "Lolin C3 mini",     "Beetle ESP32-C3",
                                         "ESP32-C3 DevKitM-1", "owoTrack",      "Wrangler",          "mocopi",
                                         "Wemos WROOM-02",  "XIAO ESP32-C3",    "Haritora",          "ESP32-C6 DevKitC-1",
                                         "IMU glove (dev)", "Gestures",         "SlimeVR v1.2",      "ESP32-S3 SuperMini",
                                         "Generic nRF",     "Butterfly (dev)",  "Butterfly",         "ESP32-C6 SuperMini"};
    if (id >= 0 && id < static_cast<int>(std::size(kNames))) return kNames[id];
    return id == 250 ? "Development" : "Unknown";
}

const char* imuName(int id) {
    static const char* const kNames[] = {"Unknown",     "MPU9250", "MPU6500", "BNO080",  "BNO085",   "BNO055",
                                         "MPU6050",     "BNO086",  "BMI160",  "ICM20948", "ICM42688", "BMI270",
                                         "LSM6DS3TR-C", "LSM6DSV", "LSM6DSO", "LSM6DSR", "ICM45686", "ICM45605"};
    if (id >= 0 && id < static_cast<int>(std::size(kNames))) return kNames[id];
    return "Unknown";
}

const char* mcuName(int id) {
    static const char* const kNames[] = {"Unknown",        "ESP8266",  "ESP32",  "owoTrack (Android)", "Wrangler",
                                         "owoTrack (iOS)", "ESP32-C3", "mocopi", "Haritora",           "nRF52",
                                         "nRF54L"};
    if (id >= 0 && id < static_cast<int>(std::size(kNames))) return kNames[id];
    return "Unknown";
}

// ---------------------------------------------------------------- entry points

void setup() {
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // no imgui.ini next to the driver binaries
    io.LogFilename = nullptr;
    io.ConfigWindowsMoveFromTitleBarOnly = true;
    if (const char* appData = std::getenv("LOCALAPPDATA"))
        g.battery.load(std::string(appData) + "\\Slime Motion Tracker\\battery_history.json");
    setupTheme();
}

void shutdown() { g.battery.save(); }

void update(DriverLink& link, bool rendering) {
    const bool connected = link.connected();
    const DriverState& s = link.state();

    if (connected) {
        const double wall = wallNow();
        for (const TrackerInfo& t : s.trackers)
            if (t.connected && t.battery >= 0.0f)
                g.battery.add(t.mac, t.battery, t.batteryVoltage, wall);
    }
    if (Clock::now() - g.lastBatterySave > std::chrono::seconds(60)) {
        g.lastBatterySave = Clock::now();
        g.battery.save();
    }

    // Countdown sounds and the calibration result
    if (connected) {
        const CalibrationInfo& cal = s.calibration;
        const bool running = isRunning(cal.phase);
        const bool wasRunning = isRunning(g.soundPhase);
        const AppPrefs& prefs = appPrefs();
        const auto play = [&](sounds::Sound sound) {
            if (!prefs.soundMuted && prefs.soundVolume > 0.0f) sounds::play(sound, prefs.soundVolume);
        };
        if (running) {
            const int second = static_cast<int>(std::ceil(std::max(0.0f, cal.countdown) - 1e-3f));
            if (cal.phase != g.soundPhase) {
                if (wasRunning) {
                    play(sounds::Sound::Done);  // stand pose captured, the ski pose starts
                    g.soundSecond = second;
                } else {
                    g.soundSecond = INT_MAX;
                }
            }
            if (second >= 1 && second < g.soundSecond) play(sounds::Sound::Tick);
            g.soundSecond = second;
        } else if (wasRunning) {
            if (cal.phase == "done") play(sounds::Sound::Done);
            if (cal.phase == "error") play(sounds::Sound::Error);
            g.resultPhase = cal.phase;
            g.resultMessage = cal.message;
            g.resultAt = steadyNow();
            g.resultDismissed = cal.phase != "done" && cal.phase != "error";
        }
        g.soundPhase = cal.phase;
    }

    // 3D views: the engine streams poses only while they are shown
    const bool want = rendering && connected && (g.wantSkeleton || !g.wantTracker.empty());
    const double now = steadyNow();
    if (want) {
        const bool changed = !g.viewSent || g.sentTracker != g.wantTracker || g.sentMag != g.wantMag;
        if (changed || now - g.lastViewSent > 0.5) {
            link.sendView(true, g.wantTracker, g.wantMag);
            g.viewSent = true;
            g.sentTracker = g.wantTracker;
            g.sentMag = g.wantMag;
            g.lastViewSent = now;
        }
    } else if (g.viewSent) {
        link.sendView(false, "", false);
        g.viewSent = false;
    }
}

void draw(DriverLink& link, const FrameInfo& info) {
    g.connected = link.connected();
    // While the engine switches between the driver and the standalone host, the last
    // state stays on screen (disabled); defaults before the first one.
    static const DriverState kPlaceholder{};
    const DriverState& s = link.hasState() ? link.state() : kPlaceholder;
    if (!g.connected) {
        g.pendingNumbers.clear();
        g.pendingStrings.clear();
    }
    updateGlow(s);
    g.wantSkeleton = false;
    g.wantTracker.clear();
    g.wantMag = false;

    const ImGuiIO& io = ImGui::GetIO();
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("Slime Motion Tracker", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    ImGui::PushFont(fonts.body);

    if (!g.magPage.empty() && !findTracker(s, g.magPage)) g.magPage.clear();
    if (!g.magPage.empty()) {
        ImGui::BeginDisabled(!g.connected);
        drawMagPage(link, s);
        ImGui::EndDisabled();
    } else {
        const float sidebarW = io.DisplaySize.x < 760.0f ? 64.0f : 84.0f;
        drawSidebar(s, info, sidebarW, io.DisplaySize.y);
        ImGui::SameLine(0, 0);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(22, 20));
        const bool home = g.page == UiState::Page::Home;
        ImGui::BeginChild("##content", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding,
                          home ? (ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse) : 0);
        ImGui::PopStyleVar();
        ImGui::BeginDisabled(!g.connected);
        switch (g.page) {
        case UiState::Page::Home:
            drawHome(link, s);
            break;
        case UiState::Page::Body:
            drawBody(link, s);
            break;
        case UiState::Page::Settings:
            drawSettings(link, s);
            break;
        }
        ImGui::EndDisabled();
        ImGui::EndChild();
    }

    ImGui::PopFont();
    ImGui::End();
}

}  // namespace smt::ui
