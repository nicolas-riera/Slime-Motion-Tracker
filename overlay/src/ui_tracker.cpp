// Tracker page (all the info, role, mag, live 3D view) and the full-screen
// magnetometer calibration page.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ui_internal.h"

namespace smt::ui {

namespace {

using v3::V3;

// Tracker body in its sensor frame (meters, half extents): thin along the sensor Z
const V3 kTrackerHalf{0.028f, 0.020f, 0.009f};

const TrackerDetail* detailFor(const DriverLink& link, const std::string& mac) {
    const TrackerDetail& d = link.detail();
    return d.mac == mac && link.detailAge() < 1.5 ? &d : nullptr;
}

std::string displayMagState(const TrackerInfo& t) {
    if (t.mag.state == "uncalibrated" && t.mag.residual < 0.999f) return "acquiring";
    return t.mag.state;
}

const char* signalQuality(int rssi) {
    if (rssi >= 0) return "";
    if (rssi >= -55) return "excellent";
    if (rssi >= -67) return "good";
    if (rssi >= -77) return "fair";
    return "weak";
}

void openMagPage(DriverLink& link, const TrackerInfo& t, bool start) {
    g.magPage = t.mac;
    g.magSawGuided = false;
    g.cloudOrbit.reset();
    g.coverageOrbit.reset();
    if (start) link.magCalibrate(t.mac);
}

// ---------------------------------------------------------------- tracker page

void drawLiveView(const TrackerDetail* d, float height) {
    const float width = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 q(p.x + width, p.y + height);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, q, sty(col::card), 14.0f);
    textAt(dl, fonts.bold, ImVec2(p.x + 18, p.y + 15), col::text, "Live orientation");

    const float legendH = 64.0f;
    const ImVec2 vmin(p.x + 1, p.y + 46), vmax(q.x - 1, q.y - legendH);
    ImGui::SetCursorScreenPos(vmin);
    g.trackerOrbit.update("##trackerView", ImVec2(vmax.x - vmin.x, vmax.y - vmin.y), 0.4f, 3.0f);

    const bool ok = d && d->hasRotation && d->connected;
    const Quatf q0 = ok ? d->q : Quatf{};
    v3::Camera cam;
    cam.fov = 0.7f;
    cam.yaw = 0.6f + g.trackerOrbit.yaw;
    cam.pitch = std::clamp(0.50f + g.trackerOrbit.pitch, -1.2f, 1.45f);
    cam.distance = 0.40f * g.trackerOrbit.zoom;
    cam.target = {0, 0, 0};
    v3::Scene sc;
    sc.begin(dl, vmin, vmax, cam);
    sc.grid({0, -0.075f, 0}, 0.22f, 0.04f, alpha(col::accentSoft, 0.28f));
    sc.box({0, 0, 0}, q0, kTrackerHalf, ok ? IM_COL32(70, 70, 92, 255) : IM_COL32(60, 60, 72, 255),
           ok ? col::accent : IM_COL32(80, 80, 96, 255));
    // A dot marking the sensor +X end of the top face
    sc.sphere(v3::rotate(q0, V3{kTrackerHalf.x * 0.6f, 0.0f, kTrackerHalf.z + 0.001f}), 0.0035f, col::white, false);

    float gyroDeg = 0, accel = 0;
    if (ok) {
        const V3 w = v3::rotate(q0, d->gyro);
        const float wn = v3::length(w);
        gyroDeg = wn * 57.2958f;
        if (wn > 0.02f) sc.arrow({0, 0, 0}, v3::normalized(w) * std::min(0.15f, 0.035f + wn * 0.025f), col::cyan, 3.0f);
        const V3 a = v3::rotate(q0, d->accel);
        accel = v3::length(a);
        if (accel > 0.05f)
            sc.arrow({0, 0, 0}, v3::normalized(a) * std::min(0.15f, 0.035f + accel * 0.02f), col::orange, 3.0f);
        if (d->hasMagField && v3::length(d->magField) > 1e-3f) {
            sc.arrow({0, 0, 0}, v3::rotate(q0, v3::normalized(d->magField)) * 0.12f, col::green, 3.0f);
        } else if (d->hasMag) {
            const V3 raw{static_cast<float>(d->magRaw[0]), static_cast<float>(d->magRaw[1]),
                         static_cast<float>(d->magRaw[2])};
            if (v3::length(raw) > 1.0f)
                sc.arrow({0, 0, 0}, v3::rotate(q0, v3::normalized(raw)) * 0.10f, alpha(col::green, 0.45f), 2.0f);
        }
    }
    sc.flush();
    if (!ok) {
        const char* msg = d ? "No orientation (tracker offline)" : "Waiting for data...";
        const ImVec2 ms = textSize(fonts.small, msg);
        textAt(dl, fonts.small, ImVec2(p.x + (width - ms.x) * 0.5f, vmin.y + 8), col::textDim, msg);
    }

    // Legend
    const float ly = q.y - legendH + 10;
    const float colW = (width - 36) / 3.0f;
    const auto legend = [&](int i, ImU32 c, const char* name, const char* value) {
        const float x = p.x + 18 + colW * static_cast<float>(i);
        dl->AddRectFilled(ImVec2(x, ly + 6), ImVec2(x + 14, ly + 10), c, 2.0f);
        textAt(dl, fonts.smallBold, ImVec2(x + 20, ly), col::textDim, name);
        textAt(dl, fonts.body, ImVec2(x, ly + 20), col::text, value);
    };
    char b1[32], b2[32], b3[48];
    std::snprintf(b1, sizeof(b1), ok ? "%.0f \xC2\xB0/s" : "--", gyroDeg);
    std::snprintf(b2, sizeof(b2), ok ? "%.2f m/s\xC2\xB2" : "--", accel);
    if (d && d->hasMagField)
        std::snprintf(b3, sizeof(b3), "%.2f \xC3\x97", v3::length(d->magField));
    else
        std::snprintf(b3, sizeof(b3), "%s", d && d->hasMag ? "raw" : "none");
    legend(0, col::cyan, "GYRO", b1);
    legend(1, col::orange, "ACCEL", b2);
    legend(2, col::green, "MAG", b3);

    ImGui::SetCursorScreenPos(ImVec2(p.x, q.y));
    ImGui::Dummy(ImVec2(width, 0));
}

void drawInfoCards(DriverLink& link, const TrackerInfo& t, const TrackerDetail* d) {
    const auto na = [&](bool known) { return known ? col::text : col::textFaint; };

    beginCard("##device");
    sectionTitle("DEVICE");
    infoRow("MAC address", col::text, "%s", t.mac.c_str());
    if (t.transport == "espnow")
        infoRow("Connection", col::text, "ESP-NOW via USB dongle");
    else
        infoRow("Connection", col::text, "WiFi  %s", t.ip.c_str());
    infoRow("Firmware", na(d && !d->firmware.empty()), "%s", d && !d->firmware.empty() ? d->firmware.c_str() : "--");
    infoRow("Board", na(d), "%s", d ? boardName(d->board) : "--");
    infoRow("IMU", na(d), "%s", d ? imuName(d->imuType) : "--");
    infoRow("MCU", na(d), "%s", d ? mcuName(d->mcu) : "--");
    if (d)
        infoRow("Protocol", col::text, "v%d", d->protocolVersion);
    else
        infoRow("Protocol", col::textFaint, "--");
    if (d)
        infoRow("Boot count", col::text, "%d", d->bootCount);
    else
        infoRow("Boot count", col::textFaint, "--");
    endCard();
    ImGui::Dummy(ImVec2(0, 2));

    beginCard("##link");
    sectionTitle("LINK & POWER");
    if (t.connected && t.rssi < 0)
        infoRow("Signal", col::text, "%d dBm  (%s)", t.rssi, signalQuality(t.rssi));
    else
        infoRow("Signal", col::textFaint, "--");
    if (t.connected && t.latencyMs >= 0)
        infoRow("Latency", t.latencyMs > 50 ? col::orange : col::text, "%.0f ms", t.latencyMs);
    else
        infoRow("Latency", col::textFaint, "--");
    infoRow("Rotation rate", na(t.connected), "%.0f Hz", t.connected ? t.rotationHz : 0.0f);
    infoRow("Mag rate", na(t.connected && t.magHz > 0), "%.0f Hz", t.connected ? t.magHz : 0.0f);
    if (t.connected && t.battery >= 0 && !BatteryEstimator::plausible(t.battery, t.batteryVoltage)) {
        infoRow("Battery", col::textDim, "No reading (USB powered?)  \xC2\xB7  %.2f V", t.batteryVoltage);
    } else if (t.connected && t.battery >= 0) {
        infoRow("Battery", t.battery < 0.15f ? col::red : col::text, "%.0f %%  \xC2\xB7  %.2f V", t.battery * 100.0f,
                t.batteryVoltage);
        ImU32 ec = col::text;
        std::string est = batteryEstimateText(t.mac, &ec);
        if (est.empty()) {
            est = "Estimating...";
            ec = col::textFaint;
        } else if (est != "charging") {
            est = "~" + est;
            ec = ec == col::textFaint ? col::text : ec;
        }
        infoRow("Remaining", ec, "%s", est.c_str());
    } else {
        infoRow("Battery", col::textFaint, "--");
    }
    if (t.connected)
        infoRow("Temperature", col::text, "%.1f \xC2\xB0""C", t.temperature);
    else
        infoRow("Temperature", col::textFaint, "--");
    endCard();
    ImGui::Dummy(ImVec2(0, 2));

    beginCard("##magcard");
    sectionTitle("MAGNETOMETER");
    const std::string state = displayMagState(t);
    const bool hasMag = t.mag.state != "absent";
    infoRow("State", t.connected ? magStateColor(state) : col::textFaint, "%s",
            state == "calibrating" ? "Calibrating" : magStateLabel(state));
    if (hasMag) {
        const bool valid = d ? d->magValid : t.mag.residual < 0.999f;
        if (valid) {
            const float residual = d ? d->magResidual : t.mag.residual;
            infoRow("Fit residual", residual < 0.08f ? col::green : col::orange, "%.3f", residual);
            infoRow("Field dip (calibration)", col::text, "%.1f\xC2\xB0", d ? d->magDip : t.mag.dip);
        }
        if (d && d->magNorm > 0) {
            infoRow("Field now", col::text, "%.2f \xC3\x97  \xC2\xB7  dip %.1f\xC2\xB0", d->magNorm, d->magFieldDip);
        }
        if (d && d->hasHeading) infoRow("Heading correction", col::text, "%.1f\xC2\xB0", d->heading);
        ImGui::Dummy(ImVec2(0, 4));
        const float w = ImGui::GetContentRegionAvail().x;
        const float bw = (w - 20) / 3.0f;
        const bool calibrating = t.mag.state == "calibrating";
        ImGui::BeginDisabled(!t.connected || t.mag.state == "disabled");
        if (button(calibrating ? "Open##cal" : "Calibrate", ImVec2(bw, 38), ButtonStyle::Primary))
            openMagPage(link, t, !calibrating);
        ImGui::EndDisabled();
        ImGui::SameLine(0, 10);
        ImGui::BeginDisabled(!t.connected);
        if (button("Samples", ImVec2(bw, 38))) openMagPage(link, t, false);
        ImGui::EndDisabled();
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Show the mag samples collected for the automatic calibration");
        ImGui::SameLine(0, 10);
        ImGui::BeginDisabled(!valid);
        if (button("Reset##mag", ImVec2(bw, 38), ButtonStyle::Danger)) link.magReset(t.mac);
        ImGui::EndDisabled();
        if (t.mag.state == "disabled") helpText("The magnetometer is turned off in the settings.");
    } else {
        helpText("This tracker has no magnetometer (or it does not report it).");
    }
    endCard();
}

// ---------------------------------------------------------------- mag page views

void drawCloudView(const TrackerDetail* d, ImVec2 size) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 q(p.x + size.x, p.y + size.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, q, sty(col::card), 14.0f);
    textAt(dl, fonts.bold, ImVec2(p.x + 18, p.y + 15), col::text, "Field samples");

    const ImVec2 vmin(p.x + 1, p.y + 44), vmax(q.x - 1, q.y - 30);
    ImGui::SetCursorScreenPos(vmin);
    g.cloudOrbit.update("##cloud", ImVec2(vmax.x - vmin.x, vmax.y - vmin.y), 0.4f, 3.0f);

    const std::vector<Vec3f>* samples = d && d->hasMagView ? &d->magView.samples : nullptr;
    // Center and size of the cloud
    V3 lo{1e9f, 1e9f, 1e9f}, hi{-1e9f, -1e9f, -1e9f};
    if (samples)
        for (const Vec3f& s : *samples) {
            lo = {std::min(lo.x, s.x), std::min(lo.y, s.y), std::min(lo.z, s.z)};
            hi = {std::max(hi.x, s.x), std::max(hi.y, s.y), std::max(hi.z, s.z)};
        }
    const bool any = samples && !samples->empty();
    const V3 center = any ? (lo + hi) * 0.5f : V3{};
    float radius = 1.0f;
    float meanR = 0.0f;
    if (any) {
        radius = 1.0f;
        for (const Vec3f& s : *samples) radius = std::max(radius, v3::length(V3(s) - center));
        for (const Vec3f& s : *samples) meanR += v3::length(V3(s) - center);
        meanR /= static_cast<float>(samples->size()) * radius;
    }

    // Slow turntable while the user does not touch it
    const double idle = ImGui::GetTime() - g.cloudOrbit.lastInteraction;
    if (idle > 3.0) g.cloudOrbit.yaw += ImGui::GetIO().DeltaTime * 0.25f;
    v3::Camera cam;
    cam.fov = 0.7f;
    cam.yaw = 0.7f + g.cloudOrbit.yaw;
    cam.pitch = std::clamp(0.35f + g.cloudOrbit.pitch, -1.3f, 1.3f);
    cam.distance = 3.6f * g.cloudOrbit.zoom;
    v3::Scene sc;
    sc.begin(dl, vmin, vmax, cam);
    const float axis = 1.25f;
    sc.line({-axis, 0, 0}, {axis, 0, 0}, alpha(col::red, 0.45f), 1.5f);
    sc.line({0, -axis, 0}, {0, axis, 0}, alpha(col::green, 0.45f), 1.5f);
    sc.line({0, 0, -axis}, {0, 0, axis}, alpha(col::cyan, 0.45f), 1.5f);
    sc.label({axis, 0, 0}, "X", alpha(col::red, 0.8f));
    sc.label({0, axis, 0}, "Y", alpha(col::green, 0.8f));
    sc.label({0, 0, axis}, "Z", alpha(col::cyan, 0.8f));
    if (any) {
        // Reference sphere: a well calibrated sensor fills it evenly
        const ImU32 ref = alpha(col::white, 0.10f);
        sc.circle({0, 0, 0}, {1, 0, 0}, meanR, ref, 1.0f);
        sc.circle({0, 0, 0}, {0, 1, 0}, meanR, ref, 1.0f);
        sc.circle({0, 0, 0}, {0, 0, 1}, meanR, ref, 1.0f);
        for (const Vec3f& s : *samples) {
            const V3 v = (V3(s) - center) * (1.0f / radius);
            const float t = std::clamp(v.y * 0.5f + 0.5f, 0.0f, 1.0f);
            sc.point(v, 3.0f, alpha(mix(col::accentSoft, col::pink, t), 0.9f));
        }
        const V3 cur = (V3{static_cast<float>(d->magRaw[0]), static_cast<float>(d->magRaw[1]),
                           static_cast<float>(d->magRaw[2])} -
                        center) *
                       (1.0f / radius);
        sc.line({0, 0, 0}, cur, alpha(col::white, 0.6f), 1.5f);
        sc.point(cur, 5.5f, col::white);
        sc.ring(cur, 10.0f, alpha(col::white, 0.5f), 1.5f);
    }
    sc.flush();
    char info[64];
    if (any)
        std::snprintf(info, sizeof(info), "%d points", static_cast<int>(samples->size()));
    else
        std::snprintf(info, sizeof(info), "%s", d ? "No samples yet: rotate the tracker" : "Waiting for data...");
    const ImVec2 is = textSize(fonts.small, info);
    textAt(dl, fonts.small, ImVec2(q.x - 18 - is.x, p.y + 18), col::textDim, info);
    const char* hint = "Raw sensor readings. A good calibration needs an evenly covered ellipsoid.";
    const ImVec2 hs = textSize(fonts.small, hint);
    if (hs.x < size.x - 30) textAt(dl, fonts.small, ImVec2(p.x + (size.x - hs.x) * 0.5f, q.y - 26), col::textFaint, hint);

    ImGui::SetCursorScreenPos(ImVec2(p.x, q.y));
    ImGui::Dummy(ImVec2(size.x, 0));
}

void drawCoverageView(const TrackerDetail* d, ImVec2 size) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 q(p.x + size.x, p.y + size.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, q, sty(col::card), 14.0f);
    textAt(dl, fonts.bold, ImVec2(p.x + 18, p.y + 15), col::text, "Orientations");

    const ImVec2 vmin(p.x + 1, p.y + 44), vmax(q.x - 1, q.y - 8);
    ImGui::SetCursorScreenPos(vmin);
    g.coverageOrbit.update("##coverage", ImVec2(vmax.x - vmin.x, vmax.y - vmin.y), 0.5f, 2.5f);

    const bool ok = d && d->hasRotation;
    const Quatf qr = ok ? d->qRaw : Quatf{};
    v3::Camera cam;
    cam.fov = 0.7f;
    cam.yaw = 0.5f + g.coverageOrbit.yaw;
    cam.pitch = std::clamp(0.30f + g.coverageOrbit.pitch, -1.2f, 1.4f);
    cam.distance = 0.62f * g.coverageOrbit.zoom;
    v3::Scene sc;
    sc.begin(dl, vmin, vmax, cam);
    const float r = 0.13f;
    sc.circle({0, 0, 0}, {0, 1, 0}, r, alpha(col::white, 0.08f), 1.0f);
    sc.box({0, 0, 0}, qr, kTrackerHalf * 1.5f, IM_COL32(70, 70, 92, 255), col::accent);
    // Where the sensor's "up" must be for each bin; the bin at the top fills now
    sc.arrow({0, r + 0.075f, 0}, {0, r + 0.025f, 0}, alpha(col::white, 0.7f), 2.0f);
    if (d && d->hasMagView && !d->magView.binDirs.empty()) {
        const MagViewInfo& mv = d->magView;
        int top = -1;
        float best = -2.0f;
        for (size_t i = 0; i < mv.binDirs.size(); ++i) {
            const V3 w = v3::rotate(qr, mv.binDirs[i]);
            if (w.y > best) {
                best = w.y;
                top = static_cast<int>(i);
            }
        }
        for (size_t i = 0; i < mv.binDirs.size(); ++i) {
            const V3 w = v3::rotate(qr, mv.binDirs[i]) * r;
            const int n = i < mv.bins.size() ? mv.bins[i] : 0;
            const float f = std::clamp(static_cast<float>(n) / static_cast<float>(mv.target), 0.0f, 1.0f);
            const ImU32 c = f >= 1.0f ? col::green : (n > 0 ? mix(col::accentDeep, col::accentSoft, 0.4f + 0.6f * f)
                                                            : IM_COL32(64, 64, 80, 255));
            sc.sphere(w, 0.0115f, c, true);
            if (static_cast<int>(i) == top) sc.ring(w, 12.0f, col::white, 2.0f);
        }
    }
    sc.flush();
    ImGui::SetCursorScreenPos(ImVec2(p.x, q.y));
    ImGui::Dummy(ImVec2(size.x, 0));
}

}  // namespace

void drawTrackerPage(DriverLink& link, const DriverState& s, const TrackerInfo& t) {
    (void)s;
    g.wantTracker = t.mac;
    const TrackerDetail* d = detailFor(link, t.mac);

    if (button("Trackers##back", ImVec2(0, 34), ButtonStyle::Ghost, icon::chevronLeft)) {
        g.trackerPage.clear();
        return;
    }
    // Title row: name, status, role
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        const std::string name = trackerName(t);
        text(fonts.title, col::text, "%s", name.c_str());
        ImGui::SameLine(0, 12);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 6);
        if (t.connected)
            pill("Online", col::green, alpha(col::green, 0.14f));
        else
            pill("Offline", col::red, alpha(col::red, 0.14f));
        const float comboW = std::min(200.0f, avail * 0.36f);
        ImGui::SetCursorScreenPos(ImVec2(p.x + avail - comboW, p.y + 2));
        roleSelector(link, t, comboW);
        ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + 44));
        ImGui::Dummy(ImVec2(0, 0));
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    if (avail.x >= 700.0f) {
        const float gap = 18.0f;
        const float rightW = std::clamp(avail.x * 0.46f, 300.0f, 560.0f);
        ImGui::BeginChild("##trackerInfo", ImVec2(avail.x - rightW - gap, avail.y), 0, ImGuiWindowFlags_NoBackground);
        drawInfoCards(link, t, d);
        ImGui::EndChild();
        ImGui::SameLine(0, gap);
        ImGui::BeginChild("##trackerView", ImVec2(rightW, avail.y), 0,
                          ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawLiveView(d, avail.y);
        ImGui::EndChild();
    } else {
        ImGui::BeginChild("##trackerScroll", avail, 0, ImGuiWindowFlags_NoBackground);
        drawLiveView(d, 340.0f);
        ImGui::Dummy(ImVec2(0, 4));
        drawInfoCards(link, t, d);
        ImGui::EndChild();
    }
}

void drawMagPage(DriverLink& link, const DriverState& s) {
    const TrackerInfo* tp = findTracker(s, g.magPage);
    if (!tp) return;
    const TrackerInfo& t = *tp;
    g.wantTracker = t.mac;
    g.wantMag = true;
    const TrackerDetail* d = detailFor(link, t.mac);
    const bool guided = t.mag.state == "calibrating" || (d && d->hasMagView && d->magView.guided);
    if (guided) g.magSawGuided = true;
    const bool finished = g.magSawGuided && !guided;
    const bool success = finished && (d ? d->magValid : t.mag.residual < 0.999f);

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(24, 18));
    ImGui::BeginChild("##magPage", ImVec2(0, 0), ImGuiChildFlags_AlwaysUseWindowPadding);
    ImGui::PopStyleVar();

    // Top bar
    {
        const float avail = ImGui::GetContentRegionAvail().x;
        const ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddCircleFilled(ImVec2(p.x + 20, p.y + 22), 20.0f, col::accentDeep, 24);
        icon::compass(dl, ImVec2(p.x + 20, p.y + 22), 24.0f, col::accentSoft, col::white);
        textAt(dl, fonts.title, ImVec2(p.x + 52, p.y), col::text, "Magnetometer calibration");
        const std::string sub = trackerName(t) + "  \xC2\xB7  " + t.mac;
        textAt(dl, fonts.small, ImVec2(p.x + 52, p.y + 30), col::textDim, sub.c_str());
        ImGui::SetCursorScreenPos(ImVec2(p.x + avail - 110, p.y + 4));
        if (button("Close", ImVec2(110, 38), ButtonStyle::Secondary, icon::cross)) {
            if (guided) link.magCancel(t.mac);
            g.magPage.clear();
        }
        ImGui::SetCursorScreenPos(ImVec2(p.x, p.y + 60));
        ImGui::Dummy(ImVec2(0, 0));
    }

    const ImVec2 avail = ImGui::GetContentRegionAvail();
    const bool wide = avail.x >= 820.0f;
    const float gap = 18.0f;
    const float rightW = wide ? std::clamp(avail.x * 0.36f, 320.0f, 460.0f) : avail.x;
    const auto statusPanel = [&] {
        beginCard("##magStatus");
        const MagViewInfo* mv = d && d->hasMagView ? &d->magView : nullptr;
        const float progress = guided ? (mv ? mv->progress : t.mag.progress) : (mv ? mv->progress : t.mag.progress);
        int filled = 0;
        if (mv)
            for (int n : mv->bins) filled += n >= mv->target ? 1 : 0;
        const char* title = guided ? "Collecting samples" : success ? "Calibration saved" : finished ? "Stopped" : "Ready";
        text(fonts.bold, success ? col::green : col::text, "%s", title);
        ImGui::SameLine();
        const float pw = ImGui::GetContentRegionAvail().x;
        char pct[16];
        std::snprintf(pct, sizeof(pct), "%.0f%%", progress * 100.0f);
        const ImVec2 ps = textSize(fonts.bold, pct);
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + pw - ps.x);
        text(fonts.bold, col::accentSoft, "%s", pct);
        progressBar(progress, 10.0f, success ? col::green : col::accent);
        infoRow("Orientations covered", col::text, "%d / %d", filled, mv ? static_cast<int>(mv->bins.size()) : 26);
        infoRow("Samples", col::text, "%d", mv ? static_cast<int>(mv->samples.size()) : 0);
        if (d && d->magValid) infoRow("Fit residual", d->magResidual < 0.08f ? col::green : col::orange, "%.3f", d->magResidual);
        ImGui::Dummy(ImVec2(0, 2));
        if (guided)
            helpText("Hold the tracker away from metal and slowly rotate it in every direction: upside down, on "
                     "each side, figure eights. Bring each dot of the sphere to the top, under the arrow.");
        else if (success)
            helpText("The calibration is saved for this tracker. Strap it back on and run a quick reset.");
        else if (finished)
            helpText("The calibration stopped before enough orientations were covered. Start again and rotate the "
                     "tracker more slowly, in every direction.");
        else
            helpText("Start, then rotate the tracker in every direction until the bar is full (it takes about a "
                     "minute). The tracker also calibrates itself slowly while worn.");
        ImGui::Dummy(ImVec2(0, 4));
        const float bw = (ImGui::GetContentRegionAvail().x - 10.0f) * 0.5f;
        if (guided) {
            if (button("Cancel", ImVec2(-1, 40))) link.magCancel(t.mac);
        } else if (success) {
            if (button("Done", ImVec2(bw, 40), ButtonStyle::Primary)) g.magPage.clear();
            ImGui::SameLine(0, 10);
            if (button("Recalibrate", ImVec2(bw, 40))) {
                g.magSawGuided = false;
                link.magCalibrate(t.mac);
            }
        } else {
            ImGui::BeginDisabled(!t.connected || t.mag.state == "absent" || t.mag.state == "disabled");
            if (button("Start calibration", ImVec2(-1, 40), ButtonStyle::Primary)) {
                g.magSawGuided = false;
                link.magCalibrate(t.mac);
            }
            ImGui::EndDisabled();
        }
        endCard();
    };

    if (wide) {
        ImGui::BeginChild("##magLeft", ImVec2(avail.x - rightW - gap, avail.y), 0,
                          ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        drawCloudView(d, ImVec2(ImGui::GetContentRegionAvail().x, avail.y));
        ImGui::EndChild();
        ImGui::SameLine(0, gap);
        ImGui::BeginChild("##magRight", ImVec2(rightW, avail.y), 0, ImGuiWindowFlags_NoBackground);
        drawCoverageView(d, ImVec2(ImGui::GetContentRegionAvail().x, std::clamp(avail.y * 0.48f, 220.0f, 360.0f)));
        ImGui::Dummy(ImVec2(0, 2));
        statusPanel();
        ImGui::EndChild();
    } else {
        ImGui::BeginChild("##magScroll", avail, 0, ImGuiWindowFlags_NoBackground);
        const float w = ImGui::GetContentRegionAvail().x;
        drawCloudView(d, ImVec2(w, 340.0f));
        ImGui::Dummy(ImVec2(0, 2));
        drawCoverageView(d, ImVec2(w, 260.0f));
        ImGui::Dummy(ImVec2(0, 2));
        statusPanel();
        ImGui::EndChild();
    }
    ImGui::EndChild();
}

}  // namespace smt::ui
