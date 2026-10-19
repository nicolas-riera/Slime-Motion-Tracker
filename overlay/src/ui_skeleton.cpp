// Home page skeleton: the solved body in 3D, orbitable.
#include <algorithm>
#include <cmath>

#include "ui_internal.h"

namespace smt::ui {

namespace {

using v3::V3;

struct Skeleton {
    V3 eyes, headCenter;
    Quatf headQ;
    V3 neck, pelvis, hip[2], knee[2], ankle[2], toe[2];
    V3 shoulder[2];
    bool arms = false;  // rest pose arms (live: only toward tracked controllers)
    V3 rest[2];  // rest pose hands
    bool hand[2] = {false, false};
    V3 handP[2];
    Quatf handQ[2];
    bool planted[2] = {false, false};
    float floor = 0.0f;
    V3 forward{0, 0, -1};
};

constexpr float kTwoPi = 6.2831853f;

float wrapAngle(float a) {
    while (a > 3.14159265f) a -= kTwoPi;
    while (a < -3.14159265f) a += kTwoPi;
    return a;
}

// Neutral standing body facing -Z (the SteamVR forward), feet on y = 0
Skeleton restSkeleton(float h) {
    Skeleton k;
    k.floor = 0.0f;
    k.eyes = {0, 0.936f * h, -0.02f * h};
    k.headCenter = k.eyes + V3{0, 0.012f, 0.07f};
    k.neck = {0, 0.82f * h, 0.01f * h};
    k.pelvis = {0, 0.56f * h, 0};
    k.arms = true;
    for (int i = 0; i < 2; ++i) {
        const float side = i == 0 ? -1.0f : 1.0f;
        k.shoulder[i] = {side * 0.115f * h, 0.805f * h, 0.012f * h};
        k.rest[i] = {side * 0.15f * h, 0.47f * h, 0.0f};
        k.hip[i] = {side * 0.05f * h, 0.53f * h, 0};
        k.knee[i] = {side * 0.055f * h, 0.285f * h, -0.008f * h};
        k.ankle[i] = {side * 0.058f * h, 0.039f * h, 0.0f};
        k.toe[i] = {side * 0.064f * h, 0.012f * h, -0.085f * h};
    }
    return k;
}

Skeleton liveSkeleton(const PoseInfo& p, float h) {
    Skeleton k;
    k.floor = p.floor;
    k.headQ = p.headQ;
    k.eyes = p.head;
    k.headCenter = V3(p.head) + v3::rotate(p.headQ, V3{0, 0.012f, 0.07f});
    k.neck = p.neck;
    k.pelvis = p.pelvis;
    // Hip joints around the smoothed pelvis (the solver's hips are unsmoothed)
    const V3 mid = (V3(p.hipL) + V3(p.hipR)) * 0.5f;
    k.hip[0] = V3(p.pelvis) + (V3(p.hipL) - mid);
    k.hip[1] = V3(p.pelvis) + (V3(p.hipR) - mid);
    k.knee[0] = p.kneeL;
    k.knee[1] = p.kneeR;
    k.ankle[0] = p.ankleL;
    k.ankle[1] = p.ankleR;
    const float footLen = 0.095f * h;
    for (int i = 0; i < 2; ++i) {
        k.toe[i] = k.ankle[i] + v3::rotate(p.footQ[i], V3{0, -0.25f, -1.0f}) * footLen;
        k.toe[i].y = std::max(k.toe[i].y, k.floor + 0.01f);
        k.planted[i] = p.planted[i];
        k.hand[i] = p.handValid[i];
        k.handP[i] = p.hand[i];
        k.handQ[i] = p.handQ[i];
    }
    // Facing: the pelvis, else the head
    V3 f = v3::rotate(p.pelvisQ, V3{0, 0, -1});
    f.y = 0;
    if (v3::length(f) < 0.3f) {
        f = v3::rotate(p.headQ, V3{0, 0, -1});
        f.y = 0;
    }
    k.forward = v3::length(f) > 0.1f ? v3::normalized(f) : V3{0, 0, -1};
    // Shoulders on the torso (not tracked: for the look and the controllers' arms)
    const V3 up = v3::normalized(k.neck - k.pelvis);
    const V3 right = v3::normalized(v3::cross(k.forward, up));
    for (int i = 0; i < 2; ++i)
        k.shoulder[i] = k.neck + right * ((i == 0 ? -1.0f : 1.0f) * 0.115f * h) - up * (0.015f * h);
    return k;
}

}  // namespace

void drawSkeletonCard(const DriverLink& link, const DriverState& s, float height) {
    g.wantSkeleton = true;
    const PoseInfo& pose = link.pose();
    const bool live = g.connected && pose.ready && pose.valid && link.poseAge() < 1.0;
    const float h = std::clamp(s.userHeight, 1.2f, 2.2f);
    const Skeleton k = live ? liveSkeleton(pose, h) : restSkeleton(h);

    const float width = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 q(p.x + width, p.y + height);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, q, sty(col::card), 14.0f);

    // Header: title + status
    textAt(dl, fonts.bold, ImVec2(p.x + 18, p.y + 15), col::text, "Body");
    const char* status;
    ImU32 statusCol;
    if (live) {
        status = s.mode == "ankles" ? "Live \xC2\xB7 ankles only" : "Live \xC2\xB7 ankles + waist";
        statusCol = col::green;
    } else if (!g.connected) {
        status = "Engine starting";
        statusCol = col::textDim;
    } else if (!s.calibrated) {
        status = "Not calibrated";
        statusCol = col::orange;
    } else {
        status = "Waiting for trackers";
        statusCol = col::orange;
    }
    {
        const ImVec2 ts = textSize(fonts.smallBold, status);
        const ImVec2 a(q.x - 16 - ts.x - 26, p.y + 14), b(q.x - 16, p.y + 14 + ts.y + 10);
        dl->AddRectFilled(a, b, alpha(statusCol, 0.13f), (b.y - a.y) * 0.5f);
        dl->AddCircleFilled(ImVec2(a.x + 12, (a.y + b.y) * 0.5f), 3.5f, statusCol, 10);
        textAt(dl, fonts.smallBold, ImVec2(a.x + 20, a.y + 5), statusCol, status);
    }

    // View
    const ImVec2 vmin(p.x + 1, p.y + 50), vmax(q.x - 1, q.y - 34);
    ImGui::SetCursorScreenPos(vmin);
    g.skeletonOrbit.update("##skeleton", ImVec2(vmax.x - vmin.x, vmax.y - vmin.y), 0.4f, 2.5f);

    // Camera follows the body (position and facing); the user's orbit is relative
    const float dt = std::min(ImGui::GetIO().DeltaTime, 0.1f);
    const float yawTarget = std::atan2(k.forward.x, k.forward.z);
    if (!g.bodyYawValid || !live) {
        g.bodyYaw = yawTarget;
        g.bodyYawValid = live;
    } else {
        g.bodyYaw = wrapAngle(g.bodyYaw + wrapAngle(yawTarget - g.bodyYaw) * (1.0f - std::exp(-3.0f * dt)));
    }
    const float bodyHeight = std::max(1.0f, (k.eyes.y - k.floor) / 0.936f);
    const V3 target{k.pelvis.x, k.floor + 0.48f * bodyHeight, k.pelvis.z};
    if (!g.bodyTargetValid || !live) {
        g.bodyTarget = target;
        g.bodyTargetValid = live;
    } else {
        g.bodyTarget = v3::lerp(g.bodyTarget, target, 1.0f - std::exp(-6.0f * dt));
    }
    v3::Camera cam;
    cam.fov = 0.70f;
    cam.yaw = g.bodyYaw + 0.55f + g.skeletonOrbit.yaw;
    cam.pitch = std::clamp(0.14f + g.skeletonOrbit.pitch, -0.30f, 1.40f);
    const float aspect = (vmax.x - vmin.x) / std::max(1.0f, vmax.y - vmin.y);
    cam.distance = 0.64f * bodyHeight / std::tan(cam.fov * 0.5f) * g.skeletonOrbit.zoom * std::max(1.0f, 0.62f / aspect);
    cam.target = g.bodyTarget;

    v3::Scene sc;
    sc.begin(dl, vmin, vmax, cam);
    // Floor: grid snapped to the world (it does not slide with the player), shadows
    const float step = 0.25f;
    const V3 gridCenter{std::round(g.bodyTarget.x / step) * step, k.floor, std::round(g.bodyTarget.z / step) * step};
    sc.grid(gridCenter, 1.75f, step, alpha(col::accentSoft, 0.30f));
    for (int i = 0; i < 2; ++i) {
        const V3 c{k.ankle[i].x * 0.6f + k.toe[i].x * 0.4f, k.floor + 0.001f, k.ankle[i].z * 0.6f + k.toe[i].z * 0.4f};
        const float lift = std::clamp((k.ankle[i].y - k.floor - 0.05f) / 0.5f, 0.0f, 1.0f);
        const ImU32 shadow = live && k.planted[i] ? alpha(col::green, 0.28f) : IM_COL32(0, 0, 0, 90);
        sc.bgDisc(c, 0.075f + 0.04f * lift, alpha(shadow, 1.0f - 0.6f * lift));
    }

    const ImU32 grey = IM_COL32(86, 86, 106, 255);
    const ImU32 spineCol = live ? col::accentSoft : grey;
    const ImU32 legCol[2] = {live ? col::cyan : grey, live ? col::pink : grey};
    const ImU32 jointCol = live ? IM_COL32(240, 240, 250, 255) : IM_COL32(118, 118, 138, 255);
    const ImU32 headCol = live ? IM_COL32(196, 188, 255, 255) : IM_COL32(104, 104, 126, 255);
    const float boneW = 0.038f, spineW = 0.048f, jointR = 0.027f;

    // Head: a bone from the top of the neck to the top of the head (follows the headset)
    const V3 headBase = k.headCenter + v3::rotate(k.headQ, V3{0, -0.10f, 0});
    const V3 headTop = k.headCenter + v3::rotate(k.headQ, V3{0, 0.11f, 0});
    sc.line(k.neck, headBase, spineCol, spineW * 0.8f, true);
    sc.line(headBase, headTop, headCol, spineW * 1.05f, true);
    sc.sphere(headBase, jointR, jointCol);
    sc.sphere(headTop, jointR * 0.9f, jointCol);
    sc.line(k.neck, k.pelvis, spineCol, spineW, true);
    sc.sphere(k.neck, jointR, jointCol);
    const ImU32 armCol = live ? alpha(col::accentSoft, 0.55f) : grey;
    for (int i = 0; i < 2; ++i) {
        sc.line(k.neck, k.shoulder[i], spineCol, spineW * 0.8f, true);
        sc.sphere(k.shoulder[i], jointR * 0.9f, jointCol);
        if (k.arms) {
            const V3 elbow = v3::lerp(k.shoulder[i], k.rest[i], 0.5f) + V3{0, 0, 0.012f * h};
            sc.line(k.shoulder[i], elbow, armCol, boneW * 0.85f, true);
            sc.line(elbow, k.rest[i], armCol, boneW * 0.8f, true);
            sc.sphere(elbow, jointR * 0.8f, jointCol);
        } else if (live && k.hand[i]) {
            sc.line(k.shoulder[i], k.handP[i], armCol, boneW * 0.6f, true);
        }
    }
    sc.sphere(k.pelvis, jointR * 1.3f, jointCol);
    for (int i = 0; i < 2; ++i) {
        sc.line(k.pelvis, k.hip[i], spineCol, spineW * 0.85f, true);
        sc.line(k.hip[i], k.knee[i], legCol[i], boneW, true);
        sc.line(k.knee[i], k.ankle[i], legCol[i], boneW, true);
        sc.line(k.ankle[i], k.toe[i], legCol[i], boneW * 0.8f, true);
        sc.sphere(k.hip[i], jointR, jointCol);
        sc.sphere(k.knee[i], jointR, jointCol);
        sc.sphere(k.ankle[i], jointR, jointCol);
        if (live && k.hand[i]) {
            sc.sphere(k.handP[i], 0.038f, col::orange);
            sc.line(k.handP[i], k.handP[i] + v3::rotate(k.handQ[i], V3{0, 0, -0.09f}), alpha(col::orange, 0.85f), 0.018f,
                    true);
        }
    }
    sc.flush();

    // Footer hint
    const char* hint = "Drag to rotate  \xC2\xB7  Scroll to zoom  \xC2\xB7  Double-click to reset";
    const ImVec2 hs = textSize(fonts.small, hint);
    if (hs.x < width - 20)
        textAt(dl, fonts.small, ImVec2(p.x + (width - hs.x) * 0.5f, q.y - 26), col::textFaint, hint);
    if (!live && g.connected) {
        bool left = false, right = false, offline = false;
        for (const TrackerInfo& t : s.trackers) {
            const std::string role = resolvedRole(t);
            left = left || role == "left_ankle";
            right = right || role == "right_ankle";
            offline = offline || (role != "none" && !t.connected);
        }
        for (const TrackerInfo& t : s.mappedOffline) {
            left = left || t.role == "left_ankle";
            right = right || t.role == "right_ankle";
            offline = true;
        }
        const char* why = !left || !right ? "Assign both ankle trackers (Body tab)"
                          : offline       ? "Every assigned tracker must be connected"
                                          : "Run a full calibration to see your body";
        const ImVec2 ws = textSize(fonts.small, why);
        if (ws.x < width - 20) textAt(dl, fonts.small, ImVec2(p.x + (width - ws.x) * 0.5f, p.y + 52), col::textDim, why);
    }

    ImGui::SetCursorScreenPos(ImVec2(p.x, q.y));
    ImGui::Dummy(ImVec2(width, 0));
}

}  // namespace smt::ui
