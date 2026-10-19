#include "widgets.h"

#include <windows.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "imgui_internal.h"

namespace smt::ui {

Fonts fonts;

namespace {

constexpr float kPi = 3.14159265358979f;

ImVec2 operator+(ImVec2 a, ImVec2 b) { return {a.x + b.x, a.y + b.y}; }
ImVec2 operator-(ImVec2 a, ImVec2 b) { return {a.x - b.x, a.y - b.y}; }
ImVec2 operator*(ImVec2 a, float s) { return {a.x * s, a.y * s}; }

// Point of an icon in its unit box: (-0.5..0.5) * s around c
ImVec2 P(ImVec2 c, float s, float x, float y) { return {c.x + x * s, c.y + y * s}; }

float stroke(float s) { return std::max(1.5f, s * 0.085f); }

// Polyline with round joints/caps (small discs), looks smoother at icon sizes
void roundLine(ImDrawList* dl, std::initializer_list<ImVec2> pts, ImU32 col, float t, bool closed = false) {
    dl->AddPolyline(pts.begin(), static_cast<int>(pts.size()), col, closed ? ImDrawFlags_Closed : 0, t);
    for (const ImVec2& p : pts) dl->AddCircleFilled(p, t * 0.5f, col, 8);
}

std::string fontPath(const char* file) {
    char windir[MAX_PATH] = {};
    const UINT n = GetWindowsDirectoryA(windir, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    return std::string(windir) + "\\Fonts\\" + file;
}

bool fileExists(const std::string& path) {
    const DWORD attr = GetFileAttributesA(path.c_str());
    return !path.empty() && attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

struct RoleOption {
    const char* value;
    const char* label;
};
constexpr RoleOption kRoles[] = {
    {"none", "Unassigned"}, {"waist", "Waist"}, {"left_ankle", "Left ankle"}, {"right_ankle", "Right ankle"}};

}  // namespace

// ---------------------------------------------------------------- colors

ImU32 alpha(ImU32 c, float a) {
    const float cur = static_cast<float>((c >> IM_COL32_A_SHIFT) & 0xFF);
    const ImU32 na = static_cast<ImU32>(std::clamp(cur * a, 0.0f, 255.0f));
    return (c & ~IM_COL32_A_MASK) | (na << IM_COL32_A_SHIFT);
}

ImU32 mix(ImU32 a, ImU32 b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    const auto ch = [&](int shift) {
        const float x = static_cast<float>((a >> shift) & 0xFF), y = static_cast<float>((b >> shift) & 0xFF);
        return static_cast<ImU32>(x + (y - x) * t + 0.5f) << shift;
    };
    return ch(IM_COL32_R_SHIFT) | ch(IM_COL32_G_SHIFT) | ch(IM_COL32_B_SHIFT) | ch(IM_COL32_A_SHIFT);
}

ImVec4 vec4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }

float animate(ImGuiID id, float target, float rate) {
    ImGuiStorage* st = ImGui::GetStateStorage();
    float* v = st->GetFloatRef(id, target);
    const float dt = ImGui::GetIO().DeltaTime;
    *v += (target - *v) * (1.0f - std::exp(-rate * dt));
    if (std::fabs(target - *v) < 1e-3f) *v = target;
    return *v;
}

// ---------------------------------------------------------------- theme

void setupTheme() {
    ImGuiIO& io = ImGui::GetIO();
    static const ImWchar kRanges[] = {0x0020, 0x00FF, 0x2010, 0x2027, 0x2190, 0x2193, 0x2212, 0x2212, 0x2248, 0x2248, 0};
    const std::string regular = fontPath("segoeui.ttf");
    const std::string semibold = fileExists(fontPath("seguisb.ttf")) ? fontPath("seguisb.ttf") : fontPath("segoeuib.ttf");
    if (fileExists(regular) && fileExists(semibold)) {
        ImFontConfig cfg;
        cfg.OversampleH = 3;
        cfg.OversampleV = 2;
        fonts.body = io.Fonts->AddFontFromFileTTF(regular.c_str(), 19.0f, &cfg, kRanges);
        fonts.bold = io.Fonts->AddFontFromFileTTF(semibold.c_str(), 19.0f, &cfg, kRanges);
        fonts.small = io.Fonts->AddFontFromFileTTF(regular.c_str(), 15.5f, &cfg, kRanges);
        fonts.smallBold = io.Fonts->AddFontFromFileTTF(semibold.c_str(), 14.5f, &cfg, kRanges);
        fonts.title = io.Fonts->AddFontFromFileTTF(semibold.c_str(), 26.0f, &cfg, kRanges);
        fonts.big = io.Fonts->AddFontFromFileTTF(semibold.c_str(), 36.0f, &cfg, kRanges);
    }
    if (!fonts.body || !fonts.bold || !fonts.small || !fonts.smallBold || !fonts.title || !fonts.big) {
        io.Fonts->Clear();
        ImFontConfig cfg;
        const auto add = [&](float size) {
            cfg.SizePixels = size;
            return io.Fonts->AddFontDefault(&cfg);
        };
        fonts.body = add(19.0f);
        fonts.bold = fonts.body;
        fonts.small = add(15.0f);
        fonts.smallBold = fonts.small;
        fonts.title = add(26.0f);
        fonts.big = add(36.0f);
    }

    ImGuiStyle& style = ImGui::GetStyle();
    style = ImGuiStyle();
    style.WindowRounding = 0.0f;
    style.WindowBorderSize = 0.0f;
    style.ChildBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(0, 0);
    style.FramePadding = ImVec2(12, 8);
    style.ItemSpacing = ImVec2(10, 10);
    style.ItemInnerSpacing = ImVec2(10, 6);
    style.CellPadding = ImVec2(8, 6);
    style.FrameRounding = 8.0f;
    style.ChildRounding = 12.0f;
    style.PopupRounding = 10.0f;
    style.GrabRounding = 8.0f;
    style.ScrollbarRounding = 8.0f;
    style.ScrollbarSize = 12.0f;
    style.GrabMinSize = 18.0f;
    style.SelectableTextAlign = ImVec2(0.0f, 0.5f);
    style.DisabledAlpha = 0.55f;

    ImVec4* c = style.Colors;
    c[ImGuiCol_Text] = vec4(col::text);
    c[ImGuiCol_TextDisabled] = vec4(col::textFaint);
    c[ImGuiCol_WindowBg] = vec4(col::bg);  // opaque: the VR overlay must not be see-through
    c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_PopupBg] = vec4(IM_COL32(30, 30, 41, 255));
    c[ImGuiCol_Border] = vec4(col::line);
    c[ImGuiCol_FrameBg] = vec4(col::field);
    c[ImGuiCol_FrameBgHovered] = vec4(IM_COL32(46, 46, 62, 255));
    c[ImGuiCol_FrameBgActive] = vec4(IM_COL32(54, 50, 84, 255));
    c[ImGuiCol_Button] = vec4(col::field);
    c[ImGuiCol_ButtonHovered] = vec4(IM_COL32(48, 48, 66, 255));
    c[ImGuiCol_ButtonActive] = vec4(IM_COL32(58, 54, 90, 255));
    c[ImGuiCol_Header] = vec4(alpha(col::accent, 0.45f));
    c[ImGuiCol_HeaderHovered] = vec4(alpha(col::accent, 0.30f));
    c[ImGuiCol_HeaderActive] = vec4(alpha(col::accent, 0.55f));
    c[ImGuiCol_CheckMark] = vec4(col::accentSoft);
    c[ImGuiCol_SliderGrab] = vec4(col::accentSoft);
    c[ImGuiCol_SliderGrabActive] = vec4(IM_COL32(176, 166, 255, 255));
    c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
    c[ImGuiCol_ScrollbarGrab] = vec4(IM_COL32(58, 58, 76, 255));
    c[ImGuiCol_ScrollbarGrabHovered] = vec4(IM_COL32(76, 76, 98, 255));
    c[ImGuiCol_ScrollbarGrabActive] = vec4(col::accent);
    c[ImGuiCol_Separator] = vec4(col::line);
    c[ImGuiCol_PlotHistogram] = vec4(col::accent);
    c[ImGuiCol_ModalWindowDimBg] = ImVec4(0.02f, 0.02f, 0.04f, 0.65f);
    c[ImGuiCol_NavHighlight] = ImVec4(0, 0, 0, 0);
}

// ---------------------------------------------------------------- icons

namespace icon {

void home(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    roundLine(dl, {P(c, s, -0.40f, -0.02f), P(c, s, 0.0f, -0.40f), P(c, s, 0.40f, -0.02f)}, col, t);
    roundLine(dl, {P(c, s, -0.28f, -0.12f), P(c, s, -0.28f, 0.38f), P(c, s, 0.28f, 0.38f), P(c, s, 0.28f, -0.12f)},
              col, t);
    dl->AddRectFilled(P(c, s, -0.08f, 0.12f), P(c, s, 0.08f, 0.38f), col, s * 0.03f);
}

void body(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s) * 1.15f;
    dl->AddCircleFilled(P(c, s, 0.0f, -0.32f), s * 0.12f, col, 20);
    roundLine(dl, {P(c, s, -0.30f, -0.10f), P(c, s, 0.30f, -0.10f)}, col, t);
    roundLine(dl, {P(c, s, 0.0f, -0.12f), P(c, s, 0.0f, 0.12f)}, col, t);
    roundLine(dl, {P(c, s, -0.16f, 0.42f), P(c, s, 0.0f, 0.10f), P(c, s, 0.16f, 0.42f)}, col, t);
}

void gear(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    constexpr int kTeeth = 8;
    ImVec2 pts[kTeeth * 4];
    const float ro = s * 0.44f, ri = s * 0.32f;
    for (int i = 0; i < kTeeth; ++i) {
        const float a0 = (static_cast<float>(i) / kTeeth) * 2 * kPi;
        const float w = 2 * kPi / kTeeth;
        const float as[4] = {a0 - w * 0.18f, a0 + w * 0.18f, a0 + w * 0.32f, a0 + w * 0.68f};
        const float rs[4] = {ro, ro, ri, ri};
        for (int k = 0; k < 4; ++k) {
            const float a = k < 2 ? as[k] : (k == 2 ? a0 + w * 0.32f : a0 + w * 0.68f);
            pts[i * 4 + k] = ImVec2(c.x + std::cos(a) * rs[k], c.y + std::sin(a) * rs[k]);
        }
    }
    dl->AddPolyline(pts, kTeeth * 4, col, ImDrawFlags_Closed, t);
    dl->AddCircle(c, s * 0.13f, col, 20, t);
}

void calibrate(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    // Scan frame corners
    const float a = 0.46f, b = 0.26f;
    roundLine(dl, {P(c, s, -a, -b), P(c, s, -a, -a), P(c, s, -b, -a)}, col, t);
    roundLine(dl, {P(c, s, b, -a), P(c, s, a, -a), P(c, s, a, -b)}, col, t);
    roundLine(dl, {P(c, s, -a, b), P(c, s, -a, a), P(c, s, -b, a)}, col, t);
    roundLine(dl, {P(c, s, b, a), P(c, s, a, a), P(c, s, a, b)}, col, t);
    // Person
    dl->AddCircleFilled(P(c, s, 0.0f, -0.20f), s * 0.085f, col, 16);
    roundLine(dl, {P(c, s, 0.0f, -0.06f), P(c, s, 0.0f, 0.12f)}, col, t);
    roundLine(dl, {P(c, s, -0.16f, 0.02f), P(c, s, 0.0f, -0.06f), P(c, s, 0.16f, 0.02f)}, col, t);
    roundLine(dl, {P(c, s, -0.11f, 0.30f), P(c, s, 0.0f, 0.12f), P(c, s, 0.11f, 0.30f)}, col, t);
}

void reset(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s) * 1.1f;
    const float r = s * 0.32f;
    const float a0 = -kPi * 0.35f, a1 = kPi * 1.25f;
    dl->PathArcTo(c, r, a0, a1, 28);
    dl->PathStroke(col, 0, t);
    // Arrow head at the start of the arc, pointing along it (counter-clockwise)
    const ImVec2 tip(c.x + std::cos(a0) * r, c.y + std::sin(a0) * r);
    const ImVec2 tangent(std::sin(a0), -std::cos(a0));  // direction of decreasing angle
    const ImVec2 normal(std::cos(a0), std::sin(a0));
    const float h = s * 0.20f;
    dl->AddTriangleFilled(tip + tangent * (h * 0.9f), tip + normal * (h * 0.6f) - tangent * (h * 0.15f),
                          tip - normal * (h * 0.6f) - tangent * (h * 0.15f), col);
    dl->AddCircleFilled(c, s * 0.06f, col, 12);
}

void tracker(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    dl->AddRect(P(c, s, -0.36f, -0.28f), P(c, s, 0.36f, 0.28f), col, s * 0.12f, 0, t);
    dl->AddCircleFilled(P(c, s, 0.18f, -0.10f), s * 0.06f, col, 10);
}

void dongle(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s) * 0.9f;
    // Body (filled) and connector (outline with two contacts), horizontal
    dl->AddRectFilled(P(c, s, -0.46f, -0.17f), P(c, s, 0.10f, 0.17f), col, s * 0.07f);
    dl->AddRect(P(c, s, 0.10f, -0.12f), P(c, s, 0.44f, 0.12f), col, s * 0.02f, 0, t);
    dl->AddRectFilled(P(c, s, 0.20f, -0.05f), P(c, s, 0.26f, 0.01f), col);
    dl->AddRectFilled(P(c, s, 0.31f, -0.05f), P(c, s, 0.37f, 0.01f), col);
}

void wifi(ImDrawList* dl, ImVec2 c, float s, int bars, ImU32 on, ImU32 off) {
    on = sty(on);
    off = sty(off);
    const float t = std::max(1.8f, s * 0.11f);
    const ImVec2 o = P(c, s, 0.0f, 0.32f);
    const float a0 = -kPi * 0.75f, a1 = -kPi * 0.25f;
    const float radii[3] = {0.26f, 0.48f, 0.70f};
    for (int i = 0; i < 3; ++i) {
        dl->PathArcTo(o, radii[i] * s, a0, a1, 16);
        dl->PathStroke(i < bars ? on : off, 0, t);
    }
    dl->AddCircleFilled(o, s * 0.08f, bars > 0 ? on : off, 12);
    if (bars < 0) dl->AddLine(P(c, s, -0.38f, -0.38f), P(c, s, 0.38f, 0.38f), on, t);
}

void signalBars(ImDrawList* dl, ImVec2 c, float s, int bars, ImU32 on, ImU32 off) {
    on = sty(on);
    off = sty(off);
    const float w = s * 0.16f, gap = s * 0.09f;
    const float x0 = c.x - (4 * w + 3 * gap) * 0.5f, bottom = c.y + s * 0.38f;
    for (int i = 0; i < 4; ++i) {
        const float h = s * (0.24f + 0.17f * static_cast<float>(i)) * 1.1f;
        const float x = x0 + static_cast<float>(i) * (w + gap);
        dl->AddRectFilled(ImVec2(x, bottom - h), ImVec2(x + w, bottom), i < bars ? on : off, w * 0.3f);
    }
}

void battery(ImDrawList* dl, ImVec2 c, float s, float level, bool charging, ImU32 outline, ImU32 fill) {
    outline = sty(outline);
    fill = sty(fill);
    const float t = std::max(1.4f, s * 0.07f);
    const ImVec2 a = P(c, s, -0.46f, -0.22f), b = P(c, s, 0.36f, 0.22f);
    dl->AddRect(a, b, outline, s * 0.08f, 0, t);
    dl->AddRectFilled(P(c, s, 0.38f, -0.09f), P(c, s, 0.48f, 0.09f), outline, s * 0.03f);
    const float in = t + s * 0.035f;
    if (level >= 0.0f) {
        const float w = (b.x - a.x - 2 * in) * std::clamp(level, 0.0f, 1.0f);
        // Charging: dimmer fill so the bolt reads on top of it
        const ImU32 f = charging ? alpha(fill, 0.45f) : fill;
        if (w > 0.5f) dl->AddRectFilled(ImVec2(a.x + in, a.y + in), ImVec2(a.x + in + w, b.y - in), f, s * 0.04f);
    }
    if (charging) {
        // Bolt: two triangles inside the body (no self-intersecting polygon)
        const float u = ((b.y - a.y) - 2.0f * in) * 0.5f;
        const ImVec2 m((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
        const ImU32 bolt = sty(col::white);
        dl->AddTriangleFilled(ImVec2(m.x + u * 0.35f, m.y - u * 1.05f), ImVec2(m.x - u * 0.55f, m.y + u * 0.12f),
                              ImVec2(m.x + u * 0.10f, m.y + u * 0.12f), bolt);
        dl->AddTriangleFilled(ImVec2(m.x - u * 0.10f, m.y - u * 0.12f), ImVec2(m.x + u * 0.55f, m.y - u * 0.12f),
                              ImVec2(m.x - u * 0.35f, m.y + u * 1.05f), bolt);
    }
}

void compass(ImDrawList* dl, ImVec2 c, float s, ImU32 ring, ImU32 needle) {
    ring = sty(ring);
    needle = sty(needle);
    const float t = stroke(s);
    dl->AddCircle(c, s * 0.42f, ring, 28, t);
    const ImVec2 n = P(c, s, 0.10f, -0.28f), so = P(c, s, -0.10f, 0.28f);
    const ImVec2 l = P(c, s, -0.09f, -0.04f), r = P(c, s, 0.09f, 0.04f);
    dl->AddTriangleFilled(n, l, r, needle);
    dl->AddTriangleFilled(so, r, l, alpha(ring, 0.8f));
}

void chevronRight(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    roundLine(dl, {P(c, s, -0.12f, -0.26f), P(c, s, 0.14f, 0.0f), P(c, s, -0.12f, 0.26f)}, col, stroke(s));
}

void chevronLeft(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    roundLine(dl, {P(c, s, 0.12f, -0.26f), P(c, s, -0.14f, 0.0f), P(c, s, 0.12f, 0.26f)}, col, stroke(s));
}

void check(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    roundLine(dl, {P(c, s, -0.30f, 0.02f), P(c, s, -0.08f, 0.24f), P(c, s, 0.32f, -0.22f)}, col, stroke(s) * 1.2f);
}

void cross(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s) * 1.1f;
    roundLine(dl, {P(c, s, -0.25f, -0.25f), P(c, s, 0.25f, 0.25f)}, col, t);
    roundLine(dl, {P(c, s, 0.25f, -0.25f), P(c, s, -0.25f, 0.25f)}, col, t);
}

void warning(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    roundLine(dl, {P(c, s, 0.0f, -0.40f), P(c, s, 0.44f, 0.36f), P(c, s, -0.44f, 0.36f)}, col, t, true);
    roundLine(dl, {P(c, s, 0.0f, -0.12f), P(c, s, 0.0f, 0.10f)}, col, t);
    dl->AddCircleFilled(P(c, s, 0.0f, 0.23f), t * 0.7f, col, 8);
}

void info(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    dl->AddCircle(c, s * 0.42f, col, 28, t);
    roundLine(dl, {P(c, s, 0.0f, -0.04f), P(c, s, 0.0f, 0.22f)}, col, t);
    dl->AddCircleFilled(P(c, s, 0.0f, -0.20f), t * 0.7f, col, 8);
}

void speaker(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const ImVec2 pts[6] = {P(c, s, -0.40f, -0.12f), P(c, s, -0.20f, -0.12f), P(c, s, 0.04f, -0.34f),
                           P(c, s, 0.04f, 0.34f),   P(c, s, -0.20f, 0.12f),  P(c, s, -0.40f, 0.12f)};
    dl->AddConcavePolyFilled(pts, 6, col);
    const float t = stroke(s);
    dl->PathArcTo(P(c, s, 0.06f, 0.0f), s * 0.18f, -kPi * 0.3f, kPi * 0.3f, 10);
    dl->PathStroke(col, 0, t);
    dl->PathArcTo(P(c, s, 0.06f, 0.0f), s * 0.34f, -kPi * 0.3f, kPi * 0.3f, 14);
    dl->PathStroke(col, 0, t);
}

void thermometer(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    dl->AddRect(P(c, s, -0.09f, -0.42f), P(c, s, 0.09f, 0.14f), col, s * 0.09f, 0, t);
    dl->AddCircleFilled(P(c, s, 0.0f, 0.26f), s * 0.16f, col, 16);
    dl->AddRectFilled(P(c, s, -0.035f, -0.10f), P(c, s, 0.035f, 0.2f), col);
}

void clock(ImDrawList* dl, ImVec2 c, float s, ImU32 col) {
    col = sty(col);
    const float t = stroke(s);
    dl->AddCircle(c, s * 0.42f, col, 28, t);
    roundLine(dl, {P(c, s, 0.0f, -0.24f), P(c, s, 0.0f, 0.0f), P(c, s, 0.17f, 0.10f)}, col, t);
}

void logo(ImDrawList* dl, ImVec2 p, float size) {
    dl->AddRectFilled(p, ImVec2(p.x + size, p.y + size), col::accent, size * 0.22f);
    ImFont* f = fonts.bold;
    const float fs = size * 0.40f;
    const ImVec2 ts = f->CalcTextSizeA(fs, FLT_MAX, 0.0f, "SMT");
    dl->AddText(f, fs, ImVec2(p.x + (size - ts.x) * 0.5f, p.y + (size - ts.y) * 0.5f), col::white, "SMT");
}

}  // namespace icon

// ---------------------------------------------------------------- widgets

void text(ImFont* font, ImU32 color, const char* fmt, ...) {
    char buf[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ImGui::PushFont(font);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::TextUnformatted(buf);
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void textWrapped(ImFont* font, ImU32 color, const char* fmt, ...) {
    char buf[2048];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ImGui::PushFont(font);
    ImGui::PushStyleColor(ImGuiCol_Text, color);
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(buf);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
    ImGui::PopFont();
}

void textAt(ImDrawList* dl, ImFont* font, ImVec2 pos, ImU32 color, const char* t) {
    dl->AddText(font, font->FontSize, pos, sty(color), t);
}

ImVec2 textSize(ImFont* font, const char* t) { return font->CalcTextSizeA(font->FontSize, FLT_MAX, 0.0f, t); }

bool navItem(const char* label, IconFn iconFn, bool active, bool collapsed, float width) {
    // Compact rail entry: icon above a short label (icon only when collapsed)
    const float h = collapsed ? 48.0f : 62.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::PushID(label);
    const bool clicked = ImGui::InvisibleButton("##nav", ImVec2(width, h));
    const bool hovered = ImGui::IsItemHovered();
    if (collapsed && hovered) ImGui::SetTooltip("%s", label);
    const float hov = animate(ImGui::GetID("hov"), hovered ? 1.0f : 0.0f, 16.0f);
    const float act = animate(ImGui::GetID("act"), active ? 1.0f : 0.0f, 14.0f);
    ImGui::PopID();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 q(p.x + width, p.y + h);
    ImU32 bg = alpha(col::white, 0.05f * hov);
    bg = mix(bg, alpha(col::accent, 0.26f), act);
    dl->AddRectFilled(p, q, sty(bg), 12.0f);
    const ImU32 fg = mix(mix(col::textDim, col::text, hov), col::white, act);
    const ImU32 ic = mix(mix(col::textDim, col::text, hov), col::accentSoft, act);
    const float iconSize = 22.0f;
    if (collapsed) {
        iconFn(dl, ImVec2(p.x + width * 0.5f, p.y + h * 0.5f), iconSize, ic);
    } else {
        iconFn(dl, ImVec2(p.x + width * 0.5f, p.y + 22.0f), iconSize, ic);
        const ImVec2 ts = textSize(fonts.smallBold, label);
        textAt(dl, fonts.smallBold, ImVec2(p.x + (width - ts.x) * 0.5f, p.y + 38.0f), fg, label);
    }
    return clicked;
}

void beginCard(const char* id, float height, ImU32 bg, float padding) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, bg);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(padding, padding));
    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 12.0f);
    const ImGuiChildFlags flags =
        ImGuiChildFlags_AlwaysUseWindowPadding | (height <= 0.0f ? ImGuiChildFlags_AutoResizeY : 0);
    ImGui::BeginChild(id, ImVec2(0, height > 0.0f ? height : 0.0f), flags, ImGuiWindowFlags_NoScrollbar);
}

void endCard() {
    ImGui::EndChild();
    ImGui::PopStyleVar(2);
    ImGui::PopStyleColor();
}

void sectionTitle(const char* t) {
    text(fonts.smallBold, col::textDim, "%s", t);
    ImGui::Dummy(ImVec2(0, 2));
}

bool button(const char* label, ImVec2 size, ButtonStyle style, IconFn iconFn) {
    const char* end = ImGui::FindRenderedTextEnd(label);
    const std::string visible(label, end);
    const ImVec2 ts = textSize(fonts.bold, visible.c_str());
    const float iconSize = 18.0f;
    const float contentW = ts.x + (iconFn ? iconSize + 8.0f : 0.0f);
    if (size.x <= 0.0f) size.x = size.x < 0.0f ? ImGui::GetContentRegionAvail().x : contentW + 32.0f;
    if (size.y <= 0.0f) size.y = 40.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(label, size);
    const bool disabled = (ImGui::GetItemFlags() & ImGuiItemFlags_Disabled) != 0;
    const bool hovered = ImGui::IsItemHovered() && !disabled;
    const bool held = ImGui::IsItemActive();
    const float hov = animate(ImGui::GetItemID(), hovered ? 1.0f : 0.0f, 16.0f);

    ImU32 bg = 0, fg = col::text, border = 0;
    switch (style) {
    case ButtonStyle::Primary:
        bg = mix(col::accent, col::accentHover, hov);
        fg = col::white;
        break;
    case ButtonStyle::Secondary:
        bg = mix(col::field, IM_COL32(50, 50, 68, 255), hov);
        break;
    case ButtonStyle::Danger:
        bg = mix(alpha(col::red, 0.14f), alpha(col::red, 0.26f), hov);
        border = alpha(col::red, 0.55f);
        fg = IM_COL32(255, 170, 172, 255);
        break;
    case ButtonStyle::Ghost:
        bg = alpha(col::white, 0.06f * hov);
        fg = mix(col::textDim, col::text, hov);
        break;
    }
    if (held) bg = mix(bg, col::white, 0.06f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 q(p.x + size.x, p.y + size.y);
    dl->AddRectFilled(p, q, sty(bg), 9.0f);
    if (border) dl->AddRect(p, q, sty(border), 9.0f, 0, 1.2f);
    float x = p.x + (size.x - contentW) * 0.5f;
    if (iconFn) {
        iconFn(dl, ImVec2(x + iconSize * 0.5f, p.y + size.y * 0.5f), iconSize, fg);
        x += iconSize + 8.0f;
    }
    textAt(dl, fonts.bold, ImVec2(x, p.y + (size.y - ts.y) * 0.5f), fg, visible.c_str());
    return clicked && !disabled;
}

bool slider(const char* id, float* value, float vMin, float vMax, bool* released) {
    const float w = ImGui::GetContentRegionAvail().x, h = 26.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton(id, ImVec2(std::max(w, 1.0f), h));
    const bool active = ImGui::IsItemActive();
    const bool hovered = ImGui::IsItemHovered();
    const float knob = 9.0f;
    const float x0 = p.x + knob, x1 = p.x + w - knob;
    bool changed = false;
    if (active && x1 > x0) {
        const float t = std::clamp((ImGui::GetIO().MousePos.x - x0) / (x1 - x0), 0.0f, 1.0f);
        const float v = vMin + t * (vMax - vMin);
        if (v != *value) {
            *value = v;
            changed = true;
        }
    }
    if (released) *released = ImGui::IsItemDeactivated();
    if (hovered || active) ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    const float t = vMax > vMin ? std::clamp((*value - vMin) / (vMax - vMin), 0.0f, 1.0f) : 0.0f;
    const float hov = animate(ImGui::GetItemID(), (hovered || active) ? 1.0f : 0.0f, 16.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float cy = p.y + h * 0.5f, th = 6.0f;
    const float kx = x0 + (x1 - x0) * t;
    dl->AddRectFilled(ImVec2(x0 - 3, cy - th * 0.5f), ImVec2(x1 + 3, cy + th * 0.5f), sty(IM_COL32(52, 52, 70, 255)),
                      th * 0.5f);
    dl->AddRectFilled(ImVec2(x0 - 3, cy - th * 0.5f), ImVec2(kx, cy + th * 0.5f), sty(col::accent), th * 0.5f);
    if (hov > 0.01f) dl->AddCircleFilled(ImVec2(kx, cy), knob + 6.0f * hov, sty(alpha(col::accentSoft, 0.18f * hov)), 24);
    dl->AddCircleFilled(ImVec2(kx, cy), knob, sty(col::white), 24);
    dl->AddCircle(ImVec2(kx, cy), knob - 0.5f, sty(col::accentSoft), 24, 2.0f);
    return changed;
}

void progressBar(float fraction, float height, ImU32 fill) {
    const float w = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::Dummy(ImVec2(w, height));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + height), sty(IM_COL32(52, 52, 70, 255)), height * 0.5f);
    const float fw = w * std::clamp(fraction, 0.0f, 1.0f);
    if (fw > 1.0f) {
        dl->PushClipRect(p, ImVec2(p.x + fw, p.y + height), true);
        dl->AddRectFilled(p, ImVec2(p.x + w, p.y + height), sty(fill), height * 0.5f);
        dl->PopClipRect();
    }
}

bool toggle(const char* id, bool* value) {
    const float h = 24.0f, w = 44.0f;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const bool clicked = ImGui::InvisibleButton(id, ImVec2(w, h));
    if (clicked) *value = !*value;
    const float on = animate(ImGui::GetItemID(), *value ? 1.0f : 0.0f, 18.0f);
    const bool hovered = ImGui::IsItemHovered();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    ImU32 track = mix(IM_COL32(58, 58, 76, 255), col::accent, on);
    if (hovered) track = mix(track, col::white, 0.08f);
    dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), sty(track), h * 0.5f);
    const float r = h * 0.5f - 3.0f;
    const float cx = p.x + 3.0f + r + (w - 6.0f - 2 * r) * on;
    dl->AddCircleFilled(ImVec2(cx, p.y + h * 0.5f), r, sty(col::white), 20);
    return clicked;
}

bool toggleRow(const char* label, const char* help, bool* value) {
    ImGui::PushID(label);
    const float avail = ImGui::GetContentRegionAvail().x;
    const ImVec2 start = ImGui::GetCursorPos();
    ImGui::BeginGroup();
    ImGui::PushTextWrapPos(start.x + avail - 64.0f);
    text(fonts.bold, col::text, "%s", label);
    if (help && *help) {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 6.0f);
        textWrapped(fonts.small, col::textDim, "%s", help);
    }
    ImGui::PopTextWrapPos();
    ImGui::EndGroup();
    const float groupH = ImGui::GetItemRectSize().y;
    const ImVec2 after = ImGui::GetCursorPos();
    ImGui::SetCursorPos(ImVec2(start.x + avail - 44.0f, start.y + std::max(0.0f, (groupH - 24.0f) * 0.5f)));
    const bool changed = toggle("##toggle", value);
    ImGui::SetCursorPos(after);
    ImGui::PopID();
    return changed;
}

void infoRow(const char* label, ImU32 valueColor, const char* fmt, ...) {
    char buf[512];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    const float avail = ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float h = fonts.body->FontSize + 8.0f;
    ImGui::Dummy(ImVec2(avail, h));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float ty = p.y + (h - fonts.body->FontSize) * 0.5f;
    textAt(dl, fonts.body, ImVec2(p.x, ty), col::textDim, label);
    const ImVec2 vs = textSize(fonts.body, buf);
    textAt(dl, fonts.body, ImVec2(p.x + avail - vs.x, ty), valueColor, buf);
    if (ImGui::IsItemHovered() && vs.x > avail * 0.6f) ImGui::SetTooltip("%s", buf);
}

void helpText(const char* t) { textWrapped(fonts.small, col::textDim, "%s", t); }

void pill(const char* t, ImU32 fg, ImU32 bg) {
    const ImVec2 ts = textSize(fonts.smallBold, t);
    const ImVec2 size(ts.x + 18.0f, ts.y + 8.0f);
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImGui::Dummy(size);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), sty(bg), size.y * 0.5f);
    textAt(dl, fonts.smallBold, ImVec2(p.x + 9.0f, p.y + 4.0f), fg, t);
}

void separator(float spacing) {
    ImGui::Dummy(ImVec2(0, spacing * 0.5f));
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float w = ImGui::GetContentRegionAvail().x;
    ImGui::GetWindowDrawList()->AddLine(p, ImVec2(p.x + w, p.y), sty(col::line), 1.0f);
    ImGui::Dummy(ImVec2(0, spacing * 0.5f));
}

const char* roleLabel(const std::string& role) {
    for (const RoleOption& r : kRoles)
        if (role == r.value) return r.label;
    return role.c_str();
}

bool roleCombo(const char* id, std::string& role, float width) {
    bool changed = false;
    ImGui::SetNextItemWidth(width);
    ImGui::PushFont(fonts.body);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(12, 7));
    ImGui::PushStyleVar(ImGuiStyleVar_PopupBorderSize, 1.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6, 6));
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(6, 4));
    ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(44, 44, 60, 255));
    ImGui::PushStyleColor(ImGuiCol_FrameBgHovered, IM_COL32(54, 54, 74, 255));
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 a = ImGui::GetCursorScreenPos();
    const ImVec2 b(a.x + width, a.y + ImGui::GetFrameHeight());
    const bool open = ImGui::BeginCombo(id, roleLabel(role), ImGuiComboFlags_HeightLarge | ImGuiComboFlags_NoArrowButton);
    ImGui::PopStyleColor(2);
    {
        const ImVec2 c(b.x - 16.0f, (a.y + b.y) * 0.5f);
        const float k = 4.5f;
        const ImVec2 pts[3] = {ImVec2(c.x - k, c.y - k * 0.5f), ImVec2(c.x, c.y + k * 0.5f), ImVec2(c.x + k, c.y - k * 0.5f)};
        dl->AddPolyline(pts, 3, sty(col::textDim), 0, 2.0f);
    }
    if (open) {
        for (const RoleOption& r : kRoles) {
            const bool selected = role == r.value;
            if (ImGui::Selectable(r.label, selected, 0, ImVec2(0, 30)) && !selected) {
                role = r.value;
                changed = true;
            }
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
    }
    ImGui::PopStyleVar(4);
    ImGui::PopFont();
    return changed;
}

}  // namespace smt::ui
