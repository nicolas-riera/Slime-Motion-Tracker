// Look of the overlay UI: palette, fonts, hand-drawn icons (ImDrawList primitives,
// no icon font) and the custom widgets shared by the pages.
#pragma once

#include <string>

#include "imgui.h"

namespace smt::ui {

// ---------------------------------------------------------------- palette

namespace col {
constexpr ImU32 bg = IM_COL32(16, 16, 22, 255);
constexpr ImU32 sidebar = IM_COL32(11, 11, 16, 255);
constexpr ImU32 card = IM_COL32(26, 26, 35, 255);
constexpr ImU32 cardHover = IM_COL32(33, 33, 45, 255);
constexpr ImU32 line = IM_COL32(44, 44, 58, 255);
constexpr ImU32 field = IM_COL32(36, 36, 49, 255);
constexpr ImU32 text = IM_COL32(234, 234, 244, 255);
constexpr ImU32 textDim = IM_COL32(156, 156, 178, 255);
constexpr ImU32 textFaint = IM_COL32(100, 100, 122, 255);
constexpr ImU32 accent = IM_COL32(90, 76, 216, 255);  // #5a4cd8, the logo
constexpr ImU32 accentHover = IM_COL32(110, 97, 234, 255);
constexpr ImU32 accentSoft = IM_COL32(146, 134, 250, 255);
constexpr ImU32 accentDeep = IM_COL32(32, 27, 70, 255);
constexpr ImU32 green = IM_COL32(64, 206, 142, 255);
constexpr ImU32 orange = IM_COL32(245, 166, 50, 255);
constexpr ImU32 red = IM_COL32(240, 86, 92, 255);
constexpr ImU32 cyan = IM_COL32(64, 190, 245, 255);
constexpr ImU32 pink = IM_COL32(236, 102, 172, 255);
constexpr ImU32 white = IM_COL32(255, 255, 255, 255);
}  // namespace col

ImU32 alpha(ImU32 c, float a);  // multiplies the alpha
// Applies the style alpha (dimmed inside ImGui::BeginDisabled) to a raw color
inline ImU32 sty(ImU32 c) { return ImGui::GetColorU32(c); }
ImU32 mix(ImU32 a, ImU32 b, float t);
ImVec4 vec4(ImU32 c);

struct Fonts {
    ImFont* body = nullptr;  // regular text
    ImFont* bold = nullptr;  // labels, buttons
    ImFont* small = nullptr;  // secondary text
    ImFont* smallBold = nullptr;  // section titles
    ImFont* title = nullptr;  // page titles
    ImFont* big = nullptr;  // large values
};
extern Fonts fonts;

// Loads the fonts and applies the theme
void setupTheme();

// Value easing toward target, stored per id (hover fades, glows)
float animate(ImGuiID id, float target, float rate = 12.0f);

// ---------------------------------------------------------------- icons
// c: center, s: size of the square the icon fits in

using IconFn = void (*)(ImDrawList*, ImVec2, float, ImU32);

namespace icon {
void home(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void body(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void gear(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void calibrate(ImDrawList* dl, ImVec2 c, float s, ImU32 col);  // person in a scan frame
void reset(ImDrawList* dl, ImVec2 c, float s, ImU32 col);  // circular arrow
void tracker(ImDrawList* dl, ImVec2 c, float s, ImU32 col);  // small box
void dongle(ImDrawList* dl, ImVec2 c, float s, ImU32 col);  // USB stick
// bars: lit arcs 0..3, -1 = disconnected (all faint, slashed)
void wifi(ImDrawList* dl, ImVec2 c, float s, int bars, ImU32 on, ImU32 off);
// bars: lit bars 0..4
void signalBars(ImDrawList* dl, ImVec2 c, float s, int bars, ImU32 on, ImU32 off);
// level 0..1, < 0 unknown
void battery(ImDrawList* dl, ImVec2 c, float s, float level, bool charging, ImU32 outline, ImU32 fill);
void compass(ImDrawList* dl, ImVec2 c, float s, ImU32 ring, ImU32 needle);
void chevronRight(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void chevronLeft(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void check(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void cross(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void warning(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void info(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void speaker(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void thermometer(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
void clock(ImDrawList* dl, ImVec2 c, float s, ImU32 col);
// The app logo (accent rounded square, "SMT"), top-left corner p
void logo(ImDrawList* dl, ImVec2 p, float size);
}  // namespace icon

// ---------------------------------------------------------------- widgets

// Text in a given font and color at the cursor
void text(ImFont* font, ImU32 color, const char* fmt, ...) IM_FMTARGS(3);
void textWrapped(ImFont* font, ImU32 color, const char* fmt, ...) IM_FMTARGS(3);
// Text drawn at a position (no layout)
void textAt(ImDrawList* dl, ImFont* font, ImVec2 pos, ImU32 color, const char* text);
ImVec2 textSize(ImFont* font, const char* text);

// Sidebar entry: icon + label (icon only when collapsed)
bool navItem(const char* label, IconFn icon, bool active, bool collapsed, float width);

// Rounded card: a child window sized to its content (height 0) or fixed
void beginCard(const char* id, float height = 0.0f, ImU32 bg = col::card, float padding = 18.0f);
void endCard();
// Small dim title at the top of a card or section
void sectionTitle(const char* text);

enum class ButtonStyle { Primary, Secondary, Danger, Ghost };
bool button(const char* label, ImVec2 size, ButtonStyle style = ButtonStyle::Secondary, IconFn icon = nullptr);
// Slim slider over the available width. Returns true while the value changes;
// `released` is set when the drag ends (the moment to send it).
bool slider(const char* id, float* value, float vMin, float vMax, bool* released = nullptr);
// Horizontal bar 0..1
void progressBar(float fraction, float height, ImU32 fill);
// On/off switch, returns true when clicked
bool toggle(const char* id, bool* value);
// Label + optional help on the left, switch on the right
bool toggleRow(const char* label, const char* help, bool* value);
// Label on the left, value on the right (key/value lists)
void infoRow(const char* label, ImU32 valueColor, const char* fmt, ...) IM_FMTARGS(3);
void helpText(const char* text);
// Small rounded label
void pill(const char* text, ImU32 fg, ImU32 bg);
void separator(float spacing = 10.0f);

// Tracker roles
const char* roleLabel(const std::string& role);
// Dropdown of the roles; returns true and sets `role` when the user picks another one
bool roleCombo(const char* id, std::string& role, float width);

}  // namespace smt::ui
