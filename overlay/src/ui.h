// Dear ImGui user interface of the overlay (desktop window and SteamVR dashboard).
#pragma once

#include "driver_link.h"

namespace slimelite::ui {

// Size of the UI surface in pixels (offscreen texture, desktop client area, VR overlay).
inline constexpr int kWidth = 1024;
inline constexpr int kHeight = 768;

// Loads the fonts and applies the style. Call once after ImGui::CreateContext().
void setup();

struct FrameInfo {
    bool vrActive = false;  // OpenVR initialized and the dashboard overlay exists
};

// Builds the whole UI for this frame (between ImGui::NewFrame() and ImGui::Render()).
void draw(DriverLink& link, const FrameInfo& info);

}  // namespace slimelite::ui
