// Dear ImGui user interface of the overlay (desktop window and SteamVR dashboard).
#pragma once

#include "driver_link.h"

namespace smt::ui {

// Size of the SteamVR dashboard overlay texture. The desktop window renders at its own
// (resizable) client size.
inline constexpr int kWidth = 1024;
inline constexpr int kHeight = 768;
// Smallest desktop client area
inline constexpr int kMinWidth = 640;
inline constexpr int kMinHeight = 500;

// Loads the fonts, applies the style and loads the battery history. Call once after
// ImGui::CreateContext().
void setup();
// Saves what must survive a restart (battery history)
void shutdown();

struct FrameInfo {
    bool vrActive = false;  // OpenVR initialized and the dashboard overlay exists
    bool standalone = false;  // the engine runs in the standalone host (no SteamVR)
};

// Every loop, rendered or not: battery history, countdown sounds and the 3D view
// requests to the engine. rendering: draw() runs this loop (window or dashboard shown).
void update(DriverLink& link, bool rendering);

// Builds the whole UI for this frame (between ImGui::NewFrame() and ImGui::Render()).
void draw(DriverLink& link, const FrameInfo& info);

}  // namespace smt::ui
