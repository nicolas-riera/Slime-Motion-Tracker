// Preferences of the desktop app itself (not the engine settings), stored in
// %LOCALAPPDATA%\Slime Motion Tracker\app.json.
#pragma once

#include <string>

namespace smt {

struct AppPrefs {
    bool minimizeToTray = false;  // the minimize button hides the window in the notification area
    bool startInTray = false;  // start hidden in the notification area
    bool soundMuted = false;  // calibration countdown sounds
    float soundVolume = 0.6f;  // 0..1

    void load();
    void save() const;
};

// The app-wide instance (loaded at startup, saved when the UI changes it).
AppPrefs& appPrefs();

}  // namespace smt
