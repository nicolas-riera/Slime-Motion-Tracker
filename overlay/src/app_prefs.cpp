#include "app_prefs.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <type_traits>

#include <nlohmann/json.hpp>

namespace smt {

namespace {

std::filesystem::path prefsPath() {
    const wchar_t* appData = _wgetenv(L"LOCALAPPDATA");
    if (!appData) return {};
    return std::filesystem::path(appData) / L"Slime Motion Tracker" / L"app.json";
}

}  // namespace

AppPrefs& appPrefs() {
    static AppPrefs prefs;
    return prefs;
}

void AppPrefs::load() {
    const std::filesystem::path path = prefsPath();
    if (path.empty()) return;
    std::ifstream in(path);
    if (!in) return;
    const nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
    if (!j.is_object()) return;
    const auto get = [&](const char* key, auto def) {
        auto it = j.find(key);
        if (it == j.end()) return def;
        if constexpr (std::is_same_v<decltype(def), bool>)
            return it->is_boolean() ? it->get<bool>() : def;
        else
            return it->is_number() ? it->get<decltype(def)>() : def;
    };
    const AppPrefs d;
    minimizeToTray = get("minimizeToTray", d.minimizeToTray);
    startInTray = get("startInTray", d.startInTray);
    soundMuted = get("soundMuted", d.soundMuted);
    soundVolume = std::clamp(get("soundVolume", d.soundVolume), 0.0f, 1.0f);
}

void AppPrefs::save() const {
    const std::filesystem::path path = prefsPath();
    if (path.empty()) return;
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    const nlohmann::json j = {{"minimizeToTray", minimizeToTray},
                              {"startInTray", startInTray},
                              {"soundMuted", soundMuted},
                              {"soundVolume", std::round(soundVolume * 100.0f) / 100.0f}};
    // Write then rename, so a crash never leaves a truncated file
    const std::filesystem::path tmp = path.wstring() + L".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) return;
        out << j.dump(2);
        if (!out) return;
    }
    std::filesystem::rename(tmp, path, ec);
}

}  // namespace smt
