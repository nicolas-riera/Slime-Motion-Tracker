// Remaining battery time per tracker, from its discharge rate.
//
// The level reported by the trackers comes from the battery voltage (the firmware's
// piecewise LiPo curve): it is noisy, sags under load, jumps when a charger is plugged in
// or removed, and moves by 0.4 % steps while the trackers use 2-5 %/h. The readings are
// reduced to one median per minute; the short-term rate is the least-squares slope of
// the last 40 minutes, restarted when the level climbs (charging, another battery), after
// a charge (the voltage relaxes for a few minutes once the charger is removed) and after
// a gap (tracker off, it may have been charged meanwhile).
//
// Forty minutes are a handful of 0.4 % steps, and the curve's percent does not fall at
// the same pace everywhere: the estimate jumped around and took 8 minutes to appear. So
// each tracker also learns, over all its sessions, how fast its level falls in each 10 %
// band (the short-term slope, minute by minute, while powered), forgetting slowly as the
// battery ages. The remaining time adds the bands' learnt paces down to empty, scaled by
// how much faster or slower the current session runs. Everything is saved to a file.
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace smt::ui {

class BatteryEstimator {
public:
    enum class State { Estimating, Charging, Ok };
    struct Estimate {
        State state = State::Estimating;
        double hours = 0;  // remaining, when Ok
    };
    static constexpr int kBands = 10;

    void load(const std::string& path);
    void save();
    // level 0..1, voltage in V (0 = unknown); now: wall-clock seconds, so the saved
    // history stays valid. Implausible readings (USB powered: ~0 %, < 3 V) are ignored.
    void add(const std::string& mac, double level, double voltage, double now);
    // A real battery reading (not USB power / no battery sense)
    [[nodiscard]] static bool plausible(double level, double voltage);
    [[nodiscard]] Estimate estimate(const std::string& mac) const;
    // Powered time learnt (s), all bands
    [[nodiscard]] double learntTime(const std::string& mac) const;

private:
    struct Minute {
        double t;
        double level;
    };
    struct History {
        std::vector<Minute> minutes;
        std::vector<double> pending;  // readings of the current minute
        double minuteStart = 0;
        double lastReading = 0;
        double lastLevel = -1;  // latest plausible reading
        bool charging = false;
        int settle = 0;  // minutes still discarded after a charge
        int riseStreak = 0;  // consecutive minutes with a rising level
    };
    // Per 10 % band: powered time (s) and the level lost meanwhile
    struct Band {
        double time = 0;
        double drop = 0;
    };
    using Profile = std::array<Band, kBands>;
    struct Fit {
        double rate;   // level lost per second
        double level;  // fitted level at the last minute
    };
    void closeMinute(History& h, Profile& p, double now, double level);
    static bool fit(const History& h, Fit& out);

    std::map<std::string, History> m_history;
    std::map<std::string, Profile> m_profiles;
    std::string m_path;
    bool m_dirty = false;
};

}  // namespace smt::ui
