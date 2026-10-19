// Remaining battery time per tracker, from its discharge rate.
//
// The level reported by the trackers comes from the battery voltage: it is noisy,
// sags under load and jumps when a charger is plugged in or removed. The readings
// are reduced to one median per minute and the discharge rate is the least-squares
// slope of the last 40 minutes. The history restarts when the level climbs
// (charging, another battery), after a charge (the voltage relaxes for a few
// minutes once the charger is removed) and after a gap (tracker off, it may have
// been charged meanwhile). It is saved to a file so an overlay restart keeps it.
#pragma once

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

    void load(const std::string& path);
    void save();
    // level 0..1, voltage in V (0 = unknown); now: wall-clock seconds, so the saved
    // history stays valid. Implausible readings (USB powered: ~0 %, < 3 V) are ignored.
    void add(const std::string& mac, double level, double voltage, double now);
    // A real battery reading (not USB power / no battery sense)
    [[nodiscard]] static bool plausible(double level, double voltage);
    [[nodiscard]] Estimate estimate(const std::string& mac) const;

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
        bool charging = false;
        int settle = 0;  // minutes still discarded after a charge
        int riseStreak = 0;  // consecutive minutes with a rising level
    };
    void closeMinute(History& h, double now, double level);

    std::map<std::string, History> m_history;
    std::string m_path;
    bool m_dirty = false;
};

}  // namespace smt::ui
