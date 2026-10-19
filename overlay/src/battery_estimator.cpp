#include "battery_estimator.h"

#include <algorithm>
#include <cstdio>
#include <fstream>

#include <nlohmann/json.hpp>

namespace smt::ui {

namespace {

constexpr double kWindow = 40 * 60;     // s of history used for the rate
constexpr double kMaxGap = 10 * 60;     // s without readings: start over
constexpr double kChargeJump = 0.02;    // level rise in a minute that means charging
constexpr double kChargeRise = 0.003;   // still rising: still charging
constexpr double kSwapDrop = 0.06;      // level drop in a minute: another battery
constexpr int kSettleMinutes = 5;       // after a charge, the voltage relaxes
constexpr size_t kMinMinutes = 8;       // before the first estimate
constexpr double kMaxHours = 99;

}  // namespace

void BatteryEstimator::load(const std::string& path) {
    m_path = path;
    std::ifstream in(path);
    if (!in) return;
    nlohmann::json j = nlohmann::json::parse(in, nullptr, false);
    if (!j.is_object()) return;
    for (const auto& [mac, hj] : j.items()) {
        if (!hj.is_object()) continue;
        History h;
        h.transport = hj.value("transport", "");
        h.lastReading = hj.value("last", 0.0);
        h.charging = hj.value("charging", false);
        h.settle = hj.value("settle", 0);
        if (hj.contains("minutes") && hj["minutes"].is_array())
            for (const auto& m : hj["minutes"])
                if (m.is_array() && m.size() == 2 && m[0].is_number() && m[1].is_number())
                    h.minutes.push_back({m[0].get<double>(), m[1].get<double>()});
        m_history[mac] = std::move(h);
    }
}

void BatteryEstimator::save() {
    if (!m_dirty || m_path.empty()) return;
    m_dirty = false;
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [mac, h] : m_history) {
        nlohmann::json minutes = nlohmann::json::array();
        for (const Minute& m : h.minutes) minutes.push_back({m.t, m.level});
        j[mac] = {{"transport", h.transport},
                  {"last", h.lastReading},
                  {"charging", h.charging},
                  {"settle", h.settle},
                  {"minutes", minutes}};
    }
    const std::string tmp = m_path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) return;
        out << j.dump();
    }
    std::remove(m_path.c_str());
    std::rename(tmp.c_str(), m_path.c_str());
}

void BatteryEstimator::add(const std::string& mac, const std::string& transport, double level, double now) {
    History& h = m_history[mac];
    if (now - h.lastReading < 1.0) return;  // one reading a second is plenty
    if (now - h.lastReading > kMaxGap || transport != h.transport) {
        h = History{};
        h.transport = transport;
        m_dirty = true;
    }
    h.lastReading = now;
    if (h.minuteStart == 0) h.minuteStart = now;
    h.pending.push_back(level);
    if (now - h.minuteStart < 60) return;
    std::nth_element(h.pending.begin(), h.pending.begin() + h.pending.size() / 2, h.pending.end());
    const double median = h.pending[h.pending.size() / 2];
    h.pending.clear();
    h.minuteStart = now;
    closeMinute(h, now, median);
    m_dirty = true;
}

void BatteryEstimator::closeMinute(History& h, double now, double level) {
    const Minute cur{now, level};
    if (!h.minutes.empty()) {
        const double rise = level - h.minutes.back().level;
        if (rise > kChargeJump || (h.charging && rise > kChargeRise)) {
            h.charging = true;
            h.settle = kSettleMinutes;
            h.minutes = {cur};
            return;
        }
        if (rise < -kSwapDrop) {
            h.minutes = {cur};
            return;
        }
    }
    if (h.settle > 0) {
        // Charger removed (or full): wait for the voltage to settle
        h.settle--;
        h.minutes = {cur};
        if (h.settle == 0) h.charging = false;
        return;
    }
    h.minutes.push_back(cur);
    h.minutes.erase(std::remove_if(h.minutes.begin(), h.minutes.end(),
                                   [&](const Minute& m) { return now - m.t > kWindow; }),
                    h.minutes.end());
}

BatteryEstimator::Estimate BatteryEstimator::estimate(const std::string& mac) const {
    Estimate e;
    auto it = m_history.find(mac);
    if (it == m_history.end()) return e;
    const History& h = it->second;
    if (h.charging) {
        e.state = State::Charging;
        return e;
    }
    if (h.minutes.size() < kMinMinutes) return e;
    // Least-squares slope (level per second)
    double mt = 0, ml = 0;
    for (const Minute& m : h.minutes) {
        mt += m.t;
        ml += m.level;
    }
    mt /= double(h.minutes.size());
    ml /= double(h.minutes.size());
    double stl = 0, stt = 0;
    for (const Minute& m : h.minutes) {
        stl += (m.t - mt) * (m.level - ml);
        stt += (m.t - mt) * (m.t - mt);
    }
    if (stt <= 0) return e;
    const double perHour = -stl / stt * 3600.0;
    // Level now: the fitted line at the last minute (less noisy than one median)
    const double level = std::clamp(ml + stl / stt * (h.minutes.back().t - mt), 0.0, 1.0);
    e.state = State::Ok;
    e.hours = perHour > level / kMaxHours ? level / perHour : kMaxHours;
    return e;
}

}  // namespace smt::ui
