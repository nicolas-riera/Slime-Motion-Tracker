#include "battery_estimator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <nlohmann/json.hpp>

namespace smt::ui {

namespace {

constexpr double kWindow = 40 * 60;     // s of history used for the short-term rate
constexpr double kMaxGap = 10 * 60;     // s without readings: start over
constexpr double kChargeJump = 0.02;    // level rise in a minute: a new baseline (not charging by itself)
constexpr double kChargeRise = 0.003;   // rise per minute that counts toward charging / keeps it
constexpr int kChargeMinutes = 3;       // consecutive rising minutes that mean charging
constexpr double kMinLevel = 0.01;      // at or below: no battery reading (USB powered)
constexpr double kMinVoltage = 3.0;     // below (and > 0): no battery reading
constexpr double kSwapDrop = 0.06;      // level drop in a minute: another battery
constexpr int kSettleMinutes = 5;       // after a charge, the voltage relaxes
constexpr size_t kMinMinutes = 8;       // before the first short-term estimate
constexpr double kMaxHours = 99;
// Learning: from this many minutes of short-term history on (a steadier slope), each
// minute adds to its band. A band remembers about kBandMemory of powered time (older
// minutes fade: the battery ages); an unlearnt band leans on the tracker's average pace
// as if it had kBandPrior of it
constexpr size_t kLearnMinutes = 20;
constexpr double kBandMemory = 10 * 3600;
constexpr double kBandPrior = 30 * 60;
constexpr double kMinLearnt = 60 * 60;  // learnt powered time before the profile is used
// The current session's pace scales the learnt one by at most this factor, with a weight
// growing to kSessionWeight over kSessionMinutes of short-term history
constexpr double kSessionMaxFactor = 2.0;
constexpr double kSessionWeight = 0.5;
constexpr double kSessionMinutes = 120;
constexpr double kMinRate = 0.002 / 3600;  // level/s: never slower than 0.2 %/h

int bandOf(double level) { return std::clamp(int(level * BatteryEstimator::kBands), 0, BatteryEstimator::kBands - 1); }

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
        h.lastReading = hj.value("last", 0.0);
        // Charging is re-detected from fresh readings (an older version flagged single jumps)
        h.charging = false;
        h.settle = 0;
        if (hj.contains("minutes") && hj["minutes"].is_array())
            for (const auto& m : hj["minutes"])
                if (m.is_array() && m.size() == 2 && m[0].is_number() && m[1].is_number())
                    h.minutes.push_back({m[0].get<double>(), m[1].get<double>()});
        if (!h.minutes.empty()) h.lastLevel = h.minutes.back().level;
        m_history[mac] = std::move(h);
        if (hj.contains("profile") && hj["profile"].is_array() && hj["profile"].size() == kBands) {
            Profile& p = m_profiles[mac];
            for (int b = 0; b < kBands; b++) {
                const auto& bj = hj["profile"][b];
                if (bj.is_array() && bj.size() == 2 && bj[0].is_number() && bj[1].is_number())
                    p[b] = {bj[0].get<double>(), bj[1].get<double>()};
            }
        }
    }
}

void BatteryEstimator::save() {
    if (!m_dirty || m_path.empty()) return;
    m_dirty = false;
    nlohmann::json j = nlohmann::json::object();
    for (const auto& [mac, h] : m_history) {
        nlohmann::json minutes = nlohmann::json::array();
        for (const Minute& m : h.minutes) minutes.push_back({m.t, m.level});
        j[mac] = {{"last", h.lastReading},
                  {"charging", h.charging},
                  {"settle", h.settle},
                  {"minutes", minutes}};
        if (auto it = m_profiles.find(mac); it != m_profiles.end()) {
            nlohmann::json profile = nlohmann::json::array();
            for (const Band& b : it->second) profile.push_back({b.time, b.drop});
            j[mac]["profile"] = profile;
        }
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

void BatteryEstimator::add(const std::string& mac, double level, double voltage, double now) {
    History& h = m_history[mac];
    if (now - h.lastReading < 1.0) return;  // one reading a second is plenty
    if (!plausible(level, voltage)) {
        // USB powered or no battery sense: not a reading. The next real reading starts
        // a new history (the jump from 0 % is not a charge)
        if (!h.minutes.empty() || !h.pending.empty() || h.charging) {
            h = History{};
            m_dirty = true;
        }
        h.lastReading = now;
        h.minuteStart = 0;
        return;
    }
    if (now - h.lastReading > kMaxGap) {
        h = History{};
        m_dirty = true;
    }
    h.lastReading = now;
    h.lastLevel = level;
    if (h.minuteStart == 0) h.minuteStart = now;
    h.pending.push_back(level);
    if (now - h.minuteStart < 60) return;
    std::nth_element(h.pending.begin(), h.pending.begin() + h.pending.size() / 2, h.pending.end());
    const double median = h.pending[h.pending.size() / 2];
    h.pending.clear();
    h.minuteStart = now;
    closeMinute(h, m_profiles[mac], now, median);
    m_dirty = true;
}

void BatteryEstimator::closeMinute(History& h, Profile& p, double now, double level) {
    const Minute cur{now, level};
    if (!h.minutes.empty()) {
        const double rise = level - h.minutes.back().level;
        h.riseStreak = rise > kChargeRise ? h.riseStreak + 1 : 0;
        // Charging: a sustained rise, never a single jump
        if (h.riseStreak >= kChargeMinutes || (h.charging && rise > kChargeRise)) {
            h.charging = true;
            h.settle = kSettleMinutes;
            h.minutes = {cur};
            return;
        }
        if (rise > kChargeJump) {
            // One jump (another battery, the reading recovering): new baseline
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

    // Long-term profile: this minute's share of the steady short-term slope. The slope's
    // noise (a step more or less in the window) averages out over the hours, so it is
    // not clipped at zero
    Fit f;
    if (h.minutes.size() < kLearnMinutes || !fit(h, f)) return;
    Band& b = p[bandOf(f.level)];
    b.time += 60;
    b.drop += f.rate * 60;
    if (b.time > kBandMemory) {
        const double k = kBandMemory / b.time;
        b.time *= k;
        b.drop *= k;
    }
}

bool BatteryEstimator::fit(const History& h, Fit& out) {
    if (h.minutes.size() < 2) return false;
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
    if (stt <= 0) return false;
    out.rate = -stl / stt;
    // Level now: the fitted line at the last minute (less noisy than one median)
    out.level = std::clamp(ml + stl / stt * (h.minutes.back().t - mt), 0.0, 1.0);
    return true;
}

bool BatteryEstimator::plausible(double level, double voltage) {
    return level > kMinLevel && !(voltage > 0.0 && voltage < kMinVoltage);
}

double BatteryEstimator::learntTime(const std::string& mac) const {
    auto it = m_profiles.find(mac);
    if (it == m_profiles.end()) return 0;
    double t = 0;
    for (const Band& b : it->second) t += b.time;
    return t;
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
    Fit f{};
    const bool shortTerm = h.minutes.size() >= kMinMinutes && fit(h, f);

    auto pit = m_profiles.find(mac);
    double learnt = 0, dropSum = 0;
    if (pit != m_profiles.end())
        for (const Band& b : pit->second) {
            learnt += b.time;
            dropSum += b.drop;
        }
    if (learnt >= kMinLearnt && dropSum > 0 && h.lastLevel >= 0) {
        // Learnt paces, unlearnt bands leaning on the tracker's average one
        const double average = dropSum / learnt;
        std::array<double, kBands> rate{};
        for (int b = 0; b < kBands; b++) {
            const Band& band = pit->second[b];
            rate[b] = std::max(kMinRate, (band.drop + average * kBandPrior) / (band.time + kBandPrior));
        }
        const double level = shortTerm ? f.level : h.lastLevel;
        double seconds = 0;
        for (int b = 0; b <= bandOf(level); b++) {
            const double lo = double(b) / kBands, hi = std::min(level, double(b + 1) / kBands);
            if (hi > lo) seconds += (hi - lo) / rate[b];
        }
        // This session against the profile (power save, signal): a bounded, gradual say
        if (shortTerm && h.minutes.size() >= kLearnMinutes) {
            const double factor =
                std::clamp(std::max(f.rate, kMinRate) / rate[bandOf(level)], 1.0 / kSessionMaxFactor, kSessionMaxFactor);
            const double w = kSessionWeight * std::min(1.0, double(h.minutes.size()) / kSessionMinutes);
            seconds /= 1.0 + w * (factor - 1.0);
        }
        e.state = State::Ok;
        e.hours = std::min(kMaxHours, seconds / 3600.0);
        return e;
    }

    if (!shortTerm) return e;
    const double perHour = f.rate * 3600.0;
    e.state = State::Ok;
    e.hours = perHour > f.level / kMaxHours ? f.level / perHour : kMaxHours;
    return e;
}

}  // namespace smt::ui
