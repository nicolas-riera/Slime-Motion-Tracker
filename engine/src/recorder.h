// Session recorder: every raw tracker packet, every SteamVR device pose (HMD,
// controllers), the engine outputs and the calibration context, as JSONL in
// %LOCALAPPDATA%\SlimeVR-lite\recordings. Used to replay sessions offline, tune
// the estimator and train models. Times are steady-clock seconds (same clock as
// the tracker hub).
//
// Line kinds ("k"):
//   hs   tracker handshake: mac, fw, board
//   rot  raw device fusion quaternion, firmware frame (Z up): q = [w, x, y, z]
//   acc  linear acceleration, tracker frame, m/s^2: a
//   mag  raw magnetometer counts: m
//   bat  battery: v (volts), p (0..1)
//   dev  SteamVR device pose (raw tracking space): i, cls, role, ser, p, q, v, w
//   trk  tracker state from the engine at 10 Hz: mac, hdg (rad), hh, ms (mag state)
//   out  engine outputs per frame: waist/lf/rf = [valid, x, y, z, qw, qx, qy, qz],
//        pl (planted), kn (knee angles, deg)
//   cfg  full config (roles, mountings, mag calibrations, settings)
//   ev   events: calibration phases, hot reloads, setting changes
#pragma once

#include <cstdio>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

#include "slimelite/protocol.h"

namespace slimelite {

class Recorder {
public:
	~Recorder();

	// Opens (appends to) path, or a new session file when path is empty
	bool open(const std::string& path = {});
	void close();
	bool isOpen() const { return m_file != nullptr; }
	std::string path() const;
	uint64_t bytes() const;

	void trackerEvent(const std::string& mac, const proto::Event& e, double t);
	void line(const char* fmt, ...);
	void json(double t, const char* kind, const nlohmann::json& body);
	void flushIfDue(double now);

private:
	mutable std::mutex m_mutex;
	FILE* m_file = nullptr;
	std::string m_path;
	uint64_t m_bytes = 0;
	double m_lastFlush = 0;
};

}  // namespace slimelite
