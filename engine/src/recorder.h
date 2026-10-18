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
//   tmp  IMU temperature: c (deg C)
//   dev  SteamVR device pose (raw tracking space): i, cls, role, ser, p, q, v, w
//   trk  tracker state from the engine at 10 Hz: mac, hdg (rad), hh, ms (mag state)
//   out  engine outputs per frame: waist/lf/rf = [valid, x, y, z, qw, qx, qy, qz],
//        pl (planted), kn (knee angles, deg)
//   cfg  full config (roles, mountings, mag calibrations, settings)
//   ev   events: calibration phases, hot reloads, setting changes
//
// Lines are queued in memory and written by a background thread: a slow disk
// never blocks the SteamVR frame or the tracker packets (a synchronous write
// froze the tracking for seconds while another program filled the same disk).
// Past kMaxPending queued bytes, lines are dropped instead.
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>

#include "slimelite/protocol.h"

namespace slimelite {

class Recorder {
public:
	~Recorder();

	// Opens (appends to) path, or a new session file when path is empty
	bool open(const std::string& path = {});
	void close();
	bool isOpen() const { return m_open; }
	std::string path() const;
	uint64_t bytes() const;

	void trackerEvent(const std::string& mac, const proto::Event& e, double t);
	void line(const char* fmt, ...);
	void json(double t, const char* kind, const nlohmann::json& body);

private:
	static constexpr size_t kMaxPending = 256u << 20;
	void writerLoop();

	mutable std::mutex m_mutex;  // everything below but the file
	std::condition_variable m_wake;
	std::string m_pending;
	bool m_stop = false;
	std::atomic<bool> m_open{false};
	std::string m_path;
	uint64_t m_bytes = 0;
	uint64_t m_dropped = 0;
	FILE* m_file = nullptr;  // written by the writer thread only while open
	std::thread m_writer;
};

}  // namespace slimelite
