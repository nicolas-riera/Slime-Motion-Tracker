// Runs the learned pose model (~4 ms per 60 Hz step) on its own thread so the
// frame loop never waits on it: each frame submits its inputs and reads the
// latest estimate (one step behind at most).
#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "slimelite/pose_model.h"

namespace slimelite {

class PoseWorker {
public:
	~PoseWorker() { stop(); }

	// Loads the model and starts the thread; false when the model cannot load
	bool start(const std::string& path);
	void stop();
	bool running() const { return m_thread.joinable(); }
	const std::string& error() const { return m_model.error(); }
	const std::string& name() const { return m_model.name(); }

	void setWristCalibration(const std::array<Quat, 2>& cal);
	void submit(const PoseModelInput& in, double now, double height);
	PoseEstimate latest() const;

private:
	void run();

	PoseModel m_model;  // touched by the worker thread only once started
	std::thread m_thread;
	mutable std::mutex m_mutex;
	std::condition_variable m_cv;
	bool m_stop = false, m_hasInput = false;
	PoseModelInput m_input;
	double m_inputTime = 0, m_height = 1.75;
	std::optional<std::array<Quat, 2>> m_pendingCal;
	PoseEstimate m_latest;
};

}  // namespace slimelite
