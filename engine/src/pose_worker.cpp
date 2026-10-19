#include "pose_worker.h"

namespace slimelite {

bool PoseWorker::start(const std::string& path) {
	stop();
	if (!m_model.load(path)) return false;
	m_stop = false;
	m_hasInput = false;
	m_thread = std::thread([this] { run(); });
	return true;
}

void PoseWorker::stop() {
	{
		std::lock_guard lock(m_mutex);
		m_stop = true;
	}
	m_cv.notify_all();
	if (m_thread.joinable()) m_thread.join();
}

void PoseWorker::setWristCalibration(const std::array<Quat, 2>& cal) {
	std::lock_guard lock(m_mutex);
	m_pendingCal = cal;
}

void PoseWorker::submit(const PoseModelInput& in, double now, double height) {
	{
		std::lock_guard lock(m_mutex);
		m_input = in;
		m_inputTime = now;
		m_height = height;
		m_hasInput = true;
	}
	m_cv.notify_one();
}

PoseEstimate PoseWorker::latest() const {
	std::lock_guard lock(m_mutex);
	return m_latest;
}

void PoseWorker::run() {
	for (;;) {
		PoseModelInput in;
		double now, height;
		std::optional<std::array<Quat, 2>> cal;
		{
			std::unique_lock lock(m_mutex);
			m_cv.wait(lock, [this] { return m_stop || m_hasInput; });
			if (m_stop) return;
			in = m_input;
			now = m_inputTime;
			height = m_height;
			cal.swap(m_pendingCal);
			m_hasInput = false;
		}
		if (cal) m_model.setWristCalibration(*cal);
		m_model.setHeight(height);
		PoseEstimate e = m_model.update(in, now);
		std::lock_guard lock(m_mutex);
		m_latest = e;
	}
}

}  // namespace slimelite
