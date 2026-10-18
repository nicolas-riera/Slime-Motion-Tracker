#include "recorder.h"

#include <chrono>
#include <cstdarg>
#include <ctime>
#include <filesystem>
#include <vector>

#include "slimelite/config.h"

namespace slimelite {

Recorder::~Recorder() { close(); }

bool Recorder::open(const std::string& path) {
	std::lock_guard lock(m_mutex);
	if (m_open) return true;
	std::string p = path;
	if (p.empty()) {
		auto dir = Config::defaultPath().parent_path() / "recordings";
		std::error_code ec;
		std::filesystem::create_directories(dir, ec);
		std::time_t now = std::time(nullptr);
		std::tm tm{};
		localtime_s(&tm, &now);
		char name[64];
		std::strftime(name, sizeof(name), "session_%Y%m%d_%H%M%S.jsonl", &tm);
		p = (dir / name).string();
	}
	m_file = std::fopen(p.c_str(), "ab");
	if (!m_file) return false;
	std::setvbuf(m_file, nullptr, _IOFBF, 1 << 20);
	m_path = p;
	std::error_code ec;
	m_bytes = std::filesystem::exists(p, ec) ? std::filesystem::file_size(p, ec) : 0;
	m_pending.clear();
	m_stop = false;
	m_open = true;
	m_writer = std::thread(&Recorder::writerLoop, this);
	return true;
}

void Recorder::close() {
	{
		std::lock_guard lock(m_mutex);
		if (!m_open) return;
		m_open = false;
		m_stop = true;
	}
	m_wake.notify_all();
	if (m_writer.joinable()) m_writer.join();
	std::fclose(m_file);
	m_file = nullptr;
}

void Recorder::writerLoop() {
	std::string batch;
	for (;;) {
		bool stop;
		{
			std::unique_lock lock(m_mutex);
			m_wake.wait_for(lock, std::chrono::seconds(1), [&] { return m_stop || m_pending.size() > (1u << 20); });
			batch.swap(m_pending);
			stop = m_stop;
			if (m_dropped > 0) {
				char note[96];
				int n = std::snprintf(note, sizeof(note), R"({"k":"ev","what":"recording_dropped","detail":"%llu"})" "\n",
									  static_cast<unsigned long long>(m_dropped));
				batch.append(note, size_t(n));
				m_dropped = 0;
			}
		}
		if (!batch.empty()) {
			std::fwrite(batch.data(), 1, batch.size(), m_file);
			std::fflush(m_file);
			batch.clear();
		}
		if (stop) return;
	}
}

std::string Recorder::path() const {
	std::lock_guard lock(m_mutex);
	return m_path;
}

uint64_t Recorder::bytes() const {
	std::lock_guard lock(m_mutex);
	return m_bytes;
}

void Recorder::line(const char* fmt, ...) {
	if (!m_open) return;
	char small[512];
	std::vector<char> large;
	char* text = small;
	va_list args;
	va_start(args, fmt);
	int n = std::vsnprintf(small, sizeof(small), fmt, args);
	va_end(args);
	if (n < 0) return;
	if (size_t(n) >= sizeof(small)) {
		large.resize(size_t(n) + 1);
		va_start(args, fmt);
		std::vsnprintf(large.data(), large.size(), fmt, args);
		va_end(args);
		text = large.data();
	}
	std::lock_guard lock(m_mutex);
	if (!m_open) return;
	if (m_pending.size() > kMaxPending) {
		m_dropped++;
		return;
	}
	m_pending.append(text, size_t(n));
	m_pending.push_back('\n');
	m_bytes += uint64_t(n) + 1;
}

void Recorder::json(double t, const char* kind, const nlohmann::json& body) {
	nlohmann::json j = body;
	j["t"] = t;
	j["k"] = kind;
	line("%s", j.dump().c_str());
}

void Recorder::trackerEvent(const std::string& mac, const proto::Event& e, double t) {
	using K = proto::Event::Kind;
	const char* m = mac.c_str();
	switch (e.kind) {
		case K::Handshake:
			line(R"({"t":%.6f,"k":"hs","mac":"%s","fw":"%s","board":%d,"imu":%d,"proto":%d})", t, m,
				 e.firmware.c_str(), e.board, e.imuType, e.protocolVersion);
			break;
		case K::Rotation:
			line(R"({"t":%.6f,"k":"rot","mac":"%s","s":%d,"q":[%.7f,%.7f,%.7f,%.7f]})", t, m, e.sensorId,
				 e.rotation.w, e.rotation.x, e.rotation.y, e.rotation.z);
			break;
		case K::Accel:
			line(R"({"t":%.6f,"k":"acc","mac":"%s","s":%d,"a":[%.5f,%.5f,%.5f]})", t, m, e.sensorId,
				 e.accel.x, e.accel.y, e.accel.z);
			break;
		case K::Mag:
			line(R"({"t":%.6f,"k":"mag","mac":"%s","s":%d,"m":[%d,%d,%d]})", t, m, e.sensorId, e.mag[0],
				 e.mag[1], e.mag[2]);
			break;
		case K::Battery:
			line(R"({"t":%.6f,"k":"bat","mac":"%s","v":%.3f,"p":%.3f})", t, m, e.batteryVoltage,
				 e.batteryLevel);
			break;
		case K::Temperature:
			line(R"({"t":%.6f,"k":"tmp","mac":"%s","c":%.2f})", t, m, e.temperature);
			break;
		default:
			break;
	}
}

}  // namespace slimelite
