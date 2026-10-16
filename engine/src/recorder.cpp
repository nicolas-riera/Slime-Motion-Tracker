#include "recorder.h"

#include <cstdarg>
#include <ctime>
#include <filesystem>

#include "slimelite/config.h"

namespace slimelite {

Recorder::~Recorder() { close(); }

bool Recorder::open(const std::string& path) {
	std::lock_guard lock(m_mutex);
	if (m_file) return true;
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
	return true;
}

void Recorder::close() {
	std::lock_guard lock(m_mutex);
	if (m_file) std::fclose(m_file);
	m_file = nullptr;
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
	std::lock_guard lock(m_mutex);
	if (!m_file) return;
	va_list args;
	va_start(args, fmt);
	int n = std::vfprintf(m_file, fmt, args);
	va_end(args);
	std::fputc('\n', m_file);
	if (n > 0) m_bytes += uint64_t(n) + 1;
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
		default:
			break;
	}
}

void Recorder::flushIfDue(double now) {
	std::lock_guard lock(m_mutex);
	if (!m_file || now - m_lastFlush < 1.0) return;
	m_lastFlush = now;
	std::fflush(m_file);
}

}  // namespace slimelite
