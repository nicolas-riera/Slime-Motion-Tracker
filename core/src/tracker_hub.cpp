#include "slimelite/tracker_hub.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>

#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif

#include <algorithm>
#include <chrono>

namespace slimelite {

const char* magStateName(MagState s) {
	switch (s) {
		case MagState::Absent: return "absent";
		case MagState::Disabled: return "disabled";
		case MagState::Uncalibrated: return "uncalibrated";
		case MagState::Calibrating: return "calibrating";
		case MagState::Ok: return "ok";
		case MagState::Disturbed: return "disturbed";
	}
	return "absent";
}

struct TrackerHub::Tracker {
	TrackerSnapshot snap;
	sockaddr_in addr{};
	double lastPacketTime = 0;

	Quat prevRotation;
	double prevRotationTime = 0;
	bool hasMagData = false;
	double lastMagTime = 0;

	MagCalibrator calibrator;
	HeadingFilter heading;
	bool guided = false;
	double guidedStart = 0;
	double lastOnlineSolve = 0;

	// Rate counters
	int rotCount = 0, magCount = 0;
	double rateWindowStart = 0;
};

double TrackerHub::now() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

TrackerHub::TrackerHub() = default;

TrackerHub::~TrackerHub() { stop(); }

bool TrackerHub::start(uint16_t port) {
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
	SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(port);
	a.sin_addr.s_addr = htonl(INADDR_ANY);
	// Exclusive bind: fail loudly if the SlimeVR server is already running
	BOOL excl = TRUE;
	setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&excl), sizeof(excl));
	if (bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
		closesocket(s);
		std::lock_guard lock(m_mutex);
		m_portError = "UDP port " + std::to_string(port)
					+ " is in use: close the SlimeVR server, then restart SteamVR";
		return false;
	}
	// Windows reports ICMP port unreachable as recv errors on UDP sockets: disable
	DWORD noReset = FALSE;
	DWORD bytes = 0;
	WSAIoctl(s, SIO_UDP_CONNRESET, &noReset, sizeof(noReset), nullptr, 0, &bytes, nullptr, nullptr);
	DWORD timeoutMs = 10;
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeoutMs), sizeof(timeoutMs));

	m_socket = s;
	m_running = true;
	m_thread = std::thread(&TrackerHub::networkLoop, this);
	return true;
}

void TrackerHub::stop() {
	if (!m_running.exchange(false)) return;
	if (m_thread.joinable()) m_thread.join();
	closesocket(static_cast<SOCKET>(m_socket));
	m_socket = ~uintptr_t(0);
}

std::string TrackerHub::portError() const {
	std::lock_guard lock(m_mutex);
	return m_portError;
}

std::vector<TrackerSnapshot> TrackerHub::snapshot() const {
	std::lock_guard lock(m_mutex);
	std::vector<TrackerSnapshot> out;
	for (const auto& [mac, t] : m_trackers) out.push_back(t->snap);
	return out;
}

void TrackerHub::setMagSettings(bool enabled, const HeadingFilterSettings& s) {
	std::lock_guard lock(m_mutex);
	m_magEnabled = enabled;
	m_headingSettings = s;
}

void TrackerHub::onMagCalibration(MagCalibrationCallback cb) {
	std::lock_guard lock(m_mutex);
	m_magCallback = std::move(cb);
}

void TrackerHub::setStoredMagCalibration(const std::string& mac, const MagCalibration& cal) {
	std::lock_guard lock(m_mutex);
	m_storedMag[mac] = cal;
	auto it = m_trackers.find(mac);
	if (it != m_trackers.end() && cal.valid) it->second->heading.setCalibration(cal);
}

void TrackerHub::startGuidedMagCalibration(const std::string& mac) {
	std::lock_guard lock(m_mutex);
	auto it = m_trackers.find(mac);
	if (it == m_trackers.end()) return;
	it->second->calibrator.reset();
	it->second->guided = true;
	it->second->guidedStart = now();
}

void TrackerHub::cancelGuidedMagCalibration(const std::string& mac) {
	std::lock_guard lock(m_mutex);
	auto it = m_trackers.find(mac);
	if (it != m_trackers.end()) it->second->guided = false;
}

void TrackerHub::resetMagCalibration(const std::string& mac) {
	MagCalibrationCallback cb;
	{
		std::lock_guard lock(m_mutex);
		m_storedMag.erase(mac);
		auto it = m_trackers.find(mac);
		if (it != m_trackers.end()) {
			it->second->heading.setCalibration(MagCalibration{});
			it->second->calibrator.reset();
		}
		cb = m_magCallback;
	}
	if (cb) cb(mac, MagCalibration{});
}

TrackerHub::Tracker& TrackerHub::trackerFor(const std::string& mac) {
	auto& p = m_trackers[mac];
	if (!p) {
		p = std::make_unique<Tracker>();
		p->snap.mac = mac;
		auto stored = m_storedMag.find(mac);
		if (stored != m_storedMag.end() && stored->second.valid)
			p->heading.setCalibration(stored->second);
	}
	return *p;
}

void TrackerHub::processEvent(const std::string& mac, const proto::Event& e, double t) {
	std::optional<MagCalibration> found;
	MagCalibrationCallback cb;
	{
		std::lock_guard lock(m_mutex);
		Tracker& tr = trackerFor(mac);
		TrackerSnapshot& s = tr.snap;
		tr.lastPacketTime = t;
		s.connected = true;

		switch (e.kind) {
			case proto::Event::Kind::Rotation: {
				if (e.sensorId != 0) break;
				Quat q = proto::firmwareToWorld(e.rotation);
				if (tr.prevRotationTime > 0) {
					double dt = t - tr.prevRotationTime;
					if (dt > 1e-4 && dt < 0.5) {
						double ang = (tr.prevRotation.conj() * q).angle();
						double w = ang / dt;
						s.angularSpeed += (w - s.angularSpeed) * std::min(1.0, dt / 0.05);
						double act = std::min(1.0, w / 6.0);
						s.activity += (act - s.activity) * std::min(1.0, dt / 0.3);
					}
				}
				tr.prevRotation = q;
				tr.prevRotationTime = t;
				s.qWorld = q;
				s.hasRotation = true;
				s.lastRotationTime = t;
				tr.rotCount++;
				break;
			}
			case proto::Event::Kind::Accel:
				if (e.sensorId == 0) s.accelLocal = e.accel;
				break;
			case proto::Event::Kind::Mag: {
				if (e.sensorId != 0 || !s.hasRotation) break;
				tr.hasMagData = true;
				tr.magCount++;
				double dt = tr.lastMagTime > 0 ? std::clamp(t - tr.lastMagTime, 0.0, 0.2) : 0.02;
				tr.lastMagTime = t;

				bool usable = tr.heading.state() != HeadingFilter::State::Disturbed;
				if (tr.guided || usable) tr.calibrator.addSample(s.qWorld, e.mag, s.angularSpeed, t);

				if (tr.guided) {
					s.magProgress = tr.calibrator.coverage();
					if (s.magProgress >= 0.85) {
						if (auto cal = tr.calibrator.solve()) {
							tr.heading.setCalibration(*cal);
							tr.guided = false;
							found = cal;
						}
					}
					if (t - tr.guidedStart > 180) tr.guided = false;
				} else if (t - tr.lastOnlineSolve > 20) {
					// Online refinement, only when clearly better than the current fit
					tr.lastOnlineSolve = t;
					if (auto cal = tr.calibrator.solve()) {
						const MagCalibration& cur = tr.heading.calibration();
						if (!cur.valid || cal->residual < 0.8 * cur.residual) {
							tr.heading.updateCalibration(*cal, s.qWorld, e.mag);
							found = tr.heading.calibration();
						}
					}
				}

				if (m_magEnabled) tr.heading.update(s.qWorld, e.mag, s.angularSpeed, dt, m_headingSettings);
				s.magNorm = tr.heading.lastNorm();
				s.magDip = rad2deg(tr.heading.lastDip());
				break;
			}
			case proto::Event::Kind::Battery:
				s.batteryVoltage = e.batteryVoltage;
				s.batteryLevel = e.batteryLevel;
				break;
			case proto::Event::Kind::Temperature:
				s.temperature = e.temperature;
				break;
			case proto::Event::Kind::Signal:
				s.rssi = e.rssi;
				break;
			default:
				break;
		}

		// Derived state
		s.magCalibration = tr.heading.calibration();
		s.hasHeading = m_magEnabled && tr.heading.hasHeading();
		s.heading = tr.heading.heading();
		if (!tr.hasMagData) s.magState = MagState::Absent;
		else if (tr.guided) s.magState = MagState::Calibrating;
		else if (!m_magEnabled) s.magState = MagState::Disabled;
		else if (!s.magCalibration.valid) s.magState = MagState::Uncalibrated;
		else if (tr.heading.state() == HeadingFilter::State::Disturbed) s.magState = MagState::Disturbed;
		else if (tr.heading.state() == HeadingFilter::State::Ok) s.magState = MagState::Ok;
		else s.magState = MagState::Uncalibrated;
		if (!tr.guided) s.magProgress = s.magCalibration.valid ? 1.0 : tr.calibrator.coverage();

		if (t - tr.rateWindowStart >= 1.0) {
			double w = t - tr.rateWindowStart;
			s.rotationHz = tr.rotCount / w;
			s.magHz = tr.magCount / w;
			tr.rotCount = tr.magCount = 0;
			tr.rateWindowStart = t;
		}

		if (found) m_storedMag[mac] = *found;
		cb = m_magCallback;
	}
	if (found && cb) cb(mac, *found);
}

void TrackerHub::networkLoop() {
	SOCKET s = static_cast<SOCKET>(m_socket);
	std::vector<uint8_t> buf(2048);
	double lastHeartbeat = 0;

	auto sendTo = [&](const sockaddr_in& addr, const std::vector<uint8_t>& data) {
		sendto(s, reinterpret_cast<const char*>(data.data()), int(data.size()), 0,
			   reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
	};

	while (m_running) {
		sockaddr_in from{};
		int fromLen = sizeof(from);
		int n = recvfrom(s, reinterpret_cast<char*>(buf.data()), int(buf.size()), 0,
						 reinterpret_cast<sockaddr*>(&from), &fromLen);
		double t = now();
		if (n > 0) {
			auto events = proto::parse(buf.data(), size_t(n));
			// Find the tracker by address; the handshake carries the MAC
			std::string mac;
			for (const auto& e : events) {
				if (e.kind == proto::Event::Kind::Handshake) {
					mac = e.mac;
					std::lock_guard lock(m_mutex);
					Tracker& tr = trackerFor(mac);
					bool isNew = !tr.snap.connected
							  || tr.addr.sin_addr.s_addr != from.sin_addr.s_addr
							  || tr.addr.sin_port != from.sin_port || t - tr.lastPacketTime > 1.0;
					tr.addr = from;
					char ip[INET_ADDRSTRLEN] = {};
					inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
					tr.snap.ip = ip;
					tr.snap.connected = true;
					tr.lastPacketTime = t;
					if (isNew) {
						tr.snap.bootCount++;
						tr.snap.hasRotation = false;
						tr.prevRotationTime = 0;
						// The device fusion restarts with an arbitrary yaw
						tr.heading.reset();
					}
					sendTo(from, proto::handshakeReply());
				}
			}
			if (mac.empty()) {
				std::lock_guard lock(m_mutex);
				for (const auto& [m, tr] : m_trackers) {
					if (tr->addr.sin_addr.s_addr == from.sin_addr.s_addr
						&& tr->addr.sin_port == from.sin_port) {
						mac = m;
						break;
					}
				}
			}
			if (!mac.empty()) {
				for (const auto& e : events) {
					if (e.kind == proto::Event::Kind::Handshake) continue;
					if (e.kind == proto::Event::Kind::SensorInfo)
						sendTo(from, proto::sensorInfoAck(e.sensorId, e.sensorState));
					if (e.kind == proto::Event::Kind::FeatureFlags)
						sendTo(from, proto::featureFlags(++m_packetNumber));
					processEvent(mac, e, t);
				}
			}
		}

		if (t - lastHeartbeat > 0.5) {
			lastHeartbeat = t;
			std::lock_guard lock(m_mutex);
			for (auto& [mac, tr] : m_trackers) {
				if (!tr->snap.connected) continue;
				if (t - tr->lastPacketTime > 5.0) {
					tr->snap.connected = false;
					tr->snap.rotationHz = tr->snap.magHz = 0;
					continue;
				}
				sendTo(tr->addr, proto::heartbeat(++m_packetNumber));
			}
		}
	}
}

}  // namespace slimelite
