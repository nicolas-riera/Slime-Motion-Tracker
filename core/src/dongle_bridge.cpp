#include "slimelite/dongle_bridge.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mstcpip.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cstdio>

namespace slimelite {

namespace dongle {

std::vector<uint8_t> encode(const Frame& f) {
	std::vector<uint8_t> out{0xA5, 0x5A, f.type, uint8_t(f.payload.size())};
	out.insert(out.end(), f.mac, f.mac + 6);
	out.insert(out.end(), f.payload.begin(), f.payload.end());
	uint8_t x = 0;
	for (size_t i = 2; i < out.size(); i++) x ^= out[i];
	out.push_back(x);
	return out;
}

std::vector<Frame> Decoder::feed(const uint8_t* data, size_t n) {
	std::vector<Frame> frames;
	m_buf.insert(m_buf.end(), data, data + n);
	size_t i = 0;
	while (m_buf.size() - i >= 4) {
		if (m_buf[i] != 0xA5 || m_buf[i + 1] != 0x5A) {
			i++;
			continue;
		}
		size_t len = m_buf[i + 3], total = 4 + 6 + len + 1;
		if (m_buf.size() - i < total) break;
		uint8_t x = 0;
		for (size_t k = i + 2; k < i + total - 1; k++) x ^= m_buf[k];
		if (x != m_buf[i + total - 1]) {
			i++;  // not a frame (or corrupted): resynchronize
			continue;
		}
		Frame f;
		f.type = m_buf[i + 2];
		std::copy(m_buf.begin() + i + 4, m_buf.begin() + i + 10, f.mac);
		f.payload.assign(m_buf.begin() + i + 10, m_buf.begin() + i + 10 + len);
		frames.push_back(std::move(f));
		i += total;
	}
	m_buf.erase(m_buf.begin(), m_buf.begin() + i);
	return frames;
}

std::string macString(const uint8_t mac[6]) {
	char s[18];
	std::snprintf(s, sizeof(s), "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return s;
}

}  // namespace dongle

namespace {

double now() {
	using namespace std::chrono;
	return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::vector<std::string> serialPorts() {
	std::vector<std::string> ports;
	HKEY key;
	if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) != ERROR_SUCCESS)
		return ports;
	for (DWORD i = 0;; i++) {
		char name[256], value[64];
		DWORD nameLen = sizeof(name), valueLen = sizeof(value), type = 0;
		if (RegEnumValueA(key, i, name, &nameLen, nullptr, &type, reinterpret_cast<BYTE*>(value), &valueLen)
			!= ERROR_SUCCESS)
			break;
		// Bluetooth serial ports (\Device\BthModemN) are skipped: opening one tries to
		// connect to the paired device, which blocks for seconds
		if (type == REG_SZ && !std::strstr(name, "BthModem")) ports.emplace_back(value, strnlen(value, valueLen));
	}
	RegCloseKey(key);
	return ports;
}

bool writeAll(HANDLE h, const std::vector<uint8_t>& data) {
	DWORD written = 0;
	return WriteFile(h, data.data(), DWORD(data.size()), &written, nullptr) && written == data.size();
}

// The dongle's info frame: "SLDONGLE" version channel
bool parseInfo(const dongle::Frame& f, DongleBridge::Status& st) {
	if (f.type != dongle::kInfo || f.payload.size() < 10 || std::memcmp(f.payload.data(), "SLDONGLE", 8) != 0)
		return false;
	st.version = f.payload[8];
	st.channel = f.payload[9];
	st.mac = dongle::macString(f.mac);
	return true;
}

}  // namespace

void DongleBridge::start(uint16_t hubPort) {
	if (m_running.exchange(true)) return;
	m_hubPort = hubPort;
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
	m_thread = std::thread(&DongleBridge::loop, this);
}

void DongleBridge::stop() {
	if (!m_running.exchange(false)) return;
	if (m_thread.joinable()) m_thread.join();
	closePort();
}

DongleBridge::Status DongleBridge::status() const {
	std::lock_guard lock(m_mutex);
	return m_status;
}

void DongleBridge::closePort() {
	for (auto& [mac, c] : m_clients) closesocket(SOCKET(c.socket));
	m_clients.clear();
	if (m_port) CloseHandle(HANDLE(m_port));
	m_port = nullptr;
	std::lock_guard lock(m_mutex);
	m_status = Status{};
}

bool DongleBridge::openAndProbe(const std::string& name) {
	std::string path = "\\\\.\\" + name;
	HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
	if (h == INVALID_HANDLE_VALUE) return false;
	DCB dcb{};
	dcb.DCBlength = sizeof(dcb);
	GetCommState(h, &dcb);
	dcb.BaudRate = 921600;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fBinary = TRUE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = FALSE;
	dcb.fOutX = dcb.fInX = FALSE;
	COMMTIMEOUTS to{};
	to.ReadIntervalTimeout = MAXDWORD;
	to.ReadTotalTimeoutMultiplier = MAXDWORD;
	to.ReadTotalTimeoutConstant = 5;
	to.WriteTotalTimeoutConstant = 50;
	if (!SetCommState(h, &dcb) || !SetCommTimeouts(h, &to)) {
		CloseHandle(h);
		return false;
	}
	PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);
	// Opening the port may reset the ESP (D1 mini auto-reset): the dongle boots and
	// scans the WiFi channels (~2-3 s) before it answers
	dongle::Decoder dec;
	dongle::Frame probe;
	probe.type = dongle::kProbe;
	const double deadline = now() + 4.0;
	double lastProbe = 0;
	while (now() < deadline && m_running) {
		if (now() - lastProbe > 0.5) {
			lastProbe = now();
			writeAll(h, dongle::encode(probe));
		}
		uint8_t buf[512];
		DWORD n = 0;
		if (!ReadFile(h, buf, sizeof(buf), &n, nullptr)) break;
		for (const auto& f : dec.feed(buf, n)) {
			Status st;
			if (!parseInfo(f, st)) continue;
			st.connected = true;
			st.port = name;
			m_port = h;
			m_decoder = dongle::Decoder{};
			m_lastInfo = now();
			std::lock_guard lock(m_mutex);
			m_status = st;
			return true;
		}
	}
	CloseHandle(h);
	return false;
}

bool DongleBridge::serve() {
	HANDLE h = HANDLE(m_port);
	uint8_t buf[2048];
	DWORD n = 0;
	if (!ReadFile(h, buf, sizeof(buf), &n, nullptr)) return false;  // unplugged
	sockaddr_in hub{};
	hub.sin_family = AF_INET;
	hub.sin_port = htons(m_hubPort);
	hub.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	for (const auto& f : m_decoder.feed(buf, n)) {
		if (f.type == dongle::kInfo) {
			Status st;
			if (parseInfo(f, st)) {
				m_lastInfo = now();
				std::lock_guard lock(m_mutex);
				m_status.channel = st.channel;
				m_status.version = st.version;
				m_status.mac = st.mac;
			}
			continue;
		}
		if (f.type != dongle::kData || f.payload.empty()) continue;
		std::string mac = dongle::macString(f.mac);
		auto it = m_clients.find(mac);
		if (it == m_clients.end()) {
			SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			if (s == INVALID_SOCKET) continue;
			// A port derived from the MAC: after a hot reload the tracker keeps the
			// address the hub knows it by
			sockaddr_in a{};
			a.sin_family = AF_INET;
			a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
			a.sin_port = htons(uint16_t(40000 + ((f.mac[3] << 16 | f.mac[4] << 8 | f.mac[5]) % 20000)));
			if (bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
				a.sin_port = 0;
				bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a));
			}
			u_long nonBlocking = 1;
			ioctlsocket(s, FIONBIO, &nonBlocking);
			DWORD noReset = FALSE, bytes = 0;
			WSAIoctl(s, SIO_UDP_CONNRESET, &noReset, sizeof(noReset), nullptr, 0, &bytes, nullptr, nullptr);
			Client c{uintptr_t(s), {}};
			std::copy(f.mac, f.mac + 6, c.mac);
			it = m_clients.emplace(mac, c).first;
			std::lock_guard lock(m_mutex);
			m_status.trackers = int(m_clients.size());
		}
		sendto(SOCKET(it->second.socket), reinterpret_cast<const char*>(f.payload.data()), int(f.payload.size()), 0,
			   reinterpret_cast<sockaddr*>(&hub), sizeof(hub));
	}
	// Hub -> trackers
	for (auto& [mac, c] : m_clients) {
		for (;;) {
			char pkt[256];
			int r = recv(SOCKET(c.socket), pkt, sizeof(pkt), 0);
			if (r <= 0) break;
			dongle::Frame f;
			f.type = dongle::kData;
			std::copy(c.mac, c.mac + 6, f.mac);
			f.payload.assign(pkt, pkt + std::min(r, 246));
			if (!writeAll(h, dongle::encode(f))) return false;
		}
	}
	return now() - m_lastInfo < 5.0;  // the dongle sends its info every second
}

void DongleBridge::loop() {
	double lastScan = 0;
	while (m_running) {
		if (m_port) {
			if (!serve()) {
				// Gone or silent: probe that port again on the next scan
				std::string port = status().port;
				closePort();
				m_probed.erase(port);
			}
			continue;
		}
		if (now() - lastScan < 2.0) {
			Sleep(100);
			continue;
		}
		lastScan = now();
		std::vector<std::string> ports = serialPorts();
		// Forget the ports that disappeared: plugged again, they are probed again
		for (auto it = m_probed.begin(); it != m_probed.end();)
			it = std::find(ports.begin(), ports.end(), *it) == ports.end() ? m_probed.erase(it) : std::next(it);
		for (const auto& p : ports) {
			if (!m_running || m_port) break;
			if (!m_probed.insert(p).second) continue;
			openAndProbe(p);
		}
	}
}

}  // namespace slimelite
