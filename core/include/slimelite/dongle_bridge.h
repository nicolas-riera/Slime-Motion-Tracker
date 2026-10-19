// USB dongle bridge. Trackers that find the dongle (firmware/slimelite-dongle)
// talk ESP-NOW to it instead of joining the WiFi network, and the dongle relays
// their packets over USB serial. The bridge finds the dongle among the serial
// ports and gives each tracker its own local UDP socket (127.0.0.1, a port
// derived from its MAC) toward the tracker hub, so the hub sees an ordinary
// tracker: discovery, handshake, heartbeats and hot reload work unchanged, and
// WiFi trackers keep working alongside. A tracker whose address is 127.0.0.1 is
// on ESP-NOW.
#pragma once

#include <atomic>
#include <functional>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace slimelite {

// Serial framing shared with the dongle firmware: A5 5A type len mac[6] payload xor
namespace dongle {
enum : uint8_t { kData = 0, kInfo = 1, kProbe = 2, kSetChannel = 3 };
struct Frame {
	uint8_t type = 0;
	uint8_t mac[6] = {};
	std::vector<uint8_t> payload;
};
std::vector<uint8_t> encode(const Frame& f);
// Incremental decoder: feed bytes, get the complete frames with a valid checksum
class Decoder {
public:
	std::vector<Frame> feed(const uint8_t* data, size_t n);

private:
	std::vector<uint8_t> m_buf;
};
std::string macString(const uint8_t mac[6]);
}  // namespace dongle

class DongleBridge {
public:
	struct Status {
		bool connected = false;
		std::string port, mac;
		int channel = 0;
		int version = 0;
		int trackers = 0;  // trackers heard through the dongle
	};

	~DongleBridge() { stop(); }
	// Connection log, from the bridge thread. Set before start().
	void setLog(std::function<void(const std::string&)> log) { m_log = std::move(log); }
	void start(uint16_t hubPort);
	void stop();
	Status status() const;

private:
	void loop();
	bool openAndProbe(const std::string& port);
	void closePort();
	bool serve();  // false when the dongle is gone

	std::function<void(const std::string&)> m_log;
	uint16_t m_hubPort = 0;
	std::atomic<bool> m_running{false};
	std::thread m_thread;
	mutable std::mutex m_mutex;
	Status m_status;
	void* m_port = nullptr;  // HANDLE
	double m_lastInfo = 0;
	dongle::Decoder m_decoder;
	std::set<std::string> m_probed;  // ports tried while present (probed once: opening resets an ESP)
	struct Client {
		uintptr_t socket;
		uint8_t mac[6];
	};
	std::map<std::string, Client> m_clients;  // by MAC string
};

}  // namespace slimelite
