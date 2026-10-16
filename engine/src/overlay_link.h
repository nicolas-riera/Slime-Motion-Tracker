// Localhost UDP JSON channel with the dashboard overlay (see docs/IPC.md)
#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <vector>

namespace slimelite {

class OverlayLink {
public:
	static constexpr uint16_t kPort = 6971;

	~OverlayLink();
	bool start();
	void stop();
	// Messages received since the last call (non-blocking)
	std::vector<nlohmann::json> poll();
	// Sends to the overlay that talked to us last; no-op if none yet
	void send(const nlohmann::json& msg);
	bool hasPeer() const { return m_hasPeer; }

private:
	uintptr_t m_socket = ~uintptr_t(0);
	bool m_hasPeer = false;
	uint8_t m_peer[16] = {};  // sockaddr_in
};

}  // namespace slimelite
