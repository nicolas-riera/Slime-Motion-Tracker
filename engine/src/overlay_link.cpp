#include "overlay_link.h"

#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>

#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif

#include <cstring>

namespace smt {

static_assert(sizeof(sockaddr_in) == 16);

OverlayLink::~OverlayLink() { stop(); }

bool OverlayLink::start() {
	WSADATA wsa;
	WSAStartup(MAKEWORD(2, 2), &wsa);
	SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sockaddr_in a{};
	a.sin_family = AF_INET;
	a.sin_port = htons(kPort);
	inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
	if (bind(s, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
		closesocket(s);
		return false;
	}
	u_long nonBlocking = 1;
	ioctlsocket(s, FIONBIO, &nonBlocking);
	DWORD noReset = FALSE, bytes = 0;
	WSAIoctl(s, SIO_UDP_CONNRESET, &noReset, sizeof(noReset), nullptr, 0, &bytes, nullptr, nullptr);
	m_socket = s;
	return true;
}

void OverlayLink::stop() {
	if (m_socket != ~uintptr_t(0)) {
		closesocket(static_cast<SOCKET>(m_socket));
		m_socket = ~uintptr_t(0);
	}
}

std::vector<nlohmann::json> OverlayLink::poll() {
	std::vector<nlohmann::json> out;
	if (m_socket == ~uintptr_t(0)) return out;
	char buf[8192];
	for (;;) {
		sockaddr_in from{};
		int fromLen = sizeof(from);
		int n = recvfrom(static_cast<SOCKET>(m_socket), buf, sizeof(buf), 0,
						 reinterpret_cast<sockaddr*>(&from), &fromLen);
		if (n <= 0) break;
		auto j = nlohmann::json::parse(buf, buf + n, nullptr, false);
		if (j.is_discarded() || !j.is_object()) continue;
		std::memcpy(m_peer, &from, sizeof(from));
		m_hasPeer = true;
		out.push_back(std::move(j));
	}
	return out;
}

void OverlayLink::send(const nlohmann::json& msg) {
	if (!m_hasPeer || m_socket == ~uintptr_t(0)) return;
	std::string s = msg.dump();
	sendto(static_cast<SOCKET>(m_socket), s.data(), int(s.size()), 0,
		   reinterpret_cast<const sockaddr*>(m_peer), sizeof(sockaddr_in));
}

}  // namespace smt
