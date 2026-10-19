/*
	SlimeVR-lite: ESP-NOW transport to the USB dongle (firmware/slimelite-dongle).

	At boot the tracker looks for the dongle (EspNow::search, ~4 s over the 13
	channels). Found: the WiFi network is never joined (no association, no
	beacons: the radio only serves ESP-NOW) and the SlimeVR packets go to the
	dongle, which relays them to the PC over USB. Not found: normal WiFi. The
	network manager searches again when the dongle or the server is lost.

	HybridUdp is the WiFiUDP subset Connection uses, routed to either transport,
	so the connection logic (discovery, handshake, timeouts) is the same for both.

	ESP-NOW frames: 'S' 'L' version kind payload. kind 0: SlimeVR packet,
	1: dongle search (tracker broadcast), 2: dongle answer or beacon (broadcast
	every 100 ms), payload[0] = the dongle's channel. Close to the dongle its
	frames are also heard on the neighbouring channels: the tracker moves to the
	announced channel.
*/
#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>

namespace SlimeVR::Network {

namespace EspNow {
constexpr uint8_t kVersion = 1;
constexpr size_t kHeader = 4;
constexpr size_t kMaxPayload = 250 - kHeader;
enum Kind : uint8_t { Data = 0, Search = 1, Here = 2 };

// Radio in ESP-NOW mode (WiFi disconnected), searching for the dongle
void begin();
// Leaves the WiFi network, keeping the saved credentials (WiFi.disconnect() on
// the ESP8266 core clears them, and erases them from flash when persistent)
void leaveNetwork();
// One search step: call from loop(); hops channel every ~150 ms. Returns true once
// the dongle answered
bool searchStep();
bool searchExhausted();  // two full rounds without an answer
// Back to WiFi: ESP-NOW off
void end();
bool active();
bool paired();
uint8_t channel();
uint32_t lastReceiveMillis();
}  // namespace EspNow

class HybridUdp {
public:
	uint8_t begin(uint16_t port);
	int beginPacket(IPAddress ip, uint16_t port);
	size_t write(const uint8_t* buffer, size_t size);
	int endPacket();
	int parsePacket();
	int read(unsigned char* buffer, size_t len);
	IPAddress remoteIP();
	uint16_t remotePort();
	int getWriteError();

private:
	WiFiUDP m_udp;
	bool m_tx = false;  // current packet goes over ESP-NOW
	uint8_t m_txBuf[EspNow::kHeader + EspNow::kMaxPayload];
	size_t m_txLen = 0;
	uint8_t m_rx[EspNow::kMaxPayload];
	size_t m_rxLen = 0, m_rxPos = 0;
	bool m_rxEspNow = false;
};

}  // namespace SlimeVR::Network
