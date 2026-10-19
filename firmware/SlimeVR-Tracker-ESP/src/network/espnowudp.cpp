#include "espnowudp.h"

#include <ESP8266WiFi.h>
#include <espnow.h>

#include "logging/Logger.h"

extern "C" {
#include <user_interface.h>
}

namespace SlimeVR::Network {

namespace {

uint8_t g_broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t g_dongle[6] = {};
volatile bool g_paired = false;
volatile uint32_t g_lastRx = 0;
bool g_active = false;
uint8_t g_channel = 1;
uint32_t g_lastHop = 0;
int g_hops = 0;

SlimeVR::Logging::Logger g_logger{"ESP-NOW"};
// Dongle heard while searching (its beacon or answer), paired from loop()
volatile bool g_found = false;
volatile uint8_t g_foundChannel = 0;
volatile int16_t g_rssi4 = 0;  // smoothed dBm x 4

// Receive ring (the callback runs in the system task, the reader in loop())
constexpr int kSlots = 4;
struct Slot {
	uint8_t len;
	uint8_t data[EspNow::kMaxPayload];
};
Slot g_slots[kSlots];
volatile uint8_t g_head = 0, g_tail = 0;

void onReceive(uint8_t* mac, uint8_t* data, uint8_t len) {
	if (len < EspNow::kHeader || data[0] != 'S' || data[1] != 'L' || data[2] != EspNow::kVersion) return;
	if (!g_paired) {
		// The dongle's beacon or answer carries its channel: close to the dongle it
		// is also heard on the neighbouring channels
		if (data[3] == EspNow::Here && !g_found) {
			memcpy(g_dongle, mac, 6);
			g_foundChannel = len > EspNow::kHeader && data[4] >= 1 && data[4] <= 13 ? data[4] : g_channel;
			g_found = true;
		}
		return;
	}
	if (memcmp(mac, g_dongle, 6) != 0) return;
	g_lastRx = millis();
	// The ESP8266 callback has no RSSI, but the frame sits in the receive buffer
	// after the radio's rx_ctrl (12 bytes, RSSI first), the 802.11 header (24) and
	// the ESP-NOW action header and vendor element (15): 51 bytes before the payload
	// (checked on hardware: the rx_ctrl length and group bit match the frames). This
	// SDK stores the RSSI above the noise floor (positive, ~42 at 1 m): dBm = x - 96
	int rssi = int8_t(data[-51]);
	if (rssi >= 0) rssi -= 96;
	if (rssi < -5 && rssi > -100) g_rssi4 = g_rssi4 == 0 ? rssi * 4 : g_rssi4 + (rssi - g_rssi4 / 4);
	if (data[3] != EspNow::Data) return;
	uint8_t next = (g_head + 1) % kSlots;
	if (next == g_tail) return;  // full: drop
	Slot& s = g_slots[g_head];
	s.len = len - EspNow::kHeader;
	memcpy(s.data, data + EspNow::kHeader, s.len);
	g_head = next;
}

void sendFrame(const uint8_t* mac, uint8_t kind, const uint8_t* payload, size_t len) {
	uint8_t buf[EspNow::kHeader + EspNow::kMaxPayload] = {'S', 'L', EspNow::kVersion, kind};
	if (len > EspNow::kMaxPayload) return;
	memcpy(buf + EspNow::kHeader, payload, len);
	esp_now_send(const_cast<uint8_t*>(mac), buf, EspNow::kHeader + len);
}

}  // namespace

namespace EspNow {

void begin() {
	if (g_active) {
		// Searching again from ESP-NOW mode: keep the radio, forget the dongle
		g_paired = false;
		g_found = false;
		g_hops = 0;
		return;
	}
	g_active = true;
	g_paired = false;
	g_found = false;
	g_hops = 0;
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	leaveNetwork();
	wifi_promiscuous_enable(0);
	// The radio must stay on to receive the server's packets
	wifi_set_sleep_type(NONE_SLEEP_T);
	wifi_set_channel(g_channel);
	esp_now_init();
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	esp_now_register_recv_cb(onReceive);
	esp_now_add_peer(g_broadcast, ESP_NOW_ROLE_COMBO, 0, nullptr, 0);
	g_lastHop = 0;
}

void leaveNetwork() {
	// The SDK would join the saved network again by itself (at boot and after a
	// disconnection), moving the radio to the AP's channel: most ESP-NOW frames to
	// the dongle were lost, the tracker looked connected but sent no data. The
	// boot-time auto-connect is a flash setting: only written when it changes.
	wifi_station_set_reconnect_policy(false);
	if (wifi_station_get_auto_connect()) wifi_station_set_auto_connect(false);
	wifi_station_disconnect();
}

bool searchStep() {
	if (g_paired) return true;
	if (g_found) {
		g_channel = g_foundChannel;
		wifi_set_channel(g_channel);
		if (esp_now_is_peer_exist(g_dongle)) esp_now_del_peer(g_dongle);
		esp_now_add_peer(g_dongle, ESP_NOW_ROLE_COMBO, g_channel, nullptr, 0);
		g_lastRx = millis();
		g_paired = true;
		g_logger.info("Dongle found on channel %d", g_channel);
		return true;
	}
	uint32_t now = millis();
	if (now - g_lastHop < 150) return false;
	if (g_lastHop != 0) {
		g_channel = g_channel % 13 + 1;
		g_hops++;
	}
	g_lastHop = now;
	wifi_set_channel(g_channel);
	sendFrame(g_broadcast, Search, nullptr, 0);
	if (g_hops == 25) g_logger.info("Dongle not found, using WiFi");
	return false;
}

bool searchExhausted() { return !g_paired && g_hops >= 26; }

void end() {
	if (!g_active) return;
	esp_now_unregister_recv_cb();
	esp_now_deinit();
	g_active = false;
	g_paired = false;
}

bool active() { return g_active; }
bool paired() { return g_paired; }
uint8_t channel() { return g_channel; }
uint32_t lastReceiveMillis() { return g_lastRx; }
int8_t rssi() { return int8_t(g_rssi4 / 4); }

}  // namespace EspNow

uint8_t HybridUdp::begin(uint16_t port) {
	if (EspNow::active()) return 1;
	return m_udp.begin(port);
}

int HybridUdp::beginPacket(IPAddress ip, uint16_t port) {
	m_tx = EspNow::active();
	if (!m_tx) return m_udp.beginPacket(ip, port);
	m_txLen = 0;
	return 1;
}

size_t HybridUdp::write(const uint8_t* buffer, size_t size) {
	if (!m_tx) return m_udp.write(buffer, size);
	if (m_txLen + size > EspNow::kMaxPayload) return 0;
	memcpy(m_txBuf + m_txLen, buffer, size);
	m_txLen += size;
	return size;
}

int HybridUdp::endPacket() {
	if (!m_tx) return m_udp.endPacket();
	// Discovery broadcasts and server packets alike go to the dongle
	if (!g_paired) return 0;
	sendFrame(g_dongle, EspNow::Data, m_txBuf, m_txLen);
	return 1;
}

int HybridUdp::parsePacket() {
	if (!EspNow::active()) {
		m_rxEspNow = false;
		return m_udp.parsePacket();
	}
	m_rxEspNow = true;
	if (g_tail == g_head) return 0;
	Slot& s = g_slots[g_tail];
	m_rxLen = s.len;
	memcpy(m_rx, s.data, s.len);
	m_rxPos = 0;
	g_tail = (g_tail + 1) % kSlots;
	return int(m_rxLen);
}

int HybridUdp::read(unsigned char* buffer, size_t len) {
	if (!m_rxEspNow) return m_udp.read(buffer, len);
	size_t n = std::min(len, m_rxLen - m_rxPos);
	memcpy(buffer, m_rx + m_rxPos, n);
	m_rxPos += n;
	return int(n);
}

IPAddress HybridUdp::remoteIP() { return m_rxEspNow ? IPAddress(10, 0, 0, 1) : m_udp.remoteIP(); }
uint16_t HybridUdp::remotePort() { return m_rxEspNow ? 6969 : m_udp.remotePort(); }
int HybridUdp::getWriteError() { return m_tx ? 0 : m_udp.getWriteError(); }

}  // namespace SlimeVR::Network
