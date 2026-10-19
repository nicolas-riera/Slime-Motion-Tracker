// SlimeVR-lite USB dongle (ESP8266 D1 mini).
//
// Relays the trackers' ESP-NOW frames (the SLIMELITE_D1MINI tracker firmware looks for it at boot)
// to the PC over USB serial and back. The PC side (core/src/dongle_bridge.cpp)
// turns each tracker into a local UDP client of the tracker hub.
//
// Serial frames, both directions, 921600 baud:
//   A5 5A type len mac[6] payload[len] xor
//   xor = XOR of type, len, mac and payload
//   type 0: data (SlimeVR packet), mac = tracker
//   type 1: dongle info, dongle -> PC, once a second and on probe:
//           payload "SLDONGLE" version channel, then uint16 counters (ESP-NOW
//           frames received, searches received, answers sent), mac = dongle
//   type 2: probe, PC -> dongle
//   type 3: set channel, PC -> dongle, payload[0] = 1..13
// LED: 3 fast blinks once ready, then off; every 5 s, one blink per tracker
// heard in the last 3 s.
// ESP-NOW frames: 'S' 'L' version kind payload. kind 0: SlimeVR packet, 1: a
// tracker searching for the dongle (broadcast), 2: the dongle's answer and its
// beacon (broadcast every 100 ms), payload[0] = its channel. A tracker
// that finds the dongle never joins the WiFi network (power saving).
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

extern "C" {
#include <user_interface.h>
}

namespace {

constexpr uint8_t kVersion = 1;
constexpr size_t kHeader = 4;
constexpr size_t kMaxPayload = 250 - kHeader;
enum Kind : uint8_t { kData = 0, kSearch = 1, kHere = 2 };
uint8_t g_channel = 6;
constexpr uint32_t kBeaconMs = 100;

// ESP-NOW receive ring (filled in the system task, drained in loop())
constexpr int kSlots = 32;
struct Slot {
	uint8_t mac[6];
	uint8_t len;
	uint8_t data[kMaxPayload];
};
Slot g_slots[kSlots];
volatile uint8_t g_head = 0, g_tail = 0;
volatile uint32_t g_dropped = 0;
// Diagnostics, appended to the info frame: ESP-NOW frames received, searches, answers sent ok
volatile uint16_t g_rxAll = 0, g_rxSearch = 0;
uint16_t g_answers = 0;

// Trackers searching for the dongle, answered from loop()
constexpr int kSearchSlots = 8;
uint8_t g_searchMac[kSearchSlots][6];
volatile uint8_t g_searchHead = 0, g_searchTail = 0;

void onReceive(uint8_t* mac, uint8_t* data, uint8_t len) {
	g_rxAll++;
	if (len < kHeader || data[0] != 'S' || data[1] != 'L' || data[2] != kVersion) return;
	if (data[3] == kSearch) {
		g_rxSearch++;
		uint8_t next = (g_searchHead + 1) % kSearchSlots;
		if (next != g_searchTail) {
			memcpy(g_searchMac[g_searchHead], mac, 6);
			g_searchHead = next;
		}
		return;
	}
	if (data[3] != kData) return;
	uint8_t next = (g_head + 1) % kSlots;
	if (next == g_tail) {
		g_dropped++;
		return;
	}
	Slot& s = g_slots[g_head];
	memcpy(s.mac, mac, 6);
	s.len = len - kHeader;
	memcpy(s.data, data + kHeader, s.len);
	g_head = next;
}

void writeFrame(uint8_t type, const uint8_t* mac, const uint8_t* payload, uint8_t len) {
	uint8_t head[10] = {0xA5, 0x5A, type, len};
	memcpy(head + 4, mac, 6);
	uint8_t x = type ^ len;
	for (int i = 0; i < 6; i++) x ^= mac[i];
	for (int i = 0; i < len; i++) x ^= payload[i];
	Serial.write(head, sizeof(head));
	Serial.write(payload, len);
	Serial.write(x);
}

void sendInfo() {
	uint8_t mac[6];
	WiFi.macAddress(mac);
	uint16_t rxAll = g_rxAll, rxSearch = g_rxSearch;
	uint8_t p[16] = {'S', 'L', 'D', 'O', 'N', 'G', 'L', 'E', kVersion, uint8_t(wifi_get_channel()),
					 uint8_t(rxAll), uint8_t(rxAll >> 8), uint8_t(rxSearch), uint8_t(rxSearch >> 8),
					 uint8_t(g_answers), uint8_t(g_answers >> 8)};
	writeFrame(1, mac, p, sizeof(p));
}

void setChannel(uint8_t ch) {
	if (ch < 1 || ch > 13) return;
	g_channel = ch;
	wifi_set_channel(ch);
}

// The least busy of channels 1, 6 and 11 (summed AP power on the overlapping ones)
uint8_t quietestChannel() {
	int n = WiFi.scanNetworks(false, true);
	double load[14] = {};
	for (int i = 0; i < n; i++) {
		int ch = WiFi.channel(i);
		double p = pow(10.0, WiFi.RSSI(i) / 10.0);
		for (int c = 1; c <= 13; c++)
			if (abs(c - ch) <= 2) load[c] += p;
	}
	WiFi.scanDelete();
	uint8_t best = 1;
	for (uint8_t c : {6, 11})
		if (load[c] < load[best]) best = c;
	return best;
}

void sendToTracker(const uint8_t* mac, uint8_t kind, const uint8_t* payload, uint8_t len) {
	if (len > kMaxPayload) return;
	uint8_t m[6];
	memcpy(m, mac, 6);
	if (!esp_now_is_peer_exist(m)) esp_now_add_peer(m, ESP_NOW_ROLE_COMBO, g_channel, nullptr, 0);
	uint8_t buf[kHeader + kMaxPayload] = {'S', 'L', kVersion, kind};
	memcpy(buf + kHeader, payload, len);
	if (esp_now_send(m, buf, kHeader + len) == 0 && kind == kHere && m[0] != 0xFF) g_answers++;
}

// Trackers heard recently (data frames), for the LED
constexpr int kMaxTrackers = 16;
constexpr uint32_t kTrackerTimeout = 3000;
struct Tracker {
	uint8_t mac[6];
	uint32_t lastSeen;
};
Tracker g_trackers[kMaxTrackers];
int g_trackerCount = 0;

void trackerSeen(const uint8_t* mac) {
	uint32_t now = millis();
	for (int i = 0; i < g_trackerCount; i++)
		if (memcmp(g_trackers[i].mac, mac, 6) == 0) {
			g_trackers[i].lastSeen = now;
			return;
		}
	int slot = 0;
	if (g_trackerCount < kMaxTrackers) slot = g_trackerCount++;
	else
		for (int i = 1; i < kMaxTrackers; i++)
			if (now - g_trackers[i].lastSeen > now - g_trackers[slot].lastSeen) slot = i;
	memcpy(g_trackers[slot].mac, mac, 6);
	g_trackers[slot].lastSeen = now;
}

int connectedTrackers() {
	int n = 0;
	for (int i = 0; i < g_trackerCount; i++)
		if (millis() - g_trackers[i].lastSeen < kTrackerTimeout) n++;
	return n;
}

// LED (GPIO2, lit when low): 3 fast blinks once ready, then off, and every
// kLedPeriod one blink per connected tracker (nothing when none)
constexpr uint8_t kLed = 2;
constexpr uint32_t kLedPeriod = 5000;
uint32_t g_blinkStart = 0, g_blinkOn = 0, g_blinkStep = 0;
int g_blinkCount = 0;

void blink(int count, uint32_t on, uint32_t step) {
	g_blinkStart = millis();
	g_blinkCount = count;
	g_blinkOn = on;
	g_blinkStep = step;
}

void updateLed() {
	static uint32_t lastCount = 0;
	uint32_t now = millis();
	uint32_t t = now - g_blinkStart;
	bool busy = t < g_blinkStep * uint32_t(g_blinkCount);
	if (!busy && now - lastCount >= kLedPeriod) {
		lastCount = now;
		blink(connectedTrackers(), 150, 400);
		t = 0;
		busy = g_blinkCount > 0;
	}
	digitalWrite(kLed, busy && t % g_blinkStep < g_blinkOn ? LOW : HIGH);
}

// Serial frame parser
uint8_t g_rx[4 + 6 + 255 + 1];
size_t g_rxLen = 0;

void handleFrame() {
	uint8_t type = g_rx[2], len = g_rx[3];
	const uint8_t* mac = g_rx + 4;
	const uint8_t* payload = g_rx + 10;
	uint8_t x = type ^ len;
	for (int i = 0; i < 6; i++) x ^= mac[i];
	for (int i = 0; i < len; i++) x ^= payload[i];
	if (x != payload[len]) return;
	if (type == 0) sendToTracker(mac, kData, payload, len);
	else if (type == 2) sendInfo();
	else if (type == 3 && len >= 1) setChannel(payload[0]);
}

void readSerial() {
	while (Serial.available()) {
		uint8_t b = Serial.read();
		if (g_rxLen == 0 && b != 0xA5) continue;
		if (g_rxLen == 1 && b != 0x5A) {
			g_rxLen = b == 0xA5 ? 1 : 0;
			continue;
		}
		g_rx[g_rxLen++] = b;
		if (g_rxLen >= 4 && g_rxLen == size_t(4 + 6 + g_rx[3] + 1)) {
			handleFrame();
			g_rxLen = 0;
		}
	}
}

}  // namespace

void setup() {
	pinMode(kLed, OUTPUT);
	digitalWrite(kLed, HIGH);
	Serial.begin(921600);
	Serial.setRxBufferSize(1024);
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
	setChannel(quietestChannel());
	wifi_set_sleep_type(NONE_SLEEP_T);
	esp_now_init();
	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
	esp_now_register_recv_cb(onReceive);
	sendInfo();
	blink(3, 60, 140);
}

void loop() {
	readSerial();
	while (g_searchTail != g_searchHead) {
		sendToTracker(g_searchMac[g_searchTail], kHere, &g_channel, 1);
		g_searchTail = (g_searchTail + 1) % kSearchSlots;
	}
	while (g_tail != g_head) {
		Slot& s = g_slots[g_tail];
		writeFrame(0, s.mac, s.data, s.len);
		trackerSeen(s.mac);
		g_tail = (g_tail + 1) % kSlots;
	}
	updateLed();
	// Beacon for the searching trackers, which listen 150 ms on each channel
	static uint32_t lastBeacon = 0;
	if (millis() - lastBeacon >= kBeaconMs) {
		lastBeacon = millis();
		static const uint8_t bc[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		sendToTracker(bc, kHere, &g_channel, 1);
	}
	static uint32_t lastInfo = 0;
	if (millis() - lastInfo > 1000) {
		lastInfo = millis();
		sendInfo();
	}
}
