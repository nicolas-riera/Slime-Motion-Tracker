/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2023 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "manager.h"

#include "GlobalVars.h"
#include "espnowudp.h"

namespace SlimeVR::Network {

// SlimeVR-lite: the USB dongle (ESP-NOW) is preferred: the WiFi network is then
// never joined, which saves power. Without a dongle, normal WiFi. A tracker that
// loses its server (or its dongle) for kResearchMs looks for the dongle again.
namespace {
constexpr uint32_t kResearchMs = 15000;
}

void Manager::setup() { startDongleSearch(); }

void Manager::startDongleSearch() {
	m_Transport = Transport::SearchingDongle;
	m_IsConnected = false;
	EspNow::begin();
}

void Manager::update() {
	if (m_Transport == Transport::SearchingDongle) {
		if (EspNow::searchStep()) {
			m_Transport = Transport::EspNow;
			m_LostSince = 0;
		} else if (EspNow::searchExhausted()) {
			EspNow::end();
			wifiNetwork.setUp();
			m_Transport = Transport::WiFi;
			m_LostSince = 0;
		}
		return;
	}

	auto wasConnected = m_IsConnected;
	if (m_Transport == Transport::EspNow) {
		m_IsConnected = true;
	} else {
		wifiNetwork.upkeep();
		m_IsConnected = wifiNetwork.isConnected();
	}

	// Server (or dongle) lost for a while: look for the dongle again
	bool serving = m_IsConnected && networkConnection.isConnected();
	if (serving) {
		m_LostSince = 0;
	} else if (m_LostSince == 0) {
		m_LostSince = millis();
	} else if (millis() - m_LostSince > kResearchMs) {
		if (m_Transport == Transport::WiFi) EspNow::leaveNetwork();
		startDongleSearch();
		return;
	}

	if (!m_IsConnected) {
		return;
	}

	if (!wasConnected) {
		// (Re)connected, rediscover the server and reconnect
		networkConnection.reset();
	}

	networkConnection.update();
}

}  // namespace SlimeVR::Network
