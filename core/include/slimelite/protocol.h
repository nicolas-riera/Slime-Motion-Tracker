// Subset of the SlimeVR tracker UDP protocol (firmware protocol 22) plus the
// SlimeVR-lite raw magnetometer extension (packet 210).
//
// Tracker -> server datagram: [u32 BE type][u64 BE packet number][payload]
// Bundle (100): header, then repeated [u16 BE length][u32 BE type][payload].
// The server's handshake reply is the raw bytes "\x03Hey OVR =D 5".
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "slimelite/linalg.h"

namespace slimelite::proto {

constexpr uint16_t kTrackerPort = 6969;

enum PacketType : uint32_t {
	kHeartbeat = 0,
	kHandshake = 3,
	kAccel = 4,
	kBatteryLevel = 12,
	kSensorInfo = 15,
	kRotationData = 17,
	kSignalStrength = 19,
	kTemperature = 20,
	kFeatureFlags = 22,
	kBundle = 100,
	kRawMagnetometer = 210,
};

struct Event {
	enum class Kind {
		Handshake,
		Heartbeat,
		Rotation,
		Accel,
		Mag,
		Battery,
		SensorInfo,
		Temperature,
		Signal,
		FeatureFlags,
	};
	Kind kind = Kind::Heartbeat;
	uint8_t sensorId = 0;

	// Handshake
	int board = 0, imuType = 0, mcu = 0, protocolVersion = 0;
	std::string firmware;
	std::string mac;  // "AA:BB:CC:DD:EE:FF"

	// Rotation: firmware frame (Z up), Hamilton quaternion
	Quat rotation;
	// Accel: linear acceleration in the tracker frame, m/s^2
	Vec3 accel;
	// Mag: raw counts
	std::array<int16_t, 3> mag{};
	// Battery
	float batteryVoltage = 0, batteryLevel = -1;
	// SensorInfo
	uint8_t sensorState = 0;
	float temperature = 0;
	int8_t rssi = 0;
};

// Parses one datagram, expanding bundles. Unknown packets are skipped.
std::vector<Event> parse(const uint8_t* data, size_t len);

std::vector<uint8_t> handshakeReply();
std::vector<uint8_t> heartbeat(uint64_t packetNumber);
// The firmware reads the ack at offset 4, i.e. without a packet number
std::vector<uint8_t> sensorInfoAck(uint8_t sensorId, uint8_t state);
// Advertises bundle support (bit 0)
std::vector<uint8_t> featureFlags(uint64_t packetNumber);

// Firmware frame (Z up, VQF earth frame) -> SteamVR world frame (Y up).
// Same convention as the SlimeVR server: q_world = Rx(-90 deg) * q_firmware.
Quat firmwareToWorld(const Quat& q);

}  // namespace slimelite::proto
