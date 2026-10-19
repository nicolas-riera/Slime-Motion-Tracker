#include "smt/protocol.h"

#include <cstdio>
#include <cstring>

namespace smt::proto {

namespace {

struct Reader {
	const uint8_t* p;
	size_t len;
	size_t off = 0;

	bool has(size_t n) const { return off + n <= len; }
	uint8_t u8() { return p[off++]; }
	uint16_t u16() {
		uint16_t v = static_cast<uint16_t>((p[off] << 8) | p[off + 1]);
		off += 2;
		return v;
	}
	uint32_t u32() {
		uint32_t v = (uint32_t(p[off]) << 24) | (uint32_t(p[off + 1]) << 16)
				   | (uint32_t(p[off + 2]) << 8) | uint32_t(p[off + 3]);
		off += 4;
		return v;
	}
	int32_t i32() { return static_cast<int32_t>(u32()); }
	int16_t i16() { return static_cast<int16_t>(u16()); }
	float f32() {
		uint32_t v = u32();
		float f;
		std::memcpy(&f, &v, 4);
		return f;
	}
	std::string shortString() {
		if (!has(1)) return {};
		size_t n = u8();
		if (!has(n)) return {};
		std::string s(reinterpret_cast<const char*>(p + off), n);
		off += n;
		return s;
	}
};

void putU32(std::vector<uint8_t>& out, uint32_t v) {
	for (int i = 3; i >= 0; i--) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

void putU64(std::vector<uint8_t>& out, uint64_t v) {
	for (int i = 7; i >= 0; i--) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

// Parses one packet body (after type and packet number) into events
void parseInner(uint32_t type, Reader r, std::vector<Event>& out) {
	Event e;
	switch (type) {
		case kHeartbeat:
			e.kind = Event::Kind::Heartbeat;
			break;
		case kPingPong:
			if (!r.has(4)) return;
			e.kind = Event::Kind::Pong;
			e.pingId = r.u32();
			break;
		case kRotationData:
			if (!r.has(2 + 16)) return;
			e.kind = Event::Kind::Rotation;
			e.sensorId = r.u8();
			r.u8();  // data type
			{
				double x = r.f32(), y = r.f32(), z = r.f32(), w = r.f32();
				e.rotation = Quat{w, x, y, z}.normalized();
			}
			break;
		case kAccel:
			if (!r.has(12)) return;
			e.kind = Event::Kind::Accel;
			e.accel.x = r.f32();
			e.accel.y = r.f32();
			e.accel.z = r.f32();
			e.sensorId = r.has(1) ? r.u8() : 0;
			break;
		case kRawMagnetometer:
			if (!r.has(7)) return;
			e.kind = Event::Kind::Mag;
			e.sensorId = r.u8();
			e.mag = {r.i16(), r.i16(), r.i16()};
			break;
		case kBatteryLevel:
			if (!r.has(8)) return;
			e.kind = Event::Kind::Battery;
			e.batteryVoltage = r.f32();
			e.batteryLevel = r.f32();
			break;
		case kSensorInfo:
			if (!r.has(2)) return;
			e.kind = Event::Kind::SensorInfo;
			e.sensorId = r.u8();
			e.sensorState = r.u8();
			break;
		case kTemperature:
			if (!r.has(5)) return;
			e.kind = Event::Kind::Temperature;
			e.sensorId = r.u8();
			e.temperature = r.f32();
			break;
		case kSignalStrength:
			if (!r.has(2)) return;
			e.kind = Event::Kind::Signal;
			e.sensorId = r.u8();
			e.rssi = static_cast<int8_t>(r.u8());
			break;
		case kFeatureFlags:
			e.kind = Event::Kind::FeatureFlags;
			break;
		default:
			return;
	}
	out.push_back(e);
}

}  // namespace

std::vector<Event> parse(const uint8_t* data, size_t len) {
	std::vector<Event> out;
	Reader r{data, len};
	if (!r.has(4)) return out;
	uint32_t type = r.u32();

	if (type == kHandshake) {
		// [u32 type][u64 packet number][7 x i32][fw short string][mac 6 bytes]...
		r.off = 12;
		if (!r.has(28)) return out;
		Event e;
		e.kind = Event::Kind::Handshake;
		e.board = r.i32();
		e.imuType = r.i32();
		e.mcu = r.i32();
		r.i32();
		r.i32();
		r.i32();
		e.protocolVersion = r.i32();
		e.firmware = r.shortString();
		if (!r.has(6)) return out;
		char mac[18];
		std::snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X", data[r.off],
					  data[r.off + 1], data[r.off + 2], data[r.off + 3], data[r.off + 4],
					  data[r.off + 5]);
		e.mac = mac;
		out.push_back(e);
		return out;
	}

	if (!r.has(8)) return out;
	r.off += 8;  // packet number

	if (type == kBundle) {
		while (r.has(2)) {
			size_t n = r.u16();
			if (!r.has(n) || n < 4) break;
			Reader inner{data + r.off, n};
			uint32_t innerType = inner.u32();
			parseInner(innerType, inner, out);
			r.off += n;
		}
		return out;
	}

	parseInner(type, Reader{data + r.off, len - r.off}, out);
	return out;
}

std::vector<uint8_t> handshakeReply() {
	const char msg[] = "\x03Hey OVR =D 5";
	return std::vector<uint8_t>(msg, msg + sizeof(msg) - 1);
}

std::vector<uint8_t> heartbeat(uint64_t packetNumber) {
	std::vector<uint8_t> out;
	putU32(out, 1);  // server heartbeat
	putU64(out, packetNumber);
	return out;
}

std::vector<uint8_t> ping(uint64_t packetNumber, uint32_t id) {
	std::vector<uint8_t> out;
	putU32(out, kPingPong);
	putU64(out, packetNumber);
	putU32(out, id);
	return out;
}

std::vector<uint8_t> sensorInfoAck(uint8_t sensorId, uint8_t state) {
	std::vector<uint8_t> out;
	putU32(out, kSensorInfo);
	out.push_back(sensorId);
	out.push_back(state);
	return out;
}

std::vector<uint8_t> featureFlags(uint64_t packetNumber) {
	std::vector<uint8_t> out;
	putU32(out, kFeatureFlags);
	putU64(out, packetNumber);
	out.push_back(0x01);
	return out;
}

Quat firmwareToWorld(const Quat& q) {
	static const Quat kAxes = Quat::fromAxisAngle({1, 0, 0}, -kPi / 2);
	return (kAxes * q).normalized();
}

}  // namespace smt::proto
