"""Minimal SlimeVR UDP server: talks to trackers, prints live data, records JSONL.

Reference implementation of the protocol subset the SteamVR driver speaks.
Do not run it while the SlimeVR server or the Slime Motion Tracker driver is running
(they all bind UDP 6969).

Usage:
  python tools/slime_server.py                 # live view
  python tools/slime_server.py --record out.jsonl
"""
import argparse
import json
import math
import socket
import struct
import time

PORT = 6969

# Tracker -> server
PKT_HEARTBEAT = 0
PKT_HANDSHAKE = 3
PKT_ACCEL = 4
PKT_BATTERY = 12
PKT_SENSOR_INFO = 15
PKT_ROTATION_DATA = 17
PKT_SIGNAL = 19
PKT_TEMPERATURE = 20
PKT_FEATURE_FLAGS = 22
PKT_BUNDLE = 100
PKT_RAW_MAG = 210  # Slime Motion Tracker extension

# Server -> tracker
SRV_HEARTBEAT = 1
SRV_SENSOR_INFO = 15
SRV_FEATURE_FLAGS = 22

HANDSHAKE_REPLY = bytes([3]) + b"Hey OVR =D 5"


class Tracker:
    def __init__(self, addr, mac, fw):
        self.addr = addr
        self.mac = mac
        self.fw = fw
        self.quat = (0.0, 0.0, 0.0, 1.0)  # x y z w
        self.accel = (0.0, 0.0, 0.0)
        self.mag = None
        self.temp = None
        self.battery = None
        self.rssi = None
        self.counts = {"rot": 0, "acc": 0, "mag": 0}
        self.last_seen = time.time()
        self.flags_sent = False


def parse_handshake(data):
    # board, imu, mcu, 3x unused, protocol (all int32 BE), fw short string, mac[6]
    board, imu, mcu, _, _, _, proto = struct.unpack_from(">7i", data, 0)
    off = 28
    n = data[off]
    fw = data[off + 1 : off + 1 + n].decode(errors="replace")
    off += 1 + n
    mac = ":".join(f"{b:02X}" for b in data[off : off + 6])
    return board, imu, mcu, proto, fw, mac


class Server:
    def __init__(self, record=None, quiet=False):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", PORT))
        self.sock.settimeout(0.05)
        self.trackers = {}
        self.pn = 0
        self.record = open(record, "w") if record else None
        self.quiet = quiet

    def send(self, addr, ptype, payload=b"", with_number=True):
        head = struct.pack(">I", ptype)
        if with_number:
            self.pn += 1
            head += struct.pack(">Q", self.pn)
        self.sock.sendto(head + payload, addr)

    def log(self, tr, kind, **kw):
        if self.record:
            kw.update(t=time.time(), mac=tr.mac, kind=kind)
            self.record.write(json.dumps(kw) + "\n")

    def handle_inner(self, tr, ptype, body):
        if ptype == PKT_ROTATION_DATA:
            sid, dtype, x, y, z, w, acc = struct.unpack_from(">BBffffB", body)
            tr.quat = (x, y, z, w)
            tr.counts["rot"] += 1
            self.log(tr, "rot", sid=sid, q=[x, y, z, w])
        elif ptype == PKT_ACCEL:
            x, y, z = struct.unpack_from(">fff", body)
            sid = body[12] if len(body) > 12 else 0
            tr.accel = (x, y, z)
            tr.counts["acc"] += 1
            self.log(tr, "acc", sid=sid, a=[x, y, z])
        elif ptype == PKT_RAW_MAG:
            sid, x, y, z = struct.unpack_from(">Bhhh", body)
            tr.mag = (x, y, z)
            tr.counts["mag"] += 1
            self.log(tr, "mag", sid=sid, m=[x, y, z])
        elif ptype == PKT_SENSOR_INFO:
            sid, state = body[0], body[1]
            # Firmware reads the ack at offset 4, i.e. without a packet number
            self.send(tr.addr, SRV_SENSOR_INFO, bytes([sid, state]), with_number=False)
        elif ptype == PKT_TEMPERATURE:
            tr.temp = struct.unpack_from(">Bf", body)[1]
        elif ptype == PKT_BATTERY:
            tr.battery = struct.unpack_from(">ff", body)
        elif ptype == PKT_SIGNAL:
            tr.rssi = struct.unpack_from(">Bb", body)[1]
        elif ptype == PKT_FEATURE_FLAGS:
            # Advertise bundle support (bit 0)
            self.send(tr.addr, SRV_FEATURE_FLAGS, bytes([0x01]))
        elif ptype == PKT_HEARTBEAT:
            pass

    def handle(self, data, addr):
        if len(data) < 4:
            return
        ptype = struct.unpack_from(">I", data)[0]
        if ptype == PKT_HANDSHAKE:
            board, imu, mcu, proto, fw, mac = parse_handshake(data[12:])
            if addr not in self.trackers:
                print(f"[+] {mac} at {addr[0]}:{addr[1]} board={board} imu={imu} fw={fw} proto={proto}")
            self.trackers[addr] = Tracker(addr, mac, fw)
            self.sock.sendto(HANDSHAKE_REPLY, addr)
            return
        tr = self.trackers.get(addr)
        if tr is None:
            return
        tr.last_seen = time.time()
        body = data[12:]
        if ptype == PKT_BUNDLE:
            off = 0
            while off + 2 <= len(body):
                n = struct.unpack_from(">H", body, off)[0]
                off += 2
                inner = body[off : off + n]
                off += n
                if len(inner) >= 4:
                    self.handle_inner(tr, struct.unpack_from(">I", inner)[0], inner[4:])
        else:
            self.handle_inner(tr, ptype, body)

    def status_line(self, dt):
        parts = []
        for tr in self.trackers.values():
            x, y, z, w = tr.quat
            # yaw around the firmware's Z-up axis, for a quick look
            yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            rates = "/".join(f"{tr.counts[k] / dt:.0f}" for k in ("rot", "acc", "mag"))
            mag = "none" if tr.mag is None else f"{tr.mag[0]:6d},{tr.mag[1]:6d},{tr.mag[2]:6d} |{math.sqrt(sum(v * v for v in tr.mag)):6.0f}|"
            parts.append(f"{tr.mac[-5:]} yaw={yaw:7.1f} Hz(r/a/m)={rates} mag={mag}")
            for k in tr.counts:
                tr.counts[k] = 0
        return " || ".join(parts) if parts else "waiting for trackers..."

    def run(self, duration=0):
        start = last_hb = last_print = time.time()
        while not duration or time.time() - start < duration:
            try:
                data, addr = self.sock.recvfrom(2048)
                self.handle(data, addr)
            except socket.timeout:
                pass
            except ConnectionResetError:
                pass
            now = time.time()
            if now - last_hb > 0.5:
                last_hb = now
                for addr, tr in list(self.trackers.items()):
                    if now - tr.last_seen > 5:
                        print(f"[-] {tr.mac} timed out")
                        del self.trackers[addr]
                        continue
                    self.send(addr, SRV_HEARTBEAT)
            if not self.quiet and now - last_print > 1.0:
                print(self.status_line(now - last_print), flush=True)
                last_print = now
        if self.record:
            self.record.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--record", help="write every sample to this JSONL file")
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds")
    args = ap.parse_args()
    srv = Server(args.record, args.quiet)
    srv.run(args.duration)


if __name__ == "__main__":
    main()
