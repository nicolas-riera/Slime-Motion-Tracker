"""Read a SlimeVR ESP tracker's boot log and query it over USB serial.

Usage: python tools/serial_probe.py COM20 [--no-reset] [--cmd "GET INFO" ...]
"""
import argparse
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-reset", action="store_true")
    ap.add_argument("--listen", type=float, default=6.0, help="seconds to read after reset")
    ap.add_argument("--cmd", action="append", default=[], help="serial command to send")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as s:
        if not args.no_reset:
            # NodeMCU/D1 style auto-reset: pulse RST via RTS, keep GPIO0 high via DTR
            s.dtr = False
            s.rts = True
            time.sleep(0.1)
            s.rts = False
        end = time.time() + args.listen
        while time.time() < end:
            data = s.read(4096)
            if data:
                print(data.decode("utf-8", "replace"), end="", flush=True)
        for cmd in args.cmd:
            print(f"\n>>> {cmd}")
            s.write((cmd + "\n").encode())
            end = time.time() + 2.0
            while time.time() < end:
                data = s.read(4096)
                if data:
                    print(data.decode("utf-8", "replace"), end="", flush=True)


if __name__ == "__main__":
    main()
