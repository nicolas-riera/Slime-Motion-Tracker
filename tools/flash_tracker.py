"""Back up, build and flash the Slime Motion Tracker firmware on a USB-connected tracker.

Steps:
  1. read the MAC address
  2. dump the full 4 MB flash to firmware/backup/<MAC>_full_4MB.bin (once per tracker)
  3. build the SMT_D1MINI PlatformIO env
  4. flash the application image only (WiFi credentials and calibration are kept)
  5. reboot and check the boot log for the magnetometer

Usage: python tools/flash_tracker.py [--port COM20] [--no-build] [--no-flash]
"""
import argparse
import hashlib
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

ROOT = Path(__file__).resolve().parent.parent
FW_DIR = ROOT / "firmware" / "SlimeVR-Tracker-ESP"
BACKUP_DIR = ROOT / "firmware" / "backup"
BOARD_JSON = ROOT / "firmware" / "smt-board.json"
VENV_SCRIPTS = ROOT / ".venv" / "Scripts"
ENV_NAME = "SMT_D1MINI"


def esptool(*args):
    cmd = [sys.executable, "-m", "esptool", *args]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print(res.stdout, res.stderr)
        raise SystemExit(f"esptool failed: {' '.join(args)}")
    return res.stdout


def pick_port(port):
    if port:
        return port
    usb = [p for p in list_ports.comports() if p.vid is not None]
    if len(usb) != 1:
        names = ", ".join(f"{p.device} ({p.description})" for p in usb) or "none"
        raise SystemExit(f"Cannot pick a port automatically, USB serial ports: {names}. Use --port.")
    return usb[0].device


def backup(port, mac):
    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    name = mac.replace(":", "") + "_full_4MB.bin"
    path = BACKUP_DIR / name
    if path.exists():
        print(f"[backup] already exists: {path.name}")
        return
    tmp = path.with_suffix(".part")
    for baud in ("460800", "230400", "115200"):
        print(f"[backup] dumping 4 MB flash to {path.name} at {baud} baud (2-6 minutes)...")
        res = subprocess.run([sys.executable, "-m", "esptool", "--port", port, "--baud", baud,
                              "read-flash", "0", "ALL", str(tmp)], capture_output=True, text=True)
        if res.returncode == 0 and tmp.exists() and tmp.stat().st_size == 4 * 1024 * 1024:
            tmp.replace(path)
            break
        print(f"[backup] failed at {baud} baud: {res.stdout.strip().splitlines()[-1] if res.stdout.strip() else res.stderr.strip()}")
    else:
        tmp.unlink(missing_ok=True)
        raise SystemExit("backup failed, nothing was flashed")
    digest = hashlib.sha256(path.read_bytes()).hexdigest().upper()
    readme = BACKUP_DIR / "README.md"
    text = readme.read_text(encoding="utf-8")
    row = f"| {mac:<20} | `{name}`       | `{digest}` |\n"
    marker = "\nAdd a row each time"
    text = text.replace(marker, row.rstrip("\n") + "\n" + marker, 1) if row not in text else text
    readme.write_text(text, encoding="utf-8")
    print(f"[backup] done, sha256 {digest}")


def build():
    env = dict(os.environ)
    env["SLIMEVR_OVERRIDE_DEFAULTS"] = BOARD_JSON.read_text(encoding="utf-8")
    env["PYTHONIOENCODING"] = "utf-8"
    print("[build] pio run ...")
    res = subprocess.run([str(VENV_SCRIPTS / "pio.exe"), "run", "-e", ENV_NAME], cwd=FW_DIR, env=env,
                         capture_output=True, text=True, encoding="utf-8", errors="replace")
    if res.returncode != 0:
        print(res.stdout[-4000:], res.stderr[-4000:])
        raise SystemExit("build failed")
    return FW_DIR / ".pio" / "build" / ENV_NAME / "firmware.bin"


def check_boot(port, seconds=8.0):
    print("[check] rebooting and reading the boot log...")
    log = ""
    with serial.Serial(port, 115200, timeout=0.1) as s:
        s.dtr = False
        s.rts = True
        time.sleep(0.1)
        s.rts = False
        end = time.time() + seconds
        while time.time() < end:
            log += s.read(4096).decode("utf-8", "replace")
    for line in log.splitlines():
        if re.search(r"MagDriver|ICM-45686|SensorManager|Aux|ERROR", line):
            print("   ", line.strip())
    if "Found mag" not in log:
        print("[check] WARNING: no magnetometer found in the boot log")
    elif "Aux" in log and "failed" in log:
        print("[check] WARNING: magnetometer I2C errors in the boot log")
    else:
        print("[check] magnetometer initialized")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--no-build", action="store_true", help="flash the last build")
    ap.add_argument("--no-flash", action="store_true", help="only back up")
    args = ap.parse_args()

    port = pick_port(args.port)
    out = esptool("--port", port, "read-mac")
    mac = re.search(r"MAC:\s*([0-9a-f:]{17})", out, re.I).group(1).upper()
    print(f"[info] tracker {mac} on {port}")

    backup(port, mac)
    if args.no_flash:
        return

    image = FW_DIR / ".pio" / "build" / ENV_NAME / "firmware.bin" if args.no_build else build()
    print(f"[flash] writing {image.name} ({image.stat().st_size // 1024} KiB) at 0x0...")
    esptool("--port", port, "--baud", "460800", "write-flash", "0x0", str(image))
    check_boot(port)


if __name__ == "__main__":
    main()
