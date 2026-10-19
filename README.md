# Slime Motion Tracker

Full-body tracking in SteamVR with only **3 SlimeVR trackers** (both ankles + waist), or
**2** (ankles only, like the Pico Motion Trackers), without the SlimeVR server.

- The trackers' magnetometers (QMC6309) are actually used: the yaw stays locked to
  magnetic north instead of drifting, so no more constant resets.
- **Anti-mag**: when the field changes in a way the gyro does not explain (a controller
  or phone brought near a tracker), the mag is ignored until the field is sane again.
- Positions come from a body model anchored to the headset: knee hinge, foot planting
  (zero-velocity detection), short inertial prediction while a foot is in the air.
- One calibration (stand, then ski pose) finds how each tracker is worn, even on the
  side or the back of the leg. After that, a quick "stand straight" reset is enough.

```
trackers (ESP8266 + ICM-45686 + QMC6309)
   | UDP 6969, SlimeVR protocol + raw mag packet
   v
smt_engine.dll (hot-reloaded)            SlimeMotionTracker.exe (dashboard UI)
   tracker hub -> mag calibration + heading      <-- localhost JSON (docs/IPC.md) -->
   body solver, calibration, session recorder
   ^ C ABI (engine/include/smt/engine_api.h)
driver_slime_motion_tracker.dll (thin SteamVR host: HMD/controller poses in, 3 virtual trackers out)
```

## Changing the algorithm without restarting SteamVR

The SteamVR driver is a thin host that loads `smt_engine.dll` from a temporary copy
and reloads it when the file changes. Rebuilding is enough:

```
cmake --build build --config Release --target smt_engine
```

A post-build step copies the engine into the installed driver folder; within a second the
running driver hands the state over (tracker addresses, mag headings, floor, recording
file) to the new engine and continues. The overlay is restarted the same way when it is
rebuilt. Only changes to `driver/` or to `engine_api.h` need a SteamVR restart (and an
`install.ps1`); the host refuses an engine built against another API version.

`smt_engine_host.exe <engine.dll>` runs the engine with a fake headset, without
SteamVR, to test the reload path.

## Session recordings

While SteamVR runs, every raw tracker packet, every SteamVR device pose (headset,
controllers), the engine outputs, calibrations and config snapshots are written to
`%LOCALAPPDATA%\Slime Motion Tracker\recordings\session_*.jsonl` (about 400 MB per hour, toggle
in the overlay settings). Format: `engine/src/recorder.h`. They are meant for offline
replay, tuning and training.

## Repository

| Path | What |
|------|------|
| `firmware/SlimeVR-Tracker-ESP` | SlimeVR tracker firmware + Slime Motion Tracker patches (mag fix, raw mag streaming) |
| `firmware/backup` | Original firmware dumps (git-ignored, contain WiFi credentials) and how to restore |
| `core/` | Protocol, tracker hub, mag calibration + anti-mag, mounting calibration, body solver |
| `driver/` | SteamVR driver |
| `overlay/` | SteamVR dashboard overlay (also opens a desktop window) |
| `tests/` | Unit tests on synthetic data |
| `tools/` | Flashing, Python test server/recorder, replay and monitor tools, installer |
| `docs/` | Design notes and the driver/overlay protocol |

## Requirements

- Windows, SteamVR, Visual Studio 2022 (C++ workload), CMake >= 3.24, Python 3.
- Python tools: `py -3 -m venv .venv` then `.venv\Scripts\pip install esptool pyserial platformio jsonschema==4.22.0 numpy`.

## 1. Flash the trackers (Wemos D1 Mini / ICM-45686 + QMC6309 only)

Plug a tracker over USB, then:

```
.venv\Scripts\python tools\flash_tracker.py
```

It backs up the full flash first (`firmware/backup/<MAC>_full_4MB.bin`, once per tracker),
builds, flashes the application only (WiFi and IMU calibration are kept) and checks that
the magnetometer starts. See `firmware/backup/README.md` to go back to the original firmware.

## 2. Build and install the driver

Close SteamVR and the SlimeVR server, then:

```
powershell -ExecutionPolicy Bypass -File tools\install.ps1
```

The trackers work as soon as SteamVR starts. The dashboard app (calibration, settings,
tracker states) is not started with SteamVR: launch it with the **Slime Motion Tracker** Start menu
shortcut the installer creates, before or after SteamVR (desktop window + "Slime Motion Tracker" tab
in the SteamVR dashboard). The SlimeVR server must not run at the same time (both use UDP
6969).

## 3. First setup

1. **Body tab**: enter your height, shake a tracker to see which row lights up and assign
   its role (left ankle, right ankle, waist). The mode follows the trackers: ankles only,
   or ankles + waist when a waist tracker is assigned and connected.
2. **Mag calibration** (once per tracker, before strapping it): on the Home tab click the
   tracker, then **Calibrate**, and slowly rotate the tracker in every direction (upside
   down, on each side, figure eights) until the bar is full. The calibration is stored and
   refined automatically afterwards.
3. Strap the trackers: ankles just above the ankle bone (any side), waist on the belt.
4. **Full calibration** (Home tab): stand straight looking ahead, then when asked take the
   ski pose (knees bent, leaning forward, arms down). 3 s each, with a beep every second.

Later sessions: just put the trackers on. If something looks rotated, use **Quick reset**
(stand straight). If you turned the mag off, a quick reset is needed after a tracker reboots.

## Tools

- `tools/slime_server.py`: minimal Python server; live view and JSONL recording.
- `build/Release/smt_monitor.exe`: runs the C++ tracker hub without SteamVR
  (`--magcal MAC` runs a guided mag calibration).
- `build/Release/smt_replay.exe rec.jsonl`: replays a recording through the mag fusion.
- `build/Release/smt_tests.exe`: unit tests.

## Limits

IMUs measure rotation, not position. Positions are inferred from the headset and the body
model, so they are plausible rather than measured: crossing legs, kneeling, lying down or
one foot on a chair are harder than standing, walking and sitting. The magnetometer is only
as good as the room: steel furniture and electronics near the floor bend the field, which
the anti-mag detects but cannot fix (the gyro takes over until the field is clean).
