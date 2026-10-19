# Driver <-> overlay protocol

The SteamVR driver (`driver_slime_motion_tracker.dll`) owns all state and logic. The dashboard
overlay (`SlimeMotionTracker.exe`) is a thin UI: it displays the state the driver
publishes and sends commands. The driver starts the overlay process when SteamVR starts.

- Transport: UDP on localhost. The driver listens on `127.0.0.1:6971`.
- Every datagram is one UTF-8 JSON object with a `type` field.
- The overlay sends `hello` once per second. The driver answers by streaming `state`
  messages (10 Hz) to the address the last overlay datagram came from. If the overlay
  receives no `state` for 2 s it shows "Driver not running".
- Lengths are meters, angles degrees, times seconds.

## Driver -> overlay

```json
{
  "type": "state",
  "version": 1,
  "driver": { "version": "0.1.0", "trackerPort": 6969, "portError": "" },
  "mode": "three",
  "userHeight": 1.75,
  "measuredEyeHeight": 1.63,
  "floorKnown": true,
  "calibrated": true,
  "calibration": { "phase": "idle", "countdown": 0.0, "message": "" },
  "lastCalibration": { "full": 1792187460.0, "reset": null },
  "settings": {
    "magEnabled": true,
    "magSensitivity": 0.5,
    "smoothing": 0.3,
    "footPlantStrength": 0.7,
    "inertialWeight": 0.5,
    "velocityPrediction": false,
    "recording": true
  },
  "recording": { "active": true, "file": "C:\...\recordings\session_20261016_223000.jsonl", "megabytes": 12.5 },
  "trackers": [
    {
      "mac": "B4:E6:2D:36:36:94",
      "ip": "192.168.137.215",
      "role": "left_ankle",
      "connected": true,
      "rotationHz": 100.0,
      "magHz": 50.0,
      "battery": 0.85,
      "batteryVoltage": 3.95,
      "rssi": -58,
      "temperature": 34.5,
      "activity": 0.12,
      "planted": true,
      "mag": {
        "state": "ok",
        "progress": 1.0,
        "residual": 0.03,
        "dip": 61.8
      }
    }
  ],
  "outputs": { "waist": true, "leftFoot": true, "rightFoot": true }
}
```

Field notes:

- `mode`: `"three"` (both ankles + waist tracker) or `"ankles"` (ankles only, waist estimated).
- `measuredEyeHeight`: HMD height above the floor captured at the last calibration, or `null`.
- `calibration.phase`: `idle` | `stand` | `ski` | `done` | `error`.
  During `stand` and `ski`, `countdown` is the time left before the pose is captured.
  `message` is a human-readable instruction or error.
- `lastCalibration`: wall clock (Unix seconds) of the last successful full calibration
  and quick reset, `null` when never done. Persisted in the config.
- `trackers[].role`: `waist` | `left_ankle` | `right_ankle` | `none`.
- `battery`: 0..1, or `-1` when unknown. `rssi` in dBm, `0` when unknown.
- `activity`: 0..1, smoothed angular speed. Lets the user find a tracker by shaking it.
- `mag.state`: `absent` (no mag on this tracker), `disabled` (mag turned off in settings),
  `uncalibrated`, `calibrating` (guided calibration running, see `progress`), `ok`,
  `disturbed` (anti-mag: field rejected, gyro only).
- `mag.residual`: fit residual of the current calibration (lower is better, < 0.08 is good).
- `outputs`: which virtual SteamVR trackers are currently published.
- `recording`: session recording status (see `engine/src/recorder.h` for the file format).

## Overlay -> driver

```json
{ "type": "hello" }
{ "type": "setMode", "mode": "ankles" }
{ "type": "setHeight", "meters": 1.78 }
{ "type": "setRole", "mac": "B4:E6:2D:36:36:94", "role": "waist" }
{ "type": "setSetting", "key": "magSensitivity", "value": 0.6 }
{ "type": "calibrate" }
{ "type": "quickReset" }
{ "type": "cancelCalibration" }
{ "type": "magCalibrate", "mac": "B4:E6:2D:36:36:94" }
{ "type": "magCancel", "mac": "B4:E6:2D:36:36:94" }
{ "type": "magReset", "mac": "B4:E6:2D:36:36:94" }
{ "type": "floor", "valid": true, "y": -0.012 }
```

- `calibrate`: full calibration. Stand straight (countdown), then ski pose (countdown).
  Needed once; it finds where each tracker is mounted (front, side, back...).
- `quickReset`: stand straight only. Re-aligns heading, keeps the mounting.
- `setSetting`: `value` is a number or a boolean depending on the key (see `settings`).
- `floor`: the overlay is a regular OpenVR client and can read
  `GetRawZeroPoseToStandingAbsoluteTrackingPose()`. It sends the floor height in raw
  tracking space once per second: `y = -M[1][3]` where `M` is that matrix (raw -> standing).
