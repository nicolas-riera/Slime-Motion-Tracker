# Driver <-> overlay protocol

The SteamVR driver (`driver_slime_motion_tracker.dll`) owns all state and logic. The dashboard
overlay (`SlimeMotionTracker.exe`) is a thin UI: it displays the state the driver
publishes and sends commands. The driver starts the overlay process when SteamVR starts.

- Transport: UDP on localhost. The driver listens on `127.0.0.1:6971`.
- Every datagram is one UTF-8 JSON object with a `type` field.
- The overlay sends `hello` once per second. The driver answers by streaming `state`
  messages (10 Hz) to the address the last overlay datagram came from. If the overlay
  receives no `state` for 2 s it keeps the last one on screen, disabled.
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
  "ready": true,
  "calibration": { "phase": "idle", "quick": false, "countdown": 0.0, "message": "" },
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
  Automatic: `three` while the tracker mapped to the waist is connected and calibrated
  (after 0.5 s), `ankles` once it has been missing for 2 s.
- `ready`: every mapped tracker is connected and calibrated, and both ankles are mapped.
- `measuredEyeHeight`: HMD height above the floor captured at the last calibration, or `null`.
- `calibration.phase`: `idle` | `stand` | `ski` | `done` | `error`.
  During `stand` and `ski`, `countdown` is the time left before the pose is captured.
  `quick`: the running (or last) calibration is a quick reset.
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
- `trackers[].transport`: `wifi` or `espnow` (through the USB dongle); `dongle`: the USB
  dongle status (`connected`, `port`, `mac`, `channel`, `version`, `trackers`).

### View streams

While the overlay shows a 3D view (see the `view` command), the driver also sends:

```json
{ "type": "pose", "valid": true, "ready": true, "floor": -0.01, "floorKnown": true, "mode": "three",
  "head": { "valid": true, "p": [0, 1.65, 0], "q": [1, 0, 0, 0] },
  "joints": { "neck": [..], "pelvis": [..], "hip0": [..], "hipL": [..], "hipR": [..],
              "kneeL": [..], "kneeR": [..], "ankleL": [..], "ankleR": [..] },
  "pelvisQ": [w, x, y, z], "feetQ": [[w, x, y, z], [w, x, y, z]], "planted": [true, false],
  "hands": [ { "valid": true, "p": [..], "q": [..] }, { "valid": false, "p": [..], "q": [..] } ] }
```

at ~30 Hz: positions in the SteamVR raw tracking space (Y up, meters), quaternions
`[w, x, y, z]`. And, when a tracker is selected, at ~20 Hz:

```json
{ "type": "detail", "mac": "D8:BF:C0:F9:F0:F9", "connected": true, "firmware": "0.6.1",
  "board": 3, "imuType": 10, "mcu": 1, "protocolVersion": 22, "bootCount": 1, "hasRotation": true,
  "q": [w, x, y, z], "qRaw": [w, x, y, z], "gyro": [x, y, z], "accel": [x, y, z],
  "hasMag": true, "magRaw": [x, y, z], "magField": [x, y, z],
  "mag": { "state": "ok", "progress": 1, "valid": true, "residual": 0.03, "dip": 61.8,
           "norm": 1.01, "fieldDip": 62.0, "hasHeading": true, "heading": 12.5 },
  "magView": { "samples": [[x, y, z], ...], "bins": [26 counts], "binDirs": [[x, y, z], ...],
               "target": 3, "guided": true, "progress": 0.4 } }
```

`q` is the mag-corrected orientation, `qRaw` the device fusion (both sensor -> world).
`gyro` (rad/s), `accel` (linear, m/s^2), `magRaw` (counts) and `magField` (calibrated,
units of the field norm, only with a valid calibration) are in the sensor frame.
`magView` (raw mag samples, at most 600, and the orientation bins: where "up" is in the
sensor frame) is included while the overlay asks for it or a guided calibration runs.

## Overlay -> driver

```json
{ "type": "hello" }
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
{ "type": "resetSettings" }
{ "type": "view", "visible": true, "tracker": "D8:BF:C0:F9:F0:F9", "magView": false }
```

- `calibrate`: full calibration. Stand straight (countdown), then ski pose (countdown).
  Needed once; it finds where each tracker is mounted (front, side, back...).
- `quickReset`: stand straight only. Re-aligns heading, keeps the mounting.
- `setSetting`: `value` is a number or a boolean depending on the key (see `settings`).
- `resetSettings`: restores the default `settings` (not the mapping nor the calibrations).
- `view`: the overlay shows a 3D view (skeleton, tracker page, mag calibration page).
  Re-sent every ~0.5 s while it applies, expires after 2 s (`visible: false` stops it
  at once). `tracker` (empty for none) selects the `detail` stream, `magView` adds the
  mag samples to it.
- `floor`: the overlay is a regular OpenVR client and can read
  `GetRawZeroPoseToStandingAbsoluteTrackingPose()`. It sends the floor height in raw
  tracking space once per second: `y = -M[1][3]` where `M` is that matrix (raw -> standing).
