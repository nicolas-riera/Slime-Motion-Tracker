# Original firmware backups

Full 4 MB flash dumps of the trackers **before** flashing the SlimeVR-lite firmware.
The `.bin` files are git-ignored on purpose: they contain your WiFi SSID and password.
Keep them somewhere safe.

## What was on the trackers

Read over USB serial (`GET INFO` / `GET CONFIG`) on 2026-10-16:

| Field            | Value                                                          |
|------------------|----------------------------------------------------------------|
| Board            | Wemos D1 Mini (ESP8266EX, 4 MB flash, 26 MHz crystal)          |
| Firmware         | Official SlimeVR-Tracker-ESP, built from `main`, protocol 22   |
| IMU              | ICM-45686 (SoftFusion driver), I2C SDA=D2(4) SCL=D1(5) INT=D5(14) |
| IMU rotation     | `DEG_90` (1.570796 rad); note the board default is `DEG_270`   |
| Magnetometer     | QMC6309 on the ICM-45686 AUX I2C bus (detected, never used)    |
| Battery          | `BAT_EXTERNAL` on A0, shield R 180k, R1 100k, R2 220k          |
| LED              | GPIO2, inverted                                                |

## Backups

| Tracker MAC          | File                              | SHA-256 |
|----------------------|-----------------------------------|---------|
| B4:E6:2D:36:36:94    | `B4E62D363694_full_4MB.bin`       | `30C7844BF91B769EAAB1E6E592C78D118C33781FB1055707E8DB8744CA54D5B2` |

Add a row each time another tracker is backed up (`tools/flash_tracker.py` does it automatically).

## Restoring the original firmware

Exact restore from a dump (firmware + WiFi credentials + calibration):

```
.venv\Scripts\python -m esptool --port COM20 --baud 460800 write-flash 0 firmware\backup\<MAC>_full_4MB.bin
```

If the dump is lost, any official SlimeVR firmware works. Use the SlimeVR web flasher
(https://slimevr-firmware-tool.vercel.app/ or the SlimeVR Server "Firmware tool") with:

- Board: **Wemos D1 Mini**
- IMU: **ICM-45686**, rotation **90°**
- SDA D2, SCL D1, INT D5
- Battery: external, A0, R1 100k / R2 220k / shield 180k
- LED: GPIO2, inverted
