# Original firmware backups

Full 4 MB flash dumps of the trackers **before** flashing the Slime Motion Tracker firmware.
The `.bin` files are git-ignored on purpose: they contain your WiFi SSID and password.
Keep them somewhere safe.

## What was on the trackers

Read over USB serial (`GET INFO` / `GET CONFIG`) on 2026-10-16:

| Field            | Value                                                          |
|------------------|----------------------------------------------------------------|
| Board            | Wemos D1 Mini (ESP8266EX, 4 MB flash, 26 MHz crystal)          |
| Firmware         | Official SlimeVR-Tracker-ESP, built from `main`, protocol 22   |
| IMU              | ICM-45686 (SoftFusion driver), I2C SDA=D2(4) SCL=D1(5) INT=D5(14) |
| IMU rotation     | +1.570796 rad = `DEG_270` in current firmware macros (board default) |
| Magnetometer     | QMC6309 on the ICM-45686 AUX I2C bus (detected, never used)    |
| Battery          | `BAT_EXTERNAL` on A0, shield R 180k, R1 100k, R2 220k          |
| LED              | GPIO2, inverted                                                |

## Backups

| Tracker MAC          | File                              | SHA-256 |
|----------------------|-----------------------------------|---------|
| B4:E6:2D:36:36:94    | `B4E62D363694_full_4MB.bin`       | `30C7844BF91B769EAAB1E6E592C78D118C33781FB1055707E8DB8744CA54D5B2` |
| 2C:F4:32:7B:FF:D1    | `2CF4327BFFD1_full_4MB.bin`       | `A2501A5A909693B261357FD1C170FDCF87BC4065C8F165CAA7AA48A2B2861392` |
| A4:CF:12:EE:B1:AB    | `A4CF12EEB1AB_full_4MB.bin`       | `2ECC5171AB8C236CBAE83C22171AAA4300DB0D3A82F82D0DA45829E091A4F3AF` |
| D8:BF:C0:F9:F0:F9    | `D8BFC0F9F0F9_full_4MB.bin`       | `B083FD9AC00DDC2F8A1290ED66FFBE80DF398B9AF24F602D1565A40A26DB5254` |
| 48:3F:DA:63:8B:80    | `483FDA638B80_full_4MB.bin`       | `E9B210DC984F1336D15CEBEE03657260916FA59CBDFF2493B84177157BA0BDED` |
| EC:FA:BC:79:B9:60    | `ECFABC79B960_full_4MB.bin`       | `AE5BFB109580711B10D94D4B512873D2892BD36E65DA006BB9445EA2540E892F` (flashed as the ESP-NOW USB dongle, support since removed: restore this backup to reuse it) |

Add a row each time another tracker is backed up (`tools/flash_tracker.py` does it automatically).

## Restoring the original firmware

Exact restore from a dump (firmware + WiFi credentials + calibration):

```
.venv\Scripts\python -m esptool --port COM20 --baud 460800 write-flash 0 firmware\backup\<MAC>_full_4MB.bin
```

If the dump is lost, any official SlimeVR firmware works. Use the SlimeVR web flasher
(https://slimevr-firmware-tool.vercel.app/ or the SlimeVR Server "Firmware tool") with:

- Board: **Wemos D1 Mini**
- IMU: **ICM-45686**, rotation **270°** (board default)
- SDA D2, SCL D1, INT D5
- Battery: external, A0, R1 100k / R2 220k / shield 180k
- LED: GPIO2, inverted
