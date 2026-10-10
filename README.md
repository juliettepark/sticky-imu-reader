# Sticky IMU readout

A small Python program that connects to a sensing sticky over Bluetooth Low Energy and saves its accelerometer and gyroscope stream to a CSV file. It uses the firmware’s default 100 Hz IMU rate. It does not change the sticky’s config, and it does not subscribe to tactile data.

## Before you run it

Turn the sticky on.

- A blinking blue light and an advertised name starting with `STICKY-SENSE` means sensing firmware is already installed. Connect with this script and leave the firmware as it is.
- A name starting with `STICKY-FB` is a feedback sticky (vibration only). It does not provide IMU data.
<!-- - No blink means the firmware has not been flashed, or the battery is dead. Flashing is documented in the [firmware setup guide](https://stickies.catherineyu.com/docs/use/firmware-setup). The scale factors below match the BMI270 sensing image (`sensing_sticky_BMI270.hex`) at ±16 g and ±2000 °/s. -->

<!-- The phone companion app can hold the only Bluetooth connection. Quit it before running this script. macOS will ask for Bluetooth permission the first time. -->

## Run

Requires Python 3.10 or newer.

```bash
pip install bleak
python sticky_imu.py
```

The script scans for about 8 seconds, connects to the first `STICKY-SENSE` device it hears, prints one line per sample, and writes `imu_YYYYMMDD_HHMMSS.csv` in the current directory. Stop with Ctrl-C.

If more than one sensing sticky is nearby:

```bash
python sticky_imu.py --name STICKY-SENSE-A1B2C3
```

`--output path.csv` chooses the CSV path. `--scan-seconds 15` listens longer.

## What is in the CSV

| Column | Meaning |
| --- | --- |
| `t_fw_ms` | Sticky clock in milliseconds (time since the sticky booted, not wall time) |
| `ax_g`, `ay_g`, `az_g` | Acceleration on the chip’s axes, in g |
| `gx_dps`, `gy_dps`, `gz_dps` | Rotation rate on those axes, in degrees per second |

These axes are fixed to the sticky. The stream does not include position or velocity.

## How the packets are decoded

The sticky advertises, then serves IMU data only after this program connects and subscribes. Notifications arrive on characteristic `00010002-7374-6963-6b79-535449434b59` (sensing service `00010000-7374-6963-6b79-535449434b59`). The short ids in the [BLE API reference](https://stickies.catherineyu.com/docs/develop/ble-api) are the first 32 bits of those ids.

Each notification is little-endian:

- byte 0: number of samples in the packet (1–12)
- each sample, 16 bytes: three signed 16-bit acceleration counts, three signed 16-bit gyro counts, one unsigned 32-bit timestamp

Counts are converted the same way as the [StickyDebugger reference parser](https://github.com/cat-Yu/sensorimotor-stickies/blob/main/app/StickyDebugger/app/src/main/java/com/example/stickydebugger/ble/DataParser.kt):

- acceleration in g = raw / 2048
- rotation in °/s = raw / 16.384

macOS chooses the Bluetooth packet size itself. This script accepts whatever batch size the sticky sends.
