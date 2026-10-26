# Sticky IMU readout

Record one sensing sticky, or the ball and hand stickies together, over Bluetooth Low Energy. Orientation is estimated after the recording. Each sticky streams at its default 100 Hz. These scripts do not change its config and do not subscribe to tactile data.

Recording uses Python 3.10 or newer and `bleak`. Orientation also uses `ahrs`. The cube view uses `matplotlib`.

```bash
pip install bleak ahrs matplotlib
```

## Before you record

Turn the sticky on.

- A blinking blue light and an advertised name starting with `STICKY-SENSE` means sensing firmware is already installed. Connect with this script and leave the firmware as it is.
- A name starting with `STICKY-FB` is a feedback sticky (vibration only). It does not provide IMU data.
<!-- - No blink means the firmware has not been flashed, or the battery is dead. Flashing is documented in the [firmware setup guide](https://stickies.catherineyu.com/docs/use/firmware-setup). The scale factors below match the BMI270 sensing image (`sensing_sticky_BMI270.hex`) at ±16 g and ±2000 °/s. -->

<!-- The phone companion app can hold the only Bluetooth connection. Quit it before running this script. macOS will ask for Bluetooth permission the first time. -->

## Record the ball and the hand

The ball is `STICKY-SENSE-07058C`. The hand is `STICKY-SENSE-57FCFE`.

```bash
python sticky_imu.py
```

The script scans for about 8 seconds and connects to both stickies before the hold starts. It prints `Hold both stickies still.` Hold both steady until it prints `Ball calibration window has passed.` and `Hand calibration window has passed.` Each window is 2 seconds on that sticky’s own clock, from its first sample. Then move them. Sample lines are prefixed with `ball` or `hand`.

Press Ctrl-C once, then wait. The script prints a summary for each sticky, runs Madgwick, and writes the combined files. It is finished when it prints `Writing .../combined.csv` and `Writing .../calibration.csv`. A second Ctrl-C during that wait stops the sync before those files are saved.

One recording creates a folder, `imu_YYYYMMDD_HHMMSS/`:

| File | Contents |
| --- | --- |
| `ball.csv`, `hand.csv` | Every sample from that sticky, including `t_host` |
| `combined.csv` | One row per ball sample, with the hand sample from that laptop time and both quaternions |
| `calibration.csv` | The gyro bias and starting quaternion passed into Madgwick for each sticky |

`t_fw_ms` is the sticky’s own clock, in milliseconds since that sticky booted. `t_host` is the laptop time of the sample. Inside one Bluetooth packet, the last sample keeps the arrival time and earlier samples are stepped backward by their firmware gaps. The two stickies can then share one clock even though each firmware clock starts at its own boot.

The ball decides the rows in `combined.csv`. The hand columns are the latest hand sample at or before that ball time. If none exists yet, those columns are empty. Quaternion cells are empty during that sticky’s hold. A hold that is missing or not still still leaves a row in `calibration.csv`, with the number columns empty, and the sensor columns in `combined.csv` are still filled.

Rebuild the combined files from a folder that already has `ball.csv` and `hand.csv`:

```bash
python sync_imu.py imu_YYYYMMDD_HHMMSS
```

`--scan-seconds 15` listens longer. `--output some_folder` chooses the recording folder.

## Record one sticky

```bash
python sticky_imu.py --name STICKY-SENSE-A1B2C3
```

This connects to that sticky only. It prints `Hold the sticky still.` Hold it steady until it prints `Calibration window has passed.` Then move it. The raw stream is written to `imu_YYYYMMDD_HHMMSS.csv` in the current directory, without `t_host`. Stop with Ctrl-C.

`--output path.csv` chooses that CSV path.

## Recording summary

When recording stops with Ctrl-C or a disconnect, the script prints a summary for each sticky:
sample count, firmware duration, effective sample rate, timestamp gaps, estimated
missing samples, malformed packets, possible saturation, and whether the opening
2-second calibration hold passed the existing stillness checks. A passing hold
also reports gyro bias. The raw CSV is not changed by the summary.

Missing-sample estimates assume 100 Hz. Possible saturation means an axis reached
99% of the assumed ±16 g or ±2000 °/s range; it is not a hardware saturation flag.
Duplicate or backward timestamps flag timing estimates as unreliable, and gaps
in the calibration hold leave its status unverified.

Run the hardware-free smoke tests with simulated BLE packets:

```bash
python3 -m unittest discover -s . -p smoke_test.py -v
```

## Calibrate

Check the opening 2 seconds and print the gyro bias. This does not change the CSV.

```bash
python calibrate_imu.py imu_YYYYMMDD_HHMMSS.csv
```

A still hold prints `Gyro bias (dps): (...)`. A window that is too short or moving prints `This CSV file sample is bad.`

## Orientation

Use that same hold as the start of a Madgwick filter, then write one quaternion per sample after the hold. The raw CSV is not changed.

```bash
python orient_imu.py imu_YYYYMMDD_HHMMSS.csv
```

This prints the gyro bias and the starting quaternion `(w, x, y, z)`, and writes `imu_YYYYMMDD_HHMMSS_orientation.csv` next to the recording. The hold rows are not in that file. The first written row is already a Madgwick update from the end of the hold. A bad hold prints the same error as calibration and writes no file. A dual-recording folder already contains both quaternions in `combined.csv`; this command is for a single-sticky CSV.

## View

Play one orientation CSV:

```bash
python visualize_orientation.py imu_YYYYMMDD_HHMMSS_orientation.csv
```

Play the ball and the hand from a recording folder. Both cubes follow `t_host`. During a hold, a cube stays at that sticky’s starting quaternion from `calibration.csv`.

```bash
python visualize_orientation.py imu_YYYYMMDD_HHMMSS
```

`--save` writes a GIF beside a single CSV, or `orientation.gif` inside the folder. Close the window to stop.

## What is in the CSV

| Column | Meaning |
| --- | --- |
| `t_fw_ms` | Sticky clock in milliseconds (time since the sticky booted, not wall time) |
| `ax_g`, `ay_g`, `az_g` | Acceleration on the chip’s axes, in g |
| `gx_dps`, `gy_dps`, `gz_dps` | Rotation rate on those axes, in degrees per second |

These axes are fixed to the sticky. The stream does not include position or velocity.

The orientation file keeps those seven columns and adds:

| Column | Meaning |
| --- | --- |
| `qw`, `qx`, `qy`, `qz` | Orientation after the hold, relative to up. Yaw 0 is the direction the sticky faced during calibration. |

`ball.csv` and `hand.csv` add `t_host`, the laptop datetime of that sample. `combined.csv` uses the ball sample’s `t_host` as the row time and prefixes the measurement and quaternion columns with `ball_` and `hand_`. `hand_t_host` is the matched hand sample’s own laptop time. `calibration.csv` has one row per sticky: `sticky`, `bx_dps`, `by_dps`, `bz_dps`, `qw`, `qx`, `qy`, `qz`.

## How the packets are decoded

The sticky advertises, then serves IMU data only after this program connects and subscribes. Notifications arrive on characteristic `00010002-7374-6963-6b79-535449434b59` (sensing service `00010000-7374-6963-6b79-535449434b59`). The short ids in the [BLE API reference](https://stickies.catherineyu.com/docs/develop/ble-api) are the first 32 bits of those ids.

Each notification is little-endian:

- byte 0: number of samples in the packet (1–12)
- each sample, 16 bytes: three signed 16-bit acceleration counts, three signed 16-bit gyro counts, one unsigned 32-bit timestamp

Counts are converted the same way as the [StickyDebugger reference parser](https://github.com/cat-Yu/sensorimotor-stickies/blob/main/app/StickyDebugger/app/src/main/java/com/example/stickydebugger/ble/DataParser.kt):

- acceleration in g = raw / 2048
- rotation in °/s = raw / 16.384

macOS chooses the Bluetooth packet size itself. This script accepts whatever batch size the sticky sends.
