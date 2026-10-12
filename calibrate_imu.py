#!/usr/bin/env python3
"""Estimate gyro bias from the first two seconds of a sticky_imu.py CSV.

The recording must begin with the sticky held still. This script uses that
opening window only. It does not modify the CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path

CSV_HEADER = ("t_fw_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")

# Constants for what we accept as a still window
CALIBRATION_WINDOW_MS = 2000
ACCEL_STILL_STD_G = 0.05
GYRO_STILL_STD_DPS = 2.0
GRAVITY_MIN_G = 0.9
GRAVITY_MAX_G = 1.1


# Read the CSV into one tuple per row: time, acceleration, then rotation rate.
def load_samples(path: Path) -> list[tuple[float, ...]]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None or tuple(reader.fieldnames) != CSV_HEADER:
            raise ValueError(f"{path} does not have the sticky IMU columns {','.join(CSV_HEADER)}")
        samples = []
        for row in reader:
            samples.append(tuple(float(row[column]) for column in CSV_HEADER))
    return samples


# Get the calibration window
# Gather up rows from the first sample until the sticky clock has advanced 2 seconds.
def first_window(samples: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    start_ms = samples[0][0]
    window = []
    for sample in samples:
        window.append(sample)
        if sample[0] - start_ms >= CALIBRATION_WINDOW_MS:
            break
    return window


# True when each axis is steady and the average acceleration is about 1 g.
def window_is_still(samples: list[tuple[float, ...]]) -> bool:
    accel_std = [statistics.stdev(sample[axis] for sample in samples) for axis in (1, 2, 3)]
    gyro_std = [statistics.stdev(sample[axis] for sample in samples) for axis in (4, 5, 6)]
    mean_accel = [statistics.mean(sample[axis] for sample in samples) for axis in (1, 2, 3)]
    gravity_g = math.sqrt(sum(value * value for value in mean_accel))
    accel_still = all(std < ACCEL_STILL_STD_G for std in accel_std)
    gyro_still = all(std < GYRO_STILL_STD_DPS for std in gyro_std)
    gravity_real = GRAVITY_MIN_G <= gravity_g <= GRAVITY_MAX_G
    return accel_still and gyro_still and gravity_real


# Average gx, gy, gz. While the sticky is still, this leftover rate is the gyro bias.
def gyro_bias(samples: list[tuple[float, ...]]) -> tuple[float, float, float]:
    return tuple(statistics.mean(sample[axis] for sample in samples) for axis in (4, 5, 6))


# For if the calibration window is too shakey - not acceptable
# Report that this recording cannot be calibrated, and return an error exit code.
def reject(message: str) -> int:
    print(f"This CSV file sample is bad. {message}", file=sys.stderr)
    return 1


# Check the opening 2 seconds. Print the gyro bias, or reject the file.
def calibrate(path: Path) -> int:
    samples = load_samples(path)
    if len(samples) < 2:
        return reject("It does not contain a 2 second still window.")

    window = first_window(samples)
    duration_ms = window[-1][0] - window[0][0]
    if duration_ms < CALIBRATION_WINDOW_MS or len(window) < 2:
        return reject("It does not cover 2 seconds from the first sample.")
    if not window_is_still(window):
        return reject("The first 2 seconds are not still enough.")

    bx, by, bz = gyro_bias(window)
    print(f"Gyro bias (dps): ({bx:+.3f}, {by:+.3f}, {bz:+.3f})")
    return 0


# Accept the path of a CSV recorded by sticky_imu.py.
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Average gyro bias from the first 2 seconds of a sticky IMU CSV."
    )
    parser.add_argument("csv_path", type=Path, help="CSV recorded by sticky_imu.py")
    return parser


# Run calibration and turn a missing or unreadable file into the same error.
def main() -> int:
    args = build_parser().parse_args()
    try:
        return calibrate(args.csv_path)
    except (OSError, ValueError, csv.Error) as error:
        print(f"This CSV file has a bad calibration window. {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
