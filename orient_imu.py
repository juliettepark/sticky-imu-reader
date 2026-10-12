#!/usr/bin/env python3
"""Estimate orientation for the rows after a sticky_imu.py calibration hold.

The first two seconds must be a still hold. Their average acceleration becomes
the starting quaternion, and their average gyro reading is removed as bias.
Madgwick then tracks orientation only for samples after that hold. The raw CSV
is not changed, and the hold itself is left out of the orientation file.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

import numpy as np
from ahrs.common.orientation import acc2q
from ahrs.filters import Madgwick

from calibrate_imu import (
    CALIBRATION_WINDOW_MS,
    CSV_HEADER,
    first_window,
    gyro_bias,
    load_samples,
    reject,
    window_is_still,
)

# Library default. Lower it and reprocess if a throw pulls the tilt off course.
MADGWICK_GAIN = 0.033
# Ignore a gap long enough to be a dropped connection rather than one sample.
MAX_DT_S = 0.2

# Add quaternion columns to the CSV header.
# Full header is (time, acceleration, rotation) + (quaternion)
ORIENTATION_HEADER = CSV_HEADER + ("qw", "qx", "qy", "qz")


# Average ax, ay, az over the still window. This is the up-push of the hold.
# We need this to provide Madgwick filter with a starting initial orientation
def mean_acceleration(samples: list[tuple[float, ...]]) -> np.ndarray:
    return np.array([statistics.mean(sample[axis] for sample in samples) for axis in (1, 2, 3)])


# Name the processed file next to the raw recording.
def orientation_path(path: Path) -> Path:
    return path.with_name(f"{path.stem}_orientation{path.suffix}")


# Return the still window, or None after reporting why this recording cannot be used.
def accepted_window(samples: list[tuple[float, ...]]) -> list[tuple[float, ...]] | None:
    if len(samples) < 2:
        reject("It does not contain a 2 second still window.")
        return None
    window = first_window(samples)
    duration_ms = window[-1][0] - window[0][0]
    if duration_ms < CALIBRATION_WINDOW_MS or len(window) < 2:
        reject("It does not cover 2 seconds from the first sample.")
        return None
    if not window_is_still(window):
        reject("The first 2 seconds are not still enough.")
        return None
    return window


# Track orientation for samples after the hold.
# q0 is the filter's starting orientation. The first written row is already a Madgwick update.
def track_orientations(
    samples: list[tuple[float, ...]],
    bias_dps: tuple[float, float, float],
    q0: np.ndarray,
    previous_t: float,
) -> list[tuple[float, ...]]:
    madgwick = Madgwick(frequency=100.0, gain=MADGWICK_GAIN)
    bias = np.array(bias_dps)
    q = np.asarray(q0, dtype=float)
    rows: list[tuple[float, ...]] = []
    for sample in samples:
        timestamp_ms = sample[0]
        dt = (timestamp_ms - previous_t) / 1000.0
        if 0 < dt <= MAX_DT_S:
            # Subtract the gyro bias from the gyro data first to eliminate the drift
            gyro_dps = np.array(sample[4:7]) - bias
            # Convert to radians per second (as expected by Madgwick filter)
            gyro_rad_s = np.deg2rad(gyro_dps)
            # Get the acceleration in g
            acc_g = np.array(sample[1:4])
            # Update the orientation using the Madgwick filter
            # q is the new orientation quaternion
            q = np.asarray(
                madgwick.updateIMU(q, gyr=gyro_rad_s, acc=acc_g, dt=dt),
                dtype=float,
            )
        previous_t = timestamp_ms
        # Record the new quaternion to the row
        rows.append((*sample, float(q[0]), float(q[1]), float(q[2]), float(q[3])))
    return rows


# Write the raw columns plus qw, qx, qy, qz.
def write_orientation(path: Path, rows: list[tuple[float, ...]]) -> None:
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(ORIENTATION_HEADER)
        for row in rows:
            timestamp_ms, ax, ay, az, gx, gy, gz, qw, qx, qy, qz = row
            writer.writerow(
                (
                    f"{timestamp_ms:.0f}",
                    f"{ax:.6f}",
                    f"{ay:.6f}",
                    f"{az:.6f}",
                    f"{gx:.4f}",
                    f"{gy:.4f}",
                    f"{gz:.4f}",
                    f"{qw:.6f}",
                    f"{qx:.6f}",
                    f"{qy:.6f}",
                    f"{qz:.6f}",
                )
            )


# Calibrate, track, and write the orientation CSV.
def orient(path: Path) -> int:
    samples = load_samples(path)
    window = accepted_window(samples)
    if window is None:
        return 1

    bias = gyro_bias(window)
    # Use the mean acceleration over the calibration window to get a better initial (ax, ay, az)
    # Pass to acc2q to get a starting quaternion from this
    q0 = np.asarray(acc2q(mean_acceleration(window)), dtype=float)
    # The hold produced q0. Madgwick starts on the next sample, timed from the end of the hold.
    # Slice off the calibration window to only track orientation of samples after calibration.
    rows = track_orientations(samples[len(window) :], bias, q0, window[-1][0])
    out_path = orientation_path(path)
    write_orientation(out_path, rows)

    bx, by, bz = bias
    print(f"Gyro bias (dps): ({bx:+.3f}, {by:+.3f}, {bz:+.3f})")
    print(
        "Starting quaternion (w, x, y, z): "
        f"({q0[0]:+.3f}, {q0[1]:+.3f}, {q0[2]:+.3f}, {q0[3]:+.3f})"
    )
    print(f"Writing {out_path.resolve()}")
    return 0


# Accept the path of a CSV recorded by sticky_imu.py.
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Add Madgwick orientation quaternions to a sticky IMU CSV."
    )
    parser.add_argument("csv_path", type=Path, help="CSV recorded by sticky_imu.py")
    return parser


# Run orientation tracking and turn a missing or unreadable file into the same error.
def main() -> int:
    args = build_parser().parse_args()
    try:
        return orient(args.csv_path)
    except (OSError, ValueError, csv.Error) as error:
        print(f"This CSV file sample is bad. {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
