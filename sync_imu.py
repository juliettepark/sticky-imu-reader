#!/usr/bin/env python3
"""Pair a ball recording and a hand recording onto one laptop clock.

The ball decides the rows. Each ball sample becomes one row at that sample's
laptop time, and the hand columns are the latest hand sample at or before that
time. Madgwick runs on each sticky alone, using that sticky's firmware clock,
the same way orient_imu.py does. The raw files are not changed.
calibration.csv records the gyro bias and starting quaternion passed into Madgwick.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from ahrs.common.orientation import acc2q

from calibrate_imu import CSV_HEADER, HOST_TIME_COLUMN
from orient_imu import accepted_window, gyro_bias, mean_acceleration, track_orientations

# One row of combined.csv. t_host is the ball sample's laptop time.
# hand_t_host is the matched hand sample's own laptop time, which can be earlier.
COMBINED_HEADER = (
    "t_host",
    "ball_t_fw_ms",
    "ball_ax_g",
    "ball_ay_g",
    "ball_az_g",
    "ball_gx_dps",
    "ball_gy_dps",
    "ball_gz_dps",
    "ball_qw",
    "ball_qx",
    "ball_qy",
    "ball_qz",
    "hand_t_host",
    "hand_t_fw_ms",
    "hand_ax_g",
    "hand_ay_g",
    "hand_az_g",
    "hand_gx_dps",
    "hand_gy_dps",
    "hand_gz_dps",
    "hand_qw",
    "hand_qx",
    "hand_qy",
    "hand_qz",
)

# One row per sticky: the bias and starting quaternion actually given to Madgwick.
# A hold that cannot be used still gets a row, with the number columns empty.
CALIBRATION_HEADER = ("sticky", "bx_dps", "by_dps", "bz_dps", "qw", "qx", "qy", "qz")

# A loaded sample: the seven measurement values, then the laptop time.
Sample = tuple[float, float, float, float, float, float, float, datetime]
Quaternion = tuple[float, float, float, float]
# None means this sticky's hold was missing or not still, so Madgwick was not started.
Calibration = tuple[tuple[float, float, float] | None, Quaternion | None]


# Read one raw sticky CSV, including t_host. Calibration's loader leaves t_host
# out because the hold uses the sticky clock. Pairing needs the laptop time.
def load_stamped_samples(path: Path) -> list[Sample]:
    with path.open(newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames
        if fieldnames is None or HOST_TIME_COLUMN not in fieldnames:
            raise ValueError(f"{path} is missing {HOST_TIME_COLUMN}")
        missing = [column for column in CSV_HEADER if column not in fieldnames]
        if missing:
            raise ValueError(f"{path} does not have the sticky IMU columns {','.join(CSV_HEADER)}")
        samples: list[Sample] = []
        for row in reader:
            t_fw, ax, ay, az, gx, gy, gz = (float(row[column]) for column in CSV_HEADER)
            host_time = datetime.fromisoformat(row[HOST_TIME_COLUMN])
            samples.append((t_fw, ax, ay, az, gx, gy, gz, host_time))
    return samples


# Run the existing hold and Madgwick path. The quaternion list lines up with
# `samples`: the hold has no quaternion yet, and each later entry is the
# orientation after that sample's update. A failed hold leaves every entry empty
# so the sensor columns can still be written. The calibration pair is the bias
# and starting quaternion passed into the filter, or None when the hold is not used.
def orientations_for(role: str, samples: list[Sample]) -> tuple[list[Quaternion | None], Calibration]:
    measurements = [sample[:7] for sample in samples]
    window = accepted_window(measurements)
    if window is None:
        return [None] * len(samples), (None, None)

    bias = gyro_bias(window)
    # The mean acceleration of the hold is the up direction Madgwick starts from.
    q0 = np.asarray(acc2q(mean_acceleration(window)), dtype=float)
    bx, by, bz = bias
    print(f"{role.capitalize()} gyro bias (dps): ({bx:+.3f}, {by:+.3f}, {bz:+.3f})")
    print(
        f"{role.capitalize()} starting quaternion (w, x, y, z): "
        f"({q0[0]:+.3f}, {q0[1]:+.3f}, {q0[2]:+.3f}, {q0[3]:+.3f})"
    )
    tracked = track_orientations(measurements[len(window) :], bias, q0, window[-1][0])
    quaternions: list[Quaternion | None] = [None] * len(window)
    quaternions.extend(
        (float(row[-4]), float(row[-3]), float(row[-2]), float(row[-1])) for row in tracked
    )
    starting = (float(q0[0]), float(q0[1]), float(q0[2]), float(q0[3]))
    return quaternions, ((float(bx), float(by), float(bz)), starting)


def _calibration_fields(bias: tuple[float, float, float] | None, quaternion: Quaternion | None) -> list[str]:
    if bias is None or quaternion is None:
        return [""] * 7
    return [f"{value:.6f}" for value in (*bias, *quaternion)]


# Save the calibration each sticky actually used, beside combined.csv.
def write_calibration(folder: Path, ball: Calibration, hand: Calibration) -> Path:
    path = folder / "calibration.csv"
    with path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(CALIBRATION_HEADER)
        for role, calibration in (("ball", ball), ("hand", hand)):
            writer.writerow([role, *_calibration_fields(*calibration)])
    return path


# Latest hand sample whose laptop time is at or before the ball sample.
# Samples are sorted by laptop time first. A hand sample from later in the
# recording is not used, even if it arrived in a packet we have already read.
def latest_hand_index(hand_times: list[datetime], ball_time: datetime) -> int | None:
    index = bisect.bisect_right(hand_times, ball_time) - 1
    if index < 0:
        return None
    return index


def _sensor_fields(sample: Sample | None) -> list[str]:
    if sample is None:
        return [""] * 7
    timestamp_ms, ax, ay, az, gx, gy, gz = sample[:7]
    return [
        f"{timestamp_ms:.0f}",
        f"{ax:.6f}",
        f"{ay:.6f}",
        f"{az:.6f}",
        f"{gx:.4f}",
        f"{gy:.4f}",
        f"{gz:.4f}",
    ]


def _quaternion_fields(quaternion: Quaternion | None) -> list[str]:
    if quaternion is None:
        return ["", "", "", ""]
    return [f"{value:.6f}" for value in quaternion]


def _host_field(sample: Sample | None) -> str:
    if sample is None:
        return ""
    return sample[7].isoformat(timespec="milliseconds")


# Build combined.csv beside the two raw files.
def sync_folder(folder: Path) -> int:
    ball = load_stamped_samples(folder / "ball.csv")
    hand = load_stamped_samples(folder / "hand.csv")
    ball_quaternions, ball_calibration = orientations_for("ball", ball)
    hand_quaternions, hand_calibration = orientations_for("hand", hand)

    # Orientation stays in recording order. Pairing searches by laptop time.
    hand_order = sorted(range(len(hand)), key=lambda index: hand[index][7])
    ordered_hand = [hand[index] for index in hand_order]
    ordered_hand_quaternions = [hand_quaternions[index] for index in hand_order]
    hand_times = [sample[7] for sample in ordered_hand]

    combined_path = folder / "combined.csv"
    with combined_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(COMBINED_HEADER)
        for sample, quaternion in zip(ball, ball_quaternions):
            match = latest_hand_index(hand_times, sample[7])
            if match is None:
                hand_sample = None
                hand_quaternion = None
            else:
                hand_sample = ordered_hand[match]
                hand_quaternion = ordered_hand_quaternions[match]
            writer.writerow(
                [
                    _host_field(sample),
                    *_sensor_fields(sample),
                    *_quaternion_fields(quaternion),
                    _host_field(hand_sample),
                    *_sensor_fields(hand_sample),
                    *_quaternion_fields(hand_quaternion),
                ]
            )
    calibration_path = write_calibration(folder, ball_calibration, hand_calibration)
    print(f"Writing {combined_path.resolve()}")
    print(f"Writing {calibration_path.resolve()}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pair ball.csv and hand.csv into combined.csv.")
    parser.add_argument("folder", type=Path, help="Recording folder that contains ball.csv and hand.csv")
    return parser


def main() -> int:
    folder = build_parser().parse_args().folder
    try:
        return sync_folder(folder)
    except (OSError, ValueError, csv.Error) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
