#!/usr/bin/env python3
"""Stream IMU samples from a sensing sticky and write them to a CSV file.

The sticky advertises as STICKY-SENSE-*, then sends batched notifications on
the IMU characteristic once a host subscribes.

Packet layout (little-endian), matching the StickyDebugger reference parser:
  byte 0: sample_count
  each sample (16 bytes): i16 ax, ay, az, i16 gx, gy, gz, u32 timestamp_ms
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import statistics
import struct
import sys
from datetime import datetime
from pathlib import Path

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

# Constants from BleConstants.kt in StickyDebugger
SENSING_SERVICE_UUID = "00010000-7374-6963-6b79-535449434b59"
IMU_DATA_UUID = "00010002-7374-6963-6b79-535449434b59"
NAME_PREFIX = "STICKY-SENSE"

IMU_SAMPLE_BYTES = 16
# Six signed int16 values followed by an unsigned int32 timestamp.
IMU_SAMPLE = struct.Struct("<6hI")
ACCEL_COUNTS_PER_G = 2048.0
GYRO_COUNTS_PER_DPS = 16.384

CSV_HEADER = ("t_fw_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")
CALIBRATION_WINDOW_MS = 2000 # Time to collect samples for calibration
ACCEL_STILL_STD_G = 0.05
GYRO_STILL_STD_DPS = 2.0
GRAVITY_MIN_G = 0.9
GRAVITY_MAX_G = 1.1
_bad_packet_count = 0


class OrientationTracker:
    """Holds the calibration window, then the orientation change since that pose.

    The calibrating phase stores samples until they span two seconds of the
    sticky's clock, then keeps that window only if the sticky was still.
    An accepted window saves the gyro bias, gravity, identity orientation,
    and the window's last timestamp, and the phase becomes tracking.
    """

    def __init__(self) -> None:
        self.phase = "calibrating"
        self.samples: list[tuple] = []
        self.window_ready = False
        # Averages of gyro samples collected during the calibration window.
        # Since sticky is supposed to be still, average these values to calculate its bias.
        self.gyro_bias: tuple[float, float, float] | None = None
        self.gravity: tuple[float, float, float] | None = None
        # w, x, y, z. None until the still window is accepted.
        self.quaternion: tuple[float, float, float, float] | None = None
        self.last_timestamp_ms: int | None = None

    def handle_sample(self, sample: tuple) -> None:
        if self.phase != "calibrating" or self.window_ready:
            return
        self.samples.append(sample)
        if len(self.samples) < 2:
            return
        duration_ms = self.samples[-1][0] - self.samples[0][0]
        if duration_ms < CALIBRATION_WINDOW_MS:
            return
        self.window_ready = True
        if not self._window_is_still():
            self.samples.clear()
            self.window_ready = False
            print("Hold still to calibrate.")
            return
        self._accept_window()

    def _accept_window(self) -> None:
        """Store the still window as the zero pose and start tracking."""
        self.gyro_bias = tuple(
            statistics.mean(sample[axis] for sample in self.samples) for axis in (4, 5, 6)
        )
        self.gravity = tuple(
            statistics.mean(sample[axis] for sample in self.samples) for axis in (1, 2, 3)
        )
        self.quaternion = (1.0, 0.0, 0.0, 0.0)
        self.last_timestamp_ms = self.samples[-1][0]
        self.phase = "tracking"
        bx, by, bz = self.gyro_bias
        gx, gy, gz = self.gravity
        print(f"Gyro bias (dps): ({bx:+.3f}, {by:+.3f}, {bz:+.3f})")
        print(f"Gravity (g): ({gx:+.3f}, {gy:+.3f}, {gz:+.3f})")
        print("Calibrated. This pose is zero.")

    def _window_is_still(self) -> bool:
        """A still window has steady axes and a mean acceleration near 1 g."""
        accel_std = [
            statistics.stdev(sample[axis] for sample in self.samples) for axis in (1, 2, 3)
        ]
        gyro_std = [
            statistics.stdev(sample[axis] for sample in self.samples) for axis in (4, 5, 6)
        ]
        mean_accel = [
            statistics.mean(sample[axis] for sample in self.samples) for axis in (1, 2, 3)
        ]
        gravity_g = math.sqrt(sum(value * value for value in mean_accel))
        accel_still = all(std < ACCEL_STILL_STD_G for std in accel_std)
        gyro_still = all(std < GYRO_STILL_STD_DPS for std in gyro_std)
        gravity_real = GRAVITY_MIN_G <= gravity_g <= GRAVITY_MAX_G
        return accel_still and gyro_still and gravity_real


def parse_imu_packet(data: bytes) -> list[tuple]:
    """Return scaled samples from one IMU notification, or an empty list."""
    if not data:
        return []

    sample_count = data[0]
    expected = 1 + IMU_SAMPLE_BYTES * sample_count
    if sample_count == 0 or len(data) != expected:
        global _bad_packet_count
        _bad_packet_count += 1
        if _bad_packet_count == 1 or _bad_packet_count % 100 == 0:
            print(
                f"Skipping IMU packet: {len(data)} bytes, sample_count={sample_count}, "
                f"expected {expected}",
                file=sys.stderr,
            )
        return []

    samples = []
    for index in range(sample_count):
        offset = 1 + index * IMU_SAMPLE_BYTES
        ax, ay, az, gx, gy, gz, timestamp_ms = IMU_SAMPLE.unpack_from(data, offset)
        samples.append(
            (
                timestamp_ms,
                ax / ACCEL_COUNTS_PER_G,
                ay / ACCEL_COUNTS_PER_G,
                az / ACCEL_COUNTS_PER_G,
                gx / GYRO_COUNTS_PER_DPS,
                gy / GYRO_COUNTS_PER_DPS,
                gz / GYRO_COUNTS_PER_DPS,
            )
        )
    return samples


def _advertised_name(device: BLEDevice, adv: AdvertisementData) -> str:
    return adv.local_name or device.name or ""


def is_sensing_sticky(device: BLEDevice, adv: AdvertisementData) -> bool:
    name = _advertised_name(device, adv)
    if name.startswith(NAME_PREFIX):
        return True
    advertised = {uuid.lower() for uuid in (adv.service_uuids or [])}
    return SENSING_SERVICE_UUID in advertised


async def find_sensing_sticky(timeout: float, name: str | None) -> BLEDevice | None:
    """Scan for sensing stickies. An exact --name returns as soon as that one appears."""
    found: list[tuple[BLEDevice, str]] = []
    seen: set[str] = set()
    wanted = asyncio.Event()

    def on_detect(device: BLEDevice, adv: AdvertisementData) -> None:
        if not is_sensing_sticky(device, adv):
            return
        if device.address in seen:
            return
        seen.add(device.address)
        label = _advertised_name(device, adv) or "(no name)"
        print(f"Found {label}  address={device.address}")
        found.append((device, label))
        if name and label == name:
            wanted.set()

    scanner = BleakScanner(detection_callback=on_detect)
    await scanner.start()
    try:
        if name:
            try:
                await asyncio.wait_for(wanted.wait(), timeout=timeout)
            except asyncio.TimeoutError:
                pass
        else:
            await asyncio.sleep(timeout)
    finally:
        await scanner.stop()

    if name:
        for device, label in found:
            if label == name:
                return device
        print(f"No sticky named {name} in this scan.", file=sys.stderr)
        return None

    if not found:
        return None
    device, label = found[0]
    if len(found) > 1:
        print(f"Several sensing stickies found; connecting to {label}.")
        print("Pass --name STICKY-SENSE-XXXXXX to choose one.")
    return device


def format_sample(sample: tuple) -> str:
    timestamp_ms, ax, ay, az, gx, gy, gz = sample
    return (
        f"t={timestamp_ms} ms  "
        f"a=({ax:+.3f}, {ay:+.3f}, {az:+.3f}) g  "
        f"g=({gx:+.1f}, {gy:+.1f}, {gz:+.1f}) dps"
    )


async def stream(device: BLEDevice, csv_path: Path) -> None:
    with csv_path.open("w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(CSV_HEADER)
        csv_file.flush()
        tracker = OrientationTracker()

        # Bleak runs this on the asyncio thread, so writing the file here is safe.
        def on_imu(_characteristic, data: bytearray) -> None:
            for sample in parse_imu_packet(bytes(data)):
                tracker.handle_sample(sample)
                writer.writerow(
                    (
                        sample[0],
                        f"{sample[1]:.6f}",
                        f"{sample[2]:.6f}",
                        f"{sample[3]:.6f}",
                        f"{sample[4]:.4f}",
                        f"{sample[5]:.4f}",
                        f"{sample[6]:.4f}",
                    )
                )
                csv_file.flush()
                print(format_sample(sample))

        async with BleakClient(device) as client:
            if not client.is_connected:
                raise RuntimeError(f"Could not connect to {device.address}")
            await client.start_notify(IMU_DATA_UUID, on_imu)
            print(f"Connected to {device.name or device.address}. Streaming IMU at the firmware default (100 Hz).")
            print(f"Writing {csv_path.resolve()}")
            print("Hold the sticky still.")
            print("Ctrl-C to stop.")
            try:
                while client.is_connected:
                    await asyncio.sleep(0.5)
            finally:
                if client.is_connected:
                    await client.stop_notify(IMU_DATA_UUID)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Stream IMU data from a sensing sticky.")
    parser.add_argument(
        "--name",
        help="Exact advertised name to connect to (for example STICKY-SENSE-A1B2C3).",
    )
    parser.add_argument(
        "--scan-seconds",
        type=float,
        default=8.0,
        help="How long to listen for advertisements (default: 8).",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV path. Default: imu_YYYYMMDD_HHMMSS.csv in the current directory.",
    )
    return parser


async def main() -> int:
    args = build_parser().parse_args()
    print(f"Scanning {args.scan_seconds:.0f}s for a sensing sticky. Turn it on and keep it nearby.")
    device = await find_sensing_sticky(args.scan_seconds, args.name)
    if device is None:
        print(
            "No sensing sticky found.",
            file=sys.stderr,
        )
        return 1

    csv_path = args.output or Path(f"imu_{datetime.now():%Y%m%d_%H%M%S}.csv")
    try:
        await stream(device, csv_path)
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)
