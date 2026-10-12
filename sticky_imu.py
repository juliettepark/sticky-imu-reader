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

# Values from StickyDebugger example application.
# BMI270 chip can sense += 16g.
# Signed int16 so largest magnitude is 32767.
# Thus, for 16gs, 32767 / 16g = 2048.0 counts per g.
ACCEL_COUNTS_PER_G = 2048.0 # Divide to get raw value in g. 2048 = 1g
GYRO_COUNTS_PER_DPS = 16.384 # Divide to get raw value in degrees per second. 16.384 = 1 degree per second

CSV_HEADER = ("t_fw_ms", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")
CALIBRATION_WINDOW_MS = 2000
_bad_packet_count = 0


# ============== DETECT STICKIES ==============

def _advertised_name(device: BLEDevice, adv: AdvertisementData) -> str:
    return adv.local_name or device.name or ""


# Helper to check if device is a sensing sticky
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

    # Callback function for when a device is detected
    # Only add sticky to list if it is a unique sensing sticky
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
            wanted.set() # Set event to indicate we found the desired sticky

    # Keep scanning for devices until we find the desired sticky
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


# ============== STREAM DATA ==============

def parse_imu_packet(data: bytes) -> list[tuple]:
    """Return scaled samples from one IMU notification, or an empty list."""
    if not data:
        return []

    sample_count = data[0]
    expected = 1 + IMU_SAMPLE_BYTES * sample_count

    # Detect if packet is not the expected size
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
        # Find the offset of the next sample within the packet
        offset = 1 + index * IMU_SAMPLE_BYTES
        ax, ay, az, gx, gy, gz, timestamp_ms = IMU_SAMPLE.unpack_from(data, offset)
        # Save samples as (time in ms, acceleration in g, rotation in °/s)
        # Divide to convert raw values to g and °/s
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
        calibration_start_ms: int | None = None
        calibration_window_passed = False

        # Bleak runs this on the asyncio thread, so writing the file here is safe.
        # Given a batch of samples, parse and write them to the CSV file
        def on_imu(_characteristic, data: bytearray) -> None:
            nonlocal calibration_start_ms, calibration_window_passed
            for sample in parse_imu_packet(bytes(data)):
                timestamp_ms = sample[0]
                if calibration_start_ms is None:
                    calibration_start_ms = timestamp_ms
                elif (
                    not calibration_window_passed
                    and timestamp_ms - calibration_start_ms >= CALIBRATION_WINDOW_MS
                ):
                    calibration_window_passed = True
                    print("Calibration window has passed.")
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
            print(f"Connected to {device.name or device.address}. Streaming IMU at the firmware default (100 Hz).")
            print(f"Writing {csv_path.resolve()}")
            print("Hold the sticky still.")
            print("Ctrl-C to stop.")
            # Subscribe to IMU characteristic notifications and write them to the CSV file
            await client.start_notify(IMU_DATA_UUID, on_imu)
            try:
                # Loop to keep connection alive and handle notifications
                while client.is_connected:
                    # loop and imu stream share one thread
                    # will check every 0.5 seconds to see if we should exit the loop
                    # In the meantime, it will handle notifications from the IMU characteristic
                    await asyncio.sleep(0.5)
            finally:
                # Unsubscribe from notifications when we're done
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

    # Scan for a sensing sticky
    device = await find_sensing_sticky(args.scan_seconds, args.name)
    if device is None:
        print(
            "No sensing sticky found.",
            file=sys.stderr,
        )
        return 1

    # If found, prepare output file and stream data
    # Continue until user stops with Ctrl-C
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
