#!/usr/bin/env python3
"""Stream IMU samples from a sensing sticky and write them to a CSV file.

The sticky advertises as STICKY-SENSE-*, then sends batched notifications on
the IMU characteristic once a host subscribes.

With no --name, this records the ball and the hand together into one folder.
Pass --name to record a single sticky the way this script did before.

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
import time
from datetime import datetime, timedelta
from pathlib import Path

from calibrate_imu import HOST_TIME_COLUMN
from recording_summary import RecordingSummary

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData

# Constants from BleConstants.kt in StickyDebugger
SENSING_SERVICE_UUID = "00010000-7374-6963-6b79-535449434b59"
IMU_DATA_UUID = "00010002-7374-6963-6b79-535449434b59"
NAME_PREFIX = "STICKY-SENSE"

BALL_IMU = "STICKY-SENSE-07058C"
HAND_IMU = "STICKY-SENSE-57FCFE"

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


# Scan once for the ball and the hand. Both must appear before recording starts,
# so their holds cover the same time the person is standing still.
async def find_named_stickies(timeout: float, names: dict[str, str]) -> dict[str, BLEDevice] | None:
    found: dict[str, BLEDevice] = {}
    seen: set[str] = set()
    both_found = asyncio.Event()
    wanted = set(names.values())

    def on_detect(device: BLEDevice, adv: AdvertisementData) -> None:
        if not is_sensing_sticky(device, adv):
            return
        if device.address in seen:
            return
        seen.add(device.address)
        label = _advertised_name(device, adv) or "(no name)"
        print(f"Found {label}  address={device.address}")
        if label not in wanted or label in found:
            return
        found[label] = device
        if len(found) == len(wanted):
            both_found.set()

    scanner = BleakScanner(detection_callback=on_detect)
    await scanner.start()
    try:
        try:
            await asyncio.wait_for(both_found.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            pass
    finally:
        await scanner.stop()

    missing = [label for label in wanted if label not in found]
    if missing:
        for label in missing:
            print(f"No sticky named {label} in this scan.", file=sys.stderr)
        return None
    return {role: found[label] for role, label in names.items()}


# ============== STREAM DATA ==============

def parse_imu_packet(data: bytes) -> list[tuple]:
    """Return scaled samples from one IMU notification, or an empty list."""
    global _bad_packet_count
    if not data:
        _bad_packet_count += 1
        return []

    sample_count = data[0]
    expected = 1 + IMU_SAMPLE_BYTES * sample_count

    # Detect if packet is not the expected size
    if not 1 <= sample_count <= 12 or len(data) != expected:
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


# Give every sample in one notification a laptop time.
# A notification is a batch that arrives together. The last sample is the one
# that just triggered the packet, so it keeps the arrival time. Each earlier
# sample was taken earlier on the sticky clock, by last_t_fw_ms minus its own
# t_fw_ms. That gap is in milliseconds and is stepped backward from the arrival.
# The two stickies can then be compared on t_host even though each t_fw_ms
# counts from its own boot.
def host_times_for_packet(samples: list[tuple], arrival: datetime) -> list[datetime]:
    if not samples:
        return []
    last_fw_ms = samples[-1][0]
    host_times = []
    for sample in samples:
        gap_ms = last_fw_ms - sample[0]
        host_times.append(arrival - timedelta(milliseconds=gap_ms))
    return host_times


def format_sample(sample: tuple) -> str:
    timestamp_ms, ax, ay, az, gx, gy, gz = sample
    return (
        f"t={timestamp_ms} ms  "
        f"a=({ax:+.3f}, {ay:+.3f}, {az:+.3f}) g  "
        f"g=({gx:+.1f}, {gy:+.1f}, {gz:+.1f}) dps"
    )


async def _stream(device: BLEDevice, csv_path: Path, summary: RecordingSummary) -> None:
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
                summary.add(sample)
                timestamp_ms = sample[0]
                already_passed = calibration_window_passed
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
                if already_passed:
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


async def stream(device: BLEDevice, csv_path: Path) -> None:
    summary = RecordingSummary()
    bad_packets_at_start = _bad_packet_count
    try:
        await _stream(device, csv_path, summary)
    finally:
        # Runs after notification cleanup and CSV close, including Ctrl-C cancellation.
        print("\n" + summary.report(_bad_packet_count - bad_packets_at_start))


# Record the ball and the hand into one folder. Rows are written as each packet arrives.
# t_host is the laptop time of that sample. Both stickies share one laptop clock,
# built from the moment streaming starts plus a monotonic elapsed time, so a
# wall-clock change during the take does not move the samples.
async def stream_pair(ball: BLEDevice, hand: BLEDevice, folder: Path) -> None:
    session_start = datetime.now()
    mono_start = time.monotonic()

    def packet_arrival() -> datetime:
        return session_start + timedelta(seconds=time.monotonic() - mono_start)

    recordings = {
        "ball": (ball, folder / "ball.csv", RecordingSummary()),
        "hand": (hand, folder / "hand.csv", RecordingSummary()),
    }
    bad_packets = {"ball": 0, "hand": 0}
    header = CSV_HEADER + (HOST_TIME_COLUMN,)

    def on_imu(role: str, writer, csv_file, summary: RecordingSummary):
        calibration_start_ms: int | None = None
        calibration_window_passed = False

        # One callback belongs to one sticky. The batch is stamped once, then
        # each sample inside it is given its own t_host from the firmware gaps.
        def callback(_characteristic, data: bytearray) -> None:
            nonlocal calibration_start_ms, calibration_window_passed
            before = _bad_packet_count
            samples = parse_imu_packet(bytes(data))
            bad_packets[role] += _bad_packet_count - before
            if not samples:
                return
            arrival = packet_arrival()
            for sample, host_time in zip(samples, host_times_for_packet(samples, arrival)):
                summary.add(sample)
                timestamp_ms = sample[0]
                already_passed = calibration_window_passed
                if calibration_start_ms is None:
                    calibration_start_ms = timestamp_ms
                elif (
                    not calibration_window_passed
                    and timestamp_ms - calibration_start_ms >= CALIBRATION_WINDOW_MS
                ):
                    calibration_window_passed = True
                    print(f"{role.capitalize()} calibration window has passed.")
                writer.writerow(
                    (
                        sample[0],
                        f"{sample[1]:.6f}",
                        f"{sample[2]:.6f}",
                        f"{sample[3]:.6f}",
                        f"{sample[4]:.4f}",
                        f"{sample[5]:.4f}",
                        f"{sample[6]:.4f}",
                        host_time.isoformat(timespec="milliseconds"),
                    )
                )
                csv_file.flush()
                if already_passed:
                    print(f"{role} {format_sample(sample)}")

        return callback

    ball_client_cm = BleakClient(ball)
    hand_client_cm = BleakClient(hand)
    try:
        async with ball_client_cm as ball_client, hand_client_cm as hand_client:
            if not ball_client.is_connected or not hand_client.is_connected:
                raise RuntimeError("Could not connect to both stickies")
            print(f"Connected to ball {BALL_IMU} and hand {HAND_IMU}.")
            # Create the folder only after both links are up, so a missed connection leaves nothing behind.
            folder.mkdir(parents=True, exist_ok=True)
            print(f"Writing {folder.resolve()}")
            print("Hold both stickies still.")
            print("Ctrl-C to stop.")
            ball_path = recordings["ball"][1]
            hand_path = recordings["hand"][1]
            with ball_path.open("w", newline="") as ball_file, hand_path.open("w", newline="") as hand_file:
                ball_writer = csv.writer(ball_file)
                hand_writer = csv.writer(hand_file)
                ball_writer.writerow(header)
                hand_writer.writerow(header)
                ball_file.flush()
                hand_file.flush()
                # Subscribe only after both links are up, so both holds start together.
                await ball_client.start_notify(
                    IMU_DATA_UUID,
                    on_imu("ball", ball_writer, ball_file, recordings["ball"][2]),
                )
                await hand_client.start_notify(
                    IMU_DATA_UUID,
                    on_imu("hand", hand_writer, hand_file, recordings["hand"][2]),
                )
                try:
                    while ball_client.is_connected and hand_client.is_connected:
                        await asyncio.sleep(0.5)
                finally:
                    if ball_client.is_connected:
                        await ball_client.stop_notify(IMU_DATA_UUID)
                    if hand_client.is_connected:
                        await hand_client.stop_notify(IMU_DATA_UUID)
    finally:
        # Each sticky has its own clock and its own hold, so each gets its own summary.
        for role in ("ball", "hand"):
            print(f"\n{role.capitalize()}")
            print(recordings[role][2].report(bad_packets[role]))
        # Pair only after both files are closed. The ball clock and Madgwick
        # results are rebuilt from the saved rows, so this can also be run later.
        if (folder / "ball.csv").exists() and (folder / "hand.csv").exists():
            try:
                from sync_imu import sync_folder
            except ImportError as error:
                print(f"Could not write combined.csv. {error}", file=sys.stderr)
            else:
                # Pair the ball and the hand on the laptop clock to create a single synced CSV file
                sync_folder(folder)


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
        help=(
            "Single-sticky CSV path, or the folder for a ball-and-hand recording. "
            "Default: imu_YYYYMMDD_HHMMSS.csv, or a folder of that name for both stickies."
        ),
    )
    return parser


async def main() -> int:
    args = build_parser().parse_args()
    # --name keeps the original one-sticky recording. Otherwise record ball and hand.
    if args.name:
        print(f"Scanning {args.scan_seconds:.0f}s for a sensing sticky. Turn it on and keep it nearby.")
        device = await find_sensing_sticky(args.scan_seconds, args.name)
        if device is None:
            print("No sensing sticky found.", file=sys.stderr)
            return 1
        csv_path = args.output or Path(f"imu_{datetime.now():%Y%m%d_%H%M%S}.csv")
        try:
            await stream(device, csv_path)
        except KeyboardInterrupt:
            print("\nStopped.")
        return 0

    print(
        f"Scanning {args.scan_seconds:.0f}s for the ball ({BALL_IMU}) and the hand ({HAND_IMU}). "
        "Turn them on and keep them nearby."
    )
    devices = await find_named_stickies(args.scan_seconds, {"ball": BALL_IMU, "hand": HAND_IMU})
    if devices is None:
        print("Both stickies are required.", file=sys.stderr)
        return 1
    folder = args.output or Path(f"imu_{datetime.now():%Y%m%d_%H%M%S}")
    try:
        await stream_pair(devices["ball"], devices["hand"], folder)
    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(asyncio.run(main()))
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)
