"""Hardware-free smoke tests. Fake only BLE transport; exercise real decoding and CSV writing."""
import asyncio
import contextlib
import csv
import io
import struct
import sys
import tempfile
import types
import unittest
from datetime import datetime, timedelta
from pathlib import Path

# These stand-ins are confined to this test process.
for name in ('bleak', 'bleak.backends', 'bleak.backends.device', 'bleak.backends.scanner'):
    sys.modules[name] = types.ModuleType(name)
sys.modules['bleak'].BleakClient = object
sys.modules['bleak'].BleakScanner = object
sys.modules['bleak.backends.device'].BLEDevice = object
sys.modules['bleak.backends.scanner'].AdvertisementData = object

import calibrate_imu
import sticky_imu
from recording_summary import RecordingSummary


def packet(t, ax=0):
    return bytes([1]) + struct.pack('<6hI', ax, 0, 2048, 0, 0, 0, t)


class SmokeTests(unittest.TestCase):
    def test_clean_recording(self):
        summary = RecordingSummary()
        for t in range(0, 3010, 10):
            summary.add(sticky_imu.parse_imu_packet(packet(t))[0])
        report = summary.report()
        self.assertIn('100.00 Hz', report)
        self.assertIn('Gaps >15 ms: 0', report)
        self.assertIn('Calibration hold: PASS', report)
        print(report)

    def test_faults(self):
        summary = RecordingSummary()
        for t in (0, 10, 40, 40, 30):
            summary.add(sticky_imu.parse_imu_packet(packet(t, 32767))[0])
        report = summary.report()
        self.assertIn('estimated missing samples: 2', report)
        self.assertIn('Duplicate/backward timestamps: 2', report)
        self.assertIn('ax=5', report)
        self.assertIn('Calibration hold: unverified', report)
        for data in (b'', b'\x01', bytes([13]) + b'\0' * 208):
            self.assertEqual(sticky_imu.parse_imu_packet(data), [])

    def test_empty_and_rollover(self):
        self.assertIn('unavailable', RecordingSummary().report())
        summary = RecordingSummary()
        for t in (2**32 - 10, 0, 10):
            summary.add((t, 0, 0, 1, 0, 0, 0))
        self.assertIn('100.00 Hz', summary.report())
        self.assertEqual(summary.invalid_timestamps, 0)

    def test_moving_and_sparse_hold(self):
        summary = RecordingSummary()
        for t in range(0, 2010, 10):
            summary.add((t, 0, 0, 1, 10 if t % 20 else -10, 0, 0))
        self.assertIn('Calibration hold: FAIL', summary.report())
        summary = RecordingSummary()
        for t in (0, 2000):
            summary.add((t, 0, 0, 1, 0, 0, 0))
        self.assertIn('unverified (timestamp gaps)', summary.report())

    def test_stream_cleanup(self):
        for cancel in (False, True):
            class FakeClient:
                def __init__(self, device):
                    self.is_connected = True
                    self.stopped = False
                async def __aenter__(self):
                    return self
                async def __aexit__(self, *args):
                    self.is_connected = False
                async def start_notify(self, uuid, callback):
                    for t in range(0, 2010, 10):
                        callback(None, bytearray(packet(t)))
                    callback(None, bytearray(b'\x01'))
                    if not cancel:
                        self.is_connected = False
                    else:
                        asyncio.get_running_loop().call_soon(asyncio.current_task().cancel)
                async def stop_notify(self, uuid):
                    self.stopped = True
            sticky_imu.BleakClient = FakeClient
            device = types.SimpleNamespace(name='TEST', address='fake')
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'imu.csv'
                output = io.StringIO()
                with contextlib.redirect_stdout(output):
                    try:
                        asyncio.run(sticky_imu.stream(device, path))
                    except asyncio.CancelledError:
                        self.assertTrue(cancel)
                with path.open(newline='') as recorded:
                    rows = list(csv.reader(recorded))
                self.assertEqual(len(rows), 202)
                self.assertEqual(tuple(rows[0]), sticky_imu.CSV_HEADER)
                self.assertIn('Calibration hold: PASS', output.getvalue())
                self.assertIn('Malformed packets skipped: 1', output.getvalue())
                self.assertEqual(output.getvalue().count('Recording summary'), 1)

    def test_load_samples_accepts_host_time(self):
        # The original seven columns still load, and t_host is allowed beside them.
        # The returned tuples stay on the sticky clock so the hold check is unchanged.
        header = list(calibrate_imu.CSV_HEADER)
        row = ["1000", "0", "0", "1", "0.1", "0.2", "0.3"]
        with tempfile.TemporaryDirectory() as directory:
            plain = Path(directory) / "plain.csv"
            stamped = Path(directory) / "stamped.csv"
            odd = Path(directory) / "odd.csv"
            for path, fields, values in (
                (plain, header, row),
                (stamped, header + ["t_host"], row + ["2026-10-26T00:00:00.032"]),
            ):
                with path.open("w", newline="") as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow(fields)
                    writer.writerow(values)
                loaded = calibrate_imu.load_samples(path)
                self.assertEqual(loaded, [(1000.0, 0.0, 0.0, 1.0, 0.1, 0.2, 0.3)])
            with odd.open("w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(header + ["other"])
                writer.writerow(row + ["x"])
            with self.assertRaises(ValueError):
                calibrate_imu.load_samples(odd)

    def test_host_times_follow_firmware_gaps(self):
        # One packet arrives at 32 ms. Firmware times 1000, 1010, 1020 ms land at 12, 22, and 32 ms.
        arrival = datetime(2026, 10, 26, 0, 0, 0) + timedelta(milliseconds=32)
        samples = [(1000, 0, 0, 1, 0, 0, 0), (1010, 0, 0, 1, 0, 0, 0), (1020, 0, 0, 1, 0, 0, 0)]
        host_times = sticky_imu.host_times_for_packet(samples, arrival)
        base = datetime(2026, 10, 26, 0, 0, 0)
        self.assertEqual(host_times, [
            base + timedelta(milliseconds=12),
            base + timedelta(milliseconds=22),
            base + timedelta(milliseconds=32),
        ])
        self.assertEqual(sticky_imu.host_times_for_packet([], arrival), [])

    def test_dual_recording_writes_both_files(self):
        # Both stickies share one folder. Each sample is written as it arrives, with t_host.
        def batch(times):
            body = b"".join(struct.pack("<6hI", 0, 0, 2048, 0, 0, 0, t) for t in times)
            return bytes([len(times)]) + body

        packets = [batch((0, 10))] + [batch((t,)) for t in range(20, 2020, 10)]

        class FakeClient:
            def __init__(self, device):
                self.device = device
                self.is_connected = True
            async def __aenter__(self):
                return self
            async def __aexit__(self, *args):
                self.is_connected = False
            async def start_notify(self, uuid, callback):
                for data in packets:
                    callback(None, bytearray(data))
                self.is_connected = False
            async def stop_notify(self, uuid):
                self.is_connected = False

        sticky_imu.BleakClient = FakeClient
        ball = types.SimpleNamespace(name=sticky_imu.BALL_IMU, address="ball")
        hand = types.SimpleNamespace(name=sticky_imu.HAND_IMU, address="hand")
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory) / "imu_test"
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                asyncio.run(sticky_imu.stream_pair(ball, hand, folder))
            text = output.getvalue()
            self.assertIn("Hold both stickies still.", text)
            self.assertIn("Ball calibration window has passed.", text)
            self.assertIn("Hand calibration window has passed.", text)
            self.assertIn("ball t=", text)
            self.assertIn("hand t=", text)
            self.assertEqual(text.count("Recording summary"), 2)
            for name in ("ball.csv", "hand.csv"):
                with (folder / name).open(newline="") as recorded:
                    rows = list(csv.reader(recorded))
                self.assertEqual(tuple(rows[0]), sticky_imu.CSV_HEADER + (calibrate_imu.HOST_TIME_COLUMN,))
                self.assertEqual(rows[1][0], "0")
                self.assertEqual(rows[2][0], "10")
                first = datetime.fromisoformat(rows[1][-1])
                second = datetime.fromisoformat(rows[2][-1])
                self.assertEqual(second - first, timedelta(milliseconds=10))
                loaded = calibrate_imu.load_samples(folder / name)
                self.assertEqual(loaded[0][0], 0.0)
                self.assertGreaterEqual(loaded[-1][0] - loaded[0][0], 2000)

    def _write_stamped(self, path, rows):
        # rows are (t_fw_ms, ax, ay, az, gx, gy, gz, t_host datetime)
        with path.open("w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(list(calibrate_imu.CSV_HEADER) + [calibrate_imu.HOST_TIME_COLUMN])
            for t_fw, ax, ay, az, gx, gy, gz, host_time in rows:
                writer.writerow([
                    t_fw, ax, ay, az, gx, gy, gz,
                    host_time.isoformat(timespec="milliseconds"),
                ])

    def test_sync_pairs_on_the_ball_clock(self):
        # The timeline: B0 has no earlier hand sample, B1 takes H0, and B3 takes H3 rather than H4.
        try:
            import sync_imu
        except ImportError:
            self.skipTest("ahrs is required to build combined.csv")
        base = datetime(2026, 10, 26)
        def at(ms):
            return base + timedelta(milliseconds=ms)
        ball = [
            (1000, 0, 0, 1, 0, 0, 0, at(12)),
            (1010, 0, 0, 1, 0, 0, 0, at(22)),
            (1020, 0, 0, 1, 0, 0, 0, at(32)),
            (1030, 0, 0, 1, 0, 0, 0, at(52)),
            (1040, 0, 0, 1, 0, 0, 0, at(62)),
        ]
        hand = [
            (5000, 0, 0, 1, 0, 0, 0, at(18)),
            (5010, 0, 0, 1, 0, 0, 0, at(28)),
            (5020, 0, 0, 1, 0, 0, 0, at(38)),
            (5030, 0, 0, 1, 0, 0, 0, at(48)),
            (5040, 0, 0, 1, 0, 0, 0, at(58)),
        ]
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self._write_stamped(folder / "ball.csv", ball)
            self._write_stamped(folder / "hand.csv", hand)
            sync_imu.sync_folder(folder)
            with (folder / "combined.csv").open(newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))
            with (folder / "calibration.csv").open(newline="") as csv_file:
                calibration = list(csv.DictReader(csv_file))
        self.assertEqual([row["ball_t_fw_ms"] for row in rows], ["1000", "1010", "1020", "1030", "1040"])
        self.assertEqual(rows[0]["hand_t_fw_ms"], "")
        self.assertEqual(rows[1]["hand_t_fw_ms"], "5000")
        self.assertEqual(rows[2]["hand_t_fw_ms"], "5010")
        self.assertEqual(rows[3]["hand_t_fw_ms"], "5030")
        self.assertEqual(rows[4]["hand_t_fw_ms"], "5040")
        self.assertEqual(rows[0]["t_host"], at(12).isoformat(timespec="milliseconds"))
        self.assertEqual([row["sticky"] for row in calibration], ["ball", "hand"])
        self.assertEqual(calibration[0]["bx_dps"], "")
        self.assertEqual(calibration[1]["qw"], "")

    def test_sync_leaves_hold_quaternions_empty(self):
        # Madgwick starts after each sticky's own 2 second hold. The hold rows stay blank.
        try:
            import sync_imu
        except ImportError:
            self.skipTest("ahrs is required to build combined.csv")
        base = datetime(2026, 10, 26, 0, 0, 0)
        rows = []
        for t_fw in range(0, 2010, 10):
            gyro = 20 if t_fw > 2000 else 0
            rows.append((t_fw, 0, 0, 1, gyro, 0, 0, base + timedelta(milliseconds=t_fw)))
        rows.append((2010, 0, 0, 1, 20, 0, 0, base + timedelta(milliseconds=2010)))
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self._write_stamped(folder / "ball.csv", rows)
            self._write_stamped(folder / "hand.csv", rows)
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                sync_imu.sync_folder(folder)
            with (folder / "combined.csv").open(newline="") as csv_file:
                combined = list(csv.DictReader(csv_file))
            with (folder / "calibration.csv").open(newline="") as csv_file:
                calibration = {row["sticky"]: row for row in csv.DictReader(csv_file)}
        self.assertEqual(combined[0]["ball_qw"], "")
        self.assertEqual(combined[0]["hand_qw"], "")
        self.assertNotEqual(combined[-1]["ball_qw"], "")
        self.assertNotEqual(combined[-1]["hand_qw"], "")
        self.assertIn("Ball gyro bias", output.getvalue())
        self.assertIn("Hand starting quaternion", output.getvalue())
        for role in ("ball", "hand"):
            self.assertEqual(calibration[role]["bx_dps"], "0.000000")
            self.assertEqual(calibration[role]["by_dps"], "0.000000")
            self.assertEqual(calibration[role]["bz_dps"], "0.000000")
            self.assertAlmostEqual(float(calibration[role]["qw"]), 1.0, places=3)
            self.assertAlmostEqual(float(calibration[role]["qx"]), 0.0, places=3)
            self.assertAlmostEqual(float(calibration[role]["qy"]), 0.0, places=3)
            self.assertAlmostEqual(float(calibration[role]["qz"]), 0.0, places=3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
