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
from pathlib import Path

# These stand-ins are confined to this test process.
for name in ('bleak', 'bleak.backends', 'bleak.backends.device', 'bleak.backends.scanner'):
    sys.modules[name] = types.ModuleType(name)
sys.modules['bleak'].BleakClient = object
sys.modules['bleak'].BleakScanner = object
sys.modules['bleak.backends.device'].BLEDevice = object
sys.modules['bleak.backends.scanner'].AdvertisementData = object

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


if __name__ == '__main__':
    unittest.main(verbosity=2)
