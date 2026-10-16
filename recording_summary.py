"""Recording diagnostics using firmware timestamps and the existing calibration check."""
import statistics

from calibrate_imu import first_window, gyro_bias, window_is_still


class RecordingSummary:
    def __init__(self):
        self.count = 0
        self.first = None
        self.previous = None
        self.elapsed_ms = 0
        self.intervals = []
        self.invalid_timestamps = 0
        self.near_limit = [0] * 6
        self.hold = []

    def add(self, sample):
        t = sample[0]
        if self.first is None:
            self.first = t
        if self.previous is not None:
            dt = t - self.previous
            # The firmware clock is an unsigned 32-bit millisecond counter.
            if self.previous > 0xF0000000 and t < 0x0FFFFFFF:
                dt += 2**32
            if dt > 0:
                self.elapsed_ms += dt
                self.intervals.append(dt)
            else:
                self.invalid_timestamps += 1
        self.previous = t
        self.count += 1
        if not self.hold or self.hold[-1][0] < 2000:
            self.hold.append((self.elapsed_ms, *sample[1:]))
        for axis, value in enumerate(sample[1:]):
            limit = 16 if axis < 3 else 2000
            self.near_limit[axis] += abs(value) >= 0.99 * limit

    def report(self, bad_packets=0):
        lines = ["Recording summary", f"Samples: {self.count}",
                 f"Malformed packets skipped: {bad_packets}"]
        if self.intervals:
            rate = 1000 * len(self.intervals) / self.elapsed_ms
            gaps = sum(dt > 15 for dt in self.intervals)
            missing = sum(max(0, round(dt / 10) - 1) for dt in self.intervals if dt > 15)
            lines += [f"Firmware duration: {self.elapsed_ms / 1000:.3f} s",
                      f"Effective sample rate: {rate:.2f} Hz (nominal 100 Hz)",
                      f"Sample intervals: median {statistics.median(self.intervals):.1f} ms, max {max(self.intervals):.1f} ms",
                      f"Gaps >15 ms: {gaps}; estimated missing samples: {missing} (assuming 100 Hz)"]
        else:
            lines.append("Effective sample rate: unavailable (fewer than two increasing timestamps)")
        lines.append(f"Duplicate/backward timestamps: {self.invalid_timestamps}")
        if self.invalid_timestamps:
            lines.append("Timing estimates are unreliable: clock reset or non-increasing timestamps detected.")
        lines.append("Near-range-limit samples per axis (99% of assumed ±16 g / ±2000 dps): " +
                     ", ".join(f"{axis}={n}" for axis, n in zip(("ax", "ay", "az", "gx", "gy", "gz"), self.near_limit)))
        if self.invalid_timestamps:
            lines.append("Calibration hold: unverified (invalid timestamps)")
        elif len(self.hold) < 2 or self.hold[-1][0] < 2000:
            lines.append("Calibration hold: incomplete (need 2 seconds)")
        else:
            hold = first_window(self.hold)
            if any(b[0] - a[0] > 15 for a, b in zip(hold, hold[1:])):
                lines.append("Calibration hold: unverified (timestamp gaps)")
            elif window_is_still(hold):
                bias = gyro_bias(hold)
                lines.append("Calibration hold: PASS; gyro bias (dps): " + ", ".join(f"{x:+.3f}" for x in bias))
            else:
                lines.append("Calibration hold: FAIL (opening 2 seconds not still enough)")
        return "\n".join(lines)
