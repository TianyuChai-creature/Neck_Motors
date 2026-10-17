"""Telemetry runs against a pseudo-terminal; no physical device is opened."""
from contextlib import contextmanager, redirect_stdout
import io
import csv
import json
import os
from pathlib import Path
import select
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
import capture_pitch_feedback as capture


@contextmanager
def emulator(voltage=1200):
    master, slave = os.openpty()
    stop = threading.Event()
    frames = []
    errors = []
    positions = {1: 10, 2: 46}
    def worker():
        pending = bytearray()
        rebounding = False
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .01)[0]:
                    continue
                pending.extend(os.read(master, 128))
                while len(pending) >= 3:
                    size = 6 if pending[2] == 14 else 5 if pending[2] in (5, 6, 8, 10) else 7
                    if len(pending) < size:
                        break
                    frame = bytes(pending[:size]); del pending[:size]
                    frames.append(frame)
                    check = 0
                    for byte in frame[:-2]: check ^= byte
                    assert frame[0] == 122 and frame[-1] == 123 and frame[-2] == check
                    axis, function = frame[1:3]
                    if function == 3:
                        target = int.from_bytes(frame[3:5], 'big') / 10
                        if axis == 2 and target < 40:
                            rebounding = True
                        if not (axis == 2 and rebounding): positions[axis] = target
                    if function != 14: continue
                    kind = frame[3]
                    if kind == 2:
                        if axis == 2 and rebounding: positions[2] = min(46, positions[2] + 1)
                        value = round(positions[axis] * 10)
                    elif kind == 0: value = -12
                    elif kind == 4: value = voltage + (10 if axis == 1 else 0)
                    else: raise AssertionError(kind)
                    body = bytes((122, axis, kind)) + value.to_bytes(4, 'big', signed=True)
                    check = 0
                    for byte in body: check ^= byte
                    reply = body + bytes((check, 123))
                    os.write(master, reply[:4])
                    os.write(master, reply[4:])
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=worker)
    thread.start()
    try:
        yield os.ttyname(slave), frames
    finally:
        stop.set(); thread.join(1)
        os.close(master); os.close(slave)
        if errors: raise errors[0]


class CaptureTests(unittest.TestCase):
    def capture(self, voltage, *, both_zero=False):
        with tempfile.TemporaryDirectory() as folder, emulator(voltage) as (device, frames):
            root = Path(folder); (root / 'logs').mkdir()
            with patch.object(capture, 'ROOT', root), redirect_stdout(io.StringIO()):
                result = capture.run(device, both_zero=both_zero)
            rows = (Path(result['directory']) / 'samples.csv').read_text()
            raw = (Path(result['directory']) / 'feedback.jsonl').read_text()
            self.assertTrue(result['cleanup_completed'])
            self.assertIn('-12,', rows)
            self.assertIn('7a 02 0e 00 76 7b', raw)
            self.assertIn('7a 02 0e 04 72 7b', raw)
            self.assertIn('7a 01 0e 04 71 7b', raw)
            samples = list(csv.DictReader(io.StringIO(rows)))
            self.assertAlmostEqual(float(samples[0]['yaw_voltage_V']), (voltage + 10) / 100)
            self.assertAlmostEqual(float(samples[0]['voltage_V']), voltage / 100)
            self.assertEqual(float(samples[0]['yaw_rpm']), -12)
            return result, frames

    def test_rebound_capture_unchanged_gains_and_both_axes_disabled(self):
        result, frames = self.capture(1200)
        self.assertEqual(result['status'], 'rebound_captured')
        self.assertTrue(result['disable_requests_sent'])
        gains = [int.from_bytes(f[3:5], 'big') for f in frames if f[1] == 2 and f[2] in (15,16,17,18)]
        self.assertEqual(gains, [40,8,70,8])
        self.assertEqual([f[1] for f in frames if f[2] == 5][-2:], [1,2])
        self.assertFalse(any(f[2] in (8,10) for f in frames))

    def test_low_voltage_baseline_aborts_before_enable(self):
        result, frames = self.capture(750)
        self.assertEqual(result['status'], 'error')
        self.assertIn('bus voltage outside', result['error'])
        self.assertFalse(result['enable_attempted'])
        self.assertFalse(any(f[2] == 6 for f in frames))

    def test_both_zero_moves_yaw_without_changing_pitch_gains(self):
        result, frames = self.capture(1200, both_zero=True)
        self.assertEqual(result['yaw_policy'], 'return_zero')
        self.assertEqual(result['status'], 'rebound_captured')
        targets = [int.from_bytes(f[3:5], 'big') for f in frames if f[1:3] == bytes((1,3))]
        self.assertLess(min(targets), targets[0])
        gains = [int.from_bytes(f[3:5], 'big') for f in frames if f[1] == 2 and f[2] in (15,16,17,18)]
        self.assertEqual(gains, [40,8,70,8])
        self.assertEqual([f[1] for f in frames if f[2] == 5][-2:], [1,2])


if __name__ == '__main__': unittest.main()
