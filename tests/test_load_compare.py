"""Offline checks for the fixed load experiment; never opens a hardware port."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from neck.control import Snapshot
from neck.driver import DriverState, Pose
from tests.fakes import FakeDriver
from tools import load_compare as tool

TEST_YAW = -13.8


class Clock:
    def __init__(self):
        self.now = 100.

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, .000001)


class LoadCompareTests(unittest.TestCase):
    def execute(self, driver, yaw_target=None):
        clock = Clock()
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "run"
            args = SimpleNamespace(device="unused", output=output, load="A", run=True, pitch_speed_kp=10, yaw_target=yaw_target)
            with patch.object(tool.Driver, "open", return_value=driver), \
                 patch.object(tool.time, "monotonic", clock.monotonic), \
                 patch.object(tool.time, "sleep", clock.sleep), \
                 contextlib.redirect_stdout(io.StringIO()):
                try:
                    tool.run(args)
                    error = None
                except BaseException as exc:
                    error = exc
            metadata = json.loads((output / "run.json").read_text())
            rows = [json.loads(line) for line in (output / "samples.jsonl").read_text().splitlines()]
            return metadata, rows, error

    def test_preview_does_not_open_or_write(self):
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(tool.Driver, "open", side_effect=AssertionError("hardware opened")), \
             contextlib.redirect_stdout(io.StringIO()):
            output = Path(tmp) / "preview"
            self.assertEqual(tool.main(["--load", "A", "--device", "unused", "--output", str(output)]), 0)
            self.assertFalse(output.exists())
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            tool.main(["--load", "A", "--device", "unused", "--output", "unused", "--pitch-speed-kp", "14"])

    def test_complete_plan_preserves_sdk_path_and_disables(self):
        driver = FakeDriver(Pose(TEST_YAW, 40))
        metadata, rows, error = self.execute(driver)
        self.assertIsNone(error)
        self.assertEqual(metadata["status"], "completed")
        self.assertEqual({r["segment"] for r in rows}, {p[0] for p in tool.PLAN})
        self.assertEqual(max(r["input"]["pitch"] for r in rows), 33)
        self.assertTrue(all(abs(r["input"]["yaw"] - TEST_YAW) < 1e-9 for r in rows))
        self.assertTrue(all(-3 <= r["snapshot"]["command"]["pitch"] <= 40 for r in rows))
        self.assertTrue(any(r["snapshot"]["command"]["pitch"] != r["input"]["pitch"] for r in rows))
        self.assertEqual((driver.zeros, driver.saves, driver.disables), (0, 0, 1))
        self.assertEqual(driver.state, DriverState.CLOSED)
        self.assertFalse(metadata["physical_stop_confirmed"])
        self.assertAlmostEqual(metadata["yaw_target_deg"], TEST_YAW)
        self.assertTrue(all(abs(r["snapshot"]["command"]["yaw"] - TEST_YAW) < 1e-9 for r in rows))
        self.assertTrue(all(yaw is None or abs(yaw - TEST_YAW) < 1e-9 for yaw, _ in driver.sent))

    def test_candidate_reuses_baseline_yaw_without_a_fixed_start_tolerance(self):
        driver = FakeDriver(Pose(TEST_YAW + 1.2, 0))
        metadata, rows, error = self.execute(driver, yaw_target=TEST_YAW)
        self.assertIsNone(error)
        self.assertAlmostEqual(metadata["yaw_target_deg"], TEST_YAW)
        self.assertTrue(all(abs(r["input"]["yaw"] - TEST_YAW) < 1e-9 for r in rows))
        self.assertAlmostEqual(rows[-1]["snapshot"]["measured"]["yaw"], TEST_YAW)
        self.assertEqual((driver.zeros, driver.saves), (0, 0))

    def test_outside_start_envelope_never_enables(self):
        driver = FakeDriver(Pose(61, 0))
        metadata, rows, error = self.execute(driver)
        self.assertIsNotNone(error)
        self.assertEqual(metadata["status"], "stopped")
        self.assertEqual(rows, [])
        self.assertEqual(driver.sent, [])
        self.assertEqual(driver.state, DriverState.CLOSED)

    def test_stall_stops_once_without_reset_or_reenable(self):
        driver = FakeDriver(Pose(TEST_YAW, 40))
        driver.follow = False
        metadata, rows, error = self.execute(driver)
        self.assertIsNotNone(error)
        self.assertTrue(rows)
        self.assertEqual(metadata["status"], "stopped")
        self.assertEqual(driver.disables, 1)
        self.assertEqual(driver.state, DriverState.CLOSED)

    def test_failed_disable_still_closes_and_fails_run(self):
        driver = FakeDriver(Pose(TEST_YAW, 0))
        driver.disable_error = True
        metadata, rows, error = self.execute(driver)
        self.assertIsNotNone(error)
        self.assertEqual(metadata["status"], "cleanup_failed")
        self.assertIn("failed", metadata["disable_request"])
        self.assertIn("failed", metadata["close"])
        self.assertTrue(driver.port_closed)

    def test_first_communication_warning_stops(self):
        driver = FakeDriver(Pose(TEST_YAW, 0))
        from neck.driver import DriverError
        original = driver.read
        def read(timeout=.1):
            if driver.reads >= 3:
                raise DriverError("communication fault")
            return original(timeout)
        with patch.object(driver, "read", side_effect=read):
            metadata, rows, error = self.execute(driver)
        self.assertIsNotNone(error)
        self.assertEqual(metadata["status"], "stopped")
        self.assertEqual(driver.state, DriverState.CLOSED)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["snapshot"]["level"], 1)

    def test_level_one_and_small_stall_are_not_ignored(self):
        from collections import deque
        snapshot = Snapshot("following", 1, "feedback_wait", Pose(0, 0), Pose(0, 0), Pose(0, 0), 0)
        with self.assertRaisesRegex(RuntimeError, "protection stop"):
            tool.check_snapshot(snapshot, deque(), 0)
        window = deque()
        snapshot = Snapshot("following", 0, "", Pose(0, 0), Pose(0, 3), Pose(0, 3), 0)
        tool.check_snapshot(snapshot, window, 0)
        with self.assertRaisesRegex(RuntimeError, "no progress"):
            tool.check_snapshot(snapshot, window, 1.5)

    def test_real_driver_on_fake_bus_records_baseline_and_no_persistent_writes(self):
        from tests.bus import Bus
        for kp in (10, 12):
            with self.subTest(pitch_speed_kp=kp), tempfile.TemporaryDirectory() as tmp, Bus({1: TEST_YAW, 2: 0.}) as bus:
                self.check_bus_run(bus, Path(tmp) / "run", kp)

    def check_bus_run(self, bus, output, kp):
        with patch.object(tool, "PLAN", [("startup", 0., 1.1)]), \
             contextlib.redirect_stdout(io.StringIO()):
            tool.run(SimpleNamespace(device=bus.path, output=output, load="A", run=True, pitch_speed_kp=kp, yaw_target=None))
        serial = (output / "serial.log").read_text()
        self.assertIn("TX", serial)
        self.assertIn("RX", serial)
        self.assertFalse(bus.errors)
        self.assertFalse(bus.commands(0x08))
        self.assertFalse(bus.commands(0x0A))
        from neck.driver.protocol import angle
        yaw_frames = [frame for frame in bus.commands(0x03) if frame[1] == 1]
        self.assertTrue(yaw_frames)
        self.assertTrue(all(frame == angle(1, TEST_YAW) for frame in yaw_frames))
        for addr, gains in ((1, dict(speed_kp=40, speed_ki=4, pos_kp=42, pos_ki=0)),
                            (2, dict(speed_kp=kp, speed_ki=4, pos_kp=40, pos_ki=0))):
            axis = bus.axes[addr]
            self.assertEqual(axis.gains, gains)
            self.assertEqual((axis.mode, axis.rpm, axis.accel), (4, 15, 2))
            self.assertFalse(axis.enabled)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp, \
             patch.object(tool.Driver, "open", side_effect=AssertionError("hardware opened")), \
             contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(FileExistsError):
                tool.run(SimpleNamespace(device="unused", output=Path(tmp), load="A", run=True, pitch_speed_kp=10, yaw_target=None))


if __name__ == "__main__":
    unittest.main()
