"""tools/ui.py on Qt's offscreen platform with a FakeDriver; never touches hardware."""

import io
import json
import os
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication

from neck.control import Neck
from neck.driver import Pose
from tests.fakes import FakeDriver
from tools import ui


class UiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.now = 100.0
        clock = patch("time.monotonic", side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.log = Path(directory.name) / "ui.log"
        self.driver = FakeDriver(Pose(30, 45.2))
        self.neck = Neck(self.driver)
        self.panel = ui.Panel(self.neck, self.log)
        self.addCleanup(self.quiet_close)

    def quiet_close(self):
        with redirect_stderr(io.StringIO()):
            self.panel.close()

    def run_ticks(self, count, until=None):
        for _ in range(count):
            self.now += 0.033
            self.panel.step_targets()
            if until and until():
                return

    def test_initial_sliders_follow_measurement_and_limits_match_defaults(self):
        self.assertEqual(self.panel.pitch.slider.maximum(), 450)
        self.assertEqual((self.panel.yaw.slider.minimum(), self.panel.yaw.slider.maximum()), (-600, 600))
        self.assertEqual(self.panel.pitch.deg(), 45.0)   # the slider stops at its own limit
        self.assertFalse(self.neck.enabled)
        self.assertEqual(self.driver.sent, [])

    def test_enable_goes_to_zero_through_the_trajectory(self):
        self.panel.enable()
        self.assertEqual(self.neck.snapshot.target, Pose(0, 0))
        self.assertEqual((self.panel.yaw.deg(), self.panel.pitch.deg()), (0, 0))
        self.assertEqual(self.driver.sent, [])          # nothing sent until the first tick
        self.run_ticks(500, until=lambda: self.neck.snapshot.reason == "target_reached")
        self.assertEqual(self.neck.snapshot.reason, "target_reached")
        self.assertEqual(self.driver.disables, 0)
        self.assertIn("保持当前位置", self.panel.status.text())

    def test_slider_moves_the_neck_and_pitch_is_clipped(self):
        self.panel.enable()
        self.panel.pitch.set_deg(90)
        self.panel.send("pitch")
        self.assertEqual(self.neck.snapshot.target.pitch, 45)
        self.panel.pitch.set_deg(-20)
        self.panel.send("pitch")
        self.assertEqual(self.neck.snapshot.target.pitch, -20)

    def test_disable_button_turns_torque_off(self):
        self.panel.enable()
        self.assertTrue(self.panel.disable())
        self.assertFalse(self.neck.enabled)
        self.assertEqual(self.driver.disables, 1)

    def test_lost_feedback_degrades_then_faults_and_writes_a_record(self):
        self.panel.enable()
        self.driver.read_error = True
        self.run_ticks(1)
        self.assertTrue(self.neck.enabled)
        phases = set()
        for _ in range(60):
            self.run_ticks(1)
            phases.add(self.neck.phase)
            if not self.neck.enabled:
                break
        self.assertIn("degraded", phases)
        self.assertFalse(self.neck.enabled)
        record = json.loads(self.log.with_suffix(".fault.json").read_text())
        self.assertEqual(record["phase"], "fault")
        self.assertIn("reason", record)
        self.assertIn("故障", self.panel.status.text())

    def test_failed_cleanup_is_reported_on_close(self):
        self.panel.enable()
        self.driver.disable_error = True
        with redirect_stderr(io.StringIO()) as errors:
            self.panel.close()
        record = json.loads(self.log.with_suffix(".close.json").read_text())
        self.assertFalse(record["cleanup_completed"])
        self.assertEqual(record["hardware_state"], "unknown")
        self.assertIn("硬件状态未知", errors.getvalue())


if __name__ == "__main__":
    unittest.main()
