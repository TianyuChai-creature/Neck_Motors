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

    def test_enable_goes_to_zero_through_the_trajectory_while_the_sliders_stream_zero(self):
        self.panel.enable()
        self.assertEqual(self.neck.snapshot.target, Pose(0, 0))
        self.assertEqual((self.panel.yaw.deg(), self.panel.pitch.deg()), (0, 0))
        self.assertEqual(self.driver.sent, [])          # nothing sent until the first tick
        self.run_ticks(500, until=lambda: abs(self.driver.measured.yaw) < .3 and abs(self.driver.measured.pitch) < .3)
        self.assertLess(abs(self.driver.measured.pitch), .3)
        self.assertEqual(self.neck.snapshot.phase, "following")
        self.assertEqual(self.driver.disables, 0)
        self.assertIn("跟随中", self.panel.status.text())

    def test_sliders_are_a_live_stream_and_pitch_is_clipped(self):
        self.panel.enable()
        self.panel.pitch.set_deg(90)
        self.run_ticks(1)
        self.assertEqual(self.neck.snapshot.target.pitch, 45)
        self.panel.pitch.set_deg(-20)
        self.panel.yaw.set_deg(10)
        self.run_ticks(1)
        self.assertEqual(self.neck.snapshot.target, Pose(10, -20))

    def test_pause_input_goes_stale_and_resume_follows_again(self):
        self.panel.enable()
        self.run_ticks(5)
        self.panel.toggle_feed()
        self.assertEqual(self.panel.btn_feed.text(), "恢复输入")
        self.run_ticks(12)                              # > 0.2 s without a target
        self.assertEqual(self.neck.snapshot.phase, "stale")
        self.assertIn("输入已暂停", self.panel.status.text())
        self.panel.toggle_feed()
        self.panel.yaw.set_deg(5)
        self.run_ticks(2)
        self.assertEqual(self.neck.snapshot.phase, "following")
        self.assertEqual(self.neck.snapshot.target.yaw, 5)

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


    def settle_at(self, yaw, pitch):
        self.panel.yaw.set_deg(yaw)
        self.panel.pitch.set_deg(pitch)
        self.run_ticks(600, until=lambda: abs(self.driver.measured.yaw - yaw) < .2
                       and abs(self.driver.measured.pitch - pitch) < .2)
        self.run_ticks(10)

    def test_write_zero_needs_confirmation_and_is_offered_unless_degraded_or_faulted(self):
        self.panel.step_targets()
        self.assertTrue(self.panel.btn_zero.isEnabled())           # off
        with patch.object(self.panel, "confirm_zero", return_value=False) as ask:
            self.panel.write_zero()
        self.assertIn("yaw 30.0", ask.call_args[0][0])
        self.assertEqual((self.driver.zeros, self.driver.saves), (0, 0))
        self.panel.enable()
        self.run_ticks(3)
        self.assertTrue(self.panel.btn_zero.isEnabled())           # following
        self.driver.read_error = True
        self.run_ticks(60)
        self.assertFalse(self.neck.enabled)
        self.assertFalse(self.panel.btn_zero.isEnabled())          # fault

    def test_write_zero_while_following_holds_zeroes_and_resumes_the_stream_at_the_new_zero(self):
        self.panel.enable()
        self.settle_at(10.0, 5.0)
        with patch.object(self.panel, "confirm_zero", return_value=True):
            self.panel.write_zero()
        self.assertEqual((self.driver.zeros, self.driver.saves, self.driver.disables), (1, 1, 0))
        self.assertIn("零位已写入", self.panel.status.text())
        self.assertEqual((self.panel.yaw.deg(), self.panel.pitch.deg()), (0, 0))
        self.assertTrue(self.panel.feeding)
        self.run_ticks(5)
        self.assertEqual(self.neck.snapshot.phase, "following")
        self.assertEqual(self.neck.snapshot.target, Pose(0, 0))
        self.assertLess(abs(self.driver.measured.yaw), .3)

    def test_unsettled_neck_refuses_zero_and_keeps_following(self):
        self.panel.enable()
        self.settle_at(10.0, 5.0)
        self.driver.follow = False
        self.driver.measured = Pose(15.0, 5.0)      # something is pushing the neck away
        with patch.object(self.panel, "confirm_zero", return_value=True):
            self.panel.write_zero()
        self.assertIn("写零位未完成", self.panel.status.text())
        self.assertEqual(self.driver.zeros, 0)
        self.assertTrue(self.neck.enabled)
        self.assertTrue(self.panel.feeding)

    def test_failed_zero_write_faults_and_records(self):
        self.panel.enable()
        self.settle_at(3.0, 2.0)
        self.driver.zero_error = True
        with patch.object(self.panel, "confirm_zero", return_value=True):
            self.panel.write_zero()
        self.assertFalse(self.neck.enabled)
        self.assertTrue(self.log.with_suffix(".fault.json").exists())


if __name__ == "__main__":
    unittest.main()
