"""tools/ui.py on Qt's offscreen platform with a fake link; never touches hardware."""

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

from neck import LinkError, Neck, NeckConfig, Pose
from tests.fake_link import FakeLink
from tools import ui


class UiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.log = Path(directory.name) / "ui.log"
        self.link = FakeLink(Pose(30, 45))
        self.neck = Neck(NeckConfig(control_period=0.01, max_lead=180), self.link)
        self.panel = ui.Panel(self.neck, self.log)
        self.addCleanup(self.quiet_close)

    def quiet_close(self):
        with redirect_stderr(io.StringIO()):
            self.panel.close()

    def test_initial_sliders_follow_measurement_and_limits_match_defaults(self):
        self.assertEqual(self.panel.pitch.slider.maximum(), 450)
        self.assertEqual((self.panel.yaw.slider.minimum(), self.panel.yaw.slider.maximum()), (-600, 600))
        self.assertEqual(self.panel.pitch.deg(), 45.0)
        self.assertFalse(self.neck.enabled)
        self.assertEqual(self.link.frames, [])

    def test_enable_homes_and_turns_on_both_axes(self):
        self.panel.enable()
        self.assertTrue(self.neck.enabled)
        self.assertEqual(self.neck.status.target, Pose(0, 0))
        self.assertEqual(self.link.enabled, {1, 2})
        self.assertEqual((self.panel.yaw.deg(), self.panel.pitch.deg()), (0, 0))
        self.assertIn("使能", self.panel.status.text())

    def test_sliders_stream_and_pitch_is_clipped(self):
        self.panel.enable()
        self.panel.pitch.set_deg(90)
        self.panel.yaw.set_deg(10)
        self.panel.step_targets()
        self.assertEqual(self.neck.status.target, Pose(10, 45))

    def test_pause_input_changes_the_button_and_stops_streaming(self):
        self.panel.enable()
        self.panel.step_targets()
        self.panel.toggle_feed()
        self.assertFalse(self.panel.feeding)
        self.assertEqual(self.panel.btn_feed.text(), "恢复输入")
        target = self.neck.status.target
        self.panel.yaw.set_deg(5)
        self.panel.step_targets()
        self.assertEqual(self.neck.status.target, target)
        self.panel.toggle_feed()
        self.panel.step_targets()
        self.assertEqual(self.neck.status.target, Pose(5, 0))

    def test_disable_button_turns_torque_off(self):
        self.panel.enable()
        self.assertTrue(self.panel.disable())
        self.assertFalse(self.neck.enabled)
        self.assertEqual(self.link.enabled, set())

    def test_write_zero_needs_confirmation_and_saves(self):
        self.panel.enable()
        with patch.object(self.panel, "confirm_zero", return_value=False) as ask:
            self.panel.write_zero()
        self.assertIn("yaw 30.0", ask.call_args[0][0])
        self.assertEqual((self.link.zeros, self.link.saves), (0, 0))
        with patch.object(self.panel, "confirm_zero", return_value=True):
            self.panel.write_zero()
        self.assertEqual((self.link.zeros, self.link.saves), (2, 2))
        self.assertIn("零位已写入", self.panel.status.text())
        self.assertEqual((self.panel.yaw.deg(), self.panel.pitch.deg()), (0, 0))

    def test_unsettled_neck_refuses_zero(self):
        self.panel.enable()
        self.link.angles = {1: 40.0, 2: 45.0}
        with patch.object(self.panel, "confirm_zero", return_value=True):
            self.panel.write_zero()
        self.assertIn("写零位未完成", self.panel.status.text())
        self.assertEqual(self.link.zeros, 0)
        self.assertTrue(self.neck.enabled)

    def test_close_writes_a_report(self):
        self.panel.enable()
        self.panel.close()
        record = json.loads(self.log.with_suffix(".close.json").read_text())
        self.assertTrue(record["cleanup_completed"])
        self.assertTrue(self.link.closed)

    def test_open_failure_reports_and_exits_nonzero(self):
        err = io.StringIO()
        with patch.object(ui, "Neck", side_effect=LinkError("no port")), \
                patch.object(ui.QMessageBox, "critical") as box, \
                patch.object(ui.sys, "argv", ["ui"]), \
                patch.object(ui, "log_directory", return_value=self.log.parent), \
                redirect_stderr(err), self.assertRaises(SystemExit) as raised:
            ui.main()
        self.assertEqual(raised.exception.code, 1)
        box.assert_called_once()
        self.assertIn("no port", err.getvalue())


if __name__ == "__main__":
    unittest.main()
