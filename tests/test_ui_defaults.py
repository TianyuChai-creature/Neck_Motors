"""Qt offscreen exercises the public SDK controller, never native hardware."""
from contextlib import redirect_stderr
import io
import json
import os
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
from PySide6.QtWidgets import QApplication
import ui
from f32c import HeadController, State, TrackingConfig
from test_controller import Motor


class Camera:
    def __init__(self): self.settings = []
    def f32c_zed_create(self): return 1
    def f32c_zed_open(self, *_args): return 1
    def f32c_zed_serial_number(self, _handle): return 10661128
    def f32c_zed_set_setting(self, _handle, setting, value):
        self.settings.append((setting, value))
        return 1
    def f32c_zed_get_setting(self, *_args): return 0
    def f32c_zed_close(self, _handle): pass
    def f32c_zed_destroy(self, _handle): pass


class UiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.now = 100.
        self.clock = patch.object(ui.time, "monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = patch.object(ui, "ROOT", Path(directory.name))
        root.start()
        self.addCleanup(root.stop)
        self.motor = Motor(State(30, 45.2))
        self.motor.start_log = lambda _path: None
        self.motor.stop_log = lambda: None
        self.head = HeadController(motor=self.motor, tracking=TrackingConfig(
            yaw_limits=(-60, 60), pitch_limits=(-180, 45),
            max_speed_deg_s=120, max_accel_deg_s2=600)).open()
        self.panel = ui.Panel(self.head)
        self.addCleanup(self.panel.close)

    def test_enable_and_zero_share_the_sdk_trajectory_and_feedback(self):
        self.assertEqual(self.panel.pitch.slider.maximum(), 450)
        self.assertFalse(hasattr(self.panel, "set_zero"))
        self.panel.enable()
        self.assertAlmostEqual(self.head.snapshot.command.pitch_deg, 45.2)
        self.assertEqual(self.head.snapshot.target, State(0, 35))
        self.assertEqual(self.panel.pitch.deg(), 35)
        self.assertFalse(self.motor.sent)
        for _ in range(500):
            self.now += .033
            self.panel.step_targets()
            if self.head.snapshot.reason == "target_reached": break
        self.assertEqual(self.head.snapshot.reason, "target_reached")
        self.assertGreater(self.motor.reads, 20)
        self.assertEqual(self.motor.disables, 0)

    def test_slider_uses_controller_and_camera_remains_optional(self):
        self.panel.enable()
        self.panel.pitch.set_deg(90)
        self.panel.send("pitch")
        self.assertEqual(self.head.snapshot.target.pitch_deg, 45)
        camera = Camera()
        with patch.object(ui, "load_zed_lib", return_value=camera): self.panel.connect_camera()
        self.assertIn((5, -1), camera.settings)
        self.assertIn((4, -1), camera.settings)
        self.panel.disconnect_camera()

    def test_read_failure_and_close_failure_are_reported(self):
        self.panel.enable()
        self.motor.read_error = True
        self.now += .033
        self.panel.step_targets()
        self.assertTrue(self.head.enabled)
        for _ in range(4):
            self.now += .033
            self.panel.step_targets()
        self.assertFalse(self.head.enabled)
        self.assertTrue(self.panel.log_path.with_suffix(".fault.json").exists())
        self.motor.close_error = True
        with redirect_stderr(io.StringIO()) as errors: self.panel.close()
        record = json.loads(self.panel.log_path.with_suffix(".close.json").read_text())
        self.assertFalse(record["cleanup_completed"])
        self.assertIn("close unknown", errors.getvalue())


if __name__ == "__main__": unittest.main()
