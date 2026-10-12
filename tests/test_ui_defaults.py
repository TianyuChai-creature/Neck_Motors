"""Hardware-free check of the UI's enable and camera-connect defaults."""

import json
import io
from contextlib import redirect_stderr
import os
import sys
import time
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))

from PySide6.QtWidgets import QApplication
import ui


class Motor:
    def __init__(self):
        self.read_ok = True
        self.sent = []
        self.disabled = 0
        self.disable_ok = True
        self.close_ok = True
        self.closed = False
        self.position = [30.0, -20.0]

    def f32c_read(self, _handle, yaw, pitch):
        if not self.read_ok:
            return 0
        yaw._obj.value, pitch._obj.value = self.position
        return 1

    def f32c_enable(self, _handle):
        return 1

    def f32c_enable_at(self, _handle, yaw, pitch):
        assert pitch <= 40
        self.position = [yaw + .5, pitch - .5]
        return 1

    def f32c_start_log(self, *_args):
        return 1

    def f32c_stop_log(self, *_args):
        pass

    def f32c_disable(self, _handle):
        self.disabled += 1
        return int(self.disable_ok)

    def f32c_set_yaw_deg(self, _handle, value):
        self.sent.append(("yaw", value))
        return 1

    def f32c_set_pitch_deg(self, _handle, value):
        self.sent.append(("pitch", value))
        return 1

    def f32c_close_checked(self, _handle):
        self.closed = True
        return int(self.close_ok)

    def f32c_last_error(self, _handle):
        return b"hardware state unknown"


class Camera:
    def __init__(self):
        self.settings = []

    def f32c_zed_create(self):
        return 1

    def f32c_zed_open(self, *_args):
        return 1

    def f32c_zed_serial_number(self, _handle):
        return 10661128

    def f32c_zed_set_setting(self, _handle, setting, value):
        self.settings.append((setting, value))
        return 1

    def f32c_zed_get_setting(self, *_args):
        return 0

    def f32c_zed_close(self, _handle):
        pass

    def f32c_zed_destroy(self, _handle):
        pass


if __name__ == "__main__":
    app = QApplication([])
    gains = dict(yaw_speed_kp=50, yaw_speed_ki=16, yaw_pos_kp=63, yaw_pos_ki=0,
                 pitch_speed_kp=30, pitch_speed_ki=8, pitch_pos_kp=60, pitch_pos_ki=0)
    native = ui.load_motor_lib()
    observed = {}
    def capture_config(pointer):
        assert pointer._obj.device == b"/dev/f32c-ui-test"
        assert (pointer._obj.travel_rpm, pointer._obj.accel) == (80, 10)
        observed.update({name: getattr(pointer._obj, name) for name in gains})
        return 1
    with patch.object(native, "f32c_open_config", side_effect=capture_config):
        assert ui.open_motor(native, b"/dev/f32c-ui-test") == 1
        assert observed == gains
    motor = Motor()
    panel = ui.Panel(motor, 1)
    assert not hasattr(panel, "btn_zero") and not hasattr(panel, "set_zero")
    assert (panel.yaw.slider.minimum(), panel.yaw.slider.maximum()) == (-600, 600)
    assert panel.pitch.slider.maximum() == 400
    assert panel.smooth_speed.value() == 120
    panel.enable()
    assert panel.logging
    assert panel.desired_angles == {"yaw": 0.0, "pitch": 0.0}
    assert panel.commanded_angles == {"yaw": 30.0, "pitch": -20.0}
    assert not motor.sent  # Enable queues a smooth return, not an immediate 0° jump.
    panel._last_command_tick = time.monotonic() - 0.02
    panel.step_targets()
    assert panel.tracker.config.yaw_limits == (-60, 60)
    assert panel.tracker.config.max_accel_deg_s2 == 600
    assert 0 < motor.sent[0][1] < 30
    assert -20 < motor.sent[1][1] < 0
    for requested, bounded in ((75, 60), (-75, -60)):
        panel.yaw.set_deg(requested)
        panel.send("yaw")
        assert panel.desired_angles["yaw"] == bounded and panel.armed
    panel.go_zero()
    panel.commanded_angles = {"yaw": 75., "pitch": 0.}
    panel.measured_angles = dict(panel.commanded_angles)
    panel.tracker = None
    panel._last_command_tick = 5.
    with patch.object(ui.time, "monotonic", return_value=5.02):
        panel.step_targets()
    assert panel.tracker.config.yaw_limits == (-60, 75)
    assert 0 < panel.commanded_angles["yaw"] < 75 and panel.armed

    camera = Camera()
    with patch.object(ui, "load_zed_lib", return_value=camera):
        panel.connect_camera()
    assert (5, -1) in camera.settings and (4, -1) in camera.settings
    panel.disconnect_camera()

    panel.commanded_angles = panel.desired_angles = {"yaw": 0., "pitch": 0.}
    panel.hold_samples.clear()
    panel.hold_target = None
    for samples in panel.motion_samples.values():
        samples.clear()
    panel.motion_samples["yaw"].append((10., 30.))
    with patch.object(ui.time, "monotonic", return_value=10.1):
        assert panel.check_feedback(5, 0)  # Normal lag during a faster trajectory.
    with patch.object(ui.time, "monotonic", return_value=10.7):
        assert not panel.check_feedback(5, 0)  # The same error is a fault after stopping.
    assert not panel.armed and motor.disabled == 1
    panel.armed = True
    assert panel.check_feedback(0, 0)
    assert panel.check_feedback(-.7, 0)
    assert not panel.check_feedback(.7, 0)
    assert not panel.armed and motor.disabled == 2

    panel.armed = True
    motor.position = [5., 0.]
    panel.poll_motors()
    assert not panel.armed and motor.disabled == 3

    panel.armed = True
    panel.commanded_angles = {"yaw": 9., "pitch": 0.}
    panel.desired_angles = {"yaw": 30., "pitch": 0.}
    panel.measured_angles = {"yaw": 0., "pitch": 0.}
    panel.tracker = None
    panel._last_command_tick = 20.
    before = len(motor.sent)
    with patch.object(ui.time, "monotonic", return_value=20.02):
        panel.step_targets()
        assert panel.commanded_angles["yaw"] == 9 and len(motor.sent) == before
        assert panel.following_limited == {"yaw"}
        assert panel.check_feedback(0, 0) and panel.armed
    panel.desired_angles["yaw"] = 0.
    with patch.object(ui.time, "monotonic", return_value=20.06):
        panel.step_targets()
        assert panel.commanded_angles["yaw"] < 9  # A reversed drag is never held back.

    panel.commanded_angles["yaw"] = 9.
    panel.progress_samples.clear()
    for tick in (21., 21.5, 22.):
        with patch.object(ui.time, "monotonic", return_value=tick):
            assert panel.check_feedback(0, 0)
    with TemporaryDirectory() as directory, patch.object(ui.time, "monotonic", return_value=22.5):
        panel.log_path = Path(directory) / "session.log"
        assert not panel.check_feedback(0, 0)
        fault = json.loads(panel.log_path.with_suffix(".fault.json").read_text())
        assert "无运动进展" in fault["reason"]
        assert fault["target"]["yaw"] == 9 and fault["measured"]["yaw"] == 0
        assert fault["disable_request_sent"] and fault["hardware_state"] == "unknown"
    assert not panel.armed and motor.disabled == 4

    motor.read_ok = False
    with patch.object(ui.QMessageBox, "critical", return_value=0):
        panel.enable()
    assert motor.disabled == 5 and not panel.armed
    motor.disable_ok = motor.close_ok = False
    assert not panel.disable() and not panel.armed
    assert "硬件状态未知" in panel.status.text()
    with TemporaryDirectory() as directory, redirect_stderr(io.StringIO()) as errors:
        panel.log_path = Path(directory) / "session.log"
        def camera_cleanup():
            assert motor.closed  # Motor cleanup must precede camera teardown.
        with patch.object(panel, "disconnect_camera", side_effect=camera_cleanup):
            panel.close()
        result = json.loads(panel.log_path.with_suffix(".close.json").read_text())
        assert not result["close_request_sent"] and result["hardware_state"] == "unknown"
        assert "硬件状态未知" in errors.getvalue()
        assert panel.handle is None
    print("UI defaults passed")
