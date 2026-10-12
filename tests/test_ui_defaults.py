"""Hardware-free check of the UI's enable and camera-connect defaults."""

import os
import sys
import time
from pathlib import Path
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
        return 1

    def f32c_set_yaw_deg(self, _handle, value):
        self.sent.append(("yaw", value))
        return 1

    def f32c_set_pitch_deg(self, _handle, value):
        self.sent.append(("pitch", value))
        return 1

    def f32c_close(self, _handle):
        pass


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
    assert panel.pitch.slider.maximum() == 400
    assert panel.smooth_speed.value() == 120
    panel.enable()
    assert panel.logging
    assert panel.desired_angles == {"yaw": 0.0, "pitch": 0.0}
    assert panel.commanded_angles == {"yaw": 30.0, "pitch": -20.0}
    assert not motor.sent  # Enable queues a smooth return, not an immediate 0° jump.
    panel._last_command_tick = time.monotonic() - 0.02
    panel.step_targets()
    assert panel.tracker.config.max_accel_deg_s2 == 600
    assert 0 < motor.sent[0][1] < 30
    assert -20 < motor.sent[1][1] < 0

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

    motor.read_ok = False
    with patch.object(ui.QMessageBox, "critical", return_value=0):
        panel.enable()
    assert motor.disabled == 4 and not panel.armed
    panel.close()
    print("UI defaults passed")
