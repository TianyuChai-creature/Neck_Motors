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

    def f32c_read(self, _handle, yaw, pitch):
        if not self.read_ok:
            return 0
        yaw._obj.value, pitch._obj.value = 30.0, -20.0
        return 1

    def f32c_enable(self, _handle):
        return 1

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
    motor = Motor()
    panel = ui.Panel(motor, 1)
    panel.enable()
    assert panel.desired_angles == {"yaw": 0.0, "pitch": 0.0}
    assert panel.commanded_angles == {"yaw": 30.0, "pitch": -20.0}
    assert not motor.sent  # Enable queues a smooth return, not an immediate 0° jump.
    panel._last_command_tick = time.monotonic() - 0.02
    panel.step_targets()
    assert 0 < motor.sent[0][1] < 30
    assert -20 < motor.sent[1][1] < 0

    camera = Camera()
    with patch.object(ui, "load_zed_lib", return_value=camera):
        panel.connect_camera()
    assert (5, -1) in camera.settings and (4, -1) in camera.settings
    panel.disconnect_camera()

    motor.read_ok = False
    with patch.object(ui.QMessageBox, "critical", return_value=0):
        panel.enable()
    assert motor.disabled == 1 and not panel.armed
    panel.close()
    print("UI defaults passed")
