"""Hardware-free check of the UI's enable and camera-connect defaults."""

import os
import json
import sys
import tempfile
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

    def f32c_enable_at(self, _handle, yaw, pitch):
        assert pitch <= 40
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
    gains = dict(yaw_speed_kp=75, yaw_speed_ki=32, yaw_pos_kp=120, yaw_pos_ki=0,
                 pitch_speed_kp=15, pitch_speed_ki=8, pitch_pos_kp=60, pitch_pos_ki=0)
    native = ui.load_motor_lib()
    observed = {}
    def capture_config(pointer):
        observed.update({name: getattr(pointer._obj, name) for name in gains})
        return 1
    with tempfile.TemporaryDirectory() as directory:
        profile = Path(directory) / "profile.json"
        profile.write_text(json.dumps({"status": "validated", "pi": gains}))
        with patch.object(ui, "PI_PROFILE", profile), patch.object(native, "f32c_open_config", side_effect=capture_config) as opener:
            assert ui.open_motor(native, None) == 1
            assert observed == gains
            profile.write_text(json.dumps({"status": "validation_failed", "pi": gains}))
            try:
                ui.open_motor(native, None)
            except ValueError:
                pass
            else:
                raise AssertionError("unvalidated UI profile accepted")
            assert opener.call_count == 1
            for invalid in ([], {"status": "validated", "pi": []},
                            {"status": "validated", "pi": dict(gains, yaw_speed_kp=True)}):
                profile.write_text(json.dumps(invalid))
                try:
                    ui.open_motor(native, None)
                except ValueError:
                    pass
                else:
                    raise AssertionError("invalid PI profile accepted")
            assert opener.call_count == 1
            profile.unlink()
            assert ui.open_motor(native, None) == 1
            assert observed == dict(yaw_speed_kp=40, yaw_speed_ki=0, yaw_pos_kp=42, yaw_pos_ki=0,
                                    pitch_speed_kp=10, pitch_speed_ki=4, pitch_pos_kp=40, pitch_pos_ki=0)
    motor = Motor()
    panel = ui.Panel(motor, 1)
    assert panel.pitch.slider.maximum() == 400
    assert ui.advance_angle(30, -170, 1, circular=False) == 29
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
