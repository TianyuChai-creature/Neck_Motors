#!/usr/bin/env python3
"""F32C 2-DoF motor panel with an optional ZED Mini camera panel."""

from __future__ import annotations

import ctypes
from collections import deque
import math
import sys
import time
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from f32c import _CConfig

MOTOR_LIB = ROOT / "build" / "libf32c.so"
ZED_LIB_CANDIDATES = (ROOT / "build-zed" / "libf32c_zed.so", ROOT / "build" / "libf32c_zed.so")

ZED_VIEWS = {"左眼": 0, "右眼": 1, "并排": 2}
ZED_SETTINGS = (
    ("曝光", 5, 0, 100),
    ("增益", 4, 0, 100),
    ("亮度", 0, 0, 8),
    ("对比度", 1, 0, 8),
    ("饱和度", 2, 0, 8),
    ("锐度", 3, 0, 8),
)


def wrap180(deg: float) -> float:
    x = (deg + 180.0) % 360.0 - 180.0
    if x <= -180.0:
        x += 360.0
    return x


def advance_angle(current: float, target: float, max_step: float, circular: bool = True) -> float:
    error = wrap180(target - current) if circular else target - current
    step = max(-max_step, min(max_step, error))
    return wrap180(current + step) if circular else current + step


class ZedImageView(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_ubyte)),
        ("size_bytes", ctypes.c_size_t),
        ("stride_bytes", ctypes.c_size_t),
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("channels", ctypes.c_int),
        ("timestamp_ns", ctypes.c_uint64),
    ]


def load_motor_lib():
    if not MOTOR_LIB.exists():
        sys.exit(f"missing {MOTOR_LIB} — run: cmake -S . -B build && cmake --build build")
    lib = ctypes.CDLL(str(MOTOR_LIB))
    lib.f32c_open.restype = ctypes.c_void_p
    lib.f32c_open.argtypes = [ctypes.c_char_p, ctypes.c_uint, ctypes.c_uint]
    lib.f32c_config_defaults.argtypes = [ctypes.POINTER(_CConfig)]
    lib.f32c_config_defaults.restype = None
    lib.f32c_open_config.argtypes = [ctypes.POINTER(_CConfig)]
    lib.f32c_open_config.restype = ctypes.c_void_p
    lib.f32c_close.argtypes = [ctypes.c_void_p]
    lib.f32c_enable.argtypes = [ctypes.c_void_p]
    lib.f32c_enable_at.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
    lib.f32c_disable.argtypes = [ctypes.c_void_p]
    lib.f32c_set_deg.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
    lib.f32c_set_yaw_deg.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.f32c_set_pitch_deg.argtypes = [ctypes.c_void_p, ctypes.c_double]
    lib.f32c_read.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.f32c_set_zero.argtypes = [ctypes.c_void_p]
    lib.f32c_start_log.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
    lib.f32c_stop_log.argtypes = [ctypes.c_void_p]
    return lib


def open_motor(lib, device):
    config = _CConfig()
    lib.f32c_config_defaults(ctypes.byref(config))
    config.device = device
    return lib.f32c_open_config(ctypes.byref(config))


def load_zed_lib():
    path = next((path for path in ZED_LIB_CANDIDATES if path.exists()), None)
    if path is None:
        raise RuntimeError("ZED library is not built; run cmake -S . -B build-zed -DF32C_BUILD_ZED=ON")
    lib = ctypes.CDLL(str(path))
    lib.f32c_zed_create.restype = ctypes.c_void_p
    lib.f32c_zed_destroy.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_open.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.f32c_zed_close.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_is_open.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_grab.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_retrieve.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ZedImageView)]
    lib.f32c_zed_set_setting.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int]
    lib.f32c_zed_get_setting.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.POINTER(ctypes.c_int)]
    lib.f32c_zed_serial_number.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_last_error.argtypes = [ctypes.c_void_p]
    lib.f32c_zed_last_error.restype = ctypes.c_char_p
    return lib


class AxisRow(QWidget):
    def __init__(self, name: str, on_change):
        super().__init__()
        self.name = name
        self._on_change = on_change
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel(name))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(-1800, 400 if name == "pitch" else 1800)
        self.slider.valueChanged.connect(self._changed)
        self.slider.sliderReleased.connect(lambda: self._on_change(self.name, True))
        layout.addWidget(self.slider, 1)
        self.cmd = QLabel("0.0°")
        self.cmd.setMinimumWidth(56)
        self.meas = QLabel("—")
        self.meas.setMinimumWidth(72)
        layout.addWidget(self.cmd)
        layout.addWidget(self.meas)

    def deg(self) -> float:
        return self.slider.value() / 10.0

    def set_deg(self, deg: float) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(int(round(wrap180(deg) * 10.0)))
        self.slider.blockSignals(False)
        self.cmd.setText(f"{self.deg():.1f}°")

    def set_meas(self, text: str) -> None:
        self.meas.setText(text)

    def _changed(self, _v: int) -> None:
        self.cmd.setText(f"{self.deg():.1f}°")
        self._on_change(self.name)


class CameraSettingRow(QWidget):
    def __init__(self, label: str, setting: int, minimum: int, maximum: int, on_release):
        super().__init__()
        self.setting = setting
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(minimum, maximum)
        self.slider.sliderReleased.connect(lambda: on_release(self.setting, self.slider.value()))
        self.value = QLabel("—")
        self.value.setMinimumWidth(48)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel(label))
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.value)

    def set_value(self, value: int) -> None:
        self.slider.blockSignals(True)
        self.slider.setValue(max(self.slider.minimum(), min(self.slider.maximum(), value)))
        self.slider.blockSignals(False)
        self.value.setText("AUTO" if value < 0 else str(value))


class Panel(QWidget):
    def __init__(self, motor_lib, motor_handle):
        super().__init__()
        self.lib = motor_lib
        self.handle = motor_handle
        self.armed = False
        self.zed_lib = None
        self.zed_handle = None
        self.camera_frame_count = 0
        self.camera_fps_started = 0.0
        self.logging = False
        self.log_path = None
        self.hold_samples = deque()
        self.hold_target = None
        self.setWindowTitle("F32C 2-DoF + ZED Mini")

        self.status = QLabel("失能")
        font = self.status.font()
        font.setBold(True)
        font.setPointSize(12)
        self.status.setFont(font)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        motor_box = QGroupBox("两轴电机")
        motor_layout = QVBoxLayout(motor_box)
        motor_layout.addWidget(QLabel("使用回零保持参数（yaw 全范围尚未验收）"))
        btn_row = QHBoxLayout()
        self.btn_en = QPushButton("使能")
        self.btn_dis = QPushButton("失能")
        self.btn_home = QPushButton("到 0°")
        self.btn_zero = QPushButton("当前设为零点")
        self.btn_en.clicked.connect(self.enable)
        self.btn_dis.clicked.connect(self.disable)
        self.btn_home.clicked.connect(self.go_zero)
        self.btn_zero.clicked.connect(self.set_zero)
        for button in (self.btn_en, self.btn_dis, self.btn_home, self.btn_zero):
            btn_row.addWidget(button)
        motor_layout.addLayout(btn_row)
        log_row = QHBoxLayout()
        self.log_button = QPushButton("开始 Log")
        self.log_button.clicked.connect(self.toggle_log)
        self.log_status = QLabel("Log: 关闭")
        log_row.addWidget(self.log_button)
        log_row.addWidget(self.log_status, 1)
        motor_layout.addLayout(log_row)
        self.yaw = AxisRow("yaw", self.send)
        self.pitch = AxisRow("pitch", self.send)
        motor_layout.addWidget(self.yaw)
        motor_layout.addWidget(self.pitch)
        smooth_row = QHBoxLayout()
        smooth_row.addWidget(QLabel("直通平滑速度"))
        self.smooth_speed = QSpinBox()
        self.smooth_speed.setRange(5, 360)
        self.smooth_speed.setValue(5)
        self.smooth_speed.setSuffix(" °/s")
        smooth_row.addWidget(self.smooth_speed)
        smooth_row.addStretch(1)
        motor_layout.addLayout(smooth_row)
        motor_layout.addWidget(QLabel("latest-wins：滑块更新目标，50 Hz 平滑下发，不建立队列"))

        camera_box = QGroupBox("ZED Mini 相机（可选）")
        camera_layout = QVBoxLayout(camera_box)
        camera_controls = QHBoxLayout()
        self.camera_connect = QPushButton("连接相机")
        self.camera_connect.clicked.connect(self.toggle_camera)
        self.camera_view = QComboBox()
        for name, view in ZED_VIEWS.items():
            self.camera_view.addItem(name, view)
        self.camera_view.setEnabled(False)
        self.camera_status = QLabel("未连接")
        self.camera_fps = QLabel("FPS: —")
        camera_controls.addWidget(self.camera_connect)
        camera_controls.addWidget(QLabel("视图"))
        camera_controls.addWidget(self.camera_view)
        camera_controls.addWidget(self.camera_status, 1)
        camera_controls.addWidget(self.camera_fps)
        camera_layout.addLayout(camera_controls)

        self.camera_image = QLabel("点击“连接相机”开始取流")
        self.camera_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_image.setMinimumSize(640, 360)
        self.camera_image.setStyleSheet("background: #111; color: #bbb;")
        camera_layout.addWidget(self.camera_image, 1)

        settings_box = QGroupBox("相机属性（运行时）")
        settings_layout = QFormLayout(settings_box)
        self.camera_settings = {}
        for label, setting, minimum, maximum in ZED_SETTINGS:
            row = CameraSettingRow(label, setting, minimum, maximum, self.set_camera_setting)
            self.camera_settings[setting] = row
            settings_layout.addRow(row)
        self.camera_auto = QPushButton("自动曝光 / 增益")
        self.camera_auto.clicked.connect(self.set_camera_auto)
        settings_layout.addRow(self.camera_auto)
        camera_layout.addWidget(settings_box)

        body = QHBoxLayout()
        body.addWidget(motor_box, 1)
        body.addWidget(camera_box, 2)
        root = QVBoxLayout(self)
        root.addWidget(self.status)
        root.addLayout(body, 1)

        y, p = ctypes.c_double(), ctypes.c_double()
        if self.lib.f32c_read(self.handle, ctypes.byref(y), ctypes.byref(p)):
            self.yaw.set_deg(y.value)
            self.pitch.set_deg(p.value)
        self.desired_angles = {"yaw": self.yaw.deg(), "pitch": self.pitch.deg()}
        self.commanded_angles = dict(self.desired_angles)
        self._last_command_tick = time.monotonic()

        self.motor_timer = QTimer(self)
        self.motor_timer.timeout.connect(self.poll_motors)
        self.motor_timer.start(20)
        self.command_timer = QTimer(self)
        self.command_timer.timeout.connect(self.step_targets)
        self.command_timer.start(20)
        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.poll_camera)
        for row in self.camera_settings.values():
            row.setEnabled(False)
        self.camera_auto.setEnabled(False)

    def send(self, axis: str = "", force: bool = False) -> None:
        if axis == "yaw":
            self.desired_angles["yaw"] = self.yaw.deg()
        elif axis == "pitch":
            self.desired_angles["pitch"] = self.pitch.deg()
        else:
            self.desired_angles = {"yaw": self.yaw.deg(), "pitch": self.pitch.deg()}

    def step_targets(self) -> None:
        now = time.monotonic()
        dt = min(0.05, max(0.0, now - self._last_command_tick))
        self._last_command_tick = now
        if not self.armed:
            return
        max_step = self.smooth_speed.value() * dt
        for axis, setter in (
            ("yaw", self.lib.f32c_set_yaw_deg),
            ("pitch", self.lib.f32c_set_pitch_deg),
        ):
            current = self.commanded_angles[axis]
            command = round(advance_angle(current, self.desired_angles[axis], max_step, circular=axis == "yaw") * 10) / 10
            if round(command * 10) == round(current * 10):
                continue
            if not setter(self.handle, command):
                self.fault(f"{axis} 目标发送失败")
                return
            self.commanded_angles[axis] = command

    def enable(self) -> None:
        if not self.logging:
            self.toggle_log()
        if not self.logging:
            self.fault("无法记录启动日志，取消使能")
            return
        y, p = ctypes.c_double(), ctypes.c_double()
        if not self.lib.f32c_read(self.handle, ctypes.byref(y), ctypes.byref(p)):
            self.disable()
            QMessageBox.critical(self, "F32C", "读取当前角度失败，取消使能")
            return
        if not math.isfinite(y.value) or not math.isfinite(p.value) or wrap180(p.value) > 43:
            self.fault("使能前角度无效或 pitch 超过已知边界")
            return
        initial = {"yaw": wrap180(y.value), "pitch": min(40.0, wrap180(p.value))}
        if not self.lib.f32c_enable_at(self.handle, initial["yaw"], initial["pitch"]):
            self.armed = False
            self.status.setText("使能失败")
            QMessageBox.critical(self, "F32C", "使能失败")
            return
        y, p = ctypes.c_double(), ctypes.c_double()
        if not self.lib.f32c_read(self.handle, ctypes.byref(y), ctypes.byref(p)):
            disabled = self.lib.f32c_disable(self.handle)
            self.armed = False
            self.status.setText("失能" if disabled else "失能失败")
            QMessageBox.critical(self, "F32C", "读取当前角度失败，取消回零；" + ("已失能" if disabled else "失能失败，请检查电机"))
            return
        self.commanded_angles = initial
        self.hold_samples.clear()
        self.hold_target = None
        self.armed = True
        self.status.setText("使能")
        self._last_command_tick = time.monotonic()
        if not self.check_feedback(y.value, p.value):
            return
        self.go_zero()

    def disable(self) -> None:
        disabled = self.lib.f32c_disable(self.handle)
        self.armed = False
        self.hold_samples.clear()
        self.hold_target = None
        self.status.setText("已发送失能" if disabled else "失能发送失败，请断电")

    def fault(self, reason: str) -> None:
        self.disable()
        self.status.setText(reason + "；" + self.status.text())

    def check_feedback(self, yaw: float, pitch: float) -> bool:
        measured = {"yaw": wrap180(yaw), "pitch": wrap180(pitch)}
        if (not all(math.isfinite(value) for value in measured.values())
                or measured["pitch"] > 43 + 1e-9
                or abs(wrap180(measured["yaw"] - self.commanded_angles["yaw"])) > 3 + 1e-9
                or abs(measured["pitch"] - self.commanded_angles["pitch"]) > 3 + 1e-9):
            self.fault("反馈越界或跟随误差超过 3°")
            return False
        target = tuple(self.commanded_angles[axis] for axis in ("yaw", "pitch"))
        if target != self.hold_target:
            self.hold_samples.clear()
            self.hold_target = target
        if any(abs(self.commanded_angles[axis] - self.desired_angles[axis]) > .1
               for axis in ("yaw", "pitch")):
            self.hold_samples.clear()
            return True
        now = time.monotonic()
        errors = tuple(wrap180(measured[axis] - self.commanded_angles[axis]) for axis in ("yaw", "pitch"))
        self.hold_samples.append((now, errors))
        while self.hold_samples and self.hold_samples[0][0] < now - 1:
            self.hold_samples.popleft()
        if len(self.hold_samples) >= 3 and any(
                max(row[1][axis] for row in self.hold_samples) - min(row[1][axis] for row in self.hold_samples) > 1 + 1e-9
                for axis in (0, 1)):
            self.fault("保持位置摆幅超过 1°")
            return False
        return True

    def toggle_log(self) -> None:
        if self.logging:
            self.lib.f32c_stop_log(self.handle)
            self.logging = False
            self.log_button.setText("开始 Log")
            self.log_status.setText("Log: 关闭")
            return

        log_dir = ROOT / "logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / f"f32c_ui_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns() % 1_000_000_000:09d}.log"
        if not self.lib.f32c_start_log(self.handle, str(path).encode()):
            QMessageBox.warning(self, "Log", f"无法打开日志文件：{path}")
            return
        self.logging = True
        self.log_path = path
        self.log_button.setText("停止 Log")
        self.log_status.setText(f"Log: {path.name}")

    def go_zero(self) -> None:
        self.yaw.set_deg(0)
        self.pitch.set_deg(0)
        self.send()

    def set_zero(self) -> None:
        if not self.armed:
            QMessageBox.information(self, "置零", "先使能再置零")
            return
        if QMessageBox.question(self, "置零", "把当前姿态写成 0 并掉电保存？") != QMessageBox.StandardButton.Yes:
            return
        if not self.lib.f32c_set_zero(self.handle):
            QMessageBox.critical(self, "F32C", "置零失败")
            return
        self.yaw.set_deg(0)
        self.pitch.set_deg(0)
        self.desired_angles = {"yaw": 0.0, "pitch": 0.0}
        self.commanded_angles = {"yaw": 0.0, "pitch": 0.0}

    def poll_motors(self) -> None:
        y, p = ctypes.c_double(), ctypes.c_double()
        if self.lib.f32c_read(self.handle, ctypes.byref(y), ctypes.byref(p)):
            self.yaw.set_meas(f"{wrap180(y.value):.1f}°")
            self.pitch.set_meas(f"{wrap180(p.value):.1f}°")
            if self.armed:
                self.check_feedback(y.value, p.value)
            else:
                self.commanded_angles = {"yaw": wrap180(y.value), "pitch": wrap180(p.value)}
        else:
            self.yaw.set_meas("读失败")
            self.pitch.set_meas("读失败")
            if self.armed:
                self.fault("电机反馈读取失败")

    def zed_error(self) -> str:
        if not self.zed_lib or not self.zed_handle:
            return "ZED 未连接"
        value = self.zed_lib.f32c_zed_last_error(self.zed_handle)
        return value.decode(errors="replace") if value else "unknown error"

    def toggle_camera(self) -> None:
        if self.zed_handle:
            self.disconnect_camera()
        else:
            self.connect_camera()

    def connect_camera(self) -> None:
        try:
            self.zed_lib = load_zed_lib()
            self.zed_handle = self.zed_lib.f32c_zed_create()
            if not self.zed_handle or not self.zed_lib.f32c_zed_open(self.zed_handle, 0):
                raise RuntimeError(self.zed_error())
        except Exception as exc:
            if self.zed_lib and self.zed_handle:
                self.zed_lib.f32c_zed_destroy(self.zed_handle)
            self.zed_handle = None
            QMessageBox.warning(self, "ZED Mini", str(exc))
            return

        serial = self.zed_lib.f32c_zed_serial_number(self.zed_handle)
        self.camera_status.setText(f"已连接 SN={serial}")
        self.camera_connect.setText("断开相机")
        self.camera_view.setEnabled(True)
        self.camera_auto.setEnabled(True)
        for row in self.camera_settings.values():
            row.setEnabled(True)
        self.set_camera_auto()
        self.camera_frame_count = 0
        self.camera_fps_started = time.monotonic()
        self.camera_timer.start(33)

    def disconnect_camera(self) -> None:
        self.camera_timer.stop()
        if self.zed_lib and self.zed_handle:
            self.zed_lib.f32c_zed_close(self.zed_handle)
            self.zed_lib.f32c_zed_destroy(self.zed_handle)
        self.zed_handle = None
        self.camera_view.setEnabled(False)
        self.camera_auto.setEnabled(False)
        for row in self.camera_settings.values():
            row.setEnabled(False)
        self.camera_connect.setText("连接相机")
        self.camera_status.setText("未连接")
        self.camera_fps.setText("FPS: —")
        self.camera_image.clear()
        self.camera_image.setText("点击“连接相机”开始取流")

    def refresh_camera_settings(self) -> None:
        for setting, row in self.camera_settings.items():
            value = ctypes.c_int()
            if self.zed_lib.f32c_zed_get_setting(self.zed_handle, setting, ctypes.byref(value)):
                row.set_value(value.value)

    def set_camera_setting(self, setting: int, value: int) -> None:
        if not self.zed_handle:
            return
        if not self.zed_lib.f32c_zed_set_setting(self.zed_handle, setting, value):
            self.camera_status.setText(f"属性失败: {self.zed_error()}")

    def set_camera_auto(self) -> None:
        if not self.zed_handle:
            return
        ok_exposure = self.zed_lib.f32c_zed_set_setting(self.zed_handle, 5, -1)
        ok_gain = self.zed_lib.f32c_zed_set_setting(self.zed_handle, 4, -1)
        if not (ok_exposure and ok_gain):
            self.camera_status.setText(f"自动曝光/增益失败: {self.zed_error()}")
        self.refresh_camera_settings()

    def poll_camera(self) -> None:
        if not self.zed_handle:
            return
        if not self.zed_lib.f32c_zed_grab(self.zed_handle):
            self.camera_status.setText(f"取帧失败: {self.zed_error()}")
            return
        image = ZedImageView()
        view = int(self.camera_view.currentData())
        if not self.zed_lib.f32c_zed_retrieve(self.zed_handle, view, ctypes.byref(image)):
            self.camera_status.setText(f"取图失败: {self.zed_error()}")
            return

        if not image.data or image.width <= 0 or image.height <= 0 or image.stride_bytes <= 0:
            self.camera_status.setText("无效图像数据")
            return
        raw = ctypes.string_at(image.data, image.size_bytes)
        qimage = QImage(raw, image.width, image.height, image.stride_bytes, QImage.Format.Format_ARGB32)
        if qimage.isNull():
            self.camera_status.setText("QImage 转换失败")
            return
        pixmap = QPixmap.fromImage(qimage.copy())
        self.camera_image.setPixmap(
            pixmap.scaled(
                self.camera_image.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation,
            )
        )
        self.camera_frame_count += 1
        elapsed = time.monotonic() - self.camera_fps_started
        if elapsed >= 1.0:
            self.camera_fps.setText(f"FPS: {self.camera_frame_count / elapsed:.1f}")
            self.camera_frame_count = 0
            self.camera_fps_started = time.monotonic()

    def closeEvent(self, event) -> None:
        self.motor_timer.stop()
        self.command_timer.stop()
        self.disable()
        self.disconnect_camera()
        if self.logging:
            self.lib.f32c_stop_log(self.handle)
            self.logging = False
        self.lib.f32c_close(self.handle)
        event.accept()


def main() -> None:
    lib = load_motor_lib()
    device = sys.argv[1].encode() if len(sys.argv) > 1 else None
    handle = open_motor(lib, device)
    if not handle:
        sys.exit("open/ping failed — check power, IDs 1/2, and the serial device")
    app = QApplication(sys.argv)
    window = Panel(lib, handle)
    window.resize(1100, 680)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
