#!/usr/bin/env python3
"""F32C neck panel: enable, sliders for yaw/pitch, live status. Uses only the `neck` package."""

from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from neck.control import Neck, NeckError
from neck.driver import Driver, DriverConfig, DriverError

PHASES = {"moving": "按轨迹移动", "hold": "保持当前位置", "following": "跟随中",
          "stale": "输入过期，保持位置", "degraded": "降级（保持使能，等待恢复）",
          "fault": "故障", "off": "未使能"}


def wrap180(deg: float) -> float:
    x = (deg + 180.0) % 360.0 - 180.0
    return x + 360.0 if x <= -180.0 else x


class AxisRow(QWidget):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel(name))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(*((-600, 600) if name == "yaw" else (-1800, 450)))
        self.slider.valueChanged.connect(lambda _v: self.cmd.setText(f"{self.deg():.1f}°"))
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


class Panel(QWidget):
    def __init__(self, neck: Neck, log_path: Path | None = None):
        super().__init__()
        self.neck = neck
        self.log_path = log_path
        self.feeding = True         # the sliders are a live input stream, like an upstream source
        self.setWindowTitle("F32C Neck")

        self.status = QLabel("未使能（硬件状态未确认）")
        font = self.status.font()
        font.setBold(True)
        font.setPointSize(12)
        self.status.setFont(font)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        box = QGroupBox("两轴电机")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("使能后先回到已存零位 yaw 0° / pitch 0°；yaw ±60°，pitch -180°…45°"))
        buttons = QHBoxLayout()
        self.btn_en = QPushButton("使能并到零位")
        self.btn_dis = QPushButton("失能")
        self.btn_home = QPushButton("到 0°")
        self.btn_feed = QPushButton("暂停输入")
        self.btn_feed.setToolTip("停止发送目标，约 0.2 秒后进入“输入过期”并保持位置；再点恢复")
        self.btn_zero = QPushButton("写零位（两轴）")
        self.btn_zero.setToolTip("把当前位置写成电机的 0° 并保存到 flash，不可撤销。仅在失能或保持状态可用")
        self.btn_en.clicked.connect(self.enable)
        self.btn_dis.clicked.connect(self.disable)
        self.btn_home.clicked.connect(self.go_zero)
        self.btn_zero.clicked.connect(self.write_zero)
        self.btn_feed.clicked.connect(self.toggle_feed)
        for button in (self.btn_en, self.btn_dis, self.btn_home, self.btn_feed, self.btn_zero):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.yaw = AxisRow("yaw")
        self.pitch = AxisRow("pitch")
        layout.addWidget(self.yaw)
        layout.addWidget(self.pitch)
        layout.addWidget(QLabel("滑条是实时输入：使能后每个周期都发送一次目标（set_target），与上游跟随一致"))
        self.log_label = QLabel(f"Log: {log_path.name}" if log_path else "Log: 关闭")
        layout.addWidget(self.log_label)

        root = QVBoxLayout(self)
        root.addWidget(self.status)
        root.addWidget(box, 1)

        initial = self.neck.driver.read()
        self.yaw.set_deg(initial.yaw)
        self.pitch.set_deg(initial.pitch)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.step_targets)
        self.timer.start(33)

    # ---- actions ---------------------------------------------------------------------------

    def go_zero(self) -> None:
        self.yaw.set_deg(0)
        self.pitch.set_deg(0)

    def toggle_feed(self) -> None:
        self.feeding = not self.feeding
        self.btn_feed.setText("暂停输入" if self.feeding else "恢复输入")

    def enable(self) -> None:
        try:
            if self.neck.phase == "fault":
                self.neck.reset_fault()     # this button is the explicit operator action
            snapshot = self.neck.enable()
        except (NeckError, DriverError) as exc:
            self.fault(str(exc))
            QMessageBox.critical(self, "F32C", str(exc))
            return
        self.show_state(snapshot)
        if snapshot.target is not None:
            self.yaw.set_deg(snapshot.target.yaw)
            self.pitch.set_deg(snapshot.target.pitch)

    def confirm_zero(self, text: str) -> bool:
        answer = QMessageBox.question(self, "写零位", text, QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                                      QMessageBox.StandardButton.No)
        return answer == QMessageBox.StandardButton.Yes

    def write_zero(self) -> None:
        """The position the neck holds now becomes 0° on both motors, saved to flash."""
        try:
            pose = self.neck.driver.read()
        except (DriverError, ValueError) as exc:
            self.status.setText("无法读取当前角度，未写零位：" + str(exc))
            return
        if not self.confirm_zero(f"把当前位置写成 0° 并保存到电机 flash（不可撤销）。\n"
                                 f"当前实测：yaw {pose.yaw:.1f}°，pitch {pose.pitch:.1f}°"):
            return
        was_feeding, self.feeding = self.feeding, False
        try:
            if self.neck.enabled:
                self.neck.hold()                # stop following; zeroing needs a held pose
            snapshot = self.neck.set_zero()
        except NeckError as exc:
            if self.neck.phase == "fault":
                self.fault(exc.reason)
            else:
                self.status.setText("写零位未完成（需先稳定，稍后再点）：" + exc.reason)
            return
        finally:
            self.feeding = was_feeding
        self.yaw.set_deg(0)
        self.pitch.set_deg(0)
        self.show_state(snapshot)
        self.status.setText("零位已写入并保存（两轴现为 0°）")

    def disable(self) -> bool:
        try:
            self.neck.disable()
        except NeckError as exc:
            self.status.setText("失能请求失败（硬件状态未知）：" + exc.reason)
            return False
        self.status.setText("已发送失能请求（硬件状态未确认）")
        return True

    # ---- periodic --------------------------------------------------------------------------

    def step_targets(self) -> None:
        self._step()
        self.btn_zero.setEnabled(self.neck.phase in ("off", "hold", "moving", "following", "stale"))
        self.btn_feed.setEnabled(self.neck.enabled)

    def _step(self) -> None:
        if not self.neck.enabled:
            self.poll_motors()
            return
        try:
            if self.feeding:
                self.neck.set_target(self.yaw.deg(), self.pitch.deg())
            self.show_state(self.neck.tick())
        except NeckError as exc:
            self.fault(exc.reason)

    def poll_motors(self) -> None:
        try:
            pose = self.neck.driver.read()
        except (DriverError, ValueError):
            self.yaw.set_meas("读失败")
            self.pitch.set_meas("读失败")
            return
        self.yaw.set_meas(f"{pose.yaw:.1f}°")
        self.pitch.set_meas(f"{pose.pitch:.1f}°")

    def show_state(self, snapshot) -> None:
        if snapshot.measured is not None:
            self.yaw.set_meas(f"{snapshot.measured.yaw:.1f}°")
            self.pitch.set_meas(f"{snapshot.measured.pitch:.1f}°")
        notice = {1: "[扰动] ", 2: "[降级] "}.get(snapshot.level, "")
        if not self.feeding and snapshot.phase not in ("off", "fault"):
            notice += "[输入已暂停] "
        reason = " · " + snapshot.reason if snapshot.reason else ""
        self.status.setText(notice + PHASES.get(snapshot.phase, snapshot.phase) + reason)

    def fault(self, reason: str) -> None:
        if self.neck.enabled:
            self.disable()
        record = asdict(self.neck.snapshot)
        record.update(reason=reason, hardware_state=self.neck.driver.state.value)
        if self.log_path is not None:
            try:
                self.log_path.with_suffix(".fault.json").write_text(json.dumps(record, indent=2) + "\n")
            except OSError as exc:
                print(f"Cannot save F32C fault: {exc}", file=sys.stderr)
        self.status.setText("Neck 故障：" + reason)

    def closeEvent(self, event) -> None:
        self.timer.stop()
        closed, reason = True, ""
        try:
            self.neck.close()
        except NeckError as exc:
            closed, reason = False, exc.reason
            print(f"F32C 关闭清理失败，硬件状态未知：{reason}", file=sys.stderr)
        if self.log_path is not None:
            try:
                self.log_path.with_suffix(".close.json").write_text(json.dumps({
                    "cleanup_completed": closed, "hardware_state": self.neck.driver.state.value,
                    "error": reason}, indent=2) + "\n")
            except OSError as exc:
                print(f"Cannot save F32C close result: {exc}", file=sys.stderr)
        event.accept()


def log_directory() -> Path:
    """Source checkout: <repo>/logs. Frozen (exe/AppImage): ~/f32c_logs, since the bundle dir is temporary."""
    if getattr(sys, "frozen", False):
        return Path.home() / "f32c_logs"
    return ROOT / "logs"


def main() -> None:
    app = QApplication.instance() or QApplication(sys.argv)
    port = sys.argv[1] if len(sys.argv) > 1 else None
    log_dir = log_directory()
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"f32c_ui_{time.strftime('%Y%m%d_%H%M%S')}.log"
    try:
        neck = Neck(Driver.open(DriverConfig(port=port, log_path=str(log_path))))
    except DriverError as exc:
        print(f"F32C open failed: {exc}", file=sys.stderr)
        QMessageBox.critical(None, "F32C 无法打开串口", str(exc))
        sys.exit(1)
    try:
        window = Panel(neck, log_path)
        window.resize(640, 260)
        window.show()
        code = app.exec()
    finally:
        try:
            neck.close()
        except NeckError as exc:
            print(f"F32C close failed: {exc}", file=sys.stderr)
    sys.exit(code)


if __name__ == "__main__":
    main()
