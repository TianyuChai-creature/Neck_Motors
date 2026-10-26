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
    def __init__(self, name: str, on_change):
        super().__init__()
        self.name = name
        self._on_change = on_change
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel(name))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(*((-600, 600) if name == "yaw" else (-1800, 450)))
        self.slider.valueChanged.connect(self._changed)
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


class Panel(QWidget):
    def __init__(self, neck: Neck, log_path: Path | None = None):
        super().__init__()
        self.neck = neck
        self.log_path = log_path
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
        self.btn_en.clicked.connect(self.enable)
        self.btn_dis.clicked.connect(self.disable)
        self.btn_home.clicked.connect(self.go_zero)
        for button in (self.btn_en, self.btn_dis, self.btn_home):
            buttons.addWidget(button)
        layout.addLayout(buttons)
        self.yaw = AxisRow("yaw", self.send)
        self.pitch = AxisRow("pitch", self.send)
        layout.addWidget(self.yaw)
        layout.addWidget(self.pitch)
        layout.addWidget(QLabel("目标即时更新，接近目标时平滑减速"))
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

    def send(self, _axis: str = "") -> None:
        if not self.neck.enabled:
            return
        try:
            self.neck.move_to(self.yaw.deg(), self.pitch.deg())
        except NeckError as exc:
            self.fault(exc.reason)

    def go_zero(self) -> None:
        self.yaw.set_deg(0)
        self.pitch.set_deg(0)
        self.send()

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
        if not self.neck.enabled:
            self.poll_motors()
            return
        try:
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


def main() -> None:
    app = QApplication(sys.argv)
    port = sys.argv[1] if len(sys.argv) > 1 else None
    log_dir = ROOT / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"f32c_ui_{time.strftime('%Y%m%d_%H%M%S')}.log"
    neck = Neck(Driver.open(DriverConfig(port=port, log_path=str(log_path))))
    try:
        window = Panel(neck, log_path)
        window.resize(560, 260)
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
