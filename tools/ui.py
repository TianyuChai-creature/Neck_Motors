#!/usr/bin/env python3
"""F32C neck panel: enable, sliders for yaw/pitch, live status, zero & save."""

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
from neck import LinkError, Neck, NeckConfig, NeckError

REASONS = {
    "": "跟随中",
    "homing": "回零中",
    "input_idle": "输入过期，保持位置",
    "feedback_stale": "反馈过期，保持位置",
    "lead_guard": "跟随误差过大，等待反馈",
    "link_lost": "链路丢失，已停止推进",
    "zero_saved": "零位已写入并保存",
    "zero_set": "零位已写入（未保存）",
    "internal_error": "控制循环异常",
}


def wrap180(deg: float) -> float:
    x = (deg + 180.0) % 360.0 - 180.0
    return x + 360.0 if x <= -180.0 else x


def torque_text(value: str) -> str:
    return {"on": "使能请求已发", "off": "失能请求已发", "unknown": "硬件状态未知"}.get(
        value, "硬件状态未知")


class AxisRow(QWidget):
    def __init__(self, name: str):
        super().__init__()
        self.name = name
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(QLabel(name))
        self.slider = QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(*((-600, 600) if name == "yaw" else (-450, 430)))
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
        self.feeding = True
        self.setWindowTitle("F32C Neck")

        self.status = QLabel("未使能")
        font = self.status.font()
        font.setBold(True)
        font.setPointSize(12)
        self.status.setFont(font)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        box = QGroupBox("两轴电机")
        layout = QVBoxLayout(box)
        layout.addWidget(QLabel("使能后保持当前位置；yaw ±60°，pitch -45°…43°"))
        buttons = QHBoxLayout()
        self.btn_en = QPushButton("使能并到零位")
        self.btn_dis = QPushButton("失能")
        self.btn_home = QPushButton("到 0°")
        self.btn_feed = QPushButton("暂停输入")
        self.btn_feed.setToolTip("停止发送目标，约 0.2 秒后进入输入过期并保持位置；再点恢复")
        self.btn_zero = QPushButton("写零位（两轴）")
        self.btn_zero.setToolTip("把当前位置写成电机的 0° 并保存到 flash，不可撤销")
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
        layout.addWidget(QLabel("滑条是实时输入：使能后每个周期发送一次目标（set_target）"))
        self.log_label = QLabel(f"Log: {log_path.name}" if log_path else "Log: 关闭")
        layout.addWidget(self.log_label)

        root = QVBoxLayout(self)
        root.addWidget(self.status)
        root.addWidget(box, 1)
        self._sync_from_status(initial=True)

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
        result = self.neck.enable(home=True)
        if not result.ok:
            self.status.setText("使能失败：" + result.message)
            QMessageBox.critical(self, "F32C", result.message)
            return
        self.feeding = True
        self.btn_feed.setText("暂停输入")
        self.go_zero()
        self._show(self.neck.status)

    def confirm_zero(self, text: str) -> bool:
        answer = QMessageBox.question(
            self, "写零位", text,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No)
        return answer == QMessageBox.StandardButton.Yes

    def write_zero(self) -> None:
        status = self.neck.status
        measured = status.measured
        if measured is None:
            self.status.setText("无法读取当前角度，未写零位")
            return
        if not self.confirm_zero(
                f"把当前位置写成 0° 并保存到电机 flash（不可撤销）。\n"
                f"当前实测：yaw {measured.yaw:.1f}°，pitch {measured.pitch:.1f}°"):
            return
        was_feeding, self.feeding = self.feeding, False
        try:
            result = self.neck.zero(save=True, require_stationary=True)
        finally:
            self.feeding = was_feeding
        if not result.ok:
            self.status.setText("写零位未完成：" + result.error)
            if result.verified:
                self.go_zero()
            return
        self.go_zero()
        self.status.setText("零位已写入并保存（两轴现为 0°）")
        self._show(self.neck.status)

    def disable(self) -> bool:
        result = self.neck.disable()
        if not result.ok:
            self.status.setText("失能请求失败：" + result.message)
            return False
        self.status.setText("已发送失能请求（硬件状态未确认）")
        return True

    # ---- periodic --------------------------------------------------------------------------

    def step_targets(self) -> None:
        if self.neck.enabled and self.feeding:
            self.neck.set_target(self.yaw.deg(), self.pitch.deg())
        self._show(self.neck.status)
        self.btn_zero.setEnabled(True)
        self.btn_feed.setEnabled(self.neck.enabled)

    def _sync_from_status(self, initial: bool = False) -> None:
        status = self.neck.status
        measured = status.measured
        if measured is not None:
            self.yaw.set_meas(f"{measured.yaw:.1f}°")
            self.pitch.set_meas(f"{measured.pitch:.1f}°")
            if initial:
                self.yaw.set_deg(measured.yaw)
                self.pitch.set_deg(measured.pitch)
        self._show(status)

    def _show(self, status) -> None:
        if status.measured is not None:
            self.yaw.set_meas(f"{status.measured.yaw:.1f}°")
            self.pitch.set_meas(f"{status.measured.pitch:.1f}°")
        link = "链路正常" if status.link_ok else "链路异常"
        reason = REASONS.get(status.reason, status.reason)
        if not status.enabled:
            text = f"未使能 · {torque_text(status.torque)}"
        else:
            text = f"使能 · {link} · {reason}"
        if not self.feeding and status.enabled:
            text += " · 输入已暂停"
        if status.feedback_age is not None:
            text += f" · 反馈 {status.feedback_age * 1000:.0f} ms"
        self.status.setText(text)

    def closeEvent(self, event) -> None:
        self.timer.stop()
        result = self.neck.close()
        if self.log_path is not None:
            try:
                self.log_path.with_suffix(".close.json").write_text(json.dumps({
                    "cleanup_completed": result.ok,
                    "torque": result.torque,
                    "error": result.message,
                    "status": asdict(self.neck.status),
                }, indent=2) + "\n")
            except OSError as exc:
                print(f"Cannot save F32C close result: {exc}", file=sys.stderr)
        if not result.ok:
            print(f"F32C 关闭清理失败，硬件状态未知：{result.message}", file=sys.stderr)
        event.accept()


def log_directory() -> Path:
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
        neck = Neck(NeckConfig(port=port, log_path=str(log_path)))
    except (LinkError, NeckError) as exc:
        print(f"F32C open failed: {exc}", file=sys.stderr)
        QMessageBox.critical(None, "F32C 无法打开串口", str(exc))
        sys.exit(1)
    neck.start()
    try:
        window = Panel(neck, log_path)
        window.resize(680, 280)
        window.show()
        code = app.exec()
    finally:
        neck.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
