#!/usr/bin/env python3
"""Record one scripted run of the real Driver and Neck on the fake bus and write a
self-contained HTML replay (docs/回放.html). No hardware, no third-party packages.

    python3 tools/replay.py [output.html]
"""

from __future__ import annotations

import json
import math
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from neck.control import Neck, NeckError
from neck.driver import Driver, DriverConfig
from tests.bus import Bus

TEMPLATE = Path(__file__).with_name("replay_template.html")
TICK = 0.02
FUNCS = {0x00: "选模式", 0x01: "设速度", 0x03: "目标角", 0x05: "失能", 0x06: "使能", 0x07: "设加速度",
         0x08: "保存参数到 flash", 0x0A: "当前位置写成 0°", 0x0E: "查询机械角",
         0x0F: "速度 KP", 0x10: "速度 KI", 0x11: "位置 KP", 0x12: "位置 KI"}
AXIS = {1: "yaw", 2: "pitch"}


def decode(kind: str, frame: bytes) -> tuple[str, str]:
    """Plain-language meaning of one frame, and the axis it concerns."""
    if len(frame) < 5 or frame[0] != 0x7A:
        return "", ""
    axis, func, data = AXIS.get(frame[1], f"ID{frame[1]}"), frame[2], frame[3:-2]
    if kind == "RX":
        deg = int.from_bytes(data, "big") / 10
        fold = f"，Driver 折成 {deg - 360:.1f}°" if deg >= 180 else ""
        return f"反馈：机械角 {deg:.1f}°{fold}", axis
    text = FUNCS.get(func, f"功能码 {func:02X}")
    value = int.from_bytes(data, "big")
    if func == 0x03:
        text += f" {(value - 3600 if value > 1800 else value) / 10:.1f}°"
    elif func in (0x01, 0x07, 0x0F, 0x10, 0x11, 0x12):
        text += f" = {value}"
    elif func == 0x00:
        text += f" {value:04d}（单圈直通）"
    return text, axis


def parse_log(path: Path, t0: float) -> list[dict]:
    frames = []
    for line in path.read_text().splitlines():
        parts = line.split(" ", 4)
        if len(parts) < 5 or parts[2] not in ("TX", "RX"):
            continue
        stamp = datetime.strptime(f"{parts[0]} {parts[1]}", "%Y-%m-%d %H:%M:%S.%f").timestamp()
        raw = bytes.fromhex(parts[4])
        text, axis = decode(parts[2], raw)
        frames.append({"t": round(stamp - t0, 4), "dir": parts[2], "axis": axis, "text": text,
                       "hex": parts[4], "query": raw[2:3] == b"\x0e" or parts[2] == "RX"})
    return frames


class Recorder:
    def __init__(self, neck: Neck, bus: Bus):
        self.neck, self.bus = neck, bus
        self.t0 = time.time()
        self.samples: list[dict] = []
        self.marks: list[dict] = []

    def now(self) -> float:
        return time.time() - self.t0

    def mark(self, title: str, note: str) -> None:
        self.marks.append({"t": round(self.now(), 3), "title": title, "note": note})

    def sample(self) -> None:
        if self.neck.enabled or self.neck.phase == "fault":
            s = self.neck.snapshot
            row = {"phase": s.phase, "level": s.level, "reason": s.reason, "m": s.measured,
                   "c": s.command, "g": s.target}
        else:
            row = {"phase": "off", "level": 0, "reason": "", "m": self.neck.driver.read(),
                   "c": None, "g": None}
        pose = lambda p: None if p is None else [round(p.yaw, 2), round(p.pitch, 2)]
        self.samples.append({"t": round(self.now(), 3), "phase": row["phase"], "level": row["level"],
                             "reason": row["reason"], "m": pose(row["m"]), "c": pose(row["c"]),
                             "g": pose(row["g"])})

    def run(self, seconds: float, each=None, until=None) -> None:
        """Tick at the control rate for `seconds` (or until `until()`), sampling every tick."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            started = time.monotonic()
            if each:
                each(self.now())
            try:
                if self.neck.enabled:
                    self.neck.tick()
            except NeckError as exc:
                self.mark("故障", f"level 3：{exc.reason}。Neck 已发失能请求并锁存，需要显式 reset_fault()。")
                self.sample()
                return
            self.sample()
            if until and until():
                return
            time.sleep(max(0.0, TICK - (time.monotonic() - started)))


def scenario(rec: Recorder) -> None:
    neck, bus = rec.neck, rec.bus
    reached = lambda: neck.snapshot.phase == "hold"

    rec.mark("打开", "Driver.open() 扫描串口并确认两个 ID 都应答。电机还没有力矩，只在读角度。")
    rec.run(0.6)

    rec.mark("使能并回零", "neck.enable()：以实测位姿使能（不跳变），然后用轨迹平滑回到已存零位 (0°, 0°)。")
    neck.enable()
    rec.run(6, until=lambda: reached() and neck.snapshot.reason == "target_reached")
    rec.run(0.5)

    rec.mark("实时跟随", "上层每 20 ms 调一次 neck.set_target(yaw, pitch)；Neck 限位、限速 90°/s、限加速度后写给 Driver。")
    start = rec.now()
    def follow(t):
        tau = t - start
        neck.set_target(35 * math.sin(2 * math.pi * 0.35 * tau), 18 * math.sin(2 * math.pi * 0.25 * tau))
    rec.run(3.2, each=follow)

    rec.mark("输入过期", "上层停止更新目标。0.2 s 后 Neck 进入 stale：不再外推，保持当前指令。")
    rec.run(0.8)

    rec.mark("点对点移动", "neck.move_to(−25°, −40°)：不会过期，按轨迹走到位，实测稳定 0.5 s 才算到达。")
    neck.move_to(-25, -40)
    rec.run(6, until=lambda: reached() and neck.snapshot.reason == "target_reached")
    rec.run(0.4)

    rec.mark("反馈中断 → 降级", "电机 0.25 s 不回角度。反馈变老 → level 2（降级）：保持使能，指令拉回实测位置，恢复稳定后继续。")
    neck.move_to(20, 10)
    rec.run(0.35)
    bus.silent.update({1, 2})
    rec.run(0.25)
    bus.silent.clear()
    rec.run(6, until=lambda: reached() and neck.snapshot.reason == "target_reached")
    rec.run(0.5)

    rec.mark("写零位", "neck.set_zero()：保持稳定时把当前 (20°, 10°) 写成新的 (0°, 0°)。每个电机在零点帧后立刻改瞄 0°，读回确认后才发保存帧。")
    neck.set_zero()
    rec.sample()
    rec.run(0.8)

    rec.mark("在新零位上移动", "零位已变：同一个 move_to(10°, 5°) 现在相对新零位。")
    neck.move_to(10, 5)
    rec.run(5, until=lambda: reached() and neck.snapshot.reason == "target_reached")
    rec.run(0.4)

    rec.mark("反馈丢失", "电机持续不回角度。≥ 0.5 s → level 3 故障：失能并锁存。")
    bus.silent.update({1, 2})
    rec.run(3)
    bus.silent.clear()
    rec.mark("结束", "关闭：Driver 保证失能后释放串口。")
    rec.run(0.4)


def build(output: Path) -> None:
    with tempfile.TemporaryDirectory() as tmp:
        log = Path(tmp) / "serial.log"
        with Bus({1: 12.3, 2: -4.5}, lag=0.05) as bus:
            driver = Driver.open(DriverConfig(port=bus.path, log_path=str(log)))
            neck = Neck(driver)
            rec = Recorder(neck, bus)
            try:
                scenario(rec)
            finally:
                try:
                    neck.close()
                except NeckError:
                    pass
            assert not bus.errors, bus.errors
        frames = parse_log(log, rec.t0)
    data = {"duration": rec.samples[-1]["t"], "samples": rec.samples, "frames": frames, "marks": rec.marks}
    html = TEMPLATE.read_text().replace("/*DATA*/null", json.dumps(data, ensure_ascii=False, separators=(",", ":")))
    output.write_text(html)
    print(f"{output}: {len(rec.samples)} samples, {len(frames)} frames, {len(rec.marks)} marks, "
          f"{data['duration']:.1f} s, {output.stat().st_size // 1024} KiB")


if __name__ == "__main__":
    build(Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "docs" / "回放.html")
