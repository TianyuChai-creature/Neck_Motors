"""Fake F32C bus on a pseudo-terminal: motors that obey frames and answer angle queries."""

from __future__ import annotations

import os
import select
import threading
import time
from dataclasses import dataclass, field


@dataclass
class Axis:
    deg: float = 0.0
    enabled: bool = False
    mode: int | None = None
    rpm: int | None = None
    accel: int | None = None
    gains: dict = field(default_factory=dict)


def xor(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


class Bus:
    """`path` is the device to hand to the code under test.

    Options: echo, fragmented, noise, corrupt (decorate each reply), silent (IDs that never
    answer), delay (seconds before replying). Recorded: frames, times, errors, axes.
    """

    GAINS = {0x0F: "speed_kp", 0x10: "speed_ki", 0x11: "pos_kp", 0x12: "pos_ki"}

    def __init__(self, positions=None, *, echo=False, fragmented=False, noise=False,
                 corrupt=False, delay=0.0):
        positions = positions or {1: 12.3, 2: -4.5}
        self.axes = {addr: Axis(deg) for addr, deg in positions.items()}
        self.echo, self.fragmented, self.noise, self.corrupt, self.delay = (
            echo, fragmented, noise, corrupt, delay)
        self.silent: set[int] = set()
        self.frames: list[bytes] = []
        self.times: list[float] = []
        self.errors: list[str] = []
        self._master, self._slave = os.openpty()
        self.path = os.ttyname(self._slave)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(1)
        os.close(self._master)
        os.close(self._slave)

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.stop()

    def commands(self, func: int) -> list[bytes]:
        return [f for f in self.frames if f[2] == func]

    def _run(self) -> None:
        pending = bytearray()
        try:
            while not self._stop.is_set():
                if not select.select([self._master], [], [], 0.01)[0]:
                    continue
                pending += os.read(self._master, 1024)
                while len(pending) >= 3:
                    size = 6 if pending[2] == 0x0E else 5 if pending[2] in (5, 6, 8, 10) else 7
                    if len(pending) < size:
                        break
                    frame = bytes(pending[:size])
                    del pending[:size]
                    self._handle(frame)
        except Exception as exc:        # surfaced by tests through `errors`
            self.errors.append(repr(exc))

    def _handle(self, frame: bytes) -> None:
        if frame[0] != 0x7A or frame[-1] != 0x7B or xor(frame[:-2]) != frame[-2]:
            self.errors.append(f"malformed frame {frame.hex(' ')}")
            return
        self.frames.append(frame)
        self.times.append(time.monotonic())
        addr, func, data = frame[1], frame[2], frame[3:-2]
        axis = self.axes.get(addr)
        if axis is None:
            return
        if func == 0x06:
            axis.enabled = True
        elif func == 0x05:
            axis.enabled = False
        elif func == 0x00:
            axis.mode = int.from_bytes(data, "big")
        elif func == 0x01:
            axis.rpm = int.from_bytes(data, "big", signed=True)
        elif func == 0x07:
            axis.accel = int.from_bytes(data, "big")
        elif func in self.GAINS:
            axis.gains[self.GAINS[func]] = int.from_bytes(data, "big")
        elif func == 0x03 and axis.enabled:
            tenths = int.from_bytes(data, "big")
            axis.deg = (tenths - 3600 if tenths > 1800 else tenths) / 10
        elif func == 0x0E and addr not in self.silent:
            self._reply(frame, addr, axis)

    def _reply(self, query: bytes, addr: int, axis: Axis) -> None:
        if self.delay:
            time.sleep(self.delay)
        body = bytes((0x7A, addr, 2)) + round(axis.deg * 10).to_bytes(4, "big", signed=True)
        reply = body + bytes((xor(body), 0x7B))
        prefix = query if self.echo else b""
        if self.noise:
            other = bytes((0x7A, 99, 2, 0, 0, 0, 123))
            prefix += b"\x00\xff" + other + bytes((xor(other), 0x7B))
        if self.corrupt:
            prefix += reply[:-2] + bytes((reply[-2] ^ 1, 0x7B))
        if self.fragmented:
            os.write(self._master, prefix + reply[:4])
            time.sleep(0.008)
            os.write(self._master, reply[4:])
        else:
            os.write(self._master, prefix + reply)
