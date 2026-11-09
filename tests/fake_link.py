"""In-memory SerialLink for controller tests."""

from __future__ import annotations

import time

from neck import protocol as p
from neck.types import LinkError, Pose


class FakeLink:
    def __init__(self, pose: Pose = Pose(0.0, 0.0), follow: bool = True):
        now = time.monotonic()
        self.angles = {1: pose.yaw, 2: pose.pitch}
        self.angle_at = {1: now, 2: now}
        self.follow = follow
        self.feedback = True
        self.frames: list[bytes] = []
        self.queried: list[int] = []
        self.zeros = 0
        self.saves = 0
        self.enabled: set[int] = set()
        self.fail_write = False
        self.fail_read = False
        self.fail_save = False
        self.closed = False
        self.tx_errors = 0
        self.rx_errors = 0
        self.last_error = ""
        self.error_at: float | None = None
        self._pending_angle: dict[int, bytes] = {}
        self._pending_query: list[int] = []
        self.now = time.monotonic()

    def poll(self, now=None):
        if now is not None:
            self.now = now
        return []

    def queue_angle(self, addr: int, deg: float) -> None:
        self._pending_angle[addr] = p.angle(addr, deg)

    def queue_query(self, addr: int) -> None:
        if addr not in self._pending_query:
            self._pending_query.append(addr)

    def clear_pending(self) -> None:
        self._pending_angle.clear()
        self._pending_query.clear()

    def pump(self, now=None, max_frames: int = 3) -> int:
        if now is not None:
            self.now = now
        sent = 0
        for _ in range(max_frames):
            if self._pending_angle:
                addr, frame = next(iter(self._pending_angle.items()))
                del self._pending_angle[addr]
            elif self._pending_query:
                addr = self._pending_query.pop(0)
                frame = p.query_angle(addr)
            else:
                break
            self._handle(frame)
            sent += 1
        return sent

    def write_frame(self, frame: bytes) -> None:
        if self.fail_write:
            raise LinkError("write failed")
        self.now = time.monotonic()
        self._handle(frame)

    def read_angle(self, addr: int, timeout: float = 0.05) -> float:
        if self.fail_read:
            raise LinkError("read failed")
        return self.angles[addr]

    def close(self) -> None:
        self.closed = True

    def _handle(self, frame: bytes) -> None:
        self.frames.append(frame)
        addr, func, data = frame[1], frame[2], frame[3:-2]
        if func == 0x0E:
            self.queried.append(addr)
        elif func == 0x03:
            tenths = int.from_bytes(data, "big")
            deg = (tenths - 3600 if tenths > 1800 else tenths) / 10
            if self.follow and self.feedback:
                self.angles[addr] = deg
                self.angle_at[addr] = self.now
        elif func == 0x0A:
            self.zeros += 1
            self.angles[addr] = 0.0
            self.angle_at[addr] = self.now
        elif func == 0x08:
            if self.fail_save:
                raise LinkError("save failed")
            self.saves += 1
        elif func == 0x06:
            self.enabled.add(addr)
        elif func == 0x05:
            self.enabled.discard(addr)
