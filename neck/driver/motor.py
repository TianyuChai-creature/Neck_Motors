"""One F32C motor on a shared bus: command frames out, angle queries in."""

from __future__ import annotations

import time

from . import protocol as p
from .port import Port
from .types import DriverError, Gains

WRITE_TIMEOUT = 0.1     # s, one frame
COMMAND_GAP = 0.008     # s, the bus needs idle time after every command
QUERY_TIMEOUT = 0.025   # s, one angle query before it is repeated
QUERY_TRIES = 2
IDLE_AFTER_REPLY = 0.003


class Motor:
    def __init__(self, port: Port, addr: int):
        self.port = port
        self.addr = addr

    def _send(self, frame: bytes) -> None:
        self.port.write(frame, time.monotonic() + WRITE_TIMEOUT)
        time.sleep(COMMAND_GAP)

    def enable(self) -> None:
        self._send(p.enable(self.addr))

    def disable(self) -> None:
        self._send(p.disable(self.addr))

    def set_gains(self, gains: Gains) -> None:
        self._send(p.speed_kp(self.addr, gains.speed_kp))
        self._send(p.speed_ki(self.addr, gains.speed_ki))
        self._send(p.pos_kp(self.addr, gains.pos_kp))
        self._send(p.pos_ki(self.addr, gains.pos_ki))

    def start(self, speed_rpm: int, accel: int, deg: float) -> None:
        """Direct single-turn position mode, aimed at `deg` before motion limits are set."""
        self._send(p.mode_single_turn_direct(self.addr))
        self._send(p.speed_rpm(self.addr, speed_rpm))
        self._send(p.angle(self.addr, deg))
        self._send(p.accel(self.addr, accel))

    def set_angle(self, deg: float) -> None:
        self._send(p.angle(self.addr, deg))

    def read_angle(self, timeout: float = 0.05) -> float:
        """Signed degrees; raises DriverError if no valid reply arrives within `timeout`."""
        deadline = time.monotonic() + timeout
        for _ in range(QUERY_TRIES):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            deg = self._query(time.monotonic() + min(QUERY_TIMEOUT, remaining))
            if deg is not None:
                return deg
        raise DriverError(f"motor {self.addr} did not answer the angle query")

    def _query(self, deadline: float) -> float | None:
        self.port.flush_input()
        try:
            self.port.write(p.query_angle(self.addr), deadline)
        except DriverError:
            return None
        raw = b""
        while time.monotonic() < deadline:
            raw += self.port.read(deadline)
            deg = p.find_angle(raw, self.addr)
            if deg is not None:
                time.sleep(IDLE_AFTER_REPLY)
                return deg
            if len(raw) > 256:
                raw = raw[-8:]      # keep a tail long enough for a frame split across reads
        return None
