"""In-memory Driver with the same contract as the real one, for Control tests."""

from __future__ import annotations

from neck.driver import DriverError, DriverState, Pose
from neck.driver.types import finite, timeout_ok


class FakeDriver:
    """Measured pose follows the command on read() unless `follow` is False.

    Failure switches: read_error, set_error, enable_error, disable_error.
    Counters and history: reads, disables, sent (every set() call as a tuple).
    """

    def __init__(self, pose: Pose = Pose(0.0, 0.0)):
        self.state = DriverState.OPEN
        self.measured = self.command = pose
        self.follow = True
        self.sent: list[tuple[float | None, float | None]] = []
        self.reads = self.disables = 0
        self.read_error = self.set_error = self.enable_error = self.disable_error = False

    def _usable(self):
        if self.state in (DriverState.CLOSED, DriverState.UNKNOWN):
            raise DriverError(f"driver is {self.state.value}")

    def enable(self, yaw: float, pitch: float) -> None:
        self._usable()
        if self.state is DriverState.ARMED:
            raise DriverError("already armed")
        pose = Pose(finite(yaw, "yaw"), finite(pitch, "pitch"))
        if self.enable_error:
            raise DriverError("enable failed")
        self.command = pose
        self.state = DriverState.ARMED

    def set(self, yaw: float | None, pitch: float | None) -> None:
        self._usable()
        if self.state is not DriverState.ARMED:
            raise DriverError("motors are not enabled")
        y = None if yaw is None else finite(yaw, "yaw")
        p = None if pitch is None else finite(pitch, "pitch")
        if self.set_error:
            raise DriverError("write failed")
        self.command = Pose(self.command.yaw if y is None else y,
                            self.command.pitch if p is None else p)
        self.sent.append((y, p))

    def read(self, timeout: float = 0.1) -> Pose:
        self._usable()
        timeout_ok(timeout)
        self.reads += 1
        if self.read_error:
            raise DriverError("read timeout")
        if self.follow and self.state is DriverState.ARMED:
            self.measured = self.command
        return self.measured

    def disable(self) -> None:
        self._usable()
        self.disables += 1
        if self.disable_error:
            self.state = DriverState.UNKNOWN
            raise DriverError("disable request failed; hardware state unknown")
        self.state = DriverState.OPEN

    def close(self) -> None:
        if self.state is DriverState.CLOSED:
            return
        if self.state in (DriverState.ARMED, DriverState.UNKNOWN):
            self.disables += 1
            if self.disable_error:
                self.state = DriverState.UNKNOWN
                raise DriverError("disable request failed; hardware state unknown")
        self.state = DriverState.CLOSED
