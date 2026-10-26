"""The Driver: two motors on one bus that obey angles and report them honestly.

A successful write is not a confirmation; the motors never acknowledge. Only read()
tells where the neck really is.
"""

from __future__ import annotations

import glob
import os
import sys
import time

from .motor import Motor
from .port import Port
from .types import DriverConfig, DriverError, DriverState, Pose, finite, timeout_ok

_AXIS_READ_TIMEOUT = 0.05


def candidates() -> list[str]:
    """Serial devices worth probing, each physical device once."""
    paths = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(
        glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    seen, unique = set(), []
    for path in paths:
        real = os.path.realpath(path)
        if real not in seen:
            seen.add(real)
            unique.append(path)
    return unique


class Driver:
    @classmethod
    def open(cls, config: DriverConfig | None = None) -> Driver:
        """Open the configured port, or scan for the one where both motors answer."""
        config = config or DriverConfig()
        if config.port is not None:
            return cls(config, Port(config.port, config.baud, config.log_path))
        for path in candidates():
            try:
                driver = cls(config, Port(path, config.baud, config.log_path))
            except DriverError:
                continue
            try:
                driver.read(0.1)
                return driver
            except DriverError:
                driver._port.close()
        raise DriverError("no serial device answered for both motor IDs")

    def __init__(self, config: DriverConfig, port: Port):
        self.config = config
        self.state = DriverState.OPEN
        self._port = port
        self._yaw = Motor(port, config.yaw_id)
        self._pitch = Motor(port, config.pitch_id)

    def enable(self, yaw: float, pitch: float) -> None:
        """Torque on, both motors aimed at the given angles (normally where they are now)."""
        yaw, pitch = finite(yaw, "yaw"), finite(pitch, "pitch")
        self._require_port()
        if self.state is DriverState.ARMED:
            raise DriverError("motors are already enabled")
        if self.state is DriverState.UNKNOWN:
            raise DriverError("hardware state unknown; disable must succeed first")
        c = self.config
        self._disable_both()                    # gains are written without torque
        try:
            self._yaw.set_gains(c.yaw_gains)
            self._pitch.set_gains(c.pitch_gains)
            self._yaw.enable()
            self._yaw.start(c.speed_rpm, c.accel, yaw)
            self._pitch.enable()
            self._pitch.start(c.speed_rpm, c.accel, pitch)
        except DriverError as exc:
            self._disable_both()                # raises, state UNKNOWN, if this fails too
            raise DriverError(f"enable failed ({exc}); disable requests sent, "
                              "hardware state unconfirmed") from exc
        self.state = DriverState.ARMED

    def set(self, yaw: float | None, pitch: float | None) -> None:
        """Aim one or both axes (None leaves an axis alone). Needs ARMED."""
        yaw = None if yaw is None else finite(yaw, "yaw")
        pitch = None if pitch is None else finite(pitch, "pitch")
        self._require_port()
        if self.state is not DriverState.ARMED:
            raise DriverError("motors are not enabled")
        if yaw is not None:
            self._yaw.set_angle(yaw)
        if pitch is not None:
            self._pitch.set_angle(pitch)

    def read(self, timeout: float = 0.1) -> Pose:
        """Measured angles of both motors within one shared deadline."""
        timeout = timeout_ok(timeout)
        self._require_port()
        deadline = time.monotonic() + timeout
        yaw = self._yaw.read_angle(min(_AXIS_READ_TIMEOUT, deadline - time.monotonic()))
        pitch = self._pitch.read_angle(min(_AXIS_READ_TIMEOUT, deadline - time.monotonic()))
        return Pose(yaw, pitch)

    def disable(self) -> None:
        """Torque off. Also the way out of UNKNOWN while the port is still open."""
        self._require_port()
        self._disable_both()

    def close(self) -> None:
        """Disable if needed, release the port. Raises if the disable could not be sent."""
        if not self._port.is_open:
            if self.state is DriverState.UNKNOWN:
                raise DriverError("port closed after a failed disable; hardware state unknown")
            return
        try:
            if self.state in (DriverState.ARMED, DriverState.UNKNOWN):
                self._disable_both()
        finally:
            self._port.close()
            if self.state is not DriverState.UNKNOWN:
                self.state = DriverState.CLOSED

    def __enter__(self) -> Driver:
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def __del__(self):
        port = getattr(self, "_port", None)
        if port is not None and port.is_open:
            try:
                self.close()
            except DriverError as exc:
                print(f"neck driver close failed: {exc}", file=sys.stderr)

    def _require_port(self) -> None:
        if not self._port.is_open:
            raise DriverError(f"driver is {self.state.value}")

    def _disable_both(self) -> None:
        failed = []
        for name, motor in (("yaw", self._yaw), ("pitch", self._pitch)):
            try:
                motor.disable()
            except DriverError:
                failed.append(name)
        if failed:
            self.state = DriverState.UNKNOWN
            raise DriverError(f"disable request failed for {' and '.join(failed)}; "
                              "hardware state unknown")
        self.state = DriverState.OPEN
