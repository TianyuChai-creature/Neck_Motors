"""Public data types of the driver layer."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from numbers import Real

__all__ = ["DriverConfig", "DriverError", "DriverState", "Gains", "Pose"]


class DriverError(RuntimeError):
    """Hardware or state failure. Bad arguments raise ValueError instead."""


class DriverState(Enum):
    """What the driver knows about the motors.

    CLOSED   port released, motors known to have been told to disable
    OPEN     port open, torque off
    ARMED    torque on; only here may set() be called
    UNKNOWN  a disable request failed to go out; torque may still be on
    """

    CLOSED = "closed"
    OPEN = "open"
    ARMED = "armed"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class Pose:
    """Yaw and pitch in degrees, signed motor coordinates (0 is the stored zero)."""

    yaw: float
    pitch: float


@dataclass(frozen=True)
class Gains:
    """PI gains of one motor, each 0..256."""

    speed_kp: int
    speed_ki: int
    pos_kp: int
    pos_ki: int

    def __post_init__(self):
        for name in ("speed_kp", "speed_ki", "pos_kp", "pos_ki"):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value <= 256:
                raise ValueError(f"{name} must be an integer between 0 and 256")


@dataclass(frozen=True)
class DriverConfig:
    port: str | None = None          # None: scan for the port that answers both IDs
    baud: int = 115200
    yaw_id: int = 1
    pitch_id: int = 2
    speed_rpm: int = 15              # travel speed of the motor's own position loop
    accel: int = 2                   # rev/s^2
    yaw_gains: Gains = Gains(40, 4, 42, 0)
    pitch_gains: Gains = Gains(10, 4, 40, 0)
    log_path: str | None = None      # hex log of every frame sent and received

    def __post_init__(self):
        if self.port is not None and not isinstance(self.port, str):
            raise ValueError("port must be a string or None")
        if self.baud not in (9600, 57600, 115200):
            raise ValueError("unsupported baud rate")
        for name in ("yaw_id", "pitch_id"):
            if type(getattr(self, name)) is not int or not 1 <= getattr(self, name) <= 255:
                raise ValueError(f"{name} must be an integer between 1 and 255")
        if self.yaw_id == self.pitch_id:
            raise ValueError("motor IDs must be distinct")
        if type(self.speed_rpm) is not int or not 0 <= self.speed_rpm <= 1000:
            raise ValueError("speed_rpm must be an integer between 0 and 1000")
        if type(self.accel) is not int or not 0 <= self.accel <= 65535:
            raise ValueError("accel must be an integer between 0 and 65535")


def finite(value, name: str) -> float:
    """Validate a number argument; bool and non-numbers are rejected."""
    try:
        if isinstance(value, bool) or not isinstance(value, Real):
            raise TypeError
        result = float(value)
    except (TypeError, OverflowError):
        raise ValueError(f"{name} must be a finite number") from None
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def timeout_ok(value) -> float:
    """Validate a read timeout in seconds, 0 < t <= 1."""
    value = finite(value, "timeout")
    if not 0 < value <= 1:
        raise ValueError("timeout must be within (0, 1] seconds")
    return value
