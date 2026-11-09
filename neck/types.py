"""Public types for the simplified neck core."""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real

__all__ = [
    "CommandResult", "Gains", "LinkError", "NeckConfig", "NeckError", "Pose",
    "ProtocolError", "Status", "ZeroResult", "finite",
]


class NeckError(RuntimeError):
    """Base error for the neck core."""


class LinkError(NeckError):
    """The serial link could not be opened, read, or written."""


class ProtocolError(NeckError):
    """A frame was malformed or an angle could not be parsed."""


@dataclass(frozen=True)
class Pose:
    """Yaw and pitch in degrees, signed motor coordinates (zero is the stored zero)."""

    yaw: float
    pitch: float


@dataclass(frozen=True)
class Gains:
    """PI gains of one motor, each 0..256."""

    speed_kp: int
    speed_ki: int
    pos_kp: int
    pos_ki: int

    def __post_init__(self) -> None:
        for name in ("speed_kp", "speed_ki", "pos_kp", "pos_ki"):
            value = getattr(self, name)
            if type(value) is not int or not 0 <= value <= 256:
                raise ValueError(f"{name} must be an integer between 0 and 256")


def _limits(value, name: str) -> tuple[float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain a lower and upper angle")
    lower, upper = (finite(v, name) for v in value)
    if not -180 <= lower < upper <= 180:
        raise ValueError(f"{name} must lie within [-180, 180], lower < upper")
    return lower, upper


@dataclass(frozen=True)
class NeckConfig:
    """Everything the controller needs. Defaults are the bench-tested values."""

    # Link
    port: str | None = None          # None: scan for the port where both IDs answer
    baud: int = 115200
    yaw_id: int = 1
    pitch_id: int = 2
    frame_gap: float = 0.0015        # manual: about 1 ms between frames
    feedback_period: float = 0.01    # one axis per period, alternating
    feedback_timeout: float = 0.1    # no fresh angle for this long -> link not OK
    log_path: str | None = None

    # Motor setup
    speed_rpm: int = 15
    accel: int = 2
    yaw_gains: Gains = Gains(40, 4, 42, 0)
    pitch_gains: Gains = Gains(10, 4, 40, 0)

    # Motion
    yaw_limits: tuple[float, float] | None = (-60.0, 60.0)
    pitch_limits: tuple[float, float] = (-180.0, 45.0)
    max_speed: float | None = 90.0      # None: no software velocity clamp
    max_accel: float | None = 1440.0    # None: no software acceleration clamp
    response_hz: float = 20.0
    control_period: float = 0.01
    input_timeout: float = 0.2
    max_lead: float | None = 20.0       # None: disable the lead guard
    lead_timeout: float = 0.3

    # Failure policy: "hold" freezes the last command; "disable" drops torque
    # after disable_after seconds without feedback.
    on_link_loss: str = "hold"
    disable_after: float = 0.5
    zero_tolerance: float = 0.5

    def __post_init__(self) -> None:
        if self.port is not None and not isinstance(self.port, str):
            raise ValueError("port must be a string or None")
        if self.baud not in (9600, 57600, 115200):
            raise ValueError("unsupported baud rate")
        for name in ("yaw_id", "pitch_id"):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= 255:
                raise ValueError(f"{name} must be an integer between 1 and 255")
        if self.yaw_id == self.pitch_id:
            raise ValueError("motor IDs must be distinct")
        if type(self.speed_rpm) is not int or not 0 <= self.speed_rpm <= 1000:
            raise ValueError("speed_rpm must be an integer between 0 and 1000")
        if type(self.accel) is not int or not 0 <= self.accel <= 65535:
            raise ValueError("accel must be an integer between 0 and 65535")
        if self.yaw_limits is not None:
            object.__setattr__(self, "yaw_limits", _limits(self.yaw_limits, "yaw_limits"))
        object.__setattr__(self, "pitch_limits", _limits(self.pitch_limits, "pitch_limits"))
        for name in ("frame_gap", "feedback_period", "feedback_timeout", "control_period",
                     "input_timeout", "lead_timeout", "disable_after",
                     "zero_tolerance"):
            if finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("max_speed", "max_accel", "response_hz", "max_lead"):
            value = getattr(self, name)
            if value is not None and finite(value, name) <= 0:
                raise ValueError(f"{name} must be positive or None")
        if self.on_link_loss not in ("hold", "disable"):
            raise ValueError('on_link_loss must be "hold" or "disable"')


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


@dataclass(frozen=True)
class CommandResult:
    ok: bool
    message: str = ""
    torque: str = ""


@dataclass(frozen=True)
class ZeroResult:
    ok: bool
    zeroed: bool
    verified: bool
    saved: bool
    measured: Pose | None
    error: str = ""


@dataclass(frozen=True)
class Status:
    enabled: bool
    link_ok: bool
    torque: str                  # "on", "off", or "unknown"
    target: Pose | None
    command: Pose
    measured: Pose | None
    feedback_age: float | None
    input_age: float | None
    reason: str
    tx_errors: int = 0
    rx_errors: int = 0
    last_error: str = ""
