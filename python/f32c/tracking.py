"""Hardware-free target conditioning for the synchronous F32C position API.

The caller owns the motor, its enable/disable lifecycle and a regular control
tick. These states describe generated commands, not measured motor motion.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import time

from . import State


def _finite(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} must be a finite number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite number")
    return result


def _wrap(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def _limits(value: tuple[float, float], name: str) -> tuple[float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain a lower and upper angle")
    lower, upper = (_finite(v, name) for v in value)
    if not -180 <= lower < upper <= 180:
        raise ValueError(f"{name} must lie within [-180, 180], lower < upper")
    if math.ceil(lower * 10) > math.floor(upper * 10):
        raise ValueError(f"{name} must contain a representable 0.1 degree target")
    return lower, upper


def _alpha(cutoff: float, dt: float) -> float:
    rate = 2 * math.pi * cutoff * dt
    return 1.0 if math.isinf(rate) else rate / (1 + rate)


@dataclass(frozen=True)
class TrackingConfig:
    """Software trajectory settings; pitch limits must come from calibration.

    Unbounded yaw uses the shortest circular path. Supplying yaw_limits makes
    yaw non-circular, like pitch. Speeds are deg/s and accelerations deg/s^2.
    Defaults are a starting point for software tests, not validated motor gains.
    """

    pitch_limits: tuple[float, float]
    yaw_limits: tuple[float, float] | None = None
    max_speed_deg_s: float = 180.0
    max_accel_deg_s2: float = 1800.0
    response_hz: float = 8.0
    min_cutoff_hz: float = 3.0
    beta: float = 0.15
    derivative_cutoff_hz: float = 1.0
    deadband_deg: float = 0.05
    hysteresis_deg: float = 0.02
    stale_after_s: float = 0.2
    max_gap_s: float = 0.1

    def __post_init__(self) -> None:
        object.__setattr__(self, "pitch_limits", _limits(self.pitch_limits, "pitch_limits"))
        if self.yaw_limits is not None:
            object.__setattr__(self, "yaw_limits", _limits(self.yaw_limits, "yaw_limits"))
        for name in ("max_speed_deg_s", "max_accel_deg_s2", "response_hz", "min_cutoff_hz",
                     "derivative_cutoff_hz", "stale_after_s", "max_gap_s"):
            value = _finite(getattr(self, name), name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("beta", "deadband_deg", "hysteresis_deg"):
            if _finite(getattr(self, name), name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.response_hz > 20 or self.max_gap_s > 0.1:
            raise ValueError("response_hz must be <= 20 and max_gap_s <= 0.1")
        if self.max_speed_deg_s * self.max_gap_s > 179:
            raise ValueError("a control gap must not allow a 180 degree target jump")
        if self.max_accel_deg_s2 > 1e6:
            raise ValueError("max_accel_deg_s2 must be <= 1e6")
        if self.hysteresis_deg >= 0.05 or self.deadband_deg > 0.5:
            raise ValueError("hysteresis_deg must be < 0.05 and deadband_deg <= 0.5")


class TrackingError(RuntimeError):
    """Control timing was interrupted; reset from fresh feedback before resuming."""


class _Axis:
    def __init__(self, angle: float, limits: tuple[float, float] | None):
        self.limits = limits
        self.position = self.raw = self.filtered = self.goal = self.previous_raw = angle
        self.velocity = self.derivative = 0.0
        self.ticks = self._bound_ticks(round(angle * 10))

    def _bound_ticks(self, ticks: int) -> int:
        if self.limits is not None:
            ticks = max(math.ceil(self.limits[0] * 10), min(math.floor(self.limits[1] * 10), ticks))
        return ticks

    def condition(self, dt: float, config: TrackingConfig) -> None:
        # Speed-adaptive low-pass filtering (1 Euro principle). Angular input
        # is already unwrapped, so crossing +/-180 does not look like a spike.
        alpha_d = _alpha(config.derivative_cutoff_hz, dt)
        derivative = (self.raw - self.previous_raw) / dt
        self.derivative += alpha_d * (derivative - self.derivative)
        self.previous_raw = self.raw
        cutoff = config.min_cutoff_hz + config.beta * abs(self.derivative)
        alpha = _alpha(cutoff, dt)
        self.filtered += alpha * (self.raw - self.filtered)
        # Anchor to the accepted goal, not the previous input sample: slow
        # cumulative movement must eventually pass the noise threshold.
        if abs(self.filtered - self.goal) > config.deadband_deg:
            self.goal = self.filtered

    def brake(self) -> None:
        self.filtered = self.goal = self.previous_raw = self.position
        self.derivative = 0.0

    def advance(self, dt: float, config: TrackingConfig, braking: bool) -> None:
        steps = max(1, math.ceil(dt / 0.001))
        h = dt / steps
        omega = 2 * math.pi * config.response_hz
        acceleration_limit = config.max_accel_deg_s2
        for _ in range(steps):
            v = self.velocity
            if braking:
                next_v = math.copysign(max(0.0, abs(v) - acceleration_limit * h), v)
            else:
                acceleration = omega * omega * (self.goal - self.position) - 2 * omega * v
                acceleration = max(-acceleration_limit, min(acceleration_limit, acceleration))
                next_v = max(-config.max_speed_deg_s,
                             min(config.max_speed_deg_s, v + acceleration * h))
            if next_v:
                direction = math.copysign(1.0, next_v)
                distances = []
                if self.limits is not None:
                    boundary = self.limits[1] if direction > 0 else self.limits[0]
                    distances.append(direction * (boundary - self.position))
                target_distance = direction * (self.goal - self.position)
                forward_v = direction * v
                if not braking and target_distance >= -1e-12 and (
                        forward_v <= 0 or forward_v * forward_v <= 2 * acceleration_limit * target_distance + 1e-8):
                    distances.append(target_distance)
                # Keep enough distance for a bounded-acceleration stop. Solve
                # h*(v+u)/2 + u^2/(2*a) <= remaining_distance for the next speed u.
                for distance in distances:
                    discriminant = (acceleration_limit * h) ** 2 - 4 * acceleration_limit * h * forward_v
                    discriminant += 8 * acceleration_limit * max(0.0, distance)
                    safe_speed = max(0.0, (math.sqrt(max(0.0, discriminant)) - acceleration_limit * h) / 2)
                    next_v = direction * min(abs(next_v), safe_speed)
            self.position += (v + next_v) * h / 2
            self.velocity = next_v
            if self.limits is not None:
                # Numerical guard; trajectory braking above handles the boundary.
                self.position = max(self.limits[0], min(self.limits[1], self.position))
        if abs(self.position - self.ticks / 10) > 0.05 + config.hysteresis_deg:
            self.ticks = self._bound_ticks(round(self.position * 10))

    def command(self) -> float:
        ticks = self.ticks if self.limits is not None else (self.ticks + 1800) % 3600 - 1800
        return ticks / 10

    def continuous(self) -> float:
        return self.position if self.limits is not None else _wrap(self.position)


class HeadTracker:
    """Generate latest-target commands without opening or writing a motor.

    Call set_target() with signed motor-coordinate angles, then call step() at
    a regular cadence (normally 50 Hz), including when no packet arrives.
    All timestamps use the same monotonic clock; use one owner thread.
    """

    def __init__(self, initial: State, config: TrackingConfig, *, timestamp: float | None = None):
        if not isinstance(config, TrackingConfig):
            raise ValueError("config must be a TrackingConfig")
        self._config = config
        self.reset(initial, timestamp=timestamp)

    @staticmethod
    def _time(timestamp: float | None) -> float:
        return _finite(time.monotonic() if timestamp is None else timestamp, "timestamp")

    def reset(self, initial: State, *, timestamp: float | None = None) -> None:
        """Synchronize to fresh motor feedback, or a simulated initial pose."""
        if not isinstance(initial, State):
            raise ValueError("initial must be a State from feedback or simulation")
        now = self._time(timestamp)
        yaw = _wrap(_finite(initial.yaw_deg, "initial yaw"))
        pitch = _wrap(_finite(initial.pitch_deg, "initial pitch"))
        if yaw == -180 and self.config.yaw_limits is not None and self.config.yaw_limits[0] > -180:
            yaw = 180.0
        if pitch == -180 and self.config.pitch_limits[0] > -180:
            pitch = 180.0
        for angle, limits, name in ((yaw, self.config.yaw_limits, "yaw"),
                                    (pitch, self.config.pitch_limits, "pitch")):
            if limits is not None and not limits[0] <= angle <= limits[1]:
                raise ValueError(f"initial {name} is outside calibrated limits")
        self._yaw = _Axis(yaw, self.config.yaw_limits)
        self._pitch = _Axis(pitch, self.config.pitch_limits)
        self._last_step = now
        self._last_input: float | None = None
        self._stale = self._faulted = False

    def set_target(self, yaw_deg: float, pitch_deg: float, *, timestamp: float | None = None) -> None:
        """Replace the latest target; reject invalid input without clipping it."""
        if self._faulted:
            raise TrackingError("reset from fresh feedback before resuming tracking")
        now = self._time(timestamp)
        if now < self._last_step or (self._last_input is not None and now < self._last_input):
            raise ValueError("target timestamp must not go backwards")
        yaw = _finite(yaw_deg, "yaw target")
        pitch = _finite(pitch_deg, "pitch target")
        for angle, limits, name in ((yaw, self.config.yaw_limits, "yaw"),
                                    (pitch, self.config.pitch_limits, "pitch")):
            if limits is not None and not limits[0] <= angle <= limits[1]:
                raise ValueError(f"{name} target is outside calibrated limits")
        if self.config.yaw_limits is None:
            yaw = self._yaw.raw + _wrap(yaw - self._yaw.raw)
        self._yaw.raw, self._pitch.raw = yaw, pitch
        self._last_input = now

    def step(self, *, timestamp: float | None = None) -> State:
        """Return a generated 0.1-degree command; this does not read feedback."""
        if self._faulted:
            raise TrackingError("reset from fresh feedback before resuming tracking")
        now = self._time(timestamp)
        if now < self._last_step or (self._last_input is not None and now < self._last_input):
            raise ValueError("step timestamp must not go backwards")
        dt = now - self._last_step
        if dt > self.config.max_gap_s + 1e-12:
            self._faulted = True
            raise TrackingError("control tick gap exceeded max_gap_s; reset from fresh feedback")
        if dt < 1e-6:
            return self.command
        stale = self._last_input is not None and now - self._last_input >= self.config.stale_after_s
        if stale and not self._stale:
            self._yaw.brake()
            self._pitch.brake()
        self._stale = stale
        for axis in (self._yaw, self._pitch):
            if not stale:
                axis.condition(dt, self.config)
            axis.advance(dt, self.config, stale)
            if stale:
                # Resume filtering from the braking endpoint, not the earlier
                # point where input loss was detected.
                axis.brake()
        self._last_step = now
        return self.command

    @property
    def config(self) -> TrackingConfig:
        return self._config

    @property
    def target(self) -> State:
        """Latest accepted input, in signed motor coordinates."""
        yaw = self._yaw.raw if self.config.yaw_limits is not None else _wrap(self._yaw.raw)
        return State(yaw, self._pitch.raw)

    @property
    def command(self) -> State:
        return State(self._yaw.command(), self._pitch.command())

    @property
    def continuous_command(self) -> State:
        """Continuous trajectory before 0.1-degree output hysteresis."""
        return State(self._yaw.continuous(), self._pitch.continuous())

    @property
    def filtered_target(self) -> State:
        yaw = self._yaw.filtered if self.config.yaw_limits is not None else _wrap(self._yaw.filtered)
        return State(yaw, self._pitch.filtered)

    @property
    def velocity_deg_s(self) -> tuple[float, float]:
        return self._yaw.velocity, self._pitch.velocity

    @property
    def stale(self) -> bool:
        return self._stale

    @property
    def faulted(self) -> bool:
        return self._faulted
