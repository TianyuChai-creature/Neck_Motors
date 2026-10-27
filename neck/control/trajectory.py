"""Turn "where to look" into a smooth stream of angle commands. Pure maths: no I/O, no clock.

Pipeline per axis: speed-adaptive low-pass filter (1 Euro) -> dead band -> critically
damped follower with speed and acceleration limits -> 0.1 degree output with hysteresis.
Yaw without limits is circular (shortest path). The caller owns the clock: every method
takes a time `t` in seconds from one monotonic source.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..driver.types import Pose, finite


def wrap(angle: float) -> float:
    """Signed angle in [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def delta(a: Pose, b: Pose) -> Pose:
    """a - b, with yaw taken the short way round."""
    return Pose(wrap(a.yaw - b.yaw), a.pitch - b.pitch)


def _limits(value, name: str) -> tuple[float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain a lower and upper angle")
    lower, upper = (finite(v, name) for v in value)
    if not -180 <= lower < upper <= 180:
        raise ValueError(f"{name} must lie within [-180, 180], lower < upper")
    if math.ceil(lower * 10) > math.floor(upper * 10):
        raise ValueError(f"{name} must contain a representable 0.1 degree target")
    return lower, upper


def _alpha(cutoff: float, dt: float) -> float:
    rate = 2 * math.pi * cutoff * dt
    return 1.0 if math.isinf(rate) else rate / (1 + rate)


@dataclass(frozen=True)
class TrajectoryConfig:
    """Speeds are deg/s, accelerations deg/s^2. Limits bound new targets; a start pose
    outside them is accepted and only ever leads inward."""

    pitch_limits: tuple[float, float] = (-180.0, 45.0)
    yaw_limits: tuple[float, float] | None = (-60.0, 60.0)
    max_speed_deg_s: float = 90.0
    max_accel_deg_s2: float = 1440.0
    response_hz: float = 15.0
    min_cutoff_hz: float = 6.0
    beta: float = 0.3
    derivative_cutoff_hz: float = 2.0
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
            if finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("beta", "deadband_deg", "hysteresis_deg"):
            if finite(getattr(self, name), name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        if self.response_hz > 20 or self.max_gap_s > 0.1:
            raise ValueError("response_hz must be <= 20 and max_gap_s <= 0.1")
        if self.max_speed_deg_s * self.max_gap_s > 179:
            raise ValueError("a control gap must not allow a 180 degree target jump")
        if self.max_accel_deg_s2 > 1e6:
            raise ValueError("max_accel_deg_s2 must be <= 1e6")
        if self.hysteresis_deg >= 0.05 or self.deadband_deg > 0.5:
            raise ValueError("hysteresis_deg must be < 0.05 and deadband_deg <= 0.5")


class TrajectoryError(RuntimeError):
    """The step cadence was broken; reset() from fresh feedback before resuming."""


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

    def condition(self, dt: float, config: TrajectoryConfig) -> None:
        # Speed-adaptive low-pass filter (1 Euro principle). Input is kept unwrapped so
        # crossing +/-180 does not look like a spike.
        alpha_d = _alpha(config.derivative_cutoff_hz, dt)
        derivative = (self.raw - self.previous_raw) / dt
        self.derivative += alpha_d * (derivative - self.derivative)
        self.previous_raw = self.raw
        cutoff = config.min_cutoff_hz + config.beta * abs(self.derivative)
        self.filtered += _alpha(cutoff, dt) * (self.raw - self.filtered)
        # Anchor to the accepted goal, not the previous sample: slow cumulative movement
        # must eventually pass the noise threshold.
        if abs(self.filtered - self.goal) > config.deadband_deg:
            self.goal = self.filtered

    def brake(self) -> None:
        self.filtered = self.goal = self.previous_raw = self.position
        self.derivative = 0.0

    def advance(self, dt: float, config: TrajectoryConfig, braking: bool) -> None:
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
                self.position = max(self.limits[0], min(self.limits[1], self.position))
        if abs(self.position - self.ticks / 10) > 0.05 + config.hysteresis_deg:
            self.ticks = self._bound_ticks(round(self.position * 10))

    def command(self) -> float:
        ticks = self.ticks if self.limits is not None else (self.ticks + 1800) % 3600 - 1800
        return ticks / 10

    def current(self) -> float:
        return self.position if self.limits is not None else wrap(self.position)


class Trajectory:
    """reset(pose, t) from measured feedback, set_target(pose, t) whenever a new target
    arrives, step(t) on every tick (also when nothing arrived). Latest target wins."""

    def __init__(self, config: TrajectoryConfig | None = None):
        self.config = config or TrajectoryConfig()
        self.reset(Pose(0.0, 0.0), 0.0)

    def reset(self, pose: Pose, t: float) -> None:
        """Start from where the motors really are (or a simulated pose)."""
        t = finite(t, "t")
        yaw, pitch = wrap(finite(pose.yaw, "yaw")), wrap(finite(pose.pitch, "pitch"))
        c = self.config
        # A start outside the target limits stretches the axis limits to include it.
        yaw_limits = None if c.yaw_limits is None else (min(c.yaw_limits[0], yaw), max(c.yaw_limits[1], yaw))
        pitch_limits = (min(c.pitch_limits[0], pitch), max(c.pitch_limits[1], pitch))
        self._yaw, self._pitch = _Axis(yaw, yaw_limits), _Axis(pitch, pitch_limits)
        self._last_step = t
        self._last_input: float | None = None
        self._stale = self._faulted = False

    def set_target(self, pose: Pose, t: float) -> None:
        """Replace the latest target; invalid input is rejected whole, never clipped."""
        if self._faulted:
            raise TrajectoryError("reset from fresh feedback before resuming")
        t = finite(t, "t")
        if t < self._last_step or (self._last_input is not None and t < self._last_input):
            raise ValueError("time must not go backwards")
        yaw, pitch = finite(pose.yaw, "yaw target"), finite(pose.pitch, "pitch target")
        for angle, limits, name in ((yaw, self.config.yaw_limits, "yaw"),
                                    (pitch, self.config.pitch_limits, "pitch")):
            if limits is not None and not limits[0] <= angle <= limits[1]:
                raise ValueError(f"{name} target is outside the limits")
        if self.config.yaw_limits is None:
            yaw = self._yaw.raw + wrap(yaw - self._yaw.raw)
        self._yaw.raw, self._pitch.raw = yaw, pitch
        self._last_input = t

    def step(self, t: float) -> Pose:
        """Next 0.1 degree command. Does not read feedback."""
        if self._faulted:
            raise TrajectoryError("reset from fresh feedback before resuming")
        t = finite(t, "t")
        if t < self._last_step or (self._last_input is not None and t < self._last_input):
            raise ValueError("time must not go backwards")
        dt = t - self._last_step
        if dt > self.config.max_gap_s + 1e-12:
            self._faulted = True
            raise TrajectoryError("step gap exceeded max_gap_s; reset from fresh feedback")
        if dt < 1e-6:
            return self.command
        stale = self._last_input is not None and t - self._last_input >= self.config.stale_after_s
        if stale and not self._stale:
            self._yaw.brake()
            self._pitch.brake()
        self._stale = stale
        for axis in (self._yaw, self._pitch):
            if not stale:
                axis.condition(dt, self.config)
            axis.advance(dt, self.config, stale)
            if stale:
                axis.brake()        # resume filtering from the braking endpoint
        self._last_step = t
        return self.command

    @property
    def command(self) -> Pose:
        return Pose(self._yaw.command(), self._pitch.command())

    @property
    def current(self) -> Pose:
        """Continuous position before the 0.1 degree output hysteresis."""
        return Pose(self._yaw.current(), self._pitch.current())

    @property
    def target(self) -> Pose:
        yaw = self._yaw.raw if self.config.yaw_limits is not None else wrap(self._yaw.raw)
        return Pose(yaw, self._pitch.raw)

    @property
    def velocity(self) -> tuple[float, float]:
        return self._yaw.velocity, self._pitch.velocity

    @property
    def stale(self) -> bool:
        return self._stale

    @property
    def faulted(self) -> bool:
        return self._faulted
