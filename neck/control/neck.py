"""The Neck: the one public entry of the control layer.

One thread owns it and calls tick() every ~20 ms. It reads the measured pose, asks the
Supervisor whether anything is wrong, lets the Trajectory produce the next safe command and
writes it through the Driver. No background threads, no clock of its own beyond time.monotonic().

Phases: off | moving (point to point) | following (live target) | hold | stale (input expired
or may_send false) | degraded (level 2) | fault (level 3, latched until reset_fault()).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, replace
from typing import Callable

from ..driver.types import DriverError, DriverState, Pose, finite
from .supervisor import AXES, Fault, Finding, ProtectionConfig, Supervisor
from .trajectory import Trajectory, TrajectoryConfig, TrajectoryError, delta, wrap

ZERO = Pose(0.0, 0.0)       # the stored zero pose that enable() returns to
ZERO_CHECK_DEG = 0.5        # after set_zero() both axes must report this close to 0
_UNSAFE = (DriverState.ARMED, DriverState.UNKNOWN)


@dataclass(frozen=True)
class Snapshot:
    phase: str
    level: int                      # 0 normal, 1 disturbance, 2 degraded, 3 fault
    reason: str
    measured: Pose | None
    command: Pose | None
    target: Pose | None
    feedback_age_s: float | None


class NeckError(RuntimeError):
    def __init__(self, reason: str, snapshot: Snapshot | None = None, cleanup_error: str = ""):
        self.reason = reason
        self.snapshot = snapshot
        self.cleanup_error = cleanup_error
        super().__init__(reason + (f"; disable failed: {cleanup_error}" if cleanup_error else ""))


class Neck:
    def __init__(self, driver, trajectory: TrajectoryConfig | None = None,
                 protection: ProtectionConfig | None = None):
        self.driver = driver
        self.config = trajectory or TrajectoryConfig()
        self.protection = protection or ProtectionConfig()
        self._trajectory = Trajectory(self.config)
        self._sup = Supervisor(self.protection)
        self.phase = "off"
        self._reason = ""
        self._measured = self._command = self._goal = self._requested = None
        self._feedback_at = self._last_tick = self._input_at = None
        self._move_deadline = self._settle_since = None
        self._synced = False                # trajectory starts from the current command
        self._resume = ("hold", None)       # phase and remaining move time after level 2

    # ---- state -----------------------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self.phase not in ("off", "fault")

    @property
    def snapshot(self) -> Snapshot:
        age = None if self._feedback_at is None else max(0., time.monotonic() - self._feedback_at)
        level = 3 if self.phase == "fault" else 2 if self.phase == "degraded" else self._sup.level
        return Snapshot(self.phase, level, self._reason, self._measured, self._command,
                        self._requested, age)

    def _require_enabled(self) -> None:
        if not self.enabled:
            raise NeckError("neck is not enabled", self.snapshot)

    # ---- commands --------------------------------------------------------------------------

    def enable(self) -> Snapshot:
        """Torque on at the measured pose, then glide to the stored zero pose."""
        if self.phase == "fault":
            raise NeckError("reset the fault explicitly before enabling", self.snapshot)
        if self.enabled:
            return self.snapshot
        try:
            pose = self._read()
            self.driver.enable(pose.yaw, pose.pitch)
            self._command = pose
            self._read()
        except (DriverError, ValueError) as exc:
            self._fault(f"enable_failed: {exc}")
        self.phase, self._reason = "hold", ""
        self._last_tick = time.monotonic()
        self._goal = self._requested = None
        self._synced = False
        self._sup.reset()
        self._resume = ("hold", None)
        self.move_to(ZERO.yaw, ZERO.pitch)
        return self.snapshot

    def set_target(self, yaw: float, pitch: float, at: float | None = None) -> None:
        """A newly received live target. `at` is when it arrived (time.monotonic()); a target
        that is already stale is refused and stops following. Latest target wins."""
        self._require_enabled()
        now = time.monotonic()
        stamp = now if at is None else finite(at, "at")
        if stamp > now or (self._input_at is not None and stamp < self._input_at):
            raise ValueError("`at` must be monotonic and must not be in the future")
        target = self._clip(yaw, pitch)
        if now - stamp >= self.config.stale_after_s:
            self._stop("input_stale", "stale")
            return
        if self._goal is None:
            self._last_tick = now
        self._goal = self._requested = target
        self._input_at = stamp
        if self.phase == "degraded":            # remembered; applied when level 2 ends
            self._resume = ("following", None)
            return
        self.phase, self._reason = "following", ""
        self._move_deadline = self._settle_since = None

    def move_to(self, yaw: float, pitch: float, timeout: float | None = None) -> None:
        """Point to point; does not expire like a live target."""
        self._require_enabled()
        duration = self.protection.move_timeout_s if timeout is None else finite(timeout, "timeout")
        if duration <= 0:
            raise ValueError("timeout must be positive")
        if self._goal is None:
            self._last_tick = time.monotonic()
        self._goal = self._requested = self._clip(yaw, pitch)
        if self.phase == "degraded":
            self._resume = ("moving", duration)
            return
        self.phase, self._reason = "moving", ""
        self._move_deadline = time.monotonic() + duration
        self._settle_since = None

    def hold(self) -> Snapshot:
        """Stop following; keep reading and watching."""
        self._require_enabled()
        return self._stop("operator_hold", "hold")

    def set_zero(self) -> Snapshot:
        """Make the pose the neck holds now the stored zero (0, 0) and save it to flash.

        Only while off (torque off, posed by hand) or holding still; torque is not interrupted.
        If the motors do not report ~0 afterwards (or the zero frames fail) while enabled, the
        neck faults and disables; a failed save alone leaves the zero set but unsaved."""
        if self.phase not in ("off", "hold"):
            raise NeckError("zeroing needs the neck off or holding still", self.snapshot)
        holding = self.phase == "hold"

        def fail(reason: str):
            if holding:
                self._fault(reason)
            raise NeckError(reason, self.snapshot)

        try:
            pose = self._read()
        except DriverError as exc:
            raise NeckError(f"cannot read the pose: {exc}", self.snapshot) from exc
        if holding:
            off = delta(pose, self._command)
            if max(abs(off.yaw), abs(off.pitch)) > self.protection.settle_tolerance_deg:
                raise NeckError(f"not settled: measured={pose}, command={self._command}", self.snapshot)
        try:
            self.driver.set_zero()
        except DriverError as exc:
            fail(f"zero_failed: {exc}")
        try:
            after = self._read()
        except DriverError as exc:
            fail(f"zero_verify_failed: {exc}")
        if max(abs(after.yaw), abs(after.pitch)) > ZERO_CHECK_DEG:
            fail(f"zero_verify_failed: measured={after}")
        if holding:
            self._command = ZERO
            self._synced = False
            self._sup.forget()
            self._last_tick = time.monotonic()
        try:
            self.driver.save()
        except DriverError as exc:
            self._reason = "zero_not_saved"
            raise NeckError(f"zero is set but not saved: {exc}", self.snapshot) from exc
        self._reason = "zero_saved"
        return self.snapshot

    def disable(self) -> None:
        if self.driver.state in _UNSAFE:
            try:
                self.driver.disable()
            except DriverError as exc:
                self.phase, self._reason = "fault", str(exc)
                raise NeckError(str(exc), self.snapshot) from exc
        self._goal = None
        self._synced = False
        self._sup.leave_degraded()
        self.phase, self._reason = "off", ""

    def reset_fault(self) -> None:
        if self.driver.state in _UNSAFE:
            raise NeckError("disable must succeed before the fault can be reset", self.snapshot)
        self.phase, self._reason = "off", ""

    def close(self) -> None:
        """Disable if needed and release the driver."""
        try:
            self.driver.close()
        except DriverError as exc:
            self.phase, self._reason = "fault", str(exc)
            raise NeckError(str(exc), self.snapshot) from exc
        else:
            if self.phase != "fault":
                self.phase = "off"
        finally:
            self._goal = None
            self._synced = False

    def __enter__(self) -> Neck:
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    # ---- the control tick ------------------------------------------------------------------

    def tick(self, may_send: Callable[[], bool] | None = None) -> Snapshot:
        """Read feedback, supervise, advance the trajectory, write changed axes."""
        self._require_enabled()
        try:
            return self._tick(may_send)
        except Fault as fault:
            self._fault(str(fault))

    def _tick(self, may_send) -> Snapshot:
        sup, started = self._sup, time.monotonic()
        sup.level = 0
        if self._goal is not None and started - self._last_tick > self.config.max_gap_s + 1e-9:
            finding = sup.gap(started - self._last_tick)
            self._synced = False        # the motors kept the last command; resume from it
            self._apply(started, finding)
        self._last_tick = started
        try:
            self._read()
        except DriverError as exc:
            age = math.inf if self._feedback_at is None else time.monotonic() - self._feedback_at
            self._synced = False
            self._apply(time.monotonic(), sup.feedback_failed(age, exc))
            return self.snapshot
        now = time.monotonic()
        late = sup.feedback_late(now - self._feedback_at)
        if late:
            self._synced = False
            self._apply(now, late)
            return self.snapshot
        if self.phase == "degraded":
            return self._tick_degraded(now, may_send)
        finding = sup.watch(now, self._measured, self._command, self._goal)
        if finding:
            self._apply(now, finding)
            if finding.level == 2:
                return self.snapshot
        if self._goal is None:
            return self.snapshot
        if may_send is not None and not may_send():
            return self._stop("permission_lost", "stale")
        if self.phase == "following" and now - self._input_at >= self.config.stale_after_s:
            return self._stop("input_stale", "stale")
        if self.phase == "moving":
            reached = self._check_move(now)
            if reached:
                return reached
        if not self._synced:
            self._trajectory.reset(self._command, now)
            self._synced = True
        try:
            self._trajectory.set_target(self._goal, now)
            candidate = self._trajectory.step(now)
        except (ValueError, TrajectoryError) as exc:
            self._fault(f"trajectory_error: {exc}")
        send, limited = {}, False
        for axis in AXES:
            current, value = getattr(self._command, axis), getattr(candidate, axis)
            step = value - current
            error = current - getattr(self._measured, axis)
            if axis == "yaw":
                step, error = wrap(step), wrap(error)
            if error * step > 0 and abs(error + step) > self.protection.reference_lead_deg:
                limited = True          # the reference must not run away from the motors
            elif round(value * 10) != round(current * 10):
                send[axis] = value
        if send:
            if may_send is not None and not may_send():
                return self._stop("permission_lost", "stale")
            self._write(send)
            sup.note_send(time.monotonic())
        if limited:
            self._trajectory.reset(self._command, time.monotonic())
            self._reason = "waiting_for_feedback"
        elif sup.level != 1:
            self._reason = ""
        return self.snapshot

    def _check_move(self, now: float) -> Snapshot | None:
        c = self.protection
        error = delta(self._measured, self._goal)
        lag = delta(self._command, self._goal)
        if (max(abs(error.yaw), abs(error.pitch)) <= c.settle_tolerance_deg
                and max(abs(lag.yaw), abs(lag.pitch)) <= .1 + 1e-9):
            self._settle_since = now if self._settle_since is None else self._settle_since
            if now - self._settle_since >= c.settle_time_s:
                return self._stop("target_reached", "hold")
        else:
            self._settle_since = None
        if now >= self._move_deadline:
            self._fault(f"move_timeout: target={self._goal}, measured={self._measured}")
        return None

    # ---- level 2 ---------------------------------------------------------------------------

    def _apply(self, now: float, finding: Finding) -> None:
        if finding.level == 1:
            self._reason = finding.reason
            self._sup.apply(now, finding)
        elif self._sup.apply(now, finding):
            moving = self.phase == "moving" and self._move_deadline is not None
            resume = self.phase if self.phase in ("moving", "following") and self._goal is not None else "hold"
            self._resume = (resume, max(0., self._move_deadline - now) if moving else None)
            self._synced = False
            self._move_deadline = self._settle_since = None
            self.phase, self._reason = "degraded", "degraded: " + finding.reason
        else:
            self._reason = "degraded: " + finding.reason

    def _tick_degraded(self, now: float, may_send) -> Snapshot:
        sup = self._sup
        sup.check_degraded(now)
        if may_send is not None and not may_send():
            return self._stop("permission_lost", "stale")
        if sup.needs_relief:
            # Once per episode: pull the reference back to the measured pose so the motors
            # stop pushing against a stall or an outside disturbance.
            send = {axis: getattr(self._measured, axis) for axis in AXES
                    if round(getattr(self._measured, axis) * 10) != round(getattr(self._command, axis) * 10)}
            if send:
                self._write(send)
            sup.relieved()
        if sup.stable(now, delta(self._measured, self._command)):
            return self._recover(now)
        return self.snapshot

    def _recover(self, now: float) -> Snapshot:
        phase, remaining = self._resume if self._goal is not None else ("hold", None)
        self._sup.leave_degraded()
        self._sup.forget()
        self.phase, self._reason = phase, "recovered"
        if phase == "moving":
            self._move_deadline = now + (remaining or self.protection.move_timeout_s)
        self._synced = False
        self._settle_since = None
        self._last_tick = now
        return self.snapshot

    # ---- internals -------------------------------------------------------------------------

    def _clip(self, yaw: float, pitch: float) -> Pose:
        yaw, pitch = finite(yaw, "yaw target"), finite(pitch, "pitch target")
        if self.config.yaw_limits is not None:
            yaw = max(self.config.yaw_limits[0], min(self.config.yaw_limits[1], yaw))
        else:
            yaw = wrap(yaw)
        lower, upper = self.config.pitch_limits
        return Pose(yaw, max(lower, min(upper, pitch)))

    def _read(self) -> Pose:
        started = time.monotonic()
        pose = self.driver.read(self.protection.feedback_timeout_s)
        # Both axes were read after this (conservative) host timestamp.
        self._measured, self._feedback_at = pose, started
        return pose

    def _write(self, axes: dict) -> None:
        try:
            self.driver.set(axes.get("yaw"), axes.get("pitch"))
        except DriverError as exc:
            self._fault(f"target_write_failed: {exc}")
        self._command = replace(self._command, **axes)

    def _stop(self, reason: str, phase: str) -> Snapshot:
        self._goal = None
        self._synced = False
        self._sup.leave_degraded()
        self._input_at = self._move_deadline = self._settle_since = None
        self.phase, self._reason = phase, reason
        return self.snapshot

    def _fault(self, reason: str):
        self.phase, self._reason = "fault", reason
        self._goal = None
        self._synced = False
        self._sup.leave_degraded()
        cleanup = ""
        if self.driver.state in _UNSAFE:
            try:
                self.driver.disable()
            except DriverError as exc:
                cleanup = str(exc)
        raise NeckError(reason, self.snapshot, cleanup)
