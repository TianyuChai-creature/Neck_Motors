"""Synchronous head control shared by the UI and integrated applications.

One caller thread owns this object and calls tick regularly. No background
threads, automatic enable, zero calibration, or application authorization live
here. HeadTracker is the sole filter, trajectory and output hysteresis engine.

Graded protection (ControlSnapshot.level): 0 normal; 1 disturbance (transient:
pause this tick, keep the last command, resynchronise, nothing is disabled);
2 degraded (phase "degraded": stay enabled, stop advancing, relax the command to
the measured pose once, resume automatically after a stable recovery); 3 fault
(stop and request disable on both axes, latched until reset_fault()).
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math
import time
from typing import Callable

from . import Config, DEFAULT_HOLD_POSITION, F32CError, Gimbal2Dof, State
from .tracking import HeadTracker, TrackingConfig, TrackingError, _finite, _wrap


@dataclass(frozen=True)
class ProtectionConfig:
    feedback_timeout_s: float = 0.08
    feedback_max_age_s: float = 0.1   # level 2 (degraded) when feedback is older
    feedback_lost_s: float = 0.5      # level 3 (fault) when feedback is older
    control_gap_fault_s: float = 1.0  # level 3 for a tick gap this long; shorter gaps are level 1
    reference_lead_deg: float = 18.0
    max_error_deg: float = 12.0  # Compatibility only; error magnitude no longer disables.
    stationary_error_deg: float = 3.0
    stall_after_s: float = 1.5
    stall_motion_deg: float = 0.3
    hold_swing_deg: float = 1.0
    settle_tolerance_deg: float = 1.0
    settle_time_s: float = 0.5
    move_timeout_s: float = 15.0
    degraded_timeout_s: float = 5.0   # level 3 when level 2 lasts this long
    recover_time_s: float = 0.3       # error must stay small this long to leave level 2
    escalation_window_s: float = 30.0
    escalation_events: int = 6        # level 1/2 events inside the window that escalate to level 3

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if _finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.feedback_timeout_s <= self.feedback_max_age_s <= 1:
            raise ValueError("feedback timeouts must satisfy 0 < timeout <= max_age <= 1")
        if not self.feedback_max_age_s < self.feedback_lost_s:
            raise ValueError("expected feedback_max_age < feedback_lost")
        if not self.stationary_error_deg < self.reference_lead_deg:
            raise ValueError("expected stationary_error < reference_lead")


@dataclass(frozen=True)
class ControlSnapshot:
    phase: str
    enabled: bool
    measured: State | None
    command: State | None
    target: State | None
    feedback_age_s: float | None
    reason: str = ""
    following_limited: tuple[str, ...] = ()
    level: int = 0  # 0 normal, 1 disturbance, 2 degraded, 3 fault


class HeadControlError(F32CError):
    def __init__(self, reason: str, snapshot: ControlSnapshot, cleanup_error: str = ""):
        self.reason = reason
        self.snapshot = snapshot
        self.cleanup_error = cleanup_error
        super().__init__(reason + (f"; disable failed: {cleanup_error}" if cleanup_error else ""))


def _signed(state: State) -> State:
    return State(_wrap(_finite(state.yaw_deg, "measured yaw")),
                 _wrap(_finite(state.pitch_deg, "measured pitch")))


def _error(a: State, b: State) -> State:
    return State(_wrap(a.yaw_deg - b.yaw_deg), a.pitch_deg - b.pitch_deg)


def clip_pitch_target(deg: float) -> float:
    """Clip a new desired pitch target; never use this to rewrite feedback."""
    return min(45., _finite(deg, "pitch target"))


class HeadController:
    """Own motor I/O, reference generation, feedback guards and cleanup.

    New pitch targets saturate at +45 degrees (or a smaller configured target
    limit). Feedback has no head-down boundary. An initial pose outside the
    target range is retained as measured, and is a valid starting point for an
    inward trajectory; its coordinates and persistent zero are never changed.
    """

    def __init__(self, config: Config | None = None,
                 tracking: TrackingConfig | None = None,
                 protection: ProtectionConfig | None = None, *,
                 motor: Gimbal2Dof | None = None):
        self.tracking = tracking or TrackingConfig(
            yaw_limits=(-60, 60), pitch_limits=(-180, 45),
            max_speed_deg_s=90, max_accel_deg_s2=720,
            response_hz=15, min_cutoff_hz=6)
        if self.tracking.pitch_limits[0] > 45:
            raise ValueError("pitch target range must include an angle at or below 45 degrees")
        self.protection = protection or ProtectionConfig()
        self.motor = motor if motor is not None else Gimbal2Dof(config)
        self.phase = "off"
        self.enabled = False
        self._engaged = False
        self._opened = False
        self._cleanup_error = ""
        self._reason = ""
        self._measured = self._command = self._goal = self._requested = None
        self._feedback_at = self._last_tick = self._input_at = None
        self._move_deadline = self._settle_since = None
        self._tracker = None
        self._limited = set()
        self._motion = {name: deque() for name in ("yaw", "pitch")}
        self._sent_at = {}
        self._progress = deque()
        self._hold_samples = deque()
        self._hold_target = None
        self._hold_since = 0.0
        self._level = 0
        self._events = deque()
        self._clear_degraded()

    def _clear_degraded(self):
        self._degraded_since = self._recover_since = None
        self._degraded_reason = ""
        self._resume_phase = "hold"
        self._resume_move_s = None
        self._relieved = False

    @property
    def snapshot(self) -> ControlSnapshot:
        age = None if self._feedback_at is None else max(0., time.monotonic() - self._feedback_at)
        level = 3 if self.phase == "fault" else 2 if self.phase == "degraded" else self._level
        return ControlSnapshot(self.phase, self.enabled, self._measured, self._command,
                               self._requested, age, self._reason, tuple(sorted(self._limited)),
                               level)

    def open(self) -> HeadController:
        if not self._opened:
            self.motor.open()
            self._opened = True
        return self

    def _read(self) -> State:
        started = time.monotonic()
        state = _signed(self.motor.read(timeout_s=self.protection.feedback_timeout_s))
        # Both axes were acquired after this conservative host timestamp.
        self._measured, self._feedback_at = state, started
        return state

    def read(self) -> State:
        """Read a signed passive pose; enabled callers should use tick()."""
        return self._read()

    def start_log(self, path):
        self.motor.start_log(path)

    def stop_log(self):
        self.motor.stop_log()

    def enable(self) -> ControlSnapshot:
        """Enable at the measured pose, then return to the stored zero (DEFAULT_HOLD_POSITION) via tick()."""
        if self.phase == "fault":
            raise F32CError("reset the controller fault explicitly before enabling")
        if self.enabled:
            return self.snapshot
        if not self._opened:
            raise F32CError("controller is not open")
        try:
            initial = self._read()
            self._engaged = True
            self.motor.enable_at(initial.yaw_deg, initial.pitch_deg)
            self._command = initial
            self._read()
        except (F32CError, ValueError) as exc:
            self._fault(f"enable_failed: {exc}")
        self.enabled = True
        self.phase, self._reason = "hold", ""
        self._last_tick = time.monotonic()
        self._sent_at = {name: self._last_tick for name in self._motion}
        self._tracker = None
        self._goal = self._requested = None
        self._progress.clear()
        self._hold_samples.clear()
        self._hold_target = None
        self._limited.clear()
        self._level = 0
        self._events.clear()
        self._clear_degraded()
        for samples in self._motion.values():
            samples.clear()
        self.move_to(DEFAULT_HOLD_POSITION.yaw_deg, DEFAULT_HOLD_POSITION.pitch_deg)
        return self.snapshot

    def _require_enabled(self):
        if not self.enabled or self.phase == "fault":
            raise F32CError("head controller is not enabled")

    def _target(self, yaw_deg: float, pitch_deg: float) -> State:
        yaw = _finite(yaw_deg, "yaw target")
        pitch = _finite(pitch_deg, "pitch target")
        if self.tracking.yaw_limits is not None:
            yaw = max(self.tracking.yaw_limits[0], min(self.tracking.yaw_limits[1], yaw))
        else:
            yaw = _wrap(yaw)
        lower, upper = self.tracking.pitch_limits
        return State(yaw, max(lower, min(upper, clip_pitch_target(pitch))))

    def set_target(self, yaw_deg: float, pitch_deg: float, *, received_at: float | None = None):
        """Submit only a newly received source sample, in signed motor coordinates."""
        self._require_enabled()
        now = time.monotonic()
        stamp = now if received_at is None else _finite(received_at, "received_at")
        if stamp > now or (self._input_at is not None and stamp < self._input_at):
            raise ValueError("source time must be monotonic and must not be in the future")
        target = self._target(yaw_deg, pitch_deg)
        # An already expired sample must never refresh a previously valid goal.
        if now - stamp >= self.tracking.stale_after_s:
            self.hold("input_stale")
            return
        if self._goal is None:
            self._last_tick = now
        self._goal = self._requested = target
        self._input_at = stamp
        if self.phase == "degraded":  # remembered; applied when level 2 ends
            self._resume_phase, self._resume_move_s = "following", None
            return
        self.phase, self._reason = "following", ""
        self._move_deadline = self._settle_since = None

    def move_to(self, yaw_deg: float, pitch_deg: float, *, timeout_s: float | None = None):
        """Explicit local point-to-point action, independent of stream freshness."""
        self._require_enabled()
        duration = self.protection.move_timeout_s if timeout_s is None else _finite(timeout_s, "timeout_s")
        if duration <= 0:
            raise ValueError("timeout_s must be positive")
        if self._goal is None:
            self._last_tick = time.monotonic()
        self._goal = self._requested = self._target(yaw_deg, pitch_deg)
        if self.phase == "degraded":  # remembered; applied when level 2 ends
            self._resume_phase, self._resume_move_s = "moving", duration
            return
        self.phase, self._reason = "moving", ""
        self._move_deadline = time.monotonic() + duration
        self._settle_since = None

    def set_motion_limits(self, speed_deg_s: float, accel_deg_s2: float):
        self.tracking = replace(self.tracking, max_speed_deg_s=speed_deg_s,
                                max_accel_deg_s2=accel_deg_s2)
        self._tracker = None

    def hold(self, reason: str = "operator_hold") -> ControlSnapshot:
        """Stop new reference output; keep feedback and recent-motion guards alive."""
        self._require_enabled()
        self._goal = None
        self._tracker = None
        self._clear_degraded()
        self._input_at = self._move_deadline = self._settle_since = None
        self.phase = "source_hold" if reason in ("permission_lost", "input_stale", "source_changed") else "hold"
        self._reason = reason
        return self.snapshot

    def _new_tracker(self, initial: State, now: float):
        # Initial feedback is accepted even outside target bounds. Only the new
        # desired target is clipped; do not fabricate a 45-degree measurement.
        yaw_limits = self.tracking.yaw_limits
        if yaw_limits is not None:
            yaw_limits = (min(yaw_limits[0], initial.yaw_deg), max(yaw_limits[1], initial.yaw_deg))
        lo, hi = self.tracking.pitch_limits
        config = replace(self.tracking, yaw_limits=yaw_limits,
                         pitch_limits=(min(lo, initial.pitch_deg), max(hi, initial.pitch_deg)))
        self._tracker = HeadTracker(initial, config, timestamp=now)

    def _guard(self, now: float):
        """Return None, ("disturb", reason) for level 1 or ("degrade", reason) for level 2."""
        errors = _error(self._measured, self._command)
        for axis in tuple(self._limited):
            if abs(getattr(errors, axis + "_deg")) < self.protection.stationary_error_deg:
                self._limited.remove(axis)
        for samples in self._motion.values():
            while samples and samples[0][0] < now - .5:
                samples.popleft()
        self._progress.append((now, self._measured, errors))
        while len(self._progress) > 1 and self._progress[1][0] <= now - self.protection.stall_after_s:
            self._progress.popleft()
        if now - self._progress[0][0] >= self.protection.stall_after_s:
            for axis in self._motion:
                values = [getattr(row[1], axis + "_deg") for row in self._progress]
                origin = values[0]
                if axis == "yaw":
                    values = [_wrap(v - origin) for v in values]
                if (all(abs(getattr(row[2], axis + "_deg")) > 6 for row in self._progress) and
                        max(values) - min(values) <= self.protection.stall_motion_deg + 1e-9):
                    return ("degrade", f"no_progress {axis}: measured={getattr(self._measured, axis + '_deg'):.3f}, "
                                       f"command={getattr(self._command, axis + '_deg'):.3f}")
        if self._command != self._hold_target:
            self._hold_target, self._hold_since = self._command, now
            self._hold_samples.clear()
        near_goal = self._goal is None or max(map(abs, vars(_error(self._command, self._goal)).values())) <= .1
        if not near_goal or any(self._motion.values()):
            self._hold_samples.clear()
            return None
        if not self._hold_samples and max(abs(errors.yaw_deg), abs(errors.pitch_deg)) > .5 and now - self._hold_since < 1:
            return None
        self._hold_samples.append((now, errors))
        while self._hold_samples and self._hold_samples[0][0] < now - 1:
            self._hold_samples.popleft()
        if len(self._hold_samples) >= 3:
            for axis in self._motion:
                values = [getattr(row[1], axis + "_deg") for row in self._hold_samples]
                if max(values) - min(values) > self.protection.hold_swing_deg + 1e-9:
                    self._hold_samples.clear()
                    return ("disturb", f"hold_oscillation {axis}")
        return None

    def _note_event(self, now: float, reason: str):
        """Count a level 1/2 event; too many in the window escalate to level 3."""
        self._events.append(now)
        while self._events and self._events[0] < now - self.protection.escalation_window_s:
            self._events.popleft()
        if len(self._events) >= self.protection.escalation_events:
            self._fault(f"repeated_disturbance: {len(self._events)} events in "
                        f"{self.protection.escalation_window_s:g}s, last={reason}")

    def _disturb(self, now: float, reason: str, *, count: bool = True):
        """Level 1: no disable, no phase change; the caller pauses output this tick."""
        self._level = 1
        self._reason = reason
        if count:
            self._note_event(now, reason)

    def _degrade(self, now: float, reason: str):
        """Level 2: stay enabled, stop advancing, wait for a stable recovery."""
        if self.phase == "degraded":
            self._degraded_reason = reason
            self._reason = "degraded: " + reason
            return
        self._note_event(now, reason)
        self._resume_phase = self.phase if self.phase in ("moving", "following") and self._goal is not None else "hold"
        self._resume_move_s = None
        if self.phase == "moving" and self._move_deadline is not None:
            self._resume_move_s = max(0., self._move_deadline - now)
        self._degraded_since, self._recover_since, self._relieved = now, None, False
        self._degraded_reason = reason
        self._tracker = None
        self._move_deadline = self._settle_since = None
        self._progress.clear()
        self._hold_samples.clear()
        self.phase, self._reason = "degraded", "degraded: " + reason

    def _tick_degraded(self, now: float, may_send: Callable[[], bool] | None) -> ControlSnapshot:
        if now - self._degraded_since >= self.protection.degraded_timeout_s:
            self._fault(f"degraded_timeout: {self._degraded_reason}")
        if may_send is not None and not may_send():
            return self.hold("permission_lost")
        if not self._relieved:
            # Once per episode: pull the reference back to the measured pose so the
            # motors stop pushing against a stall or an external disturbance.
            for axis, setter in (("yaw", self.motor.set_yaw_deg), ("pitch", self.motor.set_pitch_deg)):
                value = getattr(self._measured, axis + "_deg")
                if round(value * 10) != round(getattr(self._command, axis + "_deg") * 10):
                    try:
                        setter(value)
                    except F32CError as exc:
                        self._fault(f"target_write_failed {axis}: {exc}")
                    self._sent_at[axis] = time.monotonic()
                    self._command = replace(self._command, **{axis + "_deg": value})
            self._relieved = True
        errors = _error(self._measured, self._command)
        if max(abs(errors.yaw_deg), abs(errors.pitch_deg)) < self.protection.stationary_error_deg:
            self._recover_since = now if self._recover_since is None else self._recover_since
            if now - self._recover_since >= self.protection.recover_time_s:
                return self._recover(now)
        else:
            self._recover_since = None
        return self.snapshot

    def _recover(self, now: float) -> ControlSnapshot:
        phase = self._resume_phase if self._goal is not None else "hold"
        remaining = self._resume_move_s
        self._clear_degraded()
        self.phase, self._reason = phase, "recovered"
        if phase == "moving":
            self._move_deadline = now + (remaining if remaining else self.protection.move_timeout_s)
        self._tracker = None
        self._settle_since = None
        self._limited.clear()
        self._progress.clear()
        self._hold_samples.clear()
        for samples in self._motion.values():
            samples.clear()
        self._last_tick = now
        return self.snapshot

    def tick(self, *, may_send: Callable[[], bool] | None = None) -> ControlSnapshot:
        """Read feedback every tick, then send changed axes under current permission."""
        self._require_enabled()
        started = time.monotonic()
        self._level = 0
        if self._goal is not None and started - self._last_tick > self.tracking.max_gap_s + 1e-9:
            gap = started - self._last_tick
            if gap >= self.protection.control_gap_fault_s:
                self._fault(f"control_gap: {gap:.2f}s without a control tick")
            # Level 1: the motor kept its last command; resume the trajectory from it.
            self._tracker = None
            self._disturb(started, f"control_gap: {gap:.2f}s")
        self._last_tick = started
        try:
            self._read()
        except ValueError as exc:
            self._fault(f"invalid_feedback: {exc}")
        except F32CError as exc:
            age = float("inf") if self._feedback_at is None else time.monotonic() - self._feedback_at
            if age >= self.protection.feedback_lost_s:
                self._fault(f"feedback_stale: {exc}")
            self._tracker = None
            if age >= self.protection.feedback_max_age_s:
                if self.phase != "degraded":
                    self._degrade(time.monotonic(), f"feedback_degraded: {exc}")
            else:
                self._disturb(started, f"feedback_wait: {exc}", count=False)
            return self.snapshot
        now = time.monotonic()
        age = now - self._feedback_at
        if age >= self.protection.feedback_lost_s:
            self._fault("feedback_stale: acquisition exceeded feedback lost budget")
        if age >= self.protection.feedback_max_age_s:
            self._tracker = None
            self._disturb(now, "feedback_slow: acquisition exceeded feedback age budget", count=False)
            return self.snapshot
        finding = self._guard(now)
        if self.phase == "degraded":
            return self._tick_degraded(now, may_send)
        if finding is not None:
            kind, reason = finding
            if kind == "degrade":
                self._degrade(now, reason)
                return self.snapshot
            self._disturb(now, reason)
        if self._goal is None:
            return self.snapshot
        if may_send is not None and not may_send():
            return self.hold("permission_lost")
        if self.phase == "following" and now - self._input_at >= self.tracking.stale_after_s:
            return self.hold("input_stale")
        if self.phase == "moving":
            error = _error(self._measured, self._goal)
            reference_error = _error(self._command, self._goal)
            if (max(abs(error.yaw_deg), abs(error.pitch_deg)) <= self.protection.settle_tolerance_deg and
                    max(abs(reference_error.yaw_deg), abs(reference_error.pitch_deg)) <= .1 + 1e-9):
                self._settle_since = now if self._settle_since is None else self._settle_since
                if now - self._settle_since >= self.protection.settle_time_s:
                    return self.hold("target_reached")
            else:
                self._settle_since = None
            if now >= self._move_deadline:
                self._fault(f"move_timeout: target={self._goal}, measured={self._measured}")
        if self._tracker is None:
            self._new_tracker(self._command if self._command is not None else self._measured, now)
        try:
            self._tracker.set_target(self._goal.yaw_deg, self._goal.pitch_deg, timestamp=now)
            candidate = self._tracker.step(timestamp=now)
        except (ValueError, TrackingError) as exc:
            self._fault(f"trajectory_error: {exc}")
        limited = False
        for axis, setter in (("yaw", self.motor.set_yaw_deg), ("pitch", self.motor.set_pitch_deg)):
            current = getattr(self._command, axis + "_deg")
            value = getattr(candidate, axis + "_deg")
            delta = value - current
            error = current - getattr(self._measured, axis + "_deg")
            if axis == "yaw":
                delta, error = _wrap(delta), _wrap(error)
            if error * delta > 0 and abs(error + delta) > self.protection.reference_lead_deg:
                self._limited.add(axis)
                limited = True
                continue
            if round(value * 10) == round(current * 10):
                continue
            # Permission may change during feedback I/O or the other axis write.
            if may_send is not None and not may_send():
                return self.hold("permission_lost")
            try:
                setter(value)
            except F32CError as exc:
                self._fault(f"target_write_failed {axis}: {exc}")
            sent_at = time.monotonic()
            self._motion[axis].append((sent_at, abs(delta) / max(sent_at - self._sent_at[axis], 1e-6)))
            self._sent_at[axis] = sent_at
            self._command = replace(self._command, **{axis + "_deg": value})
        if limited:
            self._new_tracker(self._command, time.monotonic())
        if limited:
            self._reason = "waiting_for_feedback"
        elif self._level != 1:
            self._reason = ""
        return self.snapshot

    def _fault(self, reason: str):
        self.phase, self._reason = "fault", reason
        self._goal = None
        self._tracker = None
        self._clear_degraded()
        if self._engaged:
            try:
                self.motor.disable()
                self._engaged = False
            except F32CError as exc:
                self._cleanup_error = str(exc)
        self.enabled = False
        raise HeadControlError(reason, self.snapshot, self._cleanup_error)

    def disable(self):
        if self._engaged:
            try:
                self.motor.disable()
            except F32CError as exc:
                self.enabled = False
                self.phase, self._cleanup_error = "fault", str(exc)
                raise
            self._engaged = False
            self._cleanup_error = ""
        self.enabled = False
        self._goal = self._tracker = None
        self._clear_degraded()
        self.phase, self._reason = "off", ""

    def reset_fault(self):
        if self._engaged or self._cleanup_error:
            raise F32CError("resolve failed disable/unknown hardware state before resetting")
        self.phase, self._reason = "off", ""

    def close(self):
        if not self._opened:
            if self._cleanup_error:
                raise F32CError(self._cleanup_error)
            return
        try:
            self.motor.close()
        except F32CError as exc:
            self.phase, self._cleanup_error = "fault", str(exc)
            raise
        else:
            self._engaged = False
            self._cleanup_error = ""
            if self.phase != "fault":
                self.phase = "off"
        finally:
            self._opened = self.enabled = False
            self._goal = self._tracker = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *_exc):
        self.close()
