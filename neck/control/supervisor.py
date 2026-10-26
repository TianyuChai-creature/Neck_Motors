"""Graded protection: judges what is going wrong. Pure logic, no I/O, no clock.

Levels: 0 normal | 1 disturbance (pause this tick, nothing disabled) |
2 degraded (stay enabled, stop advancing, relax the command to the measured pose, wait for
a stable recovery) | 3 fault (raised as Fault; the Neck disables and latches).

The Neck passes in what it sees; the Supervisor answers with a Finding or raises Fault.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from ..driver.types import Pose, finite
from .trajectory import delta, wrap

AXES = ("yaw", "pitch")


@dataclass(frozen=True)
class ProtectionConfig:
    feedback_timeout_s: float = 0.08   # one read() of both motors
    feedback_max_age_s: float = 0.1    # level 2 when feedback is older than this
    feedback_lost_s: float = 0.5       # level 3 when feedback is older than this
    control_gap_fault_s: float = 1.0   # level 3 for a tick gap this long; shorter gaps are level 1
    reference_lead_deg: float = 18.0   # the command may lead the measured pose by at most this
    stationary_error_deg: float = 3.0
    stall_after_s: float = 1.5
    stall_motion_deg: float = 0.3
    hold_swing_deg: float = 1.0
    settle_tolerance_deg: float = 1.0
    settle_time_s: float = 0.5
    move_timeout_s: float = 15.0
    degraded_timeout_s: float = 5.0    # level 3 when level 2 lasts this long
    recover_time_s: float = 0.3        # error must stay small this long to leave level 2
    escalation_window_s: float = 30.0
    escalation_events: int = 6         # level 1/2 events inside the window that make level 3

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            if finite(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not self.feedback_timeout_s <= self.feedback_max_age_s <= 1:
            raise ValueError("feedback timeouts must satisfy 0 < timeout <= max_age <= 1")
        if not self.feedback_max_age_s < self.feedback_lost_s:
            raise ValueError("expected feedback_max_age < feedback_lost")
        if not self.stationary_error_deg < self.reference_lead_deg:
            raise ValueError("expected stationary_error < reference_lead")


class Fault(Exception):
    """Level 3. The message is the reason."""


@dataclass(frozen=True)
class Finding:
    level: int              # 1 or 2
    reason: str
    count: bool = True      # whether it counts towards escalation


class Supervisor:
    def __init__(self, config: ProtectionConfig):
        self.config = config
        self._progress: deque = deque()
        self._hold_samples: deque = deque()
        self.reset()

    def reset(self) -> None:
        self.level = 0
        self._events: deque = deque()
        self.forget()
        self.leave_degraded()

    def forget(self) -> None:
        """Drop motion history (after a pause, resync or a change of plan)."""
        self._progress.clear()
        self._hold_samples.clear()
        self._hold_target = None
        self._hold_since = 0.0
        self._last_send: float | None = None

    def note_send(self, now: float) -> None:
        self._last_send = now

    # ---- what the Neck reports ------------------------------------------------------------

    def gap(self, gap: float) -> Finding:
        """The control loop did not tick for `gap` seconds."""
        if gap >= self.config.control_gap_fault_s:
            raise Fault(f"control_gap: {gap:.2f}s without a control tick")
        return Finding(1, f"control_gap: {gap:.2f}s")

    def feedback_failed(self, age: float, why) -> Finding:
        """A read failed; `age` is how old the last good feedback is."""
        if age >= self.config.feedback_lost_s:
            raise Fault(f"feedback_stale: {why}")
        if age >= self.config.feedback_max_age_s:
            return Finding(2, f"feedback_degraded: {why}")
        return Finding(1, f"feedback_wait: {why}", count=False)

    def feedback_late(self, age: float) -> Finding | None:
        """A read succeeded but took long enough that the sample is already old."""
        if age >= self.config.feedback_lost_s:
            raise Fault("feedback_stale: acquisition exceeded feedback lost budget")
        if age >= self.config.feedback_max_age_s:
            return Finding(1, "feedback_slow: acquisition exceeded feedback age budget", count=False)
        return None

    def watch(self, now: float, measured: Pose, command: Pose, goal: Pose | None) -> Finding | None:
        """Look for a stalled axis (level 2) or a shaking hold (level 1)."""
        c = self.config
        errors = delta(measured, command)
        self._progress.append((now, measured, errors))
        while len(self._progress) > 1 and self._progress[1][0] <= now - c.stall_after_s:
            self._progress.popleft()
        if now - self._progress[0][0] >= c.stall_after_s:
            for axis in AXES:
                values = [getattr(row[1], axis) for row in self._progress]
                if axis == "yaw":
                    values = [wrap(v - values[0]) for v in values]
                if (all(abs(getattr(row[2], axis)) > 6 for row in self._progress)
                        and max(values) - min(values) <= c.stall_motion_deg + 1e-9):
                    return Finding(2, f"no_progress {axis}: measured={getattr(measured, axis):.3f}, "
                                      f"command={getattr(command, axis):.3f}")
        if command != self._hold_target:
            self._hold_target, self._hold_since = command, now
            self._hold_samples.clear()
        to_goal = None if goal is None else delta(command, goal)
        near_goal = to_goal is None or max(abs(to_goal.yaw), abs(to_goal.pitch)) <= .1
        sent_recently = self._last_send is not None and self._last_send >= now - .5
        if not near_goal or sent_recently:
            self._hold_samples.clear()
            return None
        if (not self._hold_samples and max(abs(errors.yaw), abs(errors.pitch)) > .5
                and now - self._hold_since < 1):
            return None
        self._hold_samples.append((now, errors))
        while self._hold_samples and self._hold_samples[0][0] < now - 1:
            self._hold_samples.popleft()
        if len(self._hold_samples) >= 3:
            for axis in AXES:
                values = [getattr(row[1], axis) for row in self._hold_samples]
                if max(values) - min(values) > c.hold_swing_deg + 1e-9:
                    self._hold_samples.clear()
                    return Finding(1, f"hold_oscillation {axis}")
        return None

    def apply(self, now: float, finding: Finding) -> bool:
        """Record a finding; may raise Fault by escalation. True when level 2 just began."""
        if finding.level == 1:
            self.level = 1
            if finding.count:
                self._note(now, finding.reason)
            return False
        if self.degraded:
            self.degraded_reason = finding.reason
            return False
        self._note(now, finding.reason)
        self.degraded = True
        self._since, self._recover_since, self._relieved = now, None, False
        self.degraded_reason = finding.reason
        self._progress.clear()
        self._hold_samples.clear()
        return True

    def _note(self, now: float, reason: str) -> None:
        c = self.config
        self._events.append(now)
        while self._events and self._events[0] < now - c.escalation_window_s:
            self._events.popleft()
        if len(self._events) >= c.escalation_events:
            raise Fault(f"repeated_disturbance: {len(self._events)} events in "
                        f"{c.escalation_window_s:g}s, last={reason}")

    # ---- level 2 ---------------------------------------------------------------------------

    def leave_degraded(self) -> None:
        self.degraded = False
        self.degraded_reason = ""
        self._since = self._recover_since = None
        self._relieved = False

    def check_degraded(self, now: float) -> None:
        if now - self._since >= self.config.degraded_timeout_s:
            raise Fault(f"degraded_timeout: {self.degraded_reason}")

    @property
    def needs_relief(self) -> bool:
        """Once per episode the command must be pulled back to the measured pose."""
        return not self._relieved

    def relieved(self) -> None:
        self._relieved = True

    def stable(self, now: float, errors: Pose) -> bool:
        """True once the error has stayed small for recover_time_s."""
        c = self.config
        if max(abs(errors.yaw), abs(errors.pitch)) < c.stationary_error_deg:
            self._recover_since = now if self._recover_since is None else self._recover_since
            return now - self._recover_since >= c.recover_time_s
        self._recover_since = None
        return False
