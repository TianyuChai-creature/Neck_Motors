"""The simplified neck controller.

One thread owns the serial link.  The loop advances a smooth reference at 100 Hz
and sends position frames; feedback is parsed separately and only gates motion.
There is no phase state machine: the useful states are ``enabled``, ``link_ok``
and ``reason``.
"""

from __future__ import annotations

import math
import threading
import time

from . import protocol as p
from .link import SerialLink
from .types import (
    CommandResult, LinkError, NeckConfig, Pose, Status, ZeroResult, finite,
)

ZERO = Pose(0.0, 0.0)
_SETTLE_DEG = 1.0       # zeroing while enabled needs command and feedback this close

__all__ = ["Neck", "ZERO"]


def wrap(angle: float) -> float:
    """Signed angle in [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class Neck:
    def __init__(self, config: NeckConfig | None = None, link=None):
        self.config = config or NeckConfig()
        self.link = link if link is not None else SerialLink.open(self.config)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._closed = False

        self._enabled = False
        self._torque = "unknown"          # requested, not acknowledged by the motor
        self._command = Pose(0.0, 0.0)
        self._velocity = [0.0, 0.0]
        self._measured: Pose | None = None
        self._latest: Pose | None = None
        self._input_at: float | None = None
        self._last_update: float | None = None
        self._next_query = 0.0
        self._query_yaw = True
        self._lead_since: float | None = None
        self._reason = ""
        self._last_error = ""

    # ---- lifecycle -------------------------------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def start(self) -> None:
        """Start the 100 Hz control thread (idempotent)."""
        with self._lock:
            if self._closed or (self._thread is not None and self._thread.is_alive()):
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="neck-loop", daemon=True)
            self._thread.start()

    def close(self) -> CommandResult:
        """Stop the loop, attempt a disable, release the port."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(1)
        with self._lock:
            result = self._disable_locked("close")
            self.link.close()
            self._closed = True
            self._enabled = False
            self._latest = None
            return result

    def __enter__(self) -> Neck:
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.close()

    def _run(self) -> None:
        next_tick = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                self.update(now)
            except Exception as exc:             # keep the loop alive; surface the reason
                with self._lock:
                    self._last_error = f"control loop: {exc}"
                    self._reason = "internal_error"
                time.sleep(self.config.control_period)
            next_tick += self.config.control_period
            if next_tick <= now:
                next_tick = now + self.config.control_period
            self._stop.wait(max(0.0, next_tick - time.monotonic()))

    # ---- commands --------------------------------------------------------------------------

    def set_target(self, yaw: float, pitch: float, at: float | None = None) -> Pose:
        """Latest target wins. Safe to call from another thread."""
        target = self._clip(Pose(finite(yaw, "yaw"), finite(pitch, "pitch")))
        stamp = time.monotonic() if at is None else finite(at, "at")
        with self._lock:
            self._latest = target
            self._input_at = stamp
        return target

    def enable(self, home: bool = False) -> CommandResult:
        """Enable at the measured pose, then optionally target the stored zero."""
        with self._lock:
            if self._closed:
                return CommandResult(False, "neck is closed", self._torque)
            if self._enabled:
                return CommandResult(True, "already enabled", self._torque)
            self.link.clear_pending()
            try:
                pose = self._read_pose()
                self._send_disable_both()
                self._torque = "off"
                self._send_gains()
                self._start_axis(self.config.yaw_id, pose.yaw)
                self._start_axis(self.config.pitch_id, pose.pitch)
            except LinkError as exc:
                self._last_error = str(exc)
                cleanup = self._disable_locked(f"enable_failed: {exc}")
                return CommandResult(False, f"enable failed: {exc}{cleanup.message}", self._torque)
            now = time.monotonic()
            self._enabled = True
            self._torque = "on"              # enable frames were sent; no ACK exists
            self._command = pose
            self._velocity = [0.0, 0.0]
            self._measured = pose
            self._latest = ZERO if home else pose
            self._input_at = now
            self._last_update = now
            self._lead_since = None
            self._reason = "homing" if home else ""
            self._last_error = ""
            return CommandResult(True, "enabled", self._torque)

    def disable(self) -> CommandResult:
        with self._lock:
            return self._disable_locked("operator")

    def zero(self, save: bool = True, require_stationary: bool = True) -> ZeroResult:
        """Make the current pose (0, 0); optionally save it to flash."""
        with self._lock:
            if self._closed:
                return ZeroResult(False, False, False, False, None, "neck is closed")
            self.link.clear_pending()
            try:
                pose = self._read_pose()
            except LinkError as exc:
                return ZeroResult(False, False, False, False, None, f"cannot read the pose: {exc}")
            if self._enabled and require_stationary:
                off = max(abs(wrap(pose.yaw - self._command.yaw)),
                          abs(pose.pitch - self._command.pitch))
                if off > _SETTLE_DEG:
                    return ZeroResult(False, False, False, False, pose,
                                      f"not settled: measured={pose}, command={self._command}")

            zeroed = False
            try:
                self.link.write_frame(p.set_zero(self.config.yaw_id))
                self.link.write_frame(p.set_zero(self.config.pitch_id))
                zeroed = True
                if self._enabled:
                    self.link.write_frame(p.angle(self.config.yaw_id, 0.0))
                    self.link.write_frame(p.angle(self.config.pitch_id, 0.0))
            except LinkError as exc:
                cleanup = self._disable_locked("zero_failed") if self._enabled else CommandResult(True)
                return ZeroResult(False, zeroed, False, False, pose,
                                  f"zero failed: {exc}{cleanup.message}")

            # The coordinate frame has changed; never keep chasing the old target.
            now = time.monotonic()
            self._command = ZERO
            self._velocity = [0.0, 0.0]
            self._latest = ZERO
            self._input_at = now
            self._last_update = now

            try:
                after = self._read_pose()
            except LinkError as exc:
                self._reason = "zero_unverified"
                return ZeroResult(False, True, False, False, None, f"zero verify failed: {exc}")
            if max(abs(after.yaw), abs(after.pitch)) > self.config.zero_tolerance:
                self._reason = "zero_unverified"
                return ZeroResult(False, True, False, False, after,
                                  f"zero unverified: measured={after}")

            saved = False
            if save:
                try:
                    self.link.write_frame(p.save(self.config.yaw_id))
                    self.link.write_frame(p.save(self.config.pitch_id))
                    saved = True
                except LinkError as exc:
                    self._reason = "zero_set"
                    return ZeroResult(False, True, True, False, after,
                                      f"zero verified but not saved: {exc}")

            self._measured = after
            self._reason = "zero_saved" if saved else "zero_set"
            self._last_error = ""
            return ZeroResult(True, True, True, saved, after, "")

    # ---- one control tick ------------------------------------------------------------------

    def update(self, now: float | None = None) -> Status:
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._closed:
                return self.status
            self.link.poll(now)
            self._measured = self._pose_from_link()
            if not self._enabled:
                self._reason = ""
                self._queue_query(now)
                self._pump(now)
                self._last_update = now
                return self.status

            age = self._feedback_age(now)
            input_age = self._input_age(now)
            if self._latest is None or (input_age is not None
                                        and input_age > self.config.input_timeout):
                self._freeze("input_idle")
            elif not self._link_ok(now, age):
                self._freeze("feedback_stale")
                if (self.config.on_link_loss == "disable" and age is not None
                        and age >= self.config.disable_after):
                    self._disable_locked("link_lost")
                    self._reason = "link_lost"
            elif self._lead_guard(now):
                self._freeze("lead_guard")
            else:
                target = self._clip(self._latest)
                self._advance(target, now)
                self.link.queue_angle(self.config.yaw_id, self._command.yaw)
                self.link.queue_angle(self.config.pitch_id, self._command.pitch)
                self._reason = ""
            self._queue_query(now)
            self._pump(now)
            self._last_update = now
            return self.status

    @property
    def status(self) -> Status:
        with self._lock:
            now = time.monotonic()
            measured = self._pose_from_link()
            age = self._feedback_age(now)
            return Status(
                enabled=self._enabled,
                link_ok=self._link_ok(now, age),
                torque=self._torque,
                target=self._latest,
                command=self._command,
                measured=measured,
                feedback_age=age,
                input_age=self._input_age(now),
                reason=self._reason,
                tx_errors=self.link.tx_errors,
                rx_errors=self.link.rx_errors,
                last_error=self._last_error,
            )

    # ---- internals -------------------------------------------------------------------------

    def _read_pose(self) -> Pose:
        yaw = self.link.read_angle(self.config.yaw_id, 0.1)
        pitch = self.link.read_angle(self.config.pitch_id, 0.1)
        pose = Pose(yaw, pitch)
        self._measured = pose
        return pose

    def _send_disable_both(self) -> None:
        errors = []
        for addr in (self.config.yaw_id, self.config.pitch_id):
            try:
                self.link.write_frame(p.disable(addr))
            except LinkError as exc:
                errors.append(f"motor {addr}: {exc}")
        if errors:
            raise LinkError("; ".join(errors))

    def _send_gains(self) -> None:
        for addr, gains in ((self.config.yaw_id, self.config.yaw_gains),
                            (self.config.pitch_id, self.config.pitch_gains)):
            self.link.write_frame(p.speed_kp(addr, gains.speed_kp))
            self.link.write_frame(p.speed_ki(addr, gains.speed_ki))
            self.link.write_frame(p.pos_kp(addr, gains.pos_kp))
            self.link.write_frame(p.pos_ki(addr, gains.pos_ki))

    def _start_axis(self, addr: int, deg: float) -> None:
        self.link.write_frame(p.enable(addr))
        self.link.write_frame(p.mode_direct(addr))
        self.link.write_frame(p.speed_rpm(addr, self.config.speed_rpm))
        self.link.write_frame(p.accel(addr, self.config.accel))
        self.link.write_frame(p.angle(addr, deg))

    def _disable_locked(self, reason: str) -> CommandResult:
        if not self._enabled and self._torque == "off":
            return CommandResult(True, "", "off")
        self.link.clear_pending()
        try:
            self._send_disable_both()
            self._torque = "off"
            ok, error = True, ""
        except LinkError as exc:
            self._torque = "unknown"
            ok, error = False, str(exc)
            self._last_error = error
        self._enabled = False
        self._latest = None
        self._input_at = None
        self._velocity = [0.0, 0.0]
        self._reason = "" if ok else f"{reason}: {error}"
        return CommandResult(ok, error, self._torque)

    def _pump(self, now: float) -> None:
        try:
            self.link.pump(now)
        except LinkError as exc:
            self._last_error = str(exc)

    def _queue_query(self, now: float) -> None:
        if now < self._next_query:
            return
        addr = self.config.yaw_id if self._query_yaw else self.config.pitch_id
        self.link.queue_query(addr)
        self._query_yaw = not self._query_yaw
        self._next_query = now + self.config.feedback_period

    def _pose_from_link(self) -> Pose | None:
        yaw = self.link.angles.get(self.config.yaw_id)
        pitch = self.link.angles.get(self.config.pitch_id)
        if yaw is None or pitch is None:
            return None
        return Pose(yaw, pitch)

    def _feedback_age(self, now: float) -> float | None:
        stamps = [self.link.angle_at.get(addr)
                  for addr in (self.config.yaw_id, self.config.pitch_id)]
        if any(stamp is None for stamp in stamps):
            return None
        return max(0.0, max(now - stamp for stamp in stamps))

    def _input_age(self, now: float) -> float | None:
        return None if self._input_at is None else max(0.0, now - self._input_at)

    def _link_ok(self, now: float, age: float | None) -> bool:
        if age is None or age > self.config.feedback_timeout:
            return False
        error_at = getattr(self.link, "error_at", None)
        return error_at is None or now - error_at > self.config.feedback_timeout

    def _lead_guard(self, now: float) -> bool:
        if self.config.max_lead is None or self._measured is None:
            return False
        error = max(abs(wrap(self._command.yaw - self._measured.yaw)),
                    abs(self._command.pitch - self._measured.pitch))
        if error <= self.config.max_lead:
            self._lead_since = None
            return False
        if self._lead_since is None:
            self._lead_since = now
        return now - self._lead_since >= self.config.lead_timeout

    def _freeze(self, reason: str) -> None:
        self._velocity = [0.0, 0.0]
        self._reason = reason

    def _advance(self, target: Pose, now: float) -> None:
        dt = self.config.control_period if self._last_update is None else now - self._last_update
        dt = _clamp(dt, 0.0, 0.05)
        if dt <= 1e-9:
            return
        # 1 ms substeps keep the explicit follower stable even when the speed
        # and acceleration clamps are disabled.
        steps = max(1, math.ceil(dt / 0.001))
        h = dt / steps
        omega = 2 * math.pi * self.config.response_hz
        command = [self._command.yaw, self._command.pitch]
        for _ in range(steps):
            for i, name in enumerate(("yaw", "pitch")):
                goal = getattr(target, name)
                error = wrap(goal - command[i]) if name == "yaw" else goal - command[i]
                velocity = self._velocity[i]
                accel = omega * omega * error - 2 * omega * velocity
                if self.config.max_accel is not None:
                    accel = _clamp(accel, -self.config.max_accel, self.config.max_accel)
                velocity += accel * h
                if self.config.max_speed is not None:
                    velocity = _clamp(velocity, -self.config.max_speed, self.config.max_speed)
                nxt = command[i] + velocity * h
                limits = self._limits(name)
                if limits is None:
                    nxt = wrap(nxt)
                else:
                    clamped = _clamp(nxt, *limits)
                    if clamped != nxt:
                        velocity = 0.0
                    nxt = clamped
                self._velocity[i] = velocity
                command[i] = nxt
        self._command = Pose(command[0], command[1])

    def _clip(self, pose: Pose) -> Pose:
        limits = self._limits("yaw")
        yaw = wrap(pose.yaw) if limits is None else _clamp(pose.yaw, *limits)
        return Pose(yaw, _clamp(pose.pitch, *self._limits("pitch")))

    def _limits(self, axis: str) -> tuple[float, float] | None:
        return self.config.yaw_limits if axis == "yaw" else self.config.pitch_limits
