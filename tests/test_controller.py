"""Control behavior with measured motion, permission changes and failed I/O."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from f32c import (Config, F32CError, HeadController, HeadControlError,
                  ProtectionConfig, State, TrackingConfig)


class Motor:
    def __init__(self, initial=State(0, 0)):
        self.position = self.target = initial
        self.follow = True
        self.sent = []
        self.reads = self.disables = self.closes = 0
        self.read_error = self.disable_error = self.close_error = False
        self.fail_pitch = False

    def open(self): pass
    def read(self, *, timeout_s):
        self.reads += 1
        if self.read_error:
            raise F32CError("read timeout")
        if self.follow:
            self.position = self.target
        return self.position
    def enable_at(self, yaw, pitch): self.target = State(yaw, pitch)
    def set_yaw_deg(self, value):
        self.target = replace(self.target, yaw_deg=value)
        self.sent.append(("yaw", value))
    def set_pitch_deg(self, value):
        if self.fail_pitch:
            raise F32CError("pitch write failed")
        self.target = replace(self.target, pitch_deg=value)
        self.sent.append(("pitch", value))
    def disable(self):
        self.disables += 1
        if self.disable_error:
            raise F32CError("disable unknown")
    def close(self):
        self.closes += 1
        if self.close_error:
            raise F32CError("close unknown")


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.clock = patch("f32c.controller.time.monotonic", side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)

    def controller(self, initial=State(0, 0), **kwargs):
        self.motor = Motor(initial)
        self.head = HeadController(motor=self.motor, **kwargs).open()
        self.head.enable()
        self.addCleanup(self.head.close)
        return self.head

    def tick(self, dt=.02, **kwargs):
        self.now += dt
        return self.head.tick(**kwargs)

    def test_pitch_target_clips_and_initial_feedback_above_45_is_accepted(self):
        with self.assertRaises(ValueError):
            HeadController(tracking=TrackingConfig(pitch_limits=(50, 60)), motor=Motor())
        head = self.controller(State(359.9, 45.2))
        self.assertAlmostEqual(head.snapshot.measured.yaw_deg, -.1)
        self.assertAlmostEqual(head.snapshot.command.pitch_deg, 45.2)
        self.assertFalse(self.motor.sent)
        self.tick()
        self.assertTrue(all(v <= 45.2 for a, v in self.motor.sent if a == "pitch"))
        head.set_target(0, 80)
        self.assertEqual(head.snapshot.target.pitch_deg, 45)
        for _ in range(5): self.tick()
        self.assertTrue(head.enabled)
        self.assertTrue(all(v <= 45.2 for a, v in self.motor.sent if a == "pitch"))

    def test_enable_moves_to_hold_position_using_feedback_and_repeated_enable_keeps_target(self):
        head = self.controller(State(7, 45.2))
        self.assertEqual(head.snapshot.phase, "moving")
        self.assertEqual(head.snapshot.target, State(0, 35))
        self.assertAlmostEqual(head.snapshot.command.yaw_deg, 7)
        self.assertAlmostEqual(head.snapshot.command.pitch_deg, 45.2)
        self.assertFalse(self.motor.sent)
        for _ in range(500):
            status = self.tick()
            if status.reason == "target_reached": break
        self.assertEqual(status.reason, "target_reached")
        self.assertEqual(status.phase, "hold")
        self.assertLessEqual(abs(status.measured.pitch_deg - 35), 1)
        self.assertGreater(self.motor.reads, 20)
        self.assertEqual(self.motor.disables, 0)
        head.move_to(5, 10)
        self.assertEqual(head.enable().target, State(5, 10))

    def test_new_goal_after_hold_does_not_inherit_inactive_control_gap(self):
        head = self.controller()
        head.set_target(10, 0)
        self.tick()
        head.hold()
        self.now += .2
        head.set_target(5, 0)
        status = head.tick()
        self.assertEqual(status.phase, "following")
        self.assertEqual(status.measured, State(0, 0))

    def test_only_changed_axes_are_sent_and_hold_continues_reading(self):
        head = self.controller()
        head.set_target(.04, -.03)
        for _ in range(6): self.tick()
        self.assertFalse(self.motor.sent)
        head.set_target(5, 0)
        for _ in range(5): self.tick()
        self.assertTrue(self.motor.sent)
        self.assertEqual({a for a, _ in self.motor.sent}, {"yaw"})
        head.hold()
        previous, reads = list(self.motor.sent), self.motor.reads
        for _ in range(10): self.tick()
        self.assertEqual(self.motor.sent, previous)
        self.assertEqual(self.motor.reads, reads + 10)

    def test_permission_is_rechecked_after_first_axis_and_partial_output_retained(self):
        head = self.controller()
        head.set_target(5, -5)
        self.tick()
        permissions = iter([True, True, False])
        status = self.tick(may_send=lambda: next(permissions))
        self.assertEqual(status.phase, "source_hold")
        self.assertEqual([a for a, _ in self.motor.sent], ["yaw"])
        self.assertGreater(status.command.yaw_deg, 0)
        self.assertEqual(status.command.pitch_deg, 0)
        self.tick()
        self.assertEqual(len(self.motor.sent), 1)

    def test_revoked_permission_does_not_send_a_braking_trajectory(self):
        head = self.controller()
        head.set_target(30, 0)
        for _ in range(5): self.tick()
        sent = list(self.motor.sent)
        status = self.tick(may_send=lambda: False)
        self.assertEqual(status.reason, "permission_lost")
        for _ in range(5): self.tick()
        self.assertEqual(self.motor.sent, sent)

    def test_stale_and_reordered_input_cannot_refresh_authority(self):
        head = self.controller(tracking=TrackingConfig(pitch_limits=(-180, 45), stale_after_s=.1))
        head.set_target(10, 0, received_at=self.now)
        for _ in range(6): self.tick()
        self.assertEqual(head.snapshot.reason, "input_stale")
        sent = list(self.motor.sent)
        head.set_target(60, 0, received_at=self.now - 1)
        self.assertEqual(head.snapshot.reason, "input_stale")
        self.assertEqual(self.motor.sent, sent)
        with self.assertRaises(ValueError): head.set_target(0, 0, received_at=self.now + 1)

    def test_transient_feedback_timeout_pauses_output_and_recovery_is_bounded(self):
        head = self.controller()
        head.set_target(10, 0)
        self.tick()
        self.motor.read_error = True
        status = self.tick()
        self.assertIn("feedback_wait", status.reason)
        self.assertEqual(self.motor.disables, 0)
        self.motor.read_error = False
        self.tick()
        self.assertTrue(head.enabled)
        self.motor.read_error = True
        with self.assertRaisesRegex(HeadControlError, "feedback_stale"):
            for _ in range(10): self.tick()
        self.assertEqual(self.motor.disables, 1)

    def test_move_stall_is_detected_before_reference_reaches_zero(self):
        head = self.controller(State(0, 39), tracking=TrackingConfig(
            pitch_limits=(-180, 45), max_speed_deg_s=90, max_accel_deg_s2=180))
        self.motor.follow = False
        head.move_to(0, 0)
        with self.assertRaisesRegex(HeadControlError, "no_progress pitch") as caught:
            for _ in range(250): self.tick()
        self.assertGreater(caught.exception.snapshot.command.pitch_deg, 25)
        self.assertEqual(caught.exception.snapshot.measured.pitch_deg, 39)
        self.assertEqual(self.motor.disables, 1)

    def test_large_following_error_does_not_disable_but_no_progress_still_does(self):
        head = self.controller()
        head._command = self.motor.target = State(0, 0)
        self.motor.follow = False
        self.motor.position = State(20, 20)
        head.hold()
        for _ in range(40): self.tick()
        self.assertTrue(head.enabled)
        self.assertEqual(self.motor.disables, 0)
        self.assertFalse(self.motor.sent)
        with self.assertRaisesRegex(HeadControlError, "no_progress"):
            for _ in range(50): self.tick()
        self.assertEqual(self.motor.disables, 1)

    def test_large_error_still_pauses_reference_until_feedback_catches_up(self):
        head = self.controller()
        head.set_target(-60, -60)
        self.tick()
        self.motor.follow = False
        self.motor.position = State(20, 20)
        status = self.tick()
        self.assertTrue(status.enabled)
        self.assertEqual(set(status.following_limited), {"yaw", "pitch"})
        self.assertFalse(self.motor.sent)

    def test_holding_oscillation_is_still_monitored(self):
        head = self.controller()
        head.hold()
        self.motor.follow = False
        self.tick()
        self.motor.position = State(-.7, 0)
        self.tick()
        self.motor.position = State(.7, 0)
        with self.assertRaisesRegex(HeadControlError, "hold_oscillation yaw"): self.tick()

    def test_tick_gap_fault_and_failed_cleanup_are_observable(self):
        head = self.controller()
        head.set_target(10, 0)
        self.motor.disable_error = True
        with self.assertRaisesRegex(HeadControlError, "control_gap") as caught: self.tick(.11)
        self.assertIn("disable unknown", caught.exception.cleanup_error)
        with self.assertRaises(F32CError): head.reset_fault()
        self.motor.disable_error = False
        head.disable()
        head.reset_fault()
        head.enable()
        self.assertTrue(head.enabled)

    def test_second_axis_failure_reports_the_first_axis_output(self):
        head = self.controller()
        head.set_target(10, -10)
        self.tick()
        self.motor.fail_pitch = True
        with self.assertRaisesRegex(HeadControlError, "target_write_failed pitch") as caught: self.tick()
        self.assertGreater(caught.exception.snapshot.command.yaw_deg, 0)
        self.assertEqual(caught.exception.snapshot.command.pitch_deg, 0)

    def test_move_timeout_does_not_report_predicted_position_as_reached(self):
        head = self.controller(protection=ProtectionConfig(settle_time_s=.2))
        self.motor.follow = False
        head.move_to(1.5, 0, timeout_s=.2)
        with self.assertRaisesRegex(HeadControlError, "move_timeout"):
            for _ in range(20): self.tick()

    def test_default_following_profile_latest_input_stale_hold_and_resume(self):
        head = self.controller(State(0, 35))
        self.assertEqual(head.tracking.yaw_limits, (-60, 60))
        self.assertEqual(head.tracking.pitch_limits, (-180, 45))
        self.assertEqual(head.tracking.max_speed_deg_s, 30)
        self.assertEqual(head.tracking.max_accel_deg_s2, 360)
        for _ in range(120):
            head.set_target(10, 35, received_at=self.now)
            head.set_target(-10, 20, received_at=self.now)
            status = self.tick()
            self.assertLessEqual(status.command.yaw_deg, 0)
        self.assertEqual(status.target, State(-10, 20))
        self.assertLessEqual(abs(status.measured.yaw_deg + 10), 1)
        self.assertLessEqual(abs(status.measured.pitch_deg - 20), 1)
        for _ in range(12): self.tick()
        self.assertEqual(head.snapshot.reason, "input_stale")
        sent = list(self.motor.sent)
        for _ in range(10): self.tick()
        self.assertEqual(self.motor.sent, sent)
        self.now += 1
        head.set_target(120, 90, received_at=self.now)
        self.assertEqual(head.snapshot.target, State(60, 45))
        self.tick()
        self.assertEqual(head.phase, "following")
        self.assertEqual(self.motor.disables, 0)

    def test_invalid_parameters_and_passive_close(self):
        for config in (Config(accel=.5), Config(id_yaw=True), Config(pitch_pos_kp=.5)):
            with self.assertRaisesRegex(ValueError, "integer"): config._validate()
        for changes in ({"feedback_timeout_s": .2}, {"stall_after_s": float("nan")},
                        {"reference_lead_deg": 1}):
            with self.assertRaises(ValueError): ProtectionConfig(**changes)
        motor = Motor()
        head = HeadController(motor=motor).open()
        head.close()
        self.assertEqual(motor.disables, 0)
        self.assertEqual(motor.closes, 1)


if __name__ == "__main__": unittest.main()
