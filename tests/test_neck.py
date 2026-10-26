"""Neck behaviour with measured motion, permission changes and failed I/O (FakeDriver)."""
import unittest
from unittest.mock import patch

from neck.control import Neck, NeckError, ProtectionConfig, TrajectoryConfig
from neck.driver import DriverState, Pose
from tests.fakes import FakeDriver


def pitch_writes(driver):
    return [p for _, p in driver.sent if p is not None]


def axes_written(driver):
    return {axis for y, p in driver.sent for axis, v in (("yaw", y), ("pitch", p)) if v is not None}


class NeckTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        clock = patch("neck.control.neck.time.monotonic", side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)

    def neck(self, initial=Pose(0, 0), **kwargs):
        self.driver = FakeDriver(initial)
        self.head = Neck(self.driver, **kwargs)
        self.head.enable()
        self.addCleanup(self._close)
        return self.head

    def _close(self):
        try:
            self.head.close()
        except NeckError:
            pass

    def tick(self, dt=.02, **kwargs):
        self.now += dt
        return self.head.tick(**kwargs)

    def test_pitch_target_clips_and_initial_feedback_above_45_is_accepted(self):
        head = self.neck(Pose(359.9, 45.2))
        self.assertAlmostEqual(head.snapshot.measured.yaw, -.1)
        self.assertAlmostEqual(head.snapshot.command.pitch, 45.2)
        self.assertFalse(self.driver.sent)
        self.tick()
        self.assertTrue(all(v <= 45.2 for v in pitch_writes(self.driver)))
        head.set_target(0, 80)
        self.assertEqual(head.snapshot.target.pitch, 45)
        for _ in range(5): self.tick()
        self.assertTrue(head.enabled)
        self.assertTrue(all(v <= 45.2 for v in pitch_writes(self.driver)))

    def test_enable_moves_to_zero_using_feedback_and_repeated_enable_keeps_target(self):
        head = self.neck(Pose(7, 45.2))
        self.assertEqual(head.snapshot.phase, "moving")
        self.assertEqual(head.snapshot.target, Pose(0, 0))
        self.assertAlmostEqual(head.snapshot.command.yaw, 7)
        self.assertAlmostEqual(head.snapshot.command.pitch, 45.2)
        self.assertFalse(self.driver.sent)
        for _ in range(500):
            status = self.tick()
            if status.reason == "target_reached": break
        self.assertEqual(status.reason, "target_reached")
        self.assertEqual(status.phase, "hold")
        self.assertLessEqual(abs(status.measured.pitch), 1)
        self.assertLessEqual(abs(status.measured.yaw), 1)
        self.assertGreater(self.driver.reads, 20)
        self.assertEqual(self.driver.disables, 0)
        head.move_to(5, 10)
        self.assertEqual(head.enable().target, Pose(5, 10))

    def test_new_goal_after_hold_does_not_inherit_inactive_control_gap(self):
        head = self.neck()
        head.set_target(10, 0)
        self.tick()
        head.hold()
        self.now += .2
        head.set_target(5, 0)
        status = head.tick()
        self.assertEqual(status.phase, "following")
        self.assertEqual(status.measured, Pose(0, 0))

    def test_only_changed_axes_are_sent_and_hold_continues_reading(self):
        head = self.neck()
        head.set_target(.04, -.03)
        for _ in range(6): self.tick()
        self.assertFalse(self.driver.sent)
        head.set_target(5, 0)
        for _ in range(5): self.tick()
        self.assertTrue(self.driver.sent)
        self.assertEqual(axes_written(self.driver), {"yaw"})
        self.assertTrue(all(p is None for _, p in self.driver.sent))
        head.hold()
        previous, reads = list(self.driver.sent), self.driver.reads
        for _ in range(10): self.tick()
        self.assertEqual(self.driver.sent, previous)
        self.assertEqual(self.driver.reads, reads + 10)

    def test_permission_is_rechecked_right_before_the_write(self):
        head = self.neck()
        head.set_target(5, -5)
        self.tick()
        permissions = iter([True, False])
        status = self.tick(may_send=lambda: next(permissions))
        self.assertEqual((status.phase, status.reason), ("stale", "permission_lost"))
        self.assertFalse(self.driver.sent)
        self.assertEqual(status.command, Pose(0, 0))

    def test_revoked_permission_does_not_send_a_braking_trajectory(self):
        head = self.neck()
        head.set_target(30, 0)
        for _ in range(5): self.tick()
        sent = list(self.driver.sent)
        status = self.tick(may_send=lambda: False)
        self.assertEqual(status.reason, "permission_lost")
        for _ in range(5): self.tick()
        self.assertEqual(self.driver.sent, sent)

    def test_stale_and_reordered_input_cannot_refresh_authority(self):
        head = self.neck(trajectory=TrajectoryConfig(stale_after_s=.1))
        head.set_target(10, 0, at=self.now)
        for _ in range(6): self.tick()
        self.assertEqual((head.snapshot.phase, head.snapshot.reason), ("stale", "input_stale"))
        sent = list(self.driver.sent)
        head.set_target(60, 0, at=self.now - 1)
        self.assertEqual(head.snapshot.reason, "input_stale")
        self.assertEqual(self.driver.sent, sent)
        with self.assertRaises(ValueError): head.set_target(0, 0, at=self.now + 1)

    def test_transient_feedback_timeout_pauses_output_and_recovery_is_bounded(self):
        head = self.neck()
        head.set_target(10, 0)
        self.tick()
        self.driver.read_error = True
        status = self.tick()
        self.assertIn("feedback_wait", status.reason)
        self.assertEqual(self.driver.disables, 0)
        self.driver.read_error = False
        self.tick()
        self.assertTrue(head.enabled)
        self.driver.read_error = True
        levels = []
        with self.assertRaisesRegex(NeckError, "feedback_stale"):
            for _ in range(60):
                levels.append(self.tick().level)
        # 1 = short wait, 2 = degraded (still enabled), then 3 = fault after feedback_lost_s.
        self.assertEqual(sorted(set(levels)), [1, 2])
        self.assertLess(levels.index(2), 10)
        self.assertEqual(self.driver.disables, 1)

    def test_feedback_outage_shorter_than_lost_budget_recovers_without_disable(self):
        head = self.neck()
        head.set_target(10, 0)
        self.tick()
        self.driver.read_error = True
        for _ in range(10): self.tick()
        self.assertEqual(head.snapshot.phase, "degraded")
        self.assertEqual(head.snapshot.level, 2)
        self.driver.read_error = False
        for _ in range(40):
            head.set_target(10, 0)
            self.tick()
        self.assertEqual(self.driver.disables, 0)
        self.assertEqual(head.snapshot.phase, "following")
        self.assertEqual(head.snapshot.level, 0)

    def stalled_move(self):
        head = self.neck(Pose(0, 39), trajectory=TrajectoryConfig(max_accel_deg_s2=180))
        self.driver.follow = False
        head.move_to(0, 0)
        for _ in range(250):
            status = self.tick()
            if status.phase == "degraded": break
        return head, status

    def test_move_stall_is_detected_before_reference_reaches_zero(self):
        head, status = self.stalled_move()
        self.assertEqual(status.phase, "degraded")
        self.assertIn("no_progress pitch", status.reason)
        self.assertEqual(status.level, 2)
        self.assertGreater(status.command.pitch, 10)  # detected well before the reference reaches 0
        self.assertEqual(status.measured.pitch, 39)
        self.assertEqual(self.driver.disables, 0)
        # A permanent stall keeps cycling degraded/recovered and is bounded by escalation.
        with self.assertRaisesRegex(NeckError, "repeated_disturbance|degraded_timeout"):
            for _ in range(2000): self.tick()
        self.assertEqual(self.driver.disables, 1)

    def test_stall_that_clears_relaxes_the_command_and_resumes_the_move(self):
        head, status = self.stalled_move()
        self.tick()
        self.assertIn((None, 39.0), self.driver.sent)  # command pulled back to measured
        self.driver.follow = True  # the disturbance is gone
        for _ in range(60): self.tick()
        self.assertNotEqual(head.snapshot.phase, "degraded")
        for _ in range(600):
            status = self.tick()
            if status.reason == "target_reached": break
        self.assertEqual(status.reason, "target_reached")
        self.assertEqual(self.driver.disables, 0)

    def test_large_following_error_does_not_disable_but_no_progress_still_degrades(self):
        head = self.neck()
        self.driver.follow = False
        self.driver.measured = Pose(20, 20)
        head.hold()
        for _ in range(40): self.tick()
        self.assertTrue(head.enabled)
        self.assertEqual(self.driver.disables, 0)
        self.assertFalse(self.driver.sent)
        for _ in range(60):
            status = self.tick()
            if status.phase == "degraded": break
        self.assertEqual(status.phase, "degraded")
        self.assertIn("no_progress", status.reason)
        self.assertTrue(head.enabled)
        self.assertEqual(self.driver.disables, 0)
        for _ in range(40): self.tick()
        self.assertEqual(head.snapshot.phase, "hold")  # relaxed to the measured pose, then recovered
        self.assertEqual(status.level, 2)
        self.assertEqual(head.snapshot.level, 0)
        self.assertEqual(self.driver.command, Pose(20, 20))
        self.assertEqual(self.driver.disables, 0)

    def test_large_error_still_pauses_reference_until_feedback_catches_up(self):
        head = self.neck()
        head.set_target(-60, -60)
        self.tick()
        self.driver.follow = False
        self.driver.measured = Pose(20, 20)
        status = self.tick()
        self.assertTrue(head.enabled)
        self.assertEqual(status.reason, "waiting_for_feedback")
        self.assertFalse(self.driver.sent)

    def test_holding_oscillation_is_still_monitored(self):
        head = self.neck()
        head.hold()
        self.driver.follow = False
        self.tick()
        self.driver.measured = Pose(-.7, 0)
        self.tick()
        self.driver.measured = Pose(.7, 0)
        status = self.tick()  # level 1: no disable, nothing is sent
        self.assertEqual(status.level, 1)
        self.assertIn("hold_oscillation yaw", status.reason)
        self.assertTrue(head.enabled)
        self.assertEqual(self.driver.disables, 0)
        self.assertFalse(self.driver.sent)

    def test_short_tick_gap_is_a_level_one_disturbance_and_motion_resumes(self):
        head = self.neck()
        head.set_target(10, 0)
        self.tick()
        status = self.tick(.11)
        self.assertEqual(status.level, 1)
        self.assertIn("control_gap", status.reason)
        self.assertTrue(head.enabled)
        self.assertEqual(self.driver.disables, 0)
        for _ in range(10):
            head.set_target(10, 0)
            self.tick()
        self.assertGreater(head.snapshot.command.yaw, 0)
        self.assertEqual(head.snapshot.level, 0)

    def test_repeated_disturbances_escalate_to_a_fault(self):
        head = self.neck()
        head.set_target(10, 0)
        with self.assertRaisesRegex(NeckError, "repeated_disturbance"):
            for _ in range(10):
                head.set_target(10, 0)
                self.tick(.11)
        self.assertEqual(self.driver.disables, 1)

    def test_long_tick_gap_fault_and_failed_cleanup_are_observable(self):
        head = self.neck()
        head.set_target(10, 0)
        self.driver.disable_error = True
        with self.assertRaisesRegex(NeckError, "control_gap") as caught: self.tick(1.2)
        self.assertIn("hardware state unknown", caught.exception.cleanup_error)
        self.assertIs(self.driver.state, DriverState.UNKNOWN)
        with self.assertRaises(NeckError): head.reset_fault()
        self.driver.disable_error = False
        head.disable()
        head.reset_fault()
        head.enable()
        self.assertTrue(head.enabled)

    def test_write_failure_is_a_fault_that_disables(self):
        head = self.neck()
        head.set_target(10, -10)
        self.tick()
        self.driver.set_error = True
        with self.assertRaisesRegex(NeckError, "target_write_failed") as caught: self.tick()
        self.assertEqual(caught.exception.snapshot.phase, "fault")
        self.assertEqual(self.driver.disables, 1)
        self.assertFalse(head.enabled)
        with self.assertRaises(NeckError): head.tick()
        with self.assertRaises(NeckError): head.enable()      # latched until reset_fault()
        head.reset_fault()
        self.driver.set_error = False
        head.enable()
        self.assertTrue(head.enabled)

    def test_failed_enable_is_a_fault(self):
        self.driver = FakeDriver()
        self.driver.enable_error = True
        head = Neck(self.driver)
        with self.assertRaisesRegex(NeckError, "enable_failed"):
            head.enable()
        self.assertEqual(head.snapshot.phase, "fault")
        self.assertEqual(self.driver.disables, 0)       # never armed, nothing to disable

    def test_move_timeout_does_not_report_predicted_position_as_reached(self):
        head = self.neck(protection=ProtectionConfig(settle_time_s=.2))
        self.driver.follow = False
        head.move_to(1.5, 0, timeout=.2)
        with self.assertRaisesRegex(NeckError, "move_timeout"):
            for _ in range(20): self.tick()

    def test_default_following_profile_latest_input_stale_hold_and_resume(self):
        head = self.neck(Pose(0, 35))
        self.assertEqual(head.config.yaw_limits, (-60, 60))
        self.assertEqual(head.config.pitch_limits, (-180, 45))
        self.assertEqual(head.config.max_speed_deg_s, 90)
        self.assertEqual(head.config.max_accel_deg_s2, 720)
        for _ in range(120):
            head.set_target(10, 35, at=self.now)
            head.set_target(-10, 20, at=self.now)
            status = self.tick()
            self.assertLessEqual(status.command.yaw, 0)
        self.assertEqual(status.target, Pose(-10, 20))
        self.assertLessEqual(abs(status.measured.yaw + 10), 1)
        self.assertLessEqual(abs(status.measured.pitch - 20), 1)
        for _ in range(12): self.tick()
        self.assertEqual(head.snapshot.reason, "input_stale")
        sent = list(self.driver.sent)
        for _ in range(10): self.tick()
        self.assertEqual(self.driver.sent, sent)
        self.now += 1
        head.set_target(120, 90, at=self.now)
        self.assertEqual(head.snapshot.target, Pose(60, 45))
        self.tick()
        self.assertEqual(head.snapshot.phase, "following")
        self.assertEqual(self.driver.disables, 0)

    def test_commands_need_an_enabled_neck(self):
        head = Neck(FakeDriver())
        for call in (lambda: head.set_target(0, 0), lambda: head.move_to(0, 0), head.hold, head.tick):
            with self.assertRaises(NeckError):
                call()

    def test_disable_returns_to_off_and_can_enable_again(self):
        head = self.neck()
        head.set_target(10, 0)
        self.tick()
        head.disable()
        self.assertEqual(head.snapshot.phase, "off")
        self.assertIs(self.driver.state, DriverState.OPEN)
        head.enable()
        self.assertEqual(head.snapshot.phase, "moving")

    def test_invalid_parameters_and_passive_close(self):
        for changes in ({"feedback_timeout_s": .2}, {"stall_after_s": float("nan")},
                        {"reference_lead_deg": 1}, {"feedback_lost_s": .05},
                        {"escalation_events": 0}):
            with self.assertRaises(ValueError): ProtectionConfig(**changes)
        driver = FakeDriver()
        Neck(driver).close()
        self.assertEqual(driver.disables, 0)
        self.assertTrue(driver.port_closed)

    def test_close_while_enabled_disables_and_close_failure_is_reported(self):
        head = self.neck()
        self.driver.disable_error = True
        with self.assertRaises(NeckError): head.close()
        self.assertEqual(head.snapshot.phase, "fault")
        self.assertIs(self.driver.state, DriverState.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
