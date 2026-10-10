"""No hardware: generated target noise, latency, limits and timing failures."""

import math
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from f32c import HeadTracker, State, TrackingConfig, TrackingError


class TrackingTests(unittest.TestCase):
    def tracker(self, initial=State(0, 0), **config):
        return HeadTracker(initial, TrackingConfig(pitch_limits=(-170, 40), **config), timestamp=0)

    def drive(self, tracker, target, start, count, dt=0.02):
        commands = []
        for i in range(1, count + 1):
            now = start + i * dt
            tracker.set_target(*target, timestamp=now)
            commands.append(tracker.step(timestamp=now))
        return commands

    def test_no_native_library_or_motor_lifecycle_is_used(self):
        with patch("f32c._motor_library", side_effect=AssertionError("motor opened")):
            tracker = self.tracker()
            self.drive(tracker, (1, 1), 0, 5)

    def test_static_input_noise_does_not_toggle_commands(self):
        for initial in (State(0, 0), State(10.05, -10.05)):
            tracker = self.tracker(initial)
            expected = tracker.command
            rng = random.Random(52)
            for i in range(1, 1001):
                now = i * 0.02
                tracker.set_target(initial.yaw_deg + rng.uniform(-0.06, 0.06),
                                   initial.pitch_deg + rng.uniform(-0.06, 0.06), timestamp=now)
                self.assertEqual(tracker.step(timestamp=now), expected)

    def test_slow_cumulative_motion_and_final_accuracy(self):
        tracker = self.tracker()
        for i in range(1, 1001):
            now = i * 0.02
            tracker.set_target(now * 0.05, -now * 0.05, timestamp=now)
            tracker.step(timestamp=now)
        self.assertGreater(tracker.command.yaw_deg, 0.8)
        commands = self.drive(tracker, (1, -1), 20, 100)
        self.assertLessEqual(abs(commands[-1].yaw_deg - 1), 0.13)
        self.assertLessEqual(abs(commands[-1].pitch_deg + 1), 0.13)

    def test_software_ramp_lag_and_stop_accuracy(self):
        tracker = self.tracker()
        lags = []
        for i in range(1, 101):
            now = i * 0.02
            target = now * 60
            tracker.set_target(target, 0, timestamp=now)
            command = tracker.step(timestamp=now)
            if now >= 0.3:
                lags.append((target - command.yaw_deg) / 60)
        self.assertLessEqual(max(lags), 0.1)  # Generated trajectory, not motor/video latency.
        stopped = self.drive(tracker, (120, 0), 2, 50)
        self.assertLessEqual(abs(stopped[-1].yaw_deg - 120), 0.13)
        self.assertLessEqual(max(c.yaw_deg for c in stopped), 120.1)

    def test_step_targets_settle_without_command_overshoot(self):
        for angle in (5, 30, 90, -5, -30, -90):
            with self.subTest(angle=angle):
                tracker = self.tracker()
                commands = self.drive(tracker, (angle, 0), 0, 150)
                self.assertLessEqual(max(c.yaw_deg * math.copysign(1, angle) for c in commands), abs(angle) + 0.05)
                self.assertLessEqual(abs(commands[-1].yaw_deg - angle), 0.13)
                self.assertEqual(len(set(c.yaw_deg for c in commands[-30:])), 1)

    def test_speed_acceleration_limits_and_reversals(self):
        tracker = self.tracker(yaw_limits=(-90, 90), max_speed_deg_s=100, max_accel_deg_s2=600)
        now = 0.0
        previous_v = (0.0, 0.0)
        for i in range(400):
            dt = (0.015, 0.025, 0.02)[i % 3]
            now += dt
            yaw = 80 if (i // 50) % 2 == 0 else -80
            pitch = 40 if (i // 40) % 2 == 0 else -170
            tracker.set_target(yaw, pitch, timestamp=now)
            command = tracker.step(timestamp=now)
            for velocity, previous in zip(tracker.velocity_deg_s, previous_v):
                self.assertLessEqual(abs(velocity), 100 + 1e-7)
                self.assertLessEqual(abs(velocity - previous) / dt, 600 + 1e-5)
            previous_v = tracker.velocity_deg_s
            self.assertTrue(-90 <= command.yaw_deg <= 90)
            self.assertTrue(-170 <= command.pitch_deg <= 40)

    def test_pitch_uses_signed_path_including_boundary_targets(self):
        tracker = self.tracker(State(0, 190))  # Mechanical feedback 190 == signed -170.
        commands = self.drive(tracker, (0, 40), 0, 200)
        self.assertTrue(all(-170 <= c.pitch_deg <= 40 for c in commands))
        self.assertGreater(commands[0].pitch_deg, -170)
        self.assertLessEqual(abs(commands[-1].pitch_deg - 40), 0.13)
        reverse = self.drive(tracker, (0, -170), 4, 200)
        self.assertTrue(all(-170 <= c.pitch_deg <= 40 for c in reverse))
        self.assertLessEqual(abs(reverse[-1].pitch_deg + 170), 0.13)

    def test_randomized_limits_with_irregular_ticks(self):
        rng = random.Random(17)
        for _ in range(20):
            speed = rng.uniform(20, 300)
            acceleration = rng.uniform(100, 5000)
            cfg = TrackingConfig(yaw_limits=(-120, 120), pitch_limits=(-50, 40),
                                 max_speed_deg_s=speed, max_accel_deg_s2=acceleration,
                                 response_hz=rng.uniform(2, 20))
            tracker = HeadTracker(State(0, 0), cfg, timestamp=0)
            now, previous_v = 0.0, (0.0, 0.0)
            for tick in range(200):
                dt = rng.uniform(0.01, 0.035)
                now += dt
                if tick % 15 == 0:
                    target = (rng.uniform(-120, 120), rng.uniform(-50, 40))
                tracker.set_target(*target, timestamp=now)
                command = tracker.step(timestamp=now)
                self.assertTrue(-120 <= command.yaw_deg <= 120)
                self.assertTrue(-50 <= command.pitch_deg <= 40)
                for velocity, previous in zip(tracker.velocity_deg_s, previous_v):
                    self.assertLessEqual(abs(velocity), speed + 1e-6)
                    self.assertLessEqual(abs(velocity - previous) / dt, acceleration + 1e-4)
                previous_v = tracker.velocity_deg_s

    def test_yaw_circular_crossing_is_small_and_bounded_yaw_is_signed(self):
        tracker = self.tracker(State(179.8, 0))
        previous = tracker.command.yaw_deg
        travel = 0.0
        for command in self.drive(tracker, (-179.7, 0), 0, 100):
            delta = (command.yaw_deg - previous + 180) % 360 - 180
            self.assertLessEqual(abs(delta), 0.2)
            travel += delta
            previous = command.yaw_deg
        self.assertAlmostEqual(travel, 0.5, delta=0.13)
        bounded = self.tracker(State(-170, 0), yaw_limits=(-170, 170))
        commands = self.drive(bounded, (170, 0), 0, 250)
        self.assertGreater(commands[0].yaw_deg, -170)
        self.assertTrue(all(-170 <= c.yaw_deg <= 170 for c in commands))
        self.assertAlmostEqual(commands[-1].yaw_deg, 170, delta=0.13)

    def test_latest_target_replaces_previous_targets(self):
        tracker = self.tracker()
        reference = self.tracker()
        for target in (80, -70, 10):
            tracker.set_target(target, 0, timestamp=0)
        reference.set_target(10, 0, timestamp=0)
        self.assertEqual(tracker.step(timestamp=0.02), reference.step(timestamp=0.02))
        self.assertEqual(tracker.target, State(10, 0))

    def test_stale_input_brakes_holds_and_resumes(self):
        tracker = self.tracker()
        self.drive(tracker, (90, 0), 0, 10)
        commands = [tracker.step(timestamp=0.2 + i * 0.02) for i in range(1, 51)]
        self.assertTrue(tracker.stale)
        self.assertEqual(tracker.velocity_deg_s, (0.0, 0.0))
        self.assertEqual(len(set(c.yaw_deg for c in commands[-20:])), 1)
        self.assertLess(commands[-1].yaw_deg, 89)
        before = tracker.command
        tracker.set_target(-10, 0, timestamp=1.22)
        after = tracker.step(timestamp=1.22)
        self.assertFalse(tracker.stale)
        self.assertLessEqual(abs(after.yaw_deg - before.yaw_deg), 3.7)
        self.assertAlmostEqual(self.drive(tracker, (-10, 0), 1.22, 100)[-1].yaw_deg, -10, delta=0.13)

    def test_control_gap_requires_explicit_feedback_reset(self):
        tracker = self.tracker()
        tracker.set_target(30, 0, timestamp=0)
        before = tracker.step(timestamp=0.02)
        with self.assertRaises(TrackingError):
            tracker.step(timestamp=0.5)
        self.assertTrue(tracker.faulted)
        self.assertEqual(tracker.command, before)
        with self.assertRaises(TrackingError):
            tracker.set_target(0, 0, timestamp=0.51)
        tracker.reset(State(5, 350), timestamp=1)
        self.assertFalse(tracker.faulted)
        self.assertEqual(tracker.step(timestamp=1.02), State(5, -10))

    def test_stale_resume_at_held_pose_does_not_pull_backwards(self):
        tracker = self.tracker()
        self.drive(tracker, (90, 0), 0, 10)
        for i in range(1, 51):
            tracker.step(timestamp=0.2 + i * 0.02)
        held = tracker.command
        pose = tracker.continuous_command
        self.assertTrue(all(command == held for command in
                            self.drive(tracker, (pose.yaw_deg, pose.pitch_deg), 1.2, 50)))

    def test_invalid_targets_are_rejected_atomically(self):
        tracker = self.tracker(yaw_limits=(-90, 90))
        tracker.set_target(10, 20, timestamp=0)
        for yaw, pitch in ((0, 41), (91, 0), (float("nan"), 0), (0, float("inf")),
                           (True, 0), (0, "1"), (2 ** 2048, 0)):
            with self.subTest(yaw=yaw, pitch=pitch):
                with self.assertRaises(ValueError):
                    tracker.set_target(yaw, pitch, timestamp=0)
                self.assertEqual(tracker.target, State(10, 20))

    def test_clock_validation_zero_ticks_and_small_time_deltas(self):
        tracker = self.tracker()
        tracker.set_target(10, 0, timestamp=0)
        initial = tracker.command
        self.assertEqual(tracker.step(timestamp=0), initial)
        self.assertEqual(tracker.step(timestamp=1e-300), initial)
        tracker.step(timestamp=0.02)
        for value in (0.01, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                tracker.step(timestamp=value)
            with self.assertRaises(ValueError):
                tracker.set_target(0, 0, timestamp=value)
        self.assertFalse(tracker.faulted)
        with patch("f32c.tracking.time.monotonic", return_value=0.04):
            tracker.set_target(0, 0)
            tracker.step()

    def test_calibration_and_config_validation(self):
        invalid = ({"pitch_limits": (-180, 181)}, {"pitch_limits": (1, 1)},
                   {"pitch_limits": (0.01, 0.02)}, {"pitch_limits": "range"},
                   {"max_gap_s": 0}, {"max_gap_s": 1}, {"max_speed_deg_s": 2000},
                   {"max_accel_deg_s2": float("inf")}, {"response_hz": 21},
                   {"beta": -1}, {"deadband_deg": 1}, {"hysteresis_deg": 0.05},
                   {"min_cutoff_hz": True}, {"stale_after_s": float("nan")})
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                TrackingConfig(**({"pitch_limits": (-30, 40)} | values))
        with self.assertRaises(ValueError):
            self.tracker(State(0, 50))
        with self.assertRaises(ValueError):
            self.tracker(State(float("nan"), 0))
        cfg = TrackingConfig(pitch_limits=(10, 180), yaw_limits=(10, 180))
        self.assertEqual(HeadTracker(State(180, 180), cfg, timestamp=0).command, State(180, 180))
        tracker = self.tracker()
        with self.assertRaises(AttributeError):
            tracker.config = TrackingConfig(pitch_limits=(-10, 10))

    def test_non_grid_calibration_never_emits_outside_limits(self):
        cfg = TrackingConfig(pitch_limits=(-29.96, 39.96))
        tracker = HeadTracker(State(0, 0), cfg, timestamp=0)
        for start, target in ((0, 39.96), (3, -29.96)):
            commands = self.drive(tracker, (0, target), start, 150)
            self.assertTrue(all(-29.96 <= c.pitch_deg <= 39.96 for c in commands))


if __name__ == "__main__":
    unittest.main()
