"""No hardware: generated target noise, latency, limits and timing failures."""

import math
import random
import unittest

from neck.driver import Pose
from neck.control.trajectory import Trajectory, TrajectoryConfig, TrajectoryError


def make(initial=Pose(0, 0), **config):
    """Circular yaw and the old, faster test profile unless overridden."""
    values = dict(pitch_limits=(-170, 40), yaw_limits=None, max_speed_deg_s=180,
                  max_accel_deg_s2=1800, response_hz=8, min_cutoff_hz=3) | config
    trajectory = Trajectory(TrajectoryConfig(**values))
    trajectory.reset(initial, 0)
    return trajectory


class TrajectoryTests(unittest.TestCase):
    def tracker(self, initial=Pose(0, 0), **config):
        return make(initial, **config)

    def drive(self, tracker, target, start, count, dt=0.02):
        commands = []
        for i in range(1, count + 1):
            now = start + i * dt
            tracker.set_target(Pose(*target), now)
            commands.append(tracker.step(now))
        return commands

    def test_static_input_noise_does_not_toggle_commands(self):
        for initial in (Pose(0, 0), Pose(10.05, -10.05)):
            tracker = self.tracker(initial)
            expected = tracker.command
            rng = random.Random(52)
            for i in range(1, 1001):
                now = i * 0.02
                tracker.set_target(Pose(initial.yaw + rng.uniform(-0.06, 0.06),
                                            initial.pitch + rng.uniform(-0.06, 0.06)), now)
                self.assertEqual(tracker.step(now), expected)

    def test_slow_cumulative_motion_and_final_accuracy(self):
        tracker = self.tracker()
        for i in range(1, 1001):
            now = i * 0.02
            tracker.set_target(Pose(now * 0.05, -now * 0.05), now)
            tracker.step(now)
        self.assertGreater(tracker.command.yaw, 0.8)
        commands = self.drive(tracker, (1, -1), 20, 100)
        self.assertLessEqual(abs(commands[-1].yaw - 1), 0.13)
        self.assertLessEqual(abs(commands[-1].pitch + 1), 0.13)

    def test_software_ramp_lag_and_stop_accuracy(self):
        tracker = self.tracker()
        lags = []
        for i in range(1, 101):
            now = i * 0.02
            target = now * 60
            tracker.set_target(Pose(target, 0), now)
            command = tracker.step(now)
            if now >= 0.3:
                lags.append((target - command.yaw) / 60)
        self.assertLessEqual(max(lags), 0.1)  # Generated trajectory, not motor/video latency.
        stopped = self.drive(tracker, (120, 0), 2, 50)
        self.assertLessEqual(abs(stopped[-1].yaw - 120), 0.13)
        self.assertLessEqual(max(c.yaw for c in stopped), 120.1)

    def test_step_targets_settle_without_command_overshoot(self):
        for angle in (5, 30, 90, -5, -30, -90):
            with self.subTest(angle=angle):
                tracker = self.tracker()
                commands = self.drive(tracker, (angle, 0), 0, 150)
                self.assertLessEqual(max(c.yaw * math.copysign(1, angle) for c in commands), abs(angle) + 0.05)
                self.assertLessEqual(abs(commands[-1].yaw - angle), 0.13)
                self.assertEqual(len(set(c.yaw for c in commands[-30:])), 1)

    def test_speed_acceleration_limits_and_reversals(self):
        tracker = self.tracker(yaw_limits=(-90, 90), max_speed_deg_s=100, max_accel_deg_s2=600)
        now = 0.0
        previous_v = (0.0, 0.0)
        for i in range(400):
            dt = (0.015, 0.025, 0.02)[i % 3]
            now += dt
            yaw = 80 if (i // 50) % 2 == 0 else -80
            pitch = 40 if (i // 40) % 2 == 0 else -170
            tracker.set_target(Pose(yaw, pitch), now)
            command = tracker.step(now)
            for velocity, previous in zip(tracker.velocity, previous_v):
                self.assertLessEqual(abs(velocity), 100 + 1e-7)
                self.assertLessEqual(abs(velocity - previous) / dt, 600 + 1e-5)
            previous_v = tracker.velocity
            self.assertTrue(-90 <= command.yaw <= 90)
            self.assertTrue(-170 <= command.pitch <= 40)

    def test_pitch_uses_signed_path_including_boundary_targets(self):
        tracker = self.tracker(Pose(0, 190))  # Mechanical feedback 190 == signed -170.
        commands = self.drive(tracker, (0, 40), 0, 200)
        self.assertTrue(all(-170 <= c.pitch <= 40 for c in commands))
        self.assertGreater(commands[0].pitch, -170)
        self.assertLessEqual(abs(commands[-1].pitch - 40), 0.13)
        reverse = self.drive(tracker, (0, -170), 4, 200)
        self.assertTrue(all(-170 <= c.pitch <= 40 for c in reverse))
        self.assertLessEqual(abs(reverse[-1].pitch + 170), 0.13)

    def test_randomized_limits_with_irregular_ticks(self):
        rng = random.Random(17)
        for _ in range(20):
            speed = rng.uniform(20, 300)
            acceleration = rng.uniform(100, 5000)
            cfg = TrajectoryConfig(yaw_limits=(-120, 120), pitch_limits=(-50, 40),
                                 max_speed_deg_s=speed, max_accel_deg_s2=acceleration,
                                 response_hz=rng.uniform(2, 20))
            tracker = Trajectory(cfg)
            tracker.reset(Pose(0, 0), 0)
            now, previous_v = 0.0, (0.0, 0.0)
            for tick in range(200):
                dt = rng.uniform(0.01, 0.035)
                now += dt
                if tick % 15 == 0:
                    target = (rng.uniform(-120, 120), rng.uniform(-50, 40))
                tracker.set_target(Pose(*target), now)
                command = tracker.step(now)
                self.assertTrue(-120 <= command.yaw <= 120)
                self.assertTrue(-50 <= command.pitch <= 40)
                for velocity, previous in zip(tracker.velocity, previous_v):
                    self.assertLessEqual(abs(velocity), speed + 1e-6)
                    self.assertLessEqual(abs(velocity - previous) / dt, acceleration + 1e-4)
                previous_v = tracker.velocity

    def test_yaw_circular_crossing_is_small_and_bounded_yaw_is_signed(self):
        tracker = self.tracker(Pose(179.8, 0))
        previous = tracker.command.yaw
        travel = 0.0
        for command in self.drive(tracker, (-179.7, 0), 0, 100):
            delta = (command.yaw - previous + 180) % 360 - 180
            self.assertLessEqual(abs(delta), 0.2)
            travel += delta
            previous = command.yaw
        self.assertAlmostEqual(travel, 0.5, delta=0.13)
        bounded = self.tracker(Pose(-170, 0), yaw_limits=(-170, 170))
        commands = self.drive(bounded, (170, 0), 0, 250)
        self.assertGreater(commands[0].yaw, -170)
        self.assertTrue(all(-170 <= c.yaw <= 170 for c in commands))
        self.assertAlmostEqual(commands[-1].yaw, 170, delta=0.13)

    def test_latest_target_replaces_previous_targets(self):
        tracker = self.tracker()
        reference = self.tracker()
        for target in (80, -70, 10):
            tracker.set_target(Pose(target, 0), 0)
        reference.set_target(Pose(10, 0), 0)
        self.assertEqual(tracker.step(0.02), reference.step(0.02))
        self.assertEqual(tracker.target, Pose(10, 0))

    def test_stale_input_brakes_holds_and_resumes(self):
        tracker = self.tracker()
        self.drive(tracker, (90, 0), 0, 10)
        commands = [tracker.step(0.2 + i * 0.02) for i in range(1, 51)]
        self.assertTrue(tracker.stale)
        self.assertEqual(tracker.velocity, (0.0, 0.0))
        self.assertEqual(len(set(c.yaw for c in commands[-20:])), 1)
        self.assertLess(commands[-1].yaw, 89)
        before = tracker.command
        tracker.set_target(Pose(-10, 0), 1.22)
        after = tracker.step(1.22)
        self.assertFalse(tracker.stale)
        self.assertLessEqual(abs(after.yaw - before.yaw), 3.7)
        self.assertAlmostEqual(self.drive(tracker, (-10, 0), 1.22, 100)[-1].yaw, -10, delta=0.13)

    def test_control_gap_requires_explicit_feedback_reset(self):
        tracker = self.tracker()
        tracker.set_target(Pose(30, 0), 0)
        before = tracker.step(0.02)
        with self.assertRaises(TrajectoryError):
            tracker.step(0.5)
        self.assertTrue(tracker.faulted)
        self.assertEqual(tracker.command, before)
        with self.assertRaises(TrajectoryError):
            tracker.set_target(Pose(0, 0), 0.51)
        tracker.reset(Pose(5, 350), 1)
        self.assertFalse(tracker.faulted)
        self.assertEqual(tracker.step(1.02), Pose(5, -10))

    def test_stale_resume_at_held_pose_does_not_pull_backwards(self):
        tracker = self.tracker()
        self.drive(tracker, (90, 0), 0, 10)
        for i in range(1, 51):
            tracker.step(0.2 + i * 0.02)
        held = tracker.command
        pose = tracker.current
        self.assertTrue(all(command == held for command in
                            self.drive(tracker, (pose.yaw, pose.pitch), 1.2, 50)))

    def test_invalid_targets_are_rejected_atomically(self):
        tracker = self.tracker(yaw_limits=(-90, 90))
        tracker.set_target(Pose(10, 20), 0)
        for yaw, pitch in ((0, 41), (91, 0), (float("nan"), 0), (0, float("inf")),
                           (True, 0), (0, "1"), (2 ** 2048, 0)):
            with self.subTest(yaw=yaw, pitch=pitch):
                with self.assertRaises(ValueError):
                    tracker.set_target(Pose(yaw, pitch), 0)
                self.assertEqual(tracker.target, Pose(10, 20))

    def test_clock_validation_zero_ticks_and_small_time_deltas(self):
        tracker = self.tracker()
        tracker.set_target(Pose(10, 0), 0)
        initial = tracker.command
        self.assertEqual(tracker.step(0), initial)
        self.assertEqual(tracker.step(1e-300), initial)
        tracker.step(0.02)
        for value in (0.01, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                tracker.step(value)
            with self.assertRaises(ValueError):
                tracker.set_target(Pose(0, 0), value)
        self.assertFalse(tracker.faulted)

    def test_calibration_and_config_validation(self):
        invalid = ({"pitch_limits": (-180, 181)}, {"pitch_limits": (1, 1)},
                   {"pitch_limits": (0.01, 0.02)}, {"pitch_limits": "range"},
                   {"max_gap_s": 0}, {"max_gap_s": 1}, {"max_speed_deg_s": 2000},
                   {"max_accel_deg_s2": float("inf")}, {"response_hz": 21},
                   {"beta": -1}, {"deadband_deg": 1}, {"hysteresis_deg": 0.05},
                   {"min_cutoff_hz": True}, {"stale_after_s": float("nan")})
        for values in invalid:
            with self.subTest(values=values), self.assertRaises(ValueError):
                TrajectoryConfig(**({"pitch_limits": (-30, 40)} | values))
        with self.assertRaises(ValueError):
            self.tracker(Pose(float("nan"), 0))

    def test_start_outside_the_limits_is_kept_and_only_leads_inward(self):
        tracker = self.tracker(Pose(0, 50))                 # limits are (-170, 40)
        self.assertEqual(tracker.command, Pose(0, 50))
        with self.assertRaises(ValueError):
            tracker.set_target(Pose(0, 45), 0.02)           # new targets must respect the limits
        commands = self.drive(tracker, (0, 30), 0, 150)
        self.assertTrue(all(c.pitch <= 50 for c in commands))
        self.assertLessEqual(abs(commands[-1].pitch - 30), 0.13)
        edge = make(Pose(180, 180), pitch_limits=(10, 180), yaw_limits=(10, 180))
        self.assertEqual(edge.command, Pose(-180, -180))    # signed coordinates, same position

    def test_non_grid_calibration_never_emits_outside_limits(self):
        tracker = make(pitch_limits=(-29.96, 39.96))
        for start, target in ((0, 39.96), (3, -29.96)):
            commands = self.drive(tracker, (0, target), start, 150)
            self.assertTrue(all(-29.96 <= c.pitch <= 39.96 for c in commands))


if __name__ == "__main__":
    unittest.main()
