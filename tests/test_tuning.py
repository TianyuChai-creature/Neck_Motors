"""No hardware: metric definitions, command validation, motion and cleanup."""

import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))
import autotune
import tune


class Clock:
    def __init__(self):
        self.now = 0.

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration


class Motor:
    failure = None

    def __init__(self, config):
        self.config = config
        self.events = []
        self.position = [0., 0.]
        self.target = [0., 0.]
        self.axis_commands = []

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    def open(self):
        self.events.append("open")

    def start_log(self, path):
        Path(path).write_text("fake serial\n")

    def read(self):
        if self.failure == "read":
            raise RuntimeError("feedback failure")
        if self.failure == "motion":
            return SimpleNamespace(yaw_deg=30., pitch_deg=0.)
        if self.failure == "stalled":
            return SimpleNamespace(yaw_deg=0., pitch_deg=0.)
        self.position = [(p + t) / 2 for p, t in zip(self.position, self.target)]
        return SimpleNamespace(yaw_deg=self.position[0], pitch_deg=self.position[1])

    def enable(self):
        self.events.append("enable")
        if self.failure == "enable":
            raise RuntimeError("enable failure")

    def set_pi(self, axis, *gains):
        self.events.append(("pi", axis, gains))

    def enable_at(self, yaw, pitch):
        self.enable()
        self.set_deg(yaw, pitch)

    def set_deg(self, yaw, pitch):
        self.target = [yaw, pitch]
        self.events.append((yaw, pitch))

    def set_yaw_deg(self, yaw):
        self.axis_commands.append("yaw")
        self.set_deg(yaw, self.target[1])

    def set_pitch_deg(self, pitch):
        self.axis_commands.append("pitch")
        self.set_deg(self.target[0], pitch)

    def disable(self):
        self.events.append("disable")
        if self.failure == "disable":
            raise RuntimeError("disable failure")

    def close(self):
        self.events.append("close")


class TuningTests(unittest.TestCase):
    def test_recovery_needs_final_continuous_stability_and_disturbance(self):
        trial = {"target": 0, "hold": [(0, -.2), (.1, .3)], "push": [(.2, 3)],
                 "release_time": .3, "release": [(.3, 2), (.4, .5), (.5, 2), (.6, .4), (.7, .2), (.8, .1)]}
        result = tune.summarize(trial, 1, .19, .11)
        self.assertAlmostEqual(result["hold_peak_to_peak_deg"], .5)
        self.assertAlmostEqual(result["recovery_time_s"], .3)
        trial["release"].append((.9, 2))
        self.assertIsNone(tune.summarize(trial, 1, .19, .11)["recovery_time_s"])
        trial["release"] = [(.4, 0), (.5, 0), (2, 0)]
        self.assertIsNone(tune.summarize(trial, 1, .19, .11)["recovery_time_s"])
        trial["push"] = [(.2, .2)]
        self.assertFalse(tune.summarize(trial, 1, .19, .11)["disturbance_observed"])

    def test_ranges_optional_and_config_defaults_unchanged(self):
        argv = ["--axis", "pitch", "--speed-kp", "12", "--speed-ki", "3", "--pos-kp", "35", "--pos-ki", "0"]
        args = tune.parse_args(argv)
        self.assertEqual(args.limits, [None, None])
        self.assertEqual(args.config.pitch_speed_kp, 12)
        self.assertEqual(tune.Config().pitch_speed_kp, 10)
        state = SimpleNamespace(yaw_deg=350, pitch_deg=359.9)
        self.assertAlmostEqual(tune.checked_position(state, [None, None])[0], -10)
        self.assertEqual(tune.checked_position(state, [[-5, 5], None])[0], -10)
        with self.assertRaises(ValueError):
            tune.checked_position(SimpleNamespace(yaw_deg=float("nan"), pitch_deg=0), [None, None])
        for extra in (["--speed-kp", "257"], ["--hz", "nan"], ["--pitch-range", "20", "10"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tune.parse_args(argv + extra)

    def test_preview_never_opens_hardware(self):
        with patch.object(autotune, "Gimbal2Dof", side_effect=AssertionError("hardware opened")), contextlib.redirect_stdout(io.StringIO()):
            autotune.run(autotune.parse_args([]))

    def test_pitch_limit_includes_validation_steps_before_hardware_opens(self):
        autotune.check_test_pose([0, 42], 1, 3)
        autotune.check_test_pose([0, -100], 1, 3)
        for center in ([0, 43], [0, 56.8]):
            with self.assertRaises(ValueError):
                autotune.check_test_pose(center, 1, 3)
        with tempfile.TemporaryDirectory() as tmp, patch.object(autotune, "Gimbal2Dof", side_effect=AssertionError("hardware opened")):
            args = autotune.parse_args([])
            with self.assertRaises(ValueError):
                autotune.evaluate(args.config, 1, [0, 43], args, Path(tmp), "invalid")

    def test_invalid_validation_plan_never_enables(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = autotune.parse_args(["--enable", "--center", "0", "30",
                                        "--validation-offsets", "0", "15", "--output", tmp])
            motor = Motor(args.config)
            with patch.object(autotune, "Gimbal2Dof", return_value=motor), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(ValueError):
                    autotune.run(args)
            self.assertEqual(motor.events, ["open", "close"])

    def test_pitch_ramp_never_wraps_through_head_down_limit(self):
        args = autotune.parse_args(["--move-speed", "60", "--hold-seconds", "1", "--recovery-seconds", "1"])
        motor = Motor(args.config)
        motor.position = motor.target = [0., 45.2]
        clock = Clock()
        with tempfile.TemporaryDirectory() as tmp, patch.object(autotune, "Gimbal2Dof", return_value=motor), patch.object(autotune, "time", clock):
            autotune.evaluate(args.config, 1, [0, -170], args, Path(tmp), "negative_pitch")
        targets = [event[1] for event in motor.events if isinstance(event, tuple) and isinstance(event[0], (int, float))]
        self.assertLessEqual(max(targets), 45.2)
        self.assertLess(min(targets), -170)

    def test_manual_exit_and_partial_enable_cleanup(self):
        for failure in (None, "enable"):
            with tempfile.TemporaryDirectory() as tmp:
                args = tune.parse_args(["--axis", "pitch", "--speed-kp", "10", "--speed-ki", "4",
                                        "--pos-kp", "40", "--pos-ki", "0", "--enable", "--output", tmp])
                motor = Motor(args.config)
                motor.position = motor.target = [0., 42.]
                motor.failure = failure
                with patch.object(tune, "Gimbal2Dof", return_value=motor), patch("builtins.input", return_value="ENABLE"), \
                     patch.object(tune.sys, "stdin", io.StringIO("q\n")), \
                     patch.object(tune.select, "select", return_value=([1], [], [])), contextlib.redirect_stdout(io.StringIO()):
                    if failure:
                        with self.assertRaises(RuntimeError):
                            tune.run(args)
                    else:
                        tune.run(args)
                self.assertEqual(motor.events[-2:], ["disable", "close"])
                targets = [e[1] for e in motor.events if isinstance(e, tuple) and isinstance(e[0], (int, float))]
                self.assertTrue(all(value <= 45 for value in targets))
                if failure is None:
                    self.assertTrue(targets)
                record = json.loads(next(Path(tmp).glob("tune_*/run.json")).read_text())
                self.assertEqual(record["status"], "fault" if failure else "finished")

    def test_candidates_are_valid_unique_and_include_integral_compensation(self):
        self.assertEqual(autotune.candidate_values(0, True), [0, 2, 4, 8, 16, 32, 64])
        self.assertEqual(autotune.candidate_values(256), [256, 192])

    def test_protocol_quantization_does_not_drift(self):
        for value in (-4.7, -1.6, 41.9, 3.1):
            self.assertEqual(tune.quantized(tune.quantized(value)), tune.quantized(value))

    def test_weak_tracking_is_not_accepted_as_stability(self):
        args = autotune.parse_args([])
        metrics = {"hold_samples": 100, "hold_peak_to_peak_deg": 0,
                   "hold_mean_error_deg": 0, "recovery_time_s": .2,
                   "offset_mean_tracking_error_deg": 2}
        self.assertFalse(autotune.passed(metrics, args))
        metrics["offset_mean_tracking_error_deg"] = .2
        self.assertTrue(autotune.passed(metrics, args))
        metrics["hold_peak_to_peak_deg"] = .60000000000001
        self.assertTrue(autotune.passed(metrics, args))

    def evaluate(self, failure=None):
        args = autotune.parse_args(["--hold-seconds", "1", "--recovery-seconds", "1", "--move-speed", "1"])
        clock = Clock()
        motor = Motor(args.config)
        motor.failure = failure
        with tempfile.TemporaryDirectory() as tmp:
            directory = Path(tmp)
            with patch.object(autotune, "Gimbal2Dof", return_value=motor), patch.object(autotune, "time", clock):
                if failure:
                    with self.assertRaises(RuntimeError):
                        autotune.evaluate(args.config, 0, [12, 0] if failure == "stalled" else [0, 0], args, directory, "test")
                else:
                    results = autotune.evaluate(args.config, 0, [0, 0], args, directory, "test")
                    self.assertEqual(len(results), 2)
                    self.assertTrue(all(r["recovery_time_s"] is not None for r in results))
                    self.assertTrue(any(isinstance(e, tuple) and e[0] > 2.8 for e in motor.events))
                    self.assertTrue(any(isinstance(e, tuple) and e[0] < -2.8 for e in motor.events))
                    self.assertTrue(motor.axis_commands)
                    self.assertEqual(set(motor.axis_commands), {"yaw"})
                record = json.loads((directory / "result.json").read_text())
                self.assertEqual(record["status"], "fault" if failure else "passed")
                if failure == "stalled":
                    self.assertIn("no progress", record["error"])
                self.assertTrue((directory / "samples.csv").exists())
        self.assertEqual(motor.events[-1], "close")
        if failure != "read":
            self.assertIn("disable", motor.events)

    def test_small_ramp_accumulates_below_protocol_resolution(self):
        self.evaluate()

    def test_cleanup_after_enable_failure_feedback_motion_and_disable_failure(self):
        for failure in ("enable", "read", "motion", "disable"):
            with self.subTest(failure=failure):
                self.evaluate(failure)

    def test_stalled_motion_stops_before_large_error_abort(self):
        self.evaluate("stalled")

    def test_session_stays_enabled_between_experiments(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = autotune.parse_args(["--enable", "--baseline-only", "--center", "0", "0",
                                        "--validation-offsets", "0", "--hold-seconds", "1",
                                        "--recovery-seconds", "1", "--output", tmp])
            motor = Motor(args.config)
            clock = Clock()
            with patch.object(autotune, "Gimbal2Dof", return_value=motor), patch.object(autotune, "time", clock), contextlib.redirect_stdout(io.StringIO()):
                autotune.run(args)
            self.assertEqual(motor.events.count("enable"), 1)
            self.assertEqual(motor.events.count("disable"), 1)
            self.assertEqual(motor.events[-2:], ["disable", "close"])
            self.assertEqual(len([e for e in motor.events if isinstance(e, tuple) and e[0] == "pi"]), 8)


if __name__ == "__main__":
    unittest.main()
