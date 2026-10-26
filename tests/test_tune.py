"""tools/tune.py without hardware: metrics, argument validation, cleanup."""

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from neck.driver import DriverState, Pose
from tests.fakes import FakeDriver
from tools import tune

ARGV = ["--axis", "pitch", "--speed-kp", "12", "--speed-ki", "3", "--pos-kp", "35", "--pos-ki", "0"]


def run_with(driver, stdin, argv):
    with tempfile.TemporaryDirectory() as tmp:
        args = tune.parse_args(argv + ["--enable", "--output", tmp])
        out = io.StringIO()
        with patch.object(tune, "Driver", SimpleNamespace(open=lambda config: driver)), \
             patch("builtins.input", return_value="ENABLE"), \
             patch.object(tune.sys, "stdin", io.StringIO(stdin)), \
             patch.object(tune.select, "select", return_value=([1], [], [])), \
             contextlib.redirect_stdout(out):
            try:
                tune.run(args)
                error = None
            except Exception as exc:  # the run re-raises after cleaning up
                error = exc
        record = json.loads(next(Path(tmp).glob("tune_*/run.json")).read_text())
        return record, error, out.getvalue()


class TuneTests(unittest.TestCase):
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

    def test_arguments_set_only_the_chosen_axis_and_defaults_stay(self):
        args = tune.parse_args(ARGV)
        self.assertEqual(args.limits, [None, None])
        self.assertEqual((args.config.pitch_gains.speed_kp, args.config.pitch_gains.speed_ki), (12, 3))
        self.assertEqual(args.config.yaw_gains, tune.DriverConfig().yaw_gains)
        self.assertEqual(tune.DriverConfig().pitch_gains.speed_kp, 10)
        for extra in (["--speed-kp", "257"], ["--hz", "nan"], ["--pitch-range", "20", "10"],
                      ["--pitch-range", "0", "60"], ["--move-speed", "61"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                tune.parse_args(ARGV + extra)

    def test_checked_position_wraps_and_rejects_non_finite(self):
        pose = Pose(350, 359.9)
        self.assertAlmostEqual(tune.checked_position(pose, [None, None])[0], -10)
        with self.assertRaises(ValueError):
            tune.checked_position(Pose(float("nan"), 0), [None, None])

    def test_quantization_matches_the_protocol_and_does_not_drift(self):
        for value in (-4.7, -1.6, 41.9, 3.1, 0.05, -0.05, 12.349):
            self.assertEqual(tune.quantized(tune.quantized(value)), tune.quantized(value))
        self.assertEqual(tune.quantized(0.06), 0.1)
        self.assertEqual(tune.quantized(-4.7), -4.7)

    def test_preview_never_opens_hardware(self):
        out = io.StringIO()
        with patch.object(tune, "Driver", SimpleNamespace(open=lambda _c: self.fail("hardware opened"))), \
             contextlib.redirect_stdout(out):
            tune.run(tune.parse_args(ARGV))
        self.assertIn("Preview only", out.getvalue())

    def test_manual_exit_disables_and_closes(self):
        driver = FakeDriver(Pose(0, 42))
        record, error, _ = run_with(driver, "q\n", ARGV)
        self.assertIsNone(error)
        self.assertEqual(record["status"], "finished")
        self.assertEqual(driver.disables, 1)
        self.assertIs(driver.state, DriverState.CLOSED)
        self.assertTrue(driver.sent)
        self.assertTrue(all(p is None or p <= 45 for _, p in driver.sent))

    def test_partial_enable_failure_still_disables_and_closes(self):
        driver = FakeDriver(Pose(0, 42))
        driver.enable_error = True
        record, error, _ = run_with(driver, "q\n", ARGV)
        self.assertIsNotNone(error)
        self.assertEqual(record["status"], "fault")
        self.assertEqual(driver.disables, 1)
        self.assertIs(driver.state, DriverState.CLOSED)

    def test_pitch_target_above_the_limit_is_rejected_and_never_sent(self):
        driver = FakeDriver(Pose(0, 42))
        record, error, out = run_with(driver, "t 60\nq\n", ARGV)
        self.assertIsNone(error)
        self.assertIn("Command rejected", out)
        self.assertTrue(all(p is None or p <= 45 for _, p in driver.sent))

    def test_ramp_moves_only_the_chosen_axis(self):
        driver = FakeDriver(Pose(0, 0))
        argv = ["--axis", "yaw", "--speed-kp", "40", "--speed-ki", "4", "--pos-kp", "42", "--pos-ki", "0"]
        _, error, _ = run_with(driver, "t 5\nq\n", argv)
        self.assertIsNone(error)
        ramp = driver.sent[1:]          # first call is the hold at the measured pose
        self.assertTrue(ramp)
        self.assertTrue(all(pitch is None and yaw is not None for yaw, pitch in ramp))


if __name__ == "__main__":
    unittest.main()
