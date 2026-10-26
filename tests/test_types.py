import unittest

from neck.driver import DriverConfig, Gains, Pose
from neck.driver.types import finite, timeout_ok


class GainsTests(unittest.TestCase):
    def test_range_and_type(self):
        Gains(0, 0, 256, 256)
        for bad in (-1, 257, 1.0, True, "1", None):
            with self.assertRaises(ValueError, msg=bad):
                Gains(bad, 0, 0, 0)
            with self.assertRaises(ValueError, msg=bad):
                Gains(0, 0, 0, bad)


class DriverConfigTests(unittest.TestCase):
    def test_defaults_are_the_field_values(self):
        c = DriverConfig()
        self.assertEqual((c.yaw_id, c.pitch_id, c.baud, c.speed_rpm, c.accel), (1, 2, 115200, 15, 2))
        self.assertEqual(c.yaw_gains, Gains(40, 4, 42, 0))
        self.assertEqual(c.pitch_gains, Gains(10, 4, 40, 0))
        self.assertIsNone(c.port)
        self.assertIsNone(c.log_path)

    def test_rejects_bad_values(self):
        for kwargs in ({"baud": 1234}, {"yaw_id": 0}, {"pitch_id": 256}, {"yaw_id": 3, "pitch_id": 3},
                       {"yaw_id": True}, {"speed_rpm": -1}, {"speed_rpm": 1001}, {"speed_rpm": 1.5},
                       {"accel": -1}, {"accel": 65536}, {"port": 5}):
            with self.assertRaises(ValueError, msg=kwargs):
                DriverConfig(**kwargs)

    def test_is_immutable(self):
        with self.assertRaises(Exception):
            DriverConfig().speed_rpm = 20


class NumberChecks(unittest.TestCase):
    def test_finite(self):
        self.assertEqual(finite(3, "x"), 3.0)
        for bad in (float("nan"), float("inf"), True, "1", None):
            with self.assertRaises(ValueError):
                finite(bad, "x")

    def test_timeout(self):
        self.assertEqual(timeout_ok(0.1), 0.1)
        for bad in (0, -1, 1.5, float("nan")):
            with self.assertRaises(ValueError):
                timeout_ok(bad)

    def test_pose_is_a_value(self):
        self.assertEqual(Pose(1, 2), Pose(1.0, 2.0))


if __name__ == "__main__":
    unittest.main()
