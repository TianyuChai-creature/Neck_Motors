"""Neck on the real Driver over a fake bus: the two layers working together."""
import time
import unittest

from neck.control import Neck, NeckError
from neck.driver import Driver, DriverConfig, DriverState
from tests.bus import Bus


class StackTests(unittest.TestCase):
    def setUp(self):
        self.bus = Bus({1: 12.3, 2: -4.5})
        self.addCleanup(self.bus.stop)

    def run_until(self, neck, done, seconds=4.0):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            tick_started = time.monotonic()
            snapshot = neck.tick()
            if done(snapshot):
                return snapshot
            time.sleep(max(0.0, 0.02 - (time.monotonic() - tick_started)))
        self.fail(f"not reached; last phase {neck.snapshot.phase} {neck.snapshot.reason}")

    def test_enable_glides_to_zero_follows_a_target_and_closes_disabled(self):
        driver = Driver.open(DriverConfig(port=self.bus.path))
        neck = Neck(driver)
        snapshot = neck.enable()
        self.assertEqual(snapshot.phase, "moving")
        self.assertTrue(all(axis.enabled for axis in self.bus.axes.values()))
        self.run_until(neck, lambda s: s.reason == "target_reached")
        self.assertAlmostEqual(self.bus.axes[1].deg, 0, delta=0.2)
        self.assertAlmostEqual(self.bus.axes[2].deg, 0, delta=0.2)

        for _ in range(100):
            neck.set_target(20, 10)
            snapshot = neck.tick()
            time.sleep(0.02)
        self.assertEqual((snapshot.phase, snapshot.level), ("following", 0))
        self.assertAlmostEqual(self.bus.axes[1].deg, 20, delta=1)
        self.assertAlmostEqual(self.bus.axes[2].deg, 10, delta=1)

        neck.close()
        self.assertIs(driver.state, DriverState.CLOSED)
        self.assertFalse(any(axis.enabled for axis in self.bus.axes.values()))
        self.assertEqual(self.bus.errors, [])

    def test_a_dead_bus_ends_in_a_fault_and_a_disable_attempt(self):
        driver = Driver.open(DriverConfig(port=self.bus.path))
        neck = Neck(driver)
        neck.enable()
        self.bus.silent.update({1, 2})
        with self.assertRaisesRegex(NeckError, "feedback_stale"):
            self.run_until(neck, lambda s: False, seconds=3)
        self.assertEqual(neck.snapshot.phase, "fault")
        self.assertIs(driver.state, DriverState.OPEN)       # the disable requests did go out
        self.assertFalse(any(axis.enabled for axis in self.bus.axes.values()))
        neck.close()


if __name__ == "__main__":
    unittest.main()
