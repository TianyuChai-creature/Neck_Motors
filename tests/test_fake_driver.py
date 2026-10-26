import unittest

from neck.driver import DriverError, DriverState, Pose
from tests.driver_contract import INITIAL, DriverContract
from tests.fakes import FakeDriver


class FakeDriverContract(DriverContract, unittest.TestCase):
    def make(self):
        return FakeDriver(INITIAL)


class FakeDriverFailures(unittest.TestCase):
    def test_failed_disable_leaves_state_unknown_even_after_close(self):
        d = FakeDriver()
        d.enable(0, 0)
        d.disable_error = True
        with self.assertRaises(DriverError):
            d.disable()
        self.assertIs(d.state, DriverState.UNKNOWN)
        with self.assertRaises(DriverError):
            d.close()
        self.assertIs(d.state, DriverState.UNKNOWN)

    def test_measured_holds_still_when_follow_is_off(self):
        d = FakeDriver(Pose(1, 1))
        d.enable(1, 1)
        d.follow = False
        d.set(9, 9)
        self.assertEqual(d.read(), Pose(1, 1))


if __name__ == "__main__":
    unittest.main()
