"""Behaviour every Driver must show; run against FakeDriver and the real Driver."""

from neck.driver import DriverError, DriverState, Pose

INITIAL = Pose(12.3, -4.5)


class DriverContract:
    """Mixin for unittest.TestCase. Subclasses provide make(): an OPEN driver whose
    motors currently sit at INITIAL, and register cleanup themselves."""

    def make(self):
        raise NotImplementedError

    def assertPose(self, pose, yaw, pitch):
        self.assertAlmostEqual(pose.yaw, yaw, places=1)
        self.assertAlmostEqual(pose.pitch, pitch, places=1)

    def test_open_reads_without_torque(self):
        d = self.make()
        self.assertIs(d.state, DriverState.OPEN)
        self.assertPose(d.read(0.1), INITIAL.yaw, INITIAL.pitch)

    def test_set_is_refused_until_enabled(self):
        d = self.make()
        with self.assertRaises(DriverError):
            d.set(1, 2)
        self.assertIs(d.state, DriverState.OPEN)

    def test_enable_set_read_disable(self):
        d = self.make()
        d.enable(INITIAL.yaw, INITIAL.pitch)
        self.assertIs(d.state, DriverState.ARMED)
        d.set(20.0, 5.5)
        self.assertPose(d.read(0.1), 20.0, 5.5)
        d.set(None, -3.0)
        self.assertPose(d.read(0.1), 20.0, -3.0)
        d.set(-7.5, None)
        self.assertPose(d.read(0.1), -7.5, -3.0)
        d.disable()
        d.disable()
        self.assertIs(d.state, DriverState.OPEN)
        with self.assertRaises(DriverError):
            d.set(0, 0)

    def test_enable_twice_is_refused(self):
        d = self.make()
        d.enable(1, 1)
        with self.assertRaises(DriverError):
            d.enable(2, 2)
        self.assertIs(d.state, DriverState.ARMED)

    def test_bad_arguments_raise_value_error(self):
        d = self.make()
        for bad in (float("nan"), float("inf"), True, "1", None):
            with self.assertRaises(ValueError):
                d.enable(bad, 0)
        d.enable(0, 0)
        for bad in (float("nan"), float("inf"), True, "1"):
            with self.assertRaises(ValueError):
                d.set(bad, 0)
        for bad in (0, -1, float("nan"), True, 2):
            with self.assertRaises(ValueError):
                d.read(bad)
        self.assertIs(d.state, DriverState.ARMED)

    def test_close_disables_and_is_final(self):
        d = self.make()
        d.enable(0, 0)
        d.close()
        self.assertIs(d.state, DriverState.CLOSED)
        d.close()
        for call in (lambda: d.read(0.1), lambda: d.enable(0, 0), lambda: d.set(0, 0), d.disable):
            with self.assertRaises(DriverError):
                call()

    def test_zero_while_off_makes_the_held_pose_zero_and_save_is_allowed(self):
        d = self.make()
        d.set_zero()
        self.assertPose(d.read(0.1), 0.0, 0.0)
        d.save()
        self.assertIs(d.state, DriverState.OPEN)

    def test_zero_while_armed_keeps_torque_and_aims_both_motors_at_the_new_zero(self):
        d = self.make()
        d.enable(INITIAL.yaw, INITIAL.pitch)
        d.set_zero()
        self.assertIs(d.state, DriverState.ARMED)
        self.assertPose(d.read(0.1), 0.0, 0.0)
        d.set(5.0, None)
        self.assertPose(d.read(0.1), 5.0, 0.0)
        d.save()
        self.assertIs(d.state, DriverState.ARMED)

    def test_zero_and_save_need_an_open_port(self):
        d = self.make()
        d.close()
        for call in (d.set_zero, d.save):
            with self.assertRaises(DriverError):
                call()
