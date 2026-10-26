import glob
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from neck.driver import Driver, DriverConfig, DriverError, DriverState, Gains
from neck.driver.port import Port
from tests.bus import Bus
from tests.driver_contract import INITIAL, DriverContract


def frame_kinds(bus):
    return [(f[1], f[2]) for f in bus.frames]


class DeviceCase(unittest.TestCase):
    def setUp(self):
        self.bus = Bus({1: INITIAL.yaw, 2: INITIAL.pitch})
        self.addCleanup(self.bus.stop)

    def driver(self, **config) -> Driver:
        d = Driver.open(DriverConfig(port=self.bus.path, **config))
        self.addCleanup(self._release, d)
        return d

    @staticmethod
    def _release(d):
        try:
            d.close()
        except DriverError:
            pass

    def failing_writes(self, wrong):
        """Make Port.write fail for the frames `wrong(frame)` selects."""
        real = Port.write

        def write(port, data, deadline):
            if wrong(data):
                raise DriverError("injected write failure")
            real(port, data, deadline)
        return patch.object(Port, "write", write)


class RealDriverContract(DriverContract, DeviceCase):
    def make(self):
        return self.driver()


class EnableTests(DeviceCase):
    def test_enable_order_gains_before_torque_then_each_axis_in_full(self):
        d = self.driver(speed_rpm=20, accel=3, yaw_gains=Gains(1, 2, 3, 4), pitch_gains=Gains(5, 6, 7, 8))
        d.enable(10.0, -5.0)
        time.sleep(0.02)
        self.assertEqual(frame_kinds(self.bus), [
            (1, 5), (2, 5),
            (1, 0x0F), (1, 0x10), (1, 0x11), (1, 0x12), (2, 0x0F), (2, 0x10), (2, 0x11), (2, 0x12),
            (1, 6), (1, 0), (1, 1), (1, 3), (1, 7),
            (2, 6), (2, 0), (2, 1), (2, 3), (2, 7)])
        yaw, pitch = self.bus.axes[1], self.bus.axes[2]
        self.assertEqual(yaw.gains, dict(speed_kp=1, speed_ki=2, pos_kp=3, pos_ki=4))
        self.assertEqual(pitch.gains, dict(speed_kp=5, speed_ki=6, pos_kp=7, pos_ki=8))
        for axis, deg in ((yaw, 10.0), (pitch, -5.0)):
            self.assertEqual((axis.enabled, axis.mode, axis.rpm, axis.accel, axis.deg), (True, 4, 20, 3, deg))

    def test_failed_enable_rolls_back_to_disabled(self):
        d = self.driver()
        with self.failing_writes(lambda f: f[1] == 2 and f[2] == 6):
            with self.assertRaises(DriverError):
                d.enable(0, 0)
        self.assertIs(d.state, DriverState.OPEN)
        self.assertEqual([f[1] for f in self.bus.commands(5)][-2:], [1, 2])
        self.assertFalse(any(axis.enabled for axis in self.bus.axes.values()))

    def test_failed_enable_and_failed_rollback_leaves_state_unknown(self):
        d = self.driver()
        calls = []
        real = Port.write

        def write(port, data, deadline):
            calls.append(data)
            if data[2] == 6 or (data[2] == 5 and len([c for c in calls if c[2] == 5]) > 2):
                raise DriverError("injected")
            real(port, data, deadline)

        with patch.object(Port, "write", write):
            with self.assertRaises(DriverError):
                d.enable(0, 0)
        self.assertIs(d.state, DriverState.UNKNOWN)
        with self.assertRaises(DriverError):
            d.enable(0, 0)          # refused until a disable succeeds
        d.disable()
        self.assertIs(d.state, DriverState.OPEN)


class FailureTests(DeviceCase):
    def test_disable_failure_on_one_axis_still_tries_the_other_and_can_be_retried(self):
        d = self.driver()
        d.enable(0, 0)
        with self.failing_writes(lambda f: f[1] == 1 and f[2] == 5):
            with self.assertRaises(DriverError):
                d.disable()
        self.assertIs(d.state, DriverState.UNKNOWN)
        self.assertFalse(self.bus.axes[2].enabled)      # pitch got its request anyway
        self.assertTrue(self.bus.axes[1].enabled)
        d.read(0.1)                                      # reading is still possible
        d.disable()
        self.assertIs(d.state, DriverState.OPEN)
        self.assertFalse(self.bus.axes[1].enabled)

    def test_close_after_failed_disable_releases_port_but_stays_unknown(self):
        d = self.driver()
        d.enable(0, 0)
        with self.failing_writes(lambda f: f[2] == 5):
            with self.assertRaises(DriverError):
                d.close()
        self.assertIs(d.state, DriverState.UNKNOWN)
        with self.assertRaises(DriverError):
            d.read(0.1)
        with self.assertRaises(DriverError):
            d.close()
        self.assertIs(d.state, DriverState.UNKNOWN)

    def test_close_retries_a_pending_disable_and_then_reports_closed(self):
        d = self.driver()
        d.enable(0, 0)
        with self.failing_writes(lambda f: f[2] == 5):
            with self.assertRaises(DriverError):
                d.disable()
        d.close()
        self.assertIs(d.state, DriverState.CLOSED)
        self.assertFalse(any(axis.enabled for axis in self.bus.axes.values()))

    def test_set_write_failure_keeps_torque_state_and_disable_still_works(self):
        d = self.driver()
        d.enable(0, 0)
        with self.failing_writes(lambda f: f[2] == 3):
            with self.assertRaises(DriverError):
                d.set(5, 5)
        self.assertIs(d.state, DriverState.ARMED)
        d.disable()
        self.assertIs(d.state, DriverState.OPEN)

    def test_read_fails_inside_its_deadline_and_keeps_state(self):
        d = self.driver()
        d.enable(0, 0)
        self.bus.silent.add(2)
        started = time.monotonic()
        with self.assertRaises(DriverError):
            d.read(0.06)
        self.assertLess(time.monotonic() - started, 0.2)
        self.assertIs(d.state, DriverState.ARMED)
        self.bus.silent.clear()
        d.read(0.1)

    def test_passive_session_never_sends_a_command(self):
        d = self.driver()
        d.read(0.1)
        d.close()
        time.sleep(0.01)
        self.assertTrue(self.bus.frames)
        self.assertTrue(all(f[2] == 0x0E for f in self.bus.frames))

    def test_close_while_armed_disables_both_motors(self):
        d = self.driver()
        d.enable(0, 0)
        d.close()
        time.sleep(0.01)
        self.assertEqual([f[1] for f in self.bus.commands(5)][-2:], [1, 2])
        self.assertIs(d.state, DriverState.CLOSED)

    def test_works_through_echo_noise_fragments_and_high_ids(self):
        bus = Bus({120: 3.0, 121: -3.0}, echo=True, fragmented=True, noise=True)
        self.addCleanup(bus.stop)
        d = Driver.open(DriverConfig(port=bus.path, yaw_id=120, pitch_id=121))
        self.addCleanup(d.close)
        self.assertEqual((d.read(0.1).yaw, d.read(0.1).pitch), (3.0, -3.0))

    def test_serial_log_is_written(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder, "serial.log")
            d = self.driver(log_path=str(log))
            d.read(0.1)
            d.close()
            text = log.read_text()
        self.assertIn("TX 6B 7A 01 0E 02 77 7B", text)
        self.assertIn("LOG stopped", text)


class OpenTests(unittest.TestCase):
    def test_missing_explicit_port_is_an_error_and_never_scans(self):
        with patch("neck.driver.device.candidates", side_effect=AssertionError("scanned")):
            with self.assertRaises(DriverError):
                Driver.open(DriverConfig(port="/dev/does-not-exist"))

    def test_scan_picks_the_port_where_both_motors_answer(self):
        only_yaw, both = Bus({1: 1.0}), Bus({1: 5.0, 2: 6.0})
        for bus in (only_yaw, both):
            self.addCleanup(bus.stop)
        found = ["/dev/does-not-exist", only_yaw.path, both.path]
        with patch("neck.driver.device.candidates", return_value=found):
            d = Driver.open()
        self.addCleanup(d.close)
        self.assertEqual(d.read(0.1).pitch, 6.0)

    def test_scan_without_an_answering_device_fails(self):
        silent = Bus({1: 0.0, 2: 0.0})
        self.addCleanup(silent.stop)
        silent.silent.update({1, 2})
        with patch("neck.driver.device.candidates", return_value=[silent.path]):
            with self.assertRaises(DriverError):
                Driver.open()

    def test_candidates_list_each_physical_device_once(self):
        with tempfile.TemporaryDirectory() as folder:
            real = Path(folder, "ttyACM0")
            real.touch()
            link = Path(folder, "by-id-usb")
            link.symlink_to(real)

            def fake_glob(pattern):
                return {"/dev/serial/by-id/*": [str(link)], "/dev/ttyACM*": [str(real)],
                        "/dev/ttyUSB*": []}[pattern]

            with patch.object(glob, "glob", fake_glob):
                from neck.driver.device import candidates
                self.assertEqual(candidates(), [str(link)])


if __name__ == "__main__":
    unittest.main()
