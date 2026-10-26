import time
import unittest

from neck.driver import DriverError, Gains
from neck.driver.motor import Motor
from neck.driver.port import Port
from tests.bus import Bus


class MotorTests(unittest.TestCase):
    def motor(self, addr=1, **bus_options):
        bus = Bus({addr: 12.3}, **bus_options)
        self.addCleanup(bus.stop)
        port = Port(bus.path, 115200)
        self.addCleanup(port.close)
        return Motor(port, addr), bus

    def test_commands_are_framed_in_order_with_idle_gaps(self):
        motor, bus = self.motor(2)
        motor.set_gains(Gains(10, 4, 40, 0))
        motor.enable()
        motor.start(15, 2, -7.5)
        motor.set_angle(20.0)
        motor.disable()
        time.sleep(0.02)
        self.assertEqual([f.hex(" ").upper() for f in bus.frames], [
            "7A 02 0F 00 0A 7D 7B", "7A 02 10 00 04 6C 7B", "7A 02 11 00 28 41 7B",
            "7A 02 12 00 00 6A 7B", "7A 02 06 7E 7B", "7A 02 00 00 04 7C 7B",
            "7A 02 01 00 0F 76 7B", "7A 02 03 0D C5 B3 7B", "7A 02 07 00 02 7D 7B",
            "7A 02 03 00 C8 B3 7B", "7A 02 05 7D 7B"])
        gaps = [b - a for a, b in zip(bus.times, bus.times[1:])]
        self.assertGreaterEqual(min(gaps), 0.007)
        self.assertEqual(bus.errors, [])

    def test_reads_through_echo_noise_fragments_and_corruption(self):
        for options in ({}, {"echo": True}, {"fragmented": True}, {"noise": True},
                        {"corrupt": True}, {"echo": True, "fragmented": True, "noise": True}):
            for addr in (1, 120):
                with self.subTest(options=options, addr=addr):
                    motor, bus = self.motor(addr, **options)
                    self.assertEqual(motor.read_angle(), 12.3)
                    self.assertTrue(all(f[2] == 0x0E for f in bus.frames))

    def test_silent_motor_fails_within_the_deadline(self):
        motor, bus = self.motor()
        bus.silent.add(1)
        started = time.monotonic()
        with self.assertRaises(DriverError):
            motor.read_angle(0.04)
        self.assertLess(time.monotonic() - started, 0.12)
        self.assertEqual(len(bus.commands(0x0E)), 2)    # first query plus one retry

    def test_a_late_reply_is_still_accepted_by_the_retry(self):
        motor, bus = self.motor(delay=0.03)
        self.assertEqual(motor.read_angle(0.1), 12.3)

    def test_noise_without_a_valid_frame_does_not_grow_without_bound(self):
        motor, bus = self.motor()
        bus.noise = bus.corrupt = True
        bus.silent.add(1)
        with self.assertRaises(DriverError):
            motor.read_angle(0.03)


if __name__ == "__main__":
    unittest.main()
