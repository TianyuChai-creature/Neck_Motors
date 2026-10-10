"""Exercise the real Python/C/C++ PI path with a pseudo-terminal motor emulator."""

import os
from pathlib import Path
import select
import sys
import threading
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from f32c import Config, F32CError, Gimbal2Dof, HeadTracker, TrackingConfig


class RuntimePiTests(unittest.TestCase):
    def test_runtime_pi_preserves_target_mode_and_enable_state(self):
        master, slave = os.openpty()
        frames = []
        errors = []
        condition = threading.Condition()
        stop = threading.Event()

        def emulate():
            buffered = bytearray()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], .05)[0]:
                        continue
                    buffered.extend(os.read(master, 128))
                    while len(buffered) >= 3:
                        size = 6 if buffered[2] == 0x0E else 5 if buffered[2] in (5, 6, 8, 10) else 7
                        if len(buffered) < size:
                            break
                        frame = bytes(buffered[:size])
                        del buffered[:size]
                        checksum = 0
                        for value in frame[:-2]:
                            checksum ^= value
                        assert frame[0] == 0x7A and frame[-1] == 0x7B and checksum == frame[-2]
                        with condition:
                            frames.append(frame)
                            condition.notify_all()
                        if frame[2] == 0x0E:
                            address = frame[1]
                            os.write(master, bytes([0x7A, address, 2, 0, 0, 0, 0, 0x7A ^ address ^ 2, 0x7B]))
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=emulate)
        worker.start()
        motor = None
        try:
            motor = Gimbal2Dof(Config(device=os.ttyname(slave))).open()
            motor.enable()
            motor.set_deg(5, -5)
            with condition:
                self.assertTrue(condition.wait_for(
                    lambda: any(f[1:3] == bytes([2, 3]) and int.from_bytes(f[3:-2], "big") == 3550 for f in frames), timeout=1))
                frames.clear()
            motor.set_pi("yaw", 50, 8, 63, 0)
            with condition:
                self.assertTrue(condition.wait_for(lambda: len(frames) >= 4, timeout=1))
                observed = list(frames)
            self.assertEqual([(f[1], f[2], int.from_bytes(f[3:-2], "big")) for f in observed],
                             [(1, 15, 50), (1, 16, 8), (1, 17, 63), (1, 18, 0)])
            self.assertEqual(motor.config.yaw_pos_kp, 63)
            self.assertEqual(Config().yaw_pos_kp, 42)
            for axis, gains in (("roll", (1, 0, 1, 0)), ("yaw", (257, 0, 1, 0)),
                                ("pitch", (True, 0, 1, 0)), ("yaw", (2**32 + 50, 0, 1, 0))):
                with self.assertRaises(ValueError):
                    motor.set_pi(axis, *gains)
            self.assertEqual(len(frames), 4)
            # Optional target conditioning feeds the unchanged native setter;
            # it must emit target frames only, without rewriting PI or mode.
            tracker = HeadTracker(motor.read(), TrackingConfig(pitch_limits=(-30, 40)), timestamp=0)
            with condition:
                self.assertTrue(condition.wait_for(lambda: len(frames) >= 6, timeout=1))
                frames.clear()
            expected = []
            for i in range(1, 11):
                now = i * .02
                tracker.set_target(3, -3, timestamp=now)
                command = tracker.step(timestamp=now)
                motor.set_deg(command.yaw_deg, command.pitch_deg)
                expected.extend([(1, 3, int(command.yaw_deg * 10) % 3600),
                                 (2, 3, int(command.pitch_deg * 10) % 3600)])
            with condition:
                self.assertTrue(condition.wait_for(lambda: len(frames) >= len(expected), timeout=1))
                observed = list(frames)
            self.assertEqual([(f[1], f[2], int.from_bytes(f[3:-2], "big")) for f in observed], expected)
            motor.disable()
            with self.assertRaises(F32CError):
                motor.set_pi("yaw", 50, 8, 63, 0)
            with condition:
                frames.clear()
            motor.enable()
            self.assertTrue(any(f[1:3] == bytes([1, 17]) and int.from_bytes(f[3:-2], "big") == 63 for f in frames))
            self.assertFalse(errors)
        finally:
            if motor is not None:
                motor.close()
            stop.set()
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)


if __name__ == "__main__":
    unittest.main()
