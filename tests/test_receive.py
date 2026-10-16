"""Real binding/native receive path; only os.openpty devices are opened."""
from contextlib import contextmanager
import os
from pathlib import Path
import select
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from f32c import Config, F32CError, Gimbal2Dof


@contextmanager
def emulator(*, echo=False, fragmented=False, noise=False, corrupt=False):
    master, slave = os.openpty()
    stop = threading.Event()
    frames, errors = [], []
    settings = {"drop": False}
    def run():
        buffered = bytearray()
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .01)[0]: continue
                buffered.extend(os.read(master, 1024))
                while len(buffered) >= 3:
                    size = 6 if buffered[2] == 14 else 5 if buffered[2] in (5, 6, 8, 10) else 7
                    if len(buffered) < size: break
                    frame = bytes(buffered[:size])
                    del buffered[:size]
                    frames.append(frame)
                    if frame[2] != 14 or settings["drop"]: continue
                    payload = bytes([0x7A, frame[1], 2, 0, 0, 0, 123])
                    checksum = 0
                    for byte in payload: checksum ^= byte
                    reply = payload + bytes([checksum, 0x7B])
                    prefix = frame if echo else b""
                    if noise:
                        other = bytes([0x7A, 99, 2, 0, 0, 0, 123, 0x7A ^ 99 ^ 2 ^ 123, 0x7B])
                        prefix += b"\x00\xFF" + other
                    if corrupt: prefix += reply[:-2] + bytes([checksum ^ 1, 0x7B])
                    if fragmented:
                        os.write(master, prefix + reply[:4])
                        time.sleep(.008)
                        os.write(master, reply[4:])
                    else:
                        os.write(master, prefix + reply)
        except Exception as exc:
            errors.append(exc)
    worker = threading.Thread(target=run)
    worker.start()
    try:
        yield os.ttyname(slave), frames, settings
    finally:
        stop.set()
        worker.join(1)
        os.close(master)
        os.close(slave)
        if errors: raise errors[0]


class ReceiveTests(unittest.TestCase):
    def test_complete_fragmented_echoed_and_unrelated_frames(self):
        scenarios = [({}, (1, 2)), ({}, (120, 121)), ({"echo": True}, (1, 2)),
                     ({"fragmented": True}, (1, 2)), ({"noise": True}, (1, 2)),
                     ({"corrupt": True}, (1, 2)),
                     ({"echo": True, "fragmented": True, "noise": True}, (120, 121))]
        for options, ids in scenarios:
            with self.subTest(options=options, ids=ids), emulator(**options) as (device, frames, _):
                with Gimbal2Dof(Config(device=device, id_yaw=ids[0], id_pitch=ids[1])) as motor:
                    state = motor.read(timeout_s=.05)
                    self.assertEqual((state.yaw_deg, state.pitch_deg), (12.3, 12.3))
                time.sleep(.01)
                self.assertTrue(frames)
                self.assertTrue(all(frame[2] == 14 for frame in frames), "passive close sent a command")

    def test_read_deadline_and_invalid_timeout(self):
        with emulator() as (device, frames, settings):
            with Gimbal2Dof(Config(device=device)) as motor:
                for timeout in (0, -1, 2, True, float("nan")):
                    with self.assertRaises(ValueError): motor.read(timeout_s=timeout)
                settings["drop"] = True
                started = time.monotonic()
                with self.assertRaises(F32CError): motor.read(timeout_s=.02)
                self.assertLess(time.monotonic() - started, .15)
            self.assertFalse(any(frame[2] == 5 for frame in frames))


if __name__ == "__main__": unittest.main()
