import unittest

from neck.driver import protocol as p


def h(text: str) -> bytes:
    return bytes.fromhex(text)


def reply(addr: int, tenths: int) -> bytes:
    body = bytes((0x7A, addr, 2)) + tenths.to_bytes(4, "big", signed=True)
    return body + bytes((p.bcc(body), 0x7B))


class FrameTests(unittest.TestCase):
    def test_golden_frames(self):
        golden = {
            "enable": (p.enable(2), "7A 02 06 7E 7B"),
            "disable": (p.disable(2), "7A 02 05 7D 7B"),
            "mode direct": (p.mode_single_turn_direct(2), "7A 02 00 00 04 7C 7B"),
            "speed kp": (p.speed_kp(2, 10), "7A 02 0F 00 0A 7D 7B"),
            "speed ki": (p.speed_ki(2, 10), "7A 02 10 00 0A 62 7B"),
            "pos kp": (p.pos_kp(2, 10), "7A 02 11 00 0A 63 7B"),
            "pos ki": (p.pos_ki(2, 10), "7A 02 12 00 0A 60 7B"),
            "100 rpm": (p.speed_rpm(2, 100), "7A 02 01 00 64 1D 7B"),
            "accel 100": (p.accel(2, 100), "7A 02 07 00 64 1B 7B"),
            "150 deg": (p.angle(2, 150.0), "7A 02 03 05 DC A2 7B"),
            "0 deg": (p.angle(2, 0.0), "7A 02 03 00 00 7B 7B"),
            "huge angle": (p.angle(2, 3600000000.0), "7A 02 03 00 00 7B 7B"),
            "query id2": (p.query_angle(2), "7A 02 0E 02 74 7B"),
            "query id1": (p.query_angle(1), "7A 01 0E 02 77 7B"),
        }
        for name, (got, want) in golden.items():
            with self.subTest(name):
                self.assertEqual(got, h(want))

    def test_angle_wraps_negative_and_rounds_to_a_tenth(self):
        value = lambda deg: int.from_bytes(p.angle(1, deg)[3:5], "big")
        self.assertEqual(value(-0.1), 3599)
        self.assertEqual(value(-12.3), 3477)
        self.assertEqual(value(5.8), 58)       # 5.8 * 10 is 57.999... in floating point
        self.assertEqual(value(359.96), 0)
        self.assertEqual(value(-0.04), 0)
        self.assertEqual(value(725.0), 50)


class FindAngleTests(unittest.TestCase):
    def test_reads_tenths_and_wraps_to_signed(self):
        self.assertEqual(p.find_angle(h("7A 01 02 00 00 05 C8 B4 7B"), 1), 148.0)
        self.assertEqual(p.find_angle(reply(2, 3477), 2), -12.3)    # motor says 347.7
        self.assertEqual(p.find_angle(reply(2, 3599), 2), -0.1)
        self.assertEqual(p.find_angle(reply(2, 1799), 2), 179.9)
        self.assertEqual(p.find_angle(reply(2, 1800), 2), -180.0)
        self.assertEqual(p.find_angle(reply(2, 0), 2), 0.0)

    def test_skips_noise_echo_other_motors_and_corrupt_frames(self):
        corrupt = bytearray(reply(1, 999))
        corrupt[7] ^= 1
        stream = b"\x00\xff" + p.query_angle(1) + reply(2, 50) + bytes(corrupt) + b"\x7a" + reply(1, 123)
        self.assertEqual(p.find_angle(stream, 1), 12.3)
        self.assertEqual(p.find_angle(stream, 2), 5.0)

    def test_rejects_incomplete_wrong_function_and_bad_tail(self):
        good = reply(1, 100)
        self.assertIsNone(p.find_angle(good[:-1], 1))
        self.assertIsNone(p.find_angle(good[:2] + b"\x03" + good[3:], 1))
        self.assertIsNone(p.find_angle(good[:-1] + b"\x00", 1))
        self.assertIsNone(p.find_angle(b"", 1))

    def test_high_ids_and_payload_containing_tail_byte(self):
        self.assertEqual(p.find_angle(reply(120, 0x0B7B), 120), -66.1)   # 7B inside the payload
        self.assertEqual(p.find_angle(reply(121, 0x0123), 121), 29.1)


if __name__ == "__main__":
    unittest.main()
