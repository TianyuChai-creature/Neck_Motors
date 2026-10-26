"""The replay generator's decoding and template, without running the 13 s scenario."""
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

from neck.driver import protocol as p
from tools import replay


class ReplayTests(unittest.TestCase):
    def test_command_frames_are_described_in_plain_language(self):
        cases = [(p.set_zero(2), "当前位置写成 0°", "pitch"), (p.save(1), "保存参数到 flash", "yaw"),
                 (p.enable(1), "使能", "yaw"), (p.speed_kp(2, 10), "速度 KP = 10", "pitch"),
                 (p.angle(1, -4.5), "目标角 -4.5°", "yaw"), (p.angle(2, 12.3), "目标角 12.3°", "pitch")]
        for frame, meaning, axis in cases:
            self.assertEqual(replay.decode("TX", frame), (meaning, axis))

    def test_feedback_shows_the_motor_value_and_the_fold(self):
        reply = p.pack(1, 0x02, (3555).to_bytes(4, "big"))          # 355.5 deg as the motor reports
        text, axis = replay.decode("RX", reply)
        self.assertEqual(axis, "yaw")
        self.assertIn("355.5", text)
        self.assertIn("-4.5", text)

    def test_log_lines_become_frames_with_relative_time(self):
        query, reply = p.query_angle(1), p.pack(1, 0x02, (0).to_bytes(4, "big"))
        hexed = lambda b: f"{len(b)}B " + " ".join(f"{x:02X}" for x in b)
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "s.log"
            log.write_text("2026-10-26 10:00:00.000 LOG started\n"
                           f"2026-10-26 10:00:01.500 TX {hexed(query)}\n"
                           f"2026-10-26 10:00:01.510 RX {hexed(reply)}\n")
            frames = replay.parse_log(log, datetime(2026, 10, 26, 10, 0, 1).timestamp())
        self.assertEqual([f["dir"] for f in frames], ["TX", "RX"])
        self.assertAlmostEqual(frames[0]["t"], 0.5, places=3)
        self.assertTrue(all(f["query"] for f in frames))

    def test_template_has_exactly_one_data_slot(self):
        self.assertEqual(replay.TEMPLATE.read_text().count("/*DATA*/null"), 1)


if __name__ == "__main__":
    unittest.main()
