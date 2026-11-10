"""Focused checks for the simplified neck core."""

from __future__ import annotations

import time
import unittest

from neck import Neck, NeckConfig, Pose, SerialLink
from neck import protocol as p
from tests.bus import Bus
from tests.fake_link import FakeLink


def reply(addr: int, tenths: int) -> bytes:
    body = bytes((0x7A, addr, 2)) + tenths.to_bytes(4, "big", signed=True)
    return body + bytes((p.bcc(body), 0x7B))


class ProtocolTests(unittest.TestCase):
    def test_golden_frames(self):
        golden = {
            "enable": (p.enable(2), "7A 02 06 7E 7B"),
            "disable": (p.disable(2), "7A 02 05 7D 7B"),
            "save": (p.save(2), "7A 02 08 70 7B"),
            "zero": (p.set_zero(2), "7A 02 0A 72 7B"),
            "direct": (p.mode_direct(2), "7A 02 00 00 04 7C 7B"),
            "100 rpm": (p.speed_rpm(2, 100), "7A 02 01 00 64 1D 7B"),
            "150 deg": (p.angle(2, 150.0), "7A 02 03 05 DC A2 7B"),
            "query": (p.query_angle(2), "7A 02 0E 02 74 7B"),
        }
        for name, (got, want) in golden.items():
            with self.subTest(name):
                self.assertEqual(got, bytes.fromhex(want))

    def test_angle_wraps_and_rounds(self):
        raw = lambda deg: int.from_bytes(p.angle(1, deg)[3:5], "big")
        self.assertEqual(raw(-0.1), 3599)
        self.assertEqual(raw(-12.3), 3477)
        self.assertEqual(raw(5.8), 58)
        self.assertEqual(raw(359.96), 0)

    def test_parse_consumes_complete_frames_only(self):
        stream = bytearray(reply(1, 123) + reply(2, 50) + reply(1, 456)[:4])
        self.assertEqual(p.parse_angle_frames(stream), [(1, 12.3), (2, 5.0)])
        self.assertEqual(len(stream), 4)
        stream += reply(1, 456)[4:]
        self.assertEqual(p.parse_angle_frames(stream), [(1, 45.6)])
        self.assertEqual(len(stream), 0)

    def test_parse_skips_noise_and_bad_bcc(self):
        bad = bytearray(reply(1, 999))
        bad[7] ^= 1
        stream = bytearray(b"\x00\xff" + bytes(bad) + reply(1, 123))
        self.assertEqual(p.parse_angle_frames(stream), [(1, 12.3)])


class ControllerTests(unittest.TestCase):
    def make(self, pose=Pose(0, 0), **kwargs):
        config = NeckConfig(control_period=0.01, **kwargs)
        link = FakeLink(pose)
        return Neck(config, link), link

    def test_enable_holds_measured_pose_and_sends_start_frames(self):
        neck, link = self.make(Pose(30, 40))
        result = neck.enable()
        self.assertTrue(result.ok)
        self.assertTrue(neck.enabled)
        self.assertEqual(neck.status.command, Pose(30, 40))
        self.assertEqual(neck.status.target, Pose(30, 40))
        self.assertEqual(link.enabled, {1, 2})
        funcs = [frame[2] for frame in link.frames]
        self.assertIn(0x06, funcs)
        self.assertIn(0x00, funcs)
        self.assertIn(0x03, funcs)
        self.assertEqual(link.zeros, 0)
        self.assertEqual(link.saves, 0)

    def test_default_config_is_the_bench_setup(self):
        config = NeckConfig()
        self.assertEqual((config.yaw_limits, config.pitch_limits), ((-60, 60), (-45, 43)))
        self.assertEqual((config.yaw_speed_rpm, config.pitch_speed_rpm), (45, 15))
        self.assertEqual((config.yaw_accel, config.pitch_accel), (2, 2))
        self.assertEqual((config.max_speed, config.max_accel), (90, 1440))

    def test_enable_sends_per_axis_motor_speed_and_accel(self):
        neck, link = self.make()
        neck.enable()
        rpm = {frame[1]: int.from_bytes(frame[3:5], "big")
               for frame in link.frames if frame[2] == 0x01}
        accel = {frame[1]: int.from_bytes(frame[3:5], "big")
                 for frame in link.frames if frame[2] == 0x07}
        self.assertEqual(rpm, {1: 45, 2: 15})
        self.assertEqual(accel, {1: 2, 2: 2})

    def test_target_is_clipped_and_ramps_to_the_limit(self):
        neck, link = self.make(max_lead=180)
        neck.enable()
        self.assertEqual(neck.set_target(100, 100), Pose(60, 43))
        now = time.monotonic()
        for step in range(1, 150):
            neck.set_target(100, 100, at=now + step * 0.01)
            neck.update(now + step * 0.01)
        self.assertAlmostEqual(neck.status.command.yaw, 60, delta=0.5)
        self.assertAlmostEqual(neck.status.command.pitch, 43, delta=0.5)
        self.assertTrue(any(frame[2] == 0x03 for frame in link.frames))

    def test_speed_and_accel_limits_can_be_disabled(self):
        neck, link = self.make(max_speed=None, max_accel=None, max_lead=None)
        neck.enable()
        neck.set_target(30, 0)
        now = time.monotonic()
        for step in range(1, 60):
            neck.set_target(30, 0, at=now + step * 0.01)
            neck.update(now + step * 0.01)
        self.assertAlmostEqual(neck.status.command.yaw, 30, delta=0.5)
        self.assertLessEqual(neck.status.command.yaw, 30.5)

    def test_input_idle_freezes_and_new_target_resumes(self):
        neck, link = self.make(max_lead=180)
        neck.enable()
        neck.set_target(20, 0)
        now = time.monotonic()
        neck.update(now)
        neck._input_at = now - 1.0
        neck.update(now + 0.01)
        frozen = neck.status.command
        self.assertEqual(neck.status.reason, "input_idle")
        neck.update(now + 0.02)
        self.assertEqual(neck.status.command, frozen)
        neck.set_target(10, 0)
        neck.update(now + 0.03)
        self.assertNotEqual(neck.status.reason, "input_idle")

    def test_feedback_loss_holds_instead_of_disabling(self):
        neck, link = self.make(max_lead=180)
        neck.enable()
        neck.set_target(20, 0)
        now = time.monotonic()
        link.feedback = False
        link.angle_at = {1: now - 1.0, 2: now - 1.0}
        neck.update(now)
        self.assertTrue(neck.enabled)
        self.assertEqual(neck.status.reason, "feedback_stale")
        frozen = neck.status.command
        neck.update(now + 0.02)
        self.assertEqual(neck.status.command, frozen)
        link.feedback = True
        link.angle_at = {1: now + 0.02, 2: now + 0.02}
        neck.update(now + 0.03)
        self.assertTrue(neck.status.link_ok)
        self.assertNotEqual(neck.status.reason, "feedback_stale")

    def test_link_loss_can_be_configured_to_disable(self):
        neck, link = self.make(max_lead=180, on_link_loss="disable", disable_after=0.01)
        neck.enable()
        now = time.monotonic()
        link.feedback = False
        link.angle_at = {1: now - 1.0, 2: now - 1.0}
        neck.update(now)
        self.assertFalse(neck.enabled)
        self.assertIn(neck.status.torque, ("off", "unknown"))
        self.assertTrue(any(frame[2] == 0x05 for frame in link.frames))

    def test_zero_while_enabled_zeroes_verifies_and_saves(self):
        neck, link = self.make(Pose(10, 5))
        neck.enable()
        result = neck.zero(save=True)
        self.assertTrue(result.ok)
        self.assertTrue(result.zeroed and result.verified and result.saved)
        self.assertEqual(link.zeros, 2)
        self.assertEqual(link.saves, 2)
        self.assertEqual(neck.status.command, Pose(0, 0))
        self.assertEqual(neck.status.target, Pose(0, 0))
        self.assertEqual(link.angles, {1: 0.0, 2: 0.0})

    def test_zero_refuses_when_not_settled(self):
        neck, link = self.make(Pose(10, 5))
        neck.enable()
        link.angles = {1: 15.0, 2: 5.0}
        result = neck.zero()
        self.assertFalse(result.ok)
        self.assertFalse(result.zeroed)
        self.assertEqual(link.zeros, 0)
        self.assertTrue(neck.enabled)

    def test_zero_save_failure_leaves_the_new_zero_held(self):
        neck, link = self.make(Pose(10, 5))
        neck.enable()
        link.fail_save = True
        result = neck.zero()
        self.assertFalse(result.ok)
        self.assertTrue(result.zeroed and result.verified)
        self.assertFalse(result.saved)
        self.assertEqual(neck.status.command, Pose(0, 0))
        self.assertEqual(neck.status.target, Pose(0, 0))
        self.assertTrue(neck.enabled)

    def test_zero_write_failure_while_enabled_attempts_disable(self):
        neck, link = self.make(Pose(10, 5))
        neck.enable()
        link.fail_write = True
        result = neck.zero()
        self.assertFalse(result.ok)
        self.assertFalse(neck.enabled)
        self.assertEqual(neck.status.torque, "unknown")

    def test_disable_failure_is_reported_as_unknown(self):
        neck, link = self.make()
        neck.enable()
        link.fail_write = True
        result = neck.disable()
        self.assertFalse(result.ok)
        self.assertEqual(result.torque, "unknown")
        self.assertFalse(neck.enabled)

    def test_close_disables_and_closes(self):
        neck, link = self.make()
        neck.enable()
        result = neck.close()
        self.assertTrue(result.ok)
        self.assertTrue(link.closed)
        self.assertFalse(neck.enabled)


class LinkTests(unittest.TestCase):
    def test_link_reads_and_writes_through_a_pty_bus(self):
        with Bus({1: 1.0, 2: -2.0}) as bus:
            link = SerialLink.open(NeckConfig(port=bus.path, feedback_timeout=0.5))
            try:
                self.assertAlmostEqual(link.read_angle(1), 1.0)
                self.assertAlmostEqual(link.read_angle(2), -2.0)
                link.write_frame(p.enable(1))
                link.queue_angle(1, 10.0)
                link.pump()
                time.sleep(0.02)
                self.assertAlmostEqual(bus.axes[1].deg, 10.0)
                self.assertAlmostEqual(link.read_angle(1), 10.0)
                self.assertFalse(bus.errors)
            finally:
                link.close()

    def test_thread_starts_and_stops(self):
        with Bus({1: 0.0, 2: 0.0}) as bus:
            neck = Neck(NeckConfig(port=bus.path, feedback_timeout=0.5,
                                   input_timeout=0.5, max_lead=180))
            try:
                neck.start()
                time.sleep(0.05)
                self.assertTrue(neck.status.link_ok, neck.status)
            finally:
                result = neck.close()
            self.assertTrue(result.ok)
            self.assertFalse(neck.enabled)

    def test_neck_follows_on_the_real_link(self):
        with Bus({1: 0.0, 2: 0.0}, lag=0.02) as bus:
            config = NeckConfig(port=bus.path, feedback_timeout=0.5,
                                input_timeout=0.5, max_lead=180, control_period=0.005)
            neck = Neck(config)
            try:
                self.assertTrue(neck.enable().ok)
                neck.set_target(5.0, 5.0)
                for _ in range(80):
                    neck.update()
                    time.sleep(0.005)
                status = neck.status
                self.assertTrue(status.link_ok, status)
                self.assertAlmostEqual(status.command.yaw, 5.0, delta=0.8)
                self.assertAlmostEqual(status.command.pitch, 5.0, delta=0.8)
                self.assertFalse(bus.errors)
            finally:
                neck.close()


if __name__ == "__main__":
    unittest.main()
