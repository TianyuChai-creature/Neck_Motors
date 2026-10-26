import errno
import os
from pathlib import Path
import re
import tempfile
import time
import unittest
from unittest.mock import patch

from neck.driver import DriverError
from neck.driver.port import Port


class PortTests(unittest.TestCase):
    def setUp(self):
        self.master, slave = os.openpty()
        self.path = os.ttyname(slave)
        self.slave = slave
        self.addCleanup(self._close_pty)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.log = os.path.join(self.folder.name, "serial.log")

    def _close_pty(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass

    def port(self, **kwargs):
        port = Port(self.path, 115200, **kwargs)
        self.addCleanup(port.close)
        return port

    def test_bytes_both_ways_and_read_deadline(self):
        port = self.port()
        port.write(b"\x7a\x01\x02", time.monotonic() + 1)
        self.assertEqual(os.read(self.master, 16), b"\x7a\x01\x02")
        started = time.monotonic()
        self.assertEqual(port.read(started + 0.03), b"")
        self.assertTrue(0.02 < time.monotonic() - started < 0.15)
        os.write(self.master, b"abc")
        self.assertEqual(port.read(time.monotonic() + 1), b"abc")

    def test_read_returns_as_soon_as_bytes_arrive(self):
        port = self.port()
        os.write(self.master, b"x")
        started = time.monotonic()
        self.assertEqual(port.read(started + 1), b"x")
        self.assertLess(time.monotonic() - started, 0.1)

    def test_flush_input_discards_pending_bytes(self):
        port = self.port()
        os.write(self.master, b"stale")
        time.sleep(0.02)
        port.flush_input()
        self.assertEqual(port.read(time.monotonic() + 0.02), b"")

    def test_write_survives_short_writes_and_eagain(self):
        port = self.port()
        real, calls = os.write, []

        def trickle(fd, data):
            calls.append(len(data))
            if len(calls) % 3 == 0:
                raise BlockingIOError(errno.EAGAIN, "again")
            return real(fd, data[:2])

        with patch("neck.driver.port.os.write", trickle):
            port.write(b"0123456789", time.monotonic() + 1)
        self.assertEqual(os.read(self.master, 64), b"0123456789")

    def test_write_deadline_when_the_line_is_full(self):
        port = self.port(log_path=self.log)
        started = time.monotonic()
        with self.assertRaises(DriverError):
            port.write(bytes(1_000_000), started + 0.05)
        self.assertLess(time.monotonic() - started, 0.3)
        port.close()
        self.assertRegex(Path(self.log).read_text(), r"TX_ERROR write timeout, sent \d+/1000000 bytes")

    def test_write_error_other_than_retry_is_reported(self):
        port = self.port(log_path=self.log)
        with patch("neck.driver.port.os.write", side_effect=OSError(errno.EIO, "I/O error")):
            with self.assertRaises(DriverError):
                port.write(b"x", time.monotonic() + 1)
        port.close()
        self.assertIn("TX_ERROR I/O error", Path(self.log).read_text())

    def test_hex_log_format(self):
        port = self.port(log_path=self.log)
        port.write(b"\x7a\x02\x05\x7d\x7b", time.monotonic() + 1)
        os.write(self.master, b"\x01\xab")
        port.read(time.monotonic() + 1)
        port.flush_input()
        port.close()
        lines = Path(self.log).read_text().splitlines()
        stamp = r"\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3} "
        kinds = [re.fullmatch(stamp + r"(.*)", line).group(1) for line in lines]
        self.assertEqual(kinds, ["LOG started", "FLUSH input", "TX 5B 7A 02 05 7D 7B",
                                 "RX 2B 01 AB", "FLUSH input", "LOG stopped"])

    def test_log_is_appended_not_truncated(self):
        for _ in range(2):
            self.port(log_path=self.log).close()
        self.assertEqual(Path(self.log).read_text().count("LOG started"), 2)

    def test_closed_port_and_missing_device(self):
        port = self.port()
        port.close()
        port.close()
        self.assertFalse(port.is_open)
        for call in (lambda: port.write(b"x", time.monotonic() + 1),
                     lambda: port.read(time.monotonic()), port.flush_input):
            with self.assertRaises(DriverError):
                call()
        with self.assertRaises(DriverError):
            Port("/dev/does-not-exist", 115200)
        with self.assertRaises(DriverError):
            Port(self.path, 115200, log_path="/nonexistent-dir/x.log")


if __name__ == "__main__":
    unittest.main()
