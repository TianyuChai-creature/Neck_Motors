"""Serial port on the standard library only: raw bytes in and out, with deadlines and a hex log.

Deadlines are absolute time.monotonic() values so callers can share one budget.
"""

from __future__ import annotations

import errno
import fcntl
import os
import select
import struct
import termios
import time
from datetime import datetime

from .types import DriverError

_BAUD = {9600: termios.B9600, 57600: termios.B57600, 115200: termios.B115200}
_RETRY = (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR)


class Port:
    def __init__(self, path: str, baud: int = 115200, log_path: str | None = None):
        self._fd = -1
        self._log = None
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise DriverError(f"cannot open serial device {path}: {exc.strerror}") from exc
        try:
            self._configure(fd, baud)
        except (OSError, termios.error) as exc:
            os.close(fd)
            raise DriverError(f"cannot configure serial device {path}: {exc}") from exc
        self._fd = fd
        if log_path:
            try:
                self._log = open(log_path, "a")
            except OSError as exc:
                self.close()
                raise DriverError(f"cannot open serial log {log_path}: {exc.strerror}") from exc
            self._note("LOG", "started")
        time.sleep(0.05)        # let a USB CDC device settle after DTR, then drop its chatter
        self.flush_input()

    @staticmethod
    def _configure(fd: int, baud: int) -> None:
        iflag, oflag, cflag, lflag, _, _, cc = termios.tcgetattr(fd)
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                   | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB | getattr(termios, "CRTSCTS", 0))
        cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, _BAUD[baud], _BAUD[baud], cc])
        try:
            fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        except OSError:
            pass                # not a modem line (pseudo-terminals); nothing to assert

    @property
    def is_open(self) -> bool:
        return self._fd >= 0

    def close(self) -> None:
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1
        if self._log:
            self._note("LOG", "stopped")
            self._log.close()
            self._log = None

    def write(self, data: bytes, deadline: float) -> None:
        """Send all of `data` or raise DriverError; never blocks past `deadline`."""
        self._require_open()
        sent = 0
        while sent < len(data):
            wait = deadline - time.monotonic()
            if wait <= 0:
                self._note("TX_ERROR", f"write timeout, sent {sent}/{len(data)} bytes")
                raise DriverError("serial write timed out")
            try:
                sent += os.write(self._fd, data[sent:])
                continue
            except OSError as exc:
                if exc.errno not in _RETRY:
                    self._note("TX_ERROR", exc.strerror or str(exc))
                    raise DriverError(f"serial write failed: {exc.strerror}") from exc
            select.select([], [self._fd], [], min(wait, 0.01))
        self._note("TX", data)

    def read(self, deadline: float) -> bytes:
        """Bytes available now, or wait for some until `deadline`; b"" means none arrived."""
        self._require_open()
        while True:
            try:
                data = os.read(self._fd, 128)
            except OSError as exc:
                if exc.errno not in _RETRY:
                    raise DriverError(f"serial read failed: {exc.strerror}") from exc
                data = b""
            if data:
                self._note("RX", data)
                return data
            wait = deadline - time.monotonic()
            if wait <= 0:
                return b""
            select.select([self._fd], [], [], min(wait, 0.01))

    def flush_input(self) -> None:
        self._require_open()
        termios.tcflush(self._fd, termios.TCIFLUSH)
        self._note("FLUSH", "input")

    def _require_open(self) -> None:
        if self._fd < 0:
            raise DriverError("serial port is closed")

    def _note(self, kind: str, what) -> None:
        if not self._log:
            return
        now = datetime.now()
        stamp = now.strftime("%Y-%m-%d %H:%M:%S") + f".{now.microsecond // 1000:03d}"
        if isinstance(what, bytes):
            what = f"{len(what)}B " + " ".join(f"{b:02X}" for b in what)
        self._log.write(f"{stamp} {kind} {what}\n")
        self._log.flush()
