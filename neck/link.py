"""Non-blocking F32C serial link: bounded TX scheduling and a streaming RX parser.

The control loop calls :meth:`SerialLink.poll` and :meth:`SerialLink.pump` every
tick.  Reads never wait for a motor reply; the only wait is the mandatory idle
gap between transmitted frames (about 1 ms per the manual).
"""

from __future__ import annotations

import errno
import fcntl
import glob
import os
import select
import struct
import termios
import time
from datetime import datetime

from . import protocol as p
from .types import LinkError, NeckConfig

_BAUD = {9600: termios.B9600, 57600: termios.B57600, 115200: termios.B115200}
_RETRY = (errno.EAGAIN, errno.EWOULDBLOCK, errno.EINTR)


def candidates() -> list[str]:
    """Serial devices worth probing, each physical device once."""
    paths = sorted(glob.glob("/dev/serial/by-id/*")) + sorted(
        glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    seen: set[str] = set()
    unique: list[str] = []
    for path in paths:
        real = os.path.realpath(path)
        if real not in seen:
            seen.add(real)
            unique.append(path)
    return unique


class SerialLink:
    def __init__(self, config: NeckConfig, path: str | None = None):
        self.config = config
        self.path = path or config.port
        if not self.path:
            raise LinkError("no serial port specified")
        self._fd = -1
        self._log = None
        self._buf = bytearray()
        self.angles: dict[int, float] = {}
        self.angle_at: dict[int, float] = {}
        self._angle_seq: dict[int, int] = {}
        self._pending_angle: dict[int, bytes] = {}
        self._pending_query: list[int] = []
        self._next_send = 0.0
        self.tx_errors = 0
        self.rx_errors = 0
        self.last_error = ""
        self.error_at: float | None = None
        self._open()

    @classmethod
    def open(cls, config: NeckConfig | None = None) -> SerialLink:
        """Open the configured port, or scan for the one where both motors answer."""
        config = config or NeckConfig()
        if config.port:
            return cls(config)
        last: Exception | None = None
        for path in candidates():
            try:
                link = cls(config, path)
            except LinkError as exc:
                last = exc
                continue
            try:
                link.read_angle(config.yaw_id, 0.1)
                link.read_angle(config.pitch_id, 0.1)
                return link
            except LinkError as exc:
                last = exc
                link.close()
        raise LinkError(f"no serial device answered for both motor IDs: {last}")

    # ---- lifecycle -------------------------------------------------------------------------

    def _open(self) -> None:
        try:
            fd = os.open(self.path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as exc:
            raise LinkError(f"cannot open serial device {self.path}: {exc.strerror}") from exc
        try:
            self._configure(fd, self.config.baud)
        except (OSError, termios.error) as exc:
            os.close(fd)
            raise LinkError(f"cannot configure serial device {self.path}: {exc}") from exc
        self._fd = fd
        if self.config.log_path:
            try:
                self._log = open(self.config.log_path, "a")
            except OSError as exc:
                self.close()
                raise LinkError(f"cannot open serial log {self.config.log_path}: {exc.strerror}") from exc
            self._note("LOG", "started")
        time.sleep(0.05)        # let a USB CDC device settle after DTR
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
        speed = _BAUD[baud]
        termios.tcsetattr(fd, termios.TCSANOW,
                          [iflag, oflag, cflag, lflag, speed, speed, cc])
        try:
            fcntl.ioctl(fd, termios.TIOCMBIS, struct.pack("I", termios.TIOCM_DTR))
        except OSError:
            pass                # pseudo-terminals have no modem line

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

    # ---- receive ---------------------------------------------------------------------------

    def poll(self, now: float | None = None) -> list[tuple[int, float]]:
        """Read whatever is available and update per-axis angles; never blocks."""
        if self._fd < 0:
            return []
        now = time.monotonic() if now is None else now
        while True:
            try:
                data = os.read(self._fd, 4096)
            except OSError as exc:
                if exc.errno in _RETRY:
                    break
                self.rx_errors += 1
                self.last_error = f"serial read failed: {exc.strerror}"
                self.error_at = now
                break
            if not data:
                break
            self._note("RX", data)
            self._buf += data
        found = p.parse_angle_frames(self._buf)
        for addr, deg in found:
            self.angles[addr] = deg
            self.angle_at[addr] = now
            self._angle_seq[addr] = self._angle_seq.get(addr, 0) + 1
        return found

    def flush_input(self) -> None:
        if self._fd >= 0:
            try:
                termios.tcflush(self._fd, termios.TCIFLUSH)
            except termios.error:
                pass
            self._note("FLUSH", "input")

    # ---- transmit --------------------------------------------------------------------------

    def queue_angle(self, addr: int, deg: float) -> None:
        """Latest angle wins; an older unsent target is replaced."""
        self._pending_angle[addr] = p.angle(addr, deg)

    def queue_query(self, addr: int) -> None:
        if addr not in self._pending_query:
            self._pending_query.append(addr)

    def clear_pending(self) -> None:
        self._pending_angle.clear()
        self._pending_query.clear()

    def pump(self, now: float | None = None, max_frames: int = 3) -> int:
        """Send due frames, honouring the inter-frame gap. Never waits for a reply."""
        sent = 0
        for _ in range(max_frames):
            frame, kind = self._take_frame()
            if frame is None:
                break
            self._wait_turn()
            ok = self._write_once(frame)
            self._next_send = time.monotonic() + self.config.frame_gap
            if not ok:
                if kind == "angle":
                    self._pending_angle[frame[1]] = frame
                else:
                    self._pending_query.append(frame[1])
                break
            sent += 1
        return sent

    def write_frame(self, frame: bytes) -> None:
        """Synchronous transaction write; bounded by a few frame gaps."""
        for _ in range(3):
            self._wait_turn()
            if self._write_once(frame):
                self._next_send = time.monotonic() + self.config.frame_gap
                return
            self._next_send = time.monotonic() + self.config.frame_gap
            self.poll()
        raise LinkError(self.last_error or "serial write failed")

    def read_angle(self, addr: int, timeout: float = 0.05) -> float:
        """Query one axis and wait for a reply received after the query."""
        self.poll()
        self.flush_input()
        self._buf.clear()
        before = self._angle_seq.get(addr, 0)
        self.write_frame(p.query_angle(addr))
        deadline = time.monotonic() + timeout
        while True:
            self.poll()
            if self._angle_seq.get(addr, 0) > before:
                return self.angles[addr]
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            self._select(min(remaining, 0.005))
        self.poll()
        if self._angle_seq.get(addr, 0) > before:
            return self.angles[addr]
        raise LinkError(f"motor {addr} did not answer the angle query")

    def _take_frame(self) -> tuple[bytes | None, str]:
        if self._pending_angle:
            addr, frame = next(iter(self._pending_angle.items()))
            del self._pending_angle[addr]
            return frame, "angle"
        if self._pending_query:
            addr = self._pending_query.pop(0)
            return p.query_angle(addr), "query"
        return None, ""

    def _wait_turn(self) -> None:
        while True:
            remaining = self._next_send - time.monotonic()
            if remaining <= 0:
                return
            self._select(min(remaining, 0.01))

    def _select(self, timeout: float) -> None:
        if self._fd < 0 or timeout <= 0:
            return
        try:
            select.select([self._fd], [], [], timeout)
        except OSError:
            return
        self.poll()

    def _write_once(self, frame: bytes) -> bool:
        if self._fd < 0:
            self.last_error = "serial port is closed"
            self.error_at = time.monotonic()
            return False
        try:
            sent = os.write(self._fd, frame)
        except OSError as exc:
            if exc.errno in _RETRY:
                return False
            self.tx_errors += 1
            self.last_error = f"serial write failed: {exc.strerror}"
            self.error_at = time.monotonic()
            return False
        if sent != len(frame):
            self.tx_errors += 1
            self.last_error = f"short write {sent}/{len(frame)}; frame retried"
            self.error_at = time.monotonic()
            self._note("TX_ERROR", self.last_error)
            return False
        self._note("TX", frame)
        return True

    # ---- log -------------------------------------------------------------------------------

    def _note(self, kind: str, what) -> None:
        if not self._log:
            return
        now = datetime.now()
        stamp = now.strftime("%Y-%m-%d %H:%M:%S") + f".{now.microsecond // 1000:03d}"
        if isinstance(what, bytes):
            what = f"{len(what)}B " + " ".join(f"{b:02X}" for b in what)
        self._log.write(f"{stamp} {kind} {what}\n")
        self._log.flush()
