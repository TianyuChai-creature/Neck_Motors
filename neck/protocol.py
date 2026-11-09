"""F32C frame format: 7A addr func data... BCC 7B, BCC = XOR of everything before it.

Pure functions only. The motor never acknowledges a command; only the angle
query gets an answer.
"""

from __future__ import annotations

import math

HEAD = 0x7A
TAIL = 0x7B
ANGLE_REPLY = 0x02
REPLY_LEN = 9

__all__ = [
    "HEAD", "TAIL", "angle", "bcc", "disable", "enable", "find_angle", "mode_direct",
    "pack", "parse_angle_frames", "pos_ki", "pos_kp", "query_angle", "save",
    "set_zero", "speed_ki", "speed_kp", "speed_rpm",
]


def bcc(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value


def pack(addr: int, func: int, data: bytes = b"") -> bytes:
    body = bytes((HEAD, addr, func)) + data
    return body + bytes((bcc(body), TAIL))


def _u16(value: int) -> bytes:
    return (value & 0xFFFF).to_bytes(2, "big")


def enable(addr: int) -> bytes:
    return pack(addr, 0x06)


def disable(addr: int) -> bytes:
    return pack(addr, 0x05)


def save(addr: int) -> bytes:
    """Write the motor's current parameters to flash (manual 4.2.8)."""
    return pack(addr, 0x08)


def set_zero(addr: int) -> bytes:
    """The current single-turn angle becomes 0 degrees (manual 4.2.10)."""
    return pack(addr, 0x0A)


def mode_direct(addr: int) -> bytes:
    """Single-turn absolute position, direct mode (0004)."""
    return pack(addr, 0x00, _u16(4))


def speed_rpm(addr: int, rpm: int) -> bytes:
    return pack(addr, 0x01, _u16(rpm))


def accel(addr: int, rev_per_s2: int) -> bytes:
    return pack(addr, 0x07, _u16(rev_per_s2))


def angle(addr: int, deg: float) -> bytes:
    """Single-turn target in 0.1 degree steps; negative angles wrap into 0..359.9."""
    return pack(addr, 0x03, _u16(round(math.fmod(deg, 360.0) * 10) % 3600))


def speed_kp(addr: int, value: int) -> bytes:
    return pack(addr, 0x0F, _u16(value))


def speed_ki(addr: int, value: int) -> bytes:
    return pack(addr, 0x10, _u16(value))


def pos_kp(addr: int, value: int) -> bytes:
    return pack(addr, 0x11, _u16(value))


def pos_ki(addr: int, value: int) -> bytes:
    return pack(addr, 0x12, _u16(value))


def query_angle(addr: int) -> bytes:
    return pack(addr, 0x0E, bytes((ANGLE_REPLY,)))


def find_angle(buf: bytes, addr: int) -> float | None:
    """First valid angle reply from ``addr`` in a noisy byte stream."""
    for i in range(len(buf) - REPLY_LEN + 1):
        if (buf[i] == HEAD and buf[i + 1] == addr and buf[i + 2] == ANGLE_REPLY
                and buf[i + 8] == TAIL and bcc(buf[i:i + 7]) == buf[i + 7]):
            tenths = int.from_bytes(buf[i + 3:i + 7], "big", signed=True)
            return _signed(tenths)
    return None


def parse_angle_frames(buf: bytearray) -> list[tuple[int, float]]:
    """Pop every complete valid angle reply from ``buf``; leave partial bytes."""
    found: list[tuple[int, float]] = []
    i = 0
    while i <= len(buf) - REPLY_LEN:
        if (buf[i] == HEAD and buf[i + 2] == ANGLE_REPLY and buf[i + 8] == TAIL
                and bcc(buf[i:i + 7]) == buf[i + 7]):
            tenths = int.from_bytes(buf[i + 3:i + 7], "big", signed=True)
            found.append((buf[i + 1], _signed(tenths)))
            i += REPLY_LEN
        else:
            i += 1
    if i:
        del buf[:i]
    if len(buf) > 4096:
        del buf[:-8]
    return found


def _signed(tenths: int) -> float:
    return ((tenths + 1800) % 3600 - 1800) / 10.0
