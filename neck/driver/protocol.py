"""F32C frame format: 7A addr func data... BCC 7B, BCC = XOR of everything before it.

The motor never acknowledges a command; only the angle query gets an answer.
"""

from __future__ import annotations

import math

HEAD = 0x7A
TAIL = 0x7B
_ANGLE_FEEDBACK = 0x02          # query type and reply function for mechanical angle x10
_REPLY_LEN = 9                  # 7A addr 02 d3 d2 d1 d0 BCC 7B


def bcc(data: bytes) -> int:
    x = 0
    for b in data:
        x ^= b
    return x


def pack(addr: int, func: int, data: bytes = b"") -> bytes:
    body = bytes((HEAD, addr, func)) + data
    return body + bytes((bcc(body), TAIL))


def _u16(value: int) -> bytes:
    return (value & 0xFFFF).to_bytes(2, "big")


def enable(addr: int) -> bytes:
    return pack(addr, 0x06)


def disable(addr: int) -> bytes:
    return pack(addr, 0x05)


def mode_single_turn_direct(addr: int) -> bytes:
    return pack(addr, 0x00, _u16(4))


def speed_rpm(addr: int, rpm: int) -> bytes:
    return pack(addr, 0x01, _u16(rpm))


def accel(addr: int, rev_per_s2: int) -> bytes:
    return pack(addr, 0x07, _u16(rev_per_s2))


def angle(addr: int, deg: float) -> bytes:
    """Single-turn target, 0.1 degree steps; negative angles wrap into 0..359.9."""
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
    return pack(addr, 0x0E, bytes((_ANGLE_FEEDBACK,)))


def find_angle(buf: bytes, addr: int) -> float | None:
    """First valid angle reply from `addr` inside a noisy byte stream, in degrees (signed).

    Echoes of our own query, other motors' replies, garbage and corrupt checksums are skipped.
    """
    for i in range(len(buf) - _REPLY_LEN + 1):
        if (buf[i] == HEAD and buf[i + 1] == addr and buf[i + 2] == _ANGLE_FEEDBACK
                and buf[i + 8] == TAIL and bcc(buf[i:i + 7]) == buf[i + 7]):
            return int.from_bytes(buf[i + 3:i + 7], "big", signed=True) / 10.0
    return None
