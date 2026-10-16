"""F32C motor bindings and head tracking for the installed native SDK."""

from __future__ import annotations

import ctypes as C
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path

__all__ = ["Config", "F32CError", "Gimbal2Dof", "State", "native_prefix",
           "HeadTracker", "TrackingConfig", "TrackingError", "HeadController",
           "ProtectionConfig", "ControlSnapshot", "HeadControlError", "clip_pitch_target"]


class F32CError(RuntimeError):
    pass


@dataclass
class Config:
    device: str | None = None
    baud: int = 115200
    id_yaw: int = 1
    id_pitch: int = 2
    travel_rpm: int = 80
    accel: int = 10
    yaw_speed_kp: int = 50
    yaw_speed_ki: int = 16
    yaw_pos_kp: int = 63
    yaw_pos_ki: int = 0
    pitch_speed_kp: int = 30
    pitch_speed_ki: int = 8
    pitch_pos_kp: int = 60
    pitch_pos_ki: int = 0

    def _validate(self) -> None:
        if self.device is not None and not isinstance(self.device, str):
            raise ValueError("device must be a string or None")
        for name, _ in _CConfig._fields_:
            if name != "device" and type(getattr(self, name)) is not int:
                raise ValueError(f"{name} must be an integer")
        if self.baud not in (9600, 57600, 115200):
            raise ValueError("unsupported baud rate")
        if not 1 <= self.id_yaw <= 255 or not 1 <= self.id_pitch <= 255 or self.id_yaw == self.id_pitch:
            raise ValueError("motor IDs must be nonzero and distinct")
        if not 0 <= self.travel_rpm <= 1000 or not 0 <= self.accel <= 65535:
            raise ValueError("travel_rpm or accel is out of range")
        for name in ("yaw_speed_kp", "yaw_speed_ki", "yaw_pos_kp", "yaw_pos_ki",
                     "pitch_speed_kp", "pitch_speed_ki", "pitch_pos_kp", "pitch_pos_ki"):
            if not 0 <= getattr(self, name) <= 256:
                raise ValueError(f"{name} must be between 0 and 256")


@dataclass(frozen=True)
class State:
    yaw_deg: float
    pitch_deg: float


class _CConfig(C.Structure):
    _fields_ = [("device", C.c_char_p), ("baud", C.c_int),
                ("id_yaw", C.c_uint), ("id_pitch", C.c_uint),
                ("travel_rpm", C.c_int), ("accel", C.c_uint)] + [
        (name, C.c_uint) for name in ("yaw_speed_kp", "yaw_speed_ki", "yaw_pos_kp", "yaw_pos_ki",
                                      "pitch_speed_kp", "pitch_speed_ki", "pitch_pos_kp", "pitch_pos_ki")
    ]


def native_prefix() -> Path:
    """Path to the C++ headers, CMake config, and shared libraries in this wheel."""
    return Path(__file__).resolve().parent / "native"


def _library(filename: str) -> C.CDLL:
    if directory := os.environ.get("F32C_LIBRARY_DIR"):
        candidates = [Path(directory) / filename]
    else:
        candidates = [native_prefix() / "lib" / filename,
                      Path(__file__).resolve().parent / filename]
        root = Path(__file__).resolve().parents[2]
        if (root / "CMakeLists.txt").exists():
            candidates.append(root / "build" / filename)
        candidates.append(filename)
    last_error = None
    for path in candidates:
        try:
            return C.CDLL(str(path))
        except OSError as exc:
            last_error = exc
    raise F32CError(f"cannot load {filename}: {last_error}; set F32C_LIBRARY_DIR only for a custom native build")


def _bind(lib: C.CDLL, name: str, result, *args) -> None:
    try:
        fn = getattr(lib, name)
    except AttributeError as exc:
        raise F32CError(f"{lib._name} lacks {name}; rebuild the native SDK") from exc
    fn.restype = result
    fn.argtypes = args


def _motor_library() -> C.CDLL:
    lib = _library("libf32c.so")
    ptr = C.c_void_p
    _bind(lib, "f32c_open_config", ptr, C.POINTER(_CConfig))
    _bind(lib, "f32c_close", None, ptr)
    _bind(lib, "f32c_close_checked", C.c_int, ptr)
    _bind(lib, "f32c_last_error", C.c_char_p, ptr)
    for name in ("ping", "enable", "disable", "set_zero"):
        _bind(lib, f"f32c_{name}", C.c_int, ptr)
    _bind(lib, "f32c_set_deg", C.c_int, ptr, C.c_double, C.c_double)
    _bind(lib, "f32c_set_yaw_deg", C.c_int, ptr, C.c_double)
    _bind(lib, "f32c_set_pitch_deg", C.c_int, ptr, C.c_double)
    _bind(lib, "f32c_read", C.c_int, ptr, C.POINTER(C.c_double), C.POINTER(C.c_double))
    _bind(lib, "f32c_start_log", C.c_int, ptr, C.c_char_p)
    _bind(lib, "f32c_stop_log", None, ptr)
    return lib


class Gimbal2Dof:
    def __init__(self, config: Config | None = None):
        self.config = config if config is not None else Config()
        self._lib = _motor_library()
        self._handle = None

    def _require_open(self):
        if not self._handle:
            raise F32CError("gimbal is not open")
        return self._handle

    def _check(self, ok: int) -> None:
        if not ok:
            raise F32CError(self.last_error() or "motor command failed")

    def open(self) -> Gimbal2Dof:
        if self._handle:
            return self
        self.config._validate()
        cfg = self.config
        device = cfg.device.encode() if cfg.device else None
        raw = _CConfig(device, cfg.baud, cfg.id_yaw, cfg.id_pitch, cfg.travel_rpm, cfg.accel,
                       cfg.yaw_speed_kp, cfg.yaw_speed_ki, cfg.yaw_pos_kp, cfg.yaw_pos_ki,
                       cfg.pitch_speed_kp, cfg.pitch_speed_ki, cfg.pitch_pos_kp, cfg.pitch_pos_ki)
        self._handle = self._lib.f32c_open_config(C.byref(raw))
        if not self._handle:
            raise F32CError(self.last_error() or "motor open failed")
        return self

    def close(self) -> None:
        if self._handle:
            ok = self._lib.f32c_close_checked(self._handle)
            self._handle = None
            self._check(ok)

    def last_error(self) -> str:
        value = self._lib.f32c_last_error(self._handle)
        return value.decode(errors="replace") if value else ""

    def ping(self) -> bool:
        return bool(self._lib.f32c_ping(self._require_open()))

    def read(self, *, timeout_s: float | None = None) -> State:
        yaw, pitch = C.c_double(), C.c_double()
        if timeout_s is None:
            ok = self._lib.f32c_read(self._require_open(), C.byref(yaw), C.byref(pitch))
        else:
            if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 1:
                raise ValueError("timeout_s must be a finite number in (0, 1]")
            _bind(self._lib, "f32c_read_timeout", C.c_int, C.c_void_p,
                  C.POINTER(C.c_double), C.POINTER(C.c_double), C.c_uint)
            ok = self._lib.f32c_read_timeout(self._require_open(), C.byref(yaw), C.byref(pitch),
                                             max(1, math.ceil(timeout_s * 1000)))
        self._check(ok)
        return State(yaw.value, pitch.value)

    def enable(self) -> None:
        """Enable the motors and hold their current position."""
        self._check(self._lib.f32c_enable(self._require_open()))

    def disable(self) -> None:
        self._check(self._lib.f32c_disable(self._require_open()))

    def enable_at(self, yaw_deg: float, pitch_deg: float) -> None:
        """Enable with explicit initial targets; targets are immediate, not smoothed."""
        _bind(self._lib, "f32c_enable_at", C.c_int, C.c_void_p, C.c_double, C.c_double)
        self._check(self._lib.f32c_enable_at(self._require_open(), yaw_deg, pitch_deg))

    def set_pi(self, axis: str, speed_kp: int, speed_ki: int, pos_kp: int, pos_ki: int) -> None:
        """Update volatile PI while enabled; preserve the current target and mode."""
        if axis not in ("yaw", "pitch"):
            raise ValueError("axis must be yaw or pitch")
        gains = (speed_kp, speed_ki, pos_kp, pos_ki)
        if any(type(v) is not int or not 0 <= v <= 256 for v in gains):
            raise ValueError("PI gains must be integers between 0 and 256")
        handle = self._require_open()
        _bind(self._lib, "f32c_set_pi", C.c_int, C.c_void_p, *([C.c_uint] * 5))
        self._check(self._lib.f32c_set_pi(handle, 0 if axis == "yaw" else 1, *gains))
        for name, value in zip(("speed_kp", "speed_ki", "pos_kp", "pos_ki"), gains):
            setattr(self.config, f"{axis}_{name}", value)

    def set_deg(self, yaw_deg: float, pitch_deg: float) -> None:
        self._check(self._lib.f32c_set_deg(self._require_open(), yaw_deg, pitch_deg))

    def set_yaw_deg(self, deg: float) -> None:
        self._check(self._lib.f32c_set_yaw_deg(self._require_open(), deg))

    def set_pitch_deg(self, deg: float) -> None:
        self._check(self._lib.f32c_set_pitch_deg(self._require_open(), deg))

    def set_zero(self) -> None:
        """Persist the current physical pose as the new zero of both motors."""
        self._check(self._lib.f32c_set_zero(self._require_open()))

    def start_log(self, path: str | os.PathLike) -> None:
        self._check(self._lib.f32c_start_log(self._require_open(), os.fsencode(path)))

    def stop_log(self) -> None:
        self._lib.f32c_stop_log(self._require_open())

    def __enter__(self) -> Gimbal2Dof:
        return self.open()

    def __exit__(self, *_exc) -> None:
        self.close()

    def __del__(self):
        if getattr(self, "_handle", None):
            try:
                self.close()
            except F32CError as exc:
                print(f"F32C close failed: {exc}", file=sys.stderr)


# The pure tracking module shares State, so import it after that type is defined.
from .tracking import HeadTracker, TrackingConfig, TrackingError
from .controller import HeadController, ProtectionConfig, ControlSnapshot, HeadControlError, clip_pitch_target
