"""Hardware-free smoke check for the installed Python binding."""

import ctypes

import f32c
from f32c import _CConfig, _motor_library


if __name__ == "__main__":
    config = f32c.Config()
    lib = _motor_library()
    lib.f32c_config_defaults.argtypes = [ctypes.POINTER(_CConfig)]
    native = _CConfig()
    lib.f32c_config_defaults(ctypes.byref(native))
    assert (config.id_yaw, config.id_pitch) == (native.id_yaw, native.id_pitch) == (1, 2)
    names = ("yaw_speed_kp", "yaw_speed_ki", "yaw_pos_kp", "yaw_pos_ki",
             "pitch_speed_kp", "pitch_speed_ki", "pitch_pos_kp", "pitch_pos_ki")
    assert tuple(getattr(config, name) for name in names) == tuple(
        getattr(native, name) for name in names) == (50, 16, 63, 0, 30, 8, 60, 0)

    try:
        f32c.Gimbal2Dof(f32c.Config(id_pitch=1)).open()
    except ValueError as exc:
        assert "distinct" in str(exc)
    else:
        raise AssertionError("duplicate motor IDs accepted")

    try:
        f32c.Gimbal2Dof(f32c.Config(device="/dev/f32c-sdk-test-missing")).open()
    except f32c.F32CError as exc:
        assert "cannot open serial device" in str(exc)
    else:
        raise AssertionError("missing serial device accepted")

    for name in ("ZedMini", "Frame", "View", "Setting", "_zed_library"):
        assert not hasattr(f32c, name), f"camera API still exposed: {name}"
    print("Python SDK passed")
