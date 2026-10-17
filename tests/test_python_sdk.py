"""Hardware-free smoke check for the installed Python binding."""

import ctypes
from unittest.mock import patch

import f32c
from f32c import _CConfig, _motor_library


if __name__ == "__main__":
    assert f32c.DEFAULT_HOLD_POSITION == f32c.State(0, 35)
    config = f32c.Config()
    lib = _motor_library()
    lib.f32c_config_defaults.argtypes = [ctypes.POINTER(_CConfig)]
    native = _CConfig()
    lib.f32c_config_defaults(ctypes.byref(native))
    assert config.travel_rpm == native.travel_rpm == 5
    assert config.accel == native.accel == 1
    assert (config.id_yaw, config.id_pitch) == (native.id_yaw, native.id_pitch) == (1, 2)
    names = ("yaw_speed_kp", "yaw_speed_ki", "yaw_pos_kp", "yaw_pos_ki",
             "pitch_speed_kp", "pitch_speed_ki", "pitch_pos_kp", "pitch_pos_ki")
    assert tuple(getattr(config, name) for name in names) == tuple(
        getattr(native, name) for name in names) == (40, 4, 42, 0, 10, 4, 40, 0)

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
    motor = f32c.Gimbal2Dof()
    motor._handle = 1
    with patch.object(motor._lib, "f32c_close_checked", return_value=0) as close, \
            patch.object(motor._lib, "f32c_last_error", return_value=b"hardware state unknown"):
        try:
            motor.close()
        except f32c.F32CError as exc:
            assert "hardware state unknown" in str(exc)
        else:
            raise AssertionError("failed cleanup was hidden")
        assert motor._handle is None
        motor.close()
        assert close.call_count == 1
    print("Python SDK passed")
