#!/usr/bin/env python3
"""Bounded pitch return-to-zero capture with unchanged SDK PI; preview by default."""
import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import sys
import tempfile
import termios
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from f32c import Config, Gimbal2Dof, HeadController, TrackingConfig


def query(fd, kind, raw, timeout=.025, *, address=2):
    """Read one documented signed 32-bit feedback value."""
    body = bytes((0x7A, address, 0x0E, kind))
    checksum = 0
    for byte in body:
        checksum ^= byte
    frame = body + bytes((checksum, 0x7B))
    termios.tcflush(fd, termios.TCIFLUSH)
    start = time.monotonic()
    def record(direction, data):
        raw.write(json.dumps(dict(monotonic_s=time.monotonic(), direction=direction,
                                  hex=data.hex(" "))) + "\n")
        raw.flush()
    sent = os.write(fd, frame)
    record("TX", frame[:sent])
    if sent != len(frame):
        raise OSError("partial telemetry query write")
    buffered = bytearray()
    while time.monotonic() - start < timeout:
        if not select.select([fd], [], [], max(0, timeout - (time.monotonic() - start)))[0]:
            break
        data = os.read(fd, 128)
        record("RX", data)
        buffered.extend(data)
        for offset in range(len(buffered) - 8):
            reply = buffered[offset:offset + 9]
            check = 0
            for byte in reply[:-2]:
                check ^= byte
            if reply[:3] == bytes((0x7A, address, kind)) and reply[-1] == 0x7B and check == reply[-2]:
                value = int.from_bytes(reply[3:7], "big", signed=True)
                finished = time.monotonic()
                time.sleep(.003)
                return value, start, finished
        if len(buffered) > 256:
            del buffered[:-8]
    raise TimeoutError(f"motor {address} telemetry type {kind} timed out")


def run(device, *, both_zero=False):
    directory = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("pitch_feedback_%Y%m%d_%H%M%S_"),
                                     dir=ROOT / "logs"))
    config = Config(device=device)
    result = dict(config=asdict(config), directory=str(directory), status="opening",
                  hardware_state="unknown", max_motion_s=4,
                  yaw_policy="return_zero" if both_zero else "hold_initial",
                  enable_attempted=False)
    print(json.dumps(result), flush=True)
    head = None
    motor = None
    fd = None
    try:
        # This descriptor only queries telemetry between SDK calls; no reader thread.
        fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        motor = Gimbal2Dof(config).open()
        head = HeadController(motor=motor, tracking=TrackingConfig(
            yaw_limits=(-60, 60), pitch_limits=(-180, 45),
            max_speed_deg_s=120, max_accel_deg_s2=600)).open()
        fcntl.ioctl(fd, termios.TIOCEXCL)
        head.start_log(directory / "serial.log")
        with (directory / "feedback.jsonl").open("x") as raw, (directory / "samples.csv").open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(("monotonic_s", "phase", "yaw_deg", "pitch_deg", "pitch_command_deg",
                             "pitch_rpm", "voltage_V", "speed_query_start_s", "speed_query_end_s",
                             "voltage_query_start_s", "voltage_query_end_s",
                             "yaw_rpm", "yaw_voltage_V", "yaw_speed_query_start_s", "yaw_speed_query_end_s",
                             "yaw_voltage_query_start_s", "yaw_voltage_query_end_s"))
            start = time.monotonic()
            moving_at = None
            initial_yaw = None
            minimum = float("inf")
            rebound_at = None
            while True:
                cycle = time.monotonic()
                if moving_at is None:
                    measured = head.read()
                    command = None
                else:
                    snapshot = head.tick()
                    measured, command = snapshot.measured, snapshot.command.pitch_deg
                speed, s0, s1 = query(fd, 0, raw)
                voltage, v0, v1 = query(fd, 4, raw)
                yaw_speed, ys0, ys1 = query(fd, 0, raw, address=1)
                yaw_voltage, yv0, yv1 = query(fd, 4, raw, address=1)
                volts = voltage / 100
                writer.writerow((cycle, "baseline" if moving_at is None else "moving",
                                 measured.yaw_deg, measured.pitch_deg, command, speed, volts, s0, s1, v0, v1,
                                 yaw_speed, yaw_voltage / 100, ys0, ys1, yv0, yv1))
                output.flush()
                if not 8 <= volts <= 15:
                    raise RuntimeError(f"bus voltage outside 8..15 V: {volts}")
                if not 8 <= yaw_voltage / 100 <= 15:
                    raise RuntimeError(f"yaw bus voltage outside 8..15 V: {yaw_voltage / 100}")
                if moving_at is None and cycle - start >= .5:
                    initial_yaw = measured.yaw_deg
                    if not -60 <= initial_yaw <= 60:
                        raise RuntimeError("yaw outside UI range; refusing an unintended yaw move")
                    result["initial"] = asdict(measured)
                    result["enable_attempted"] = True
                    head.enable()
                    head.move_to(0 if both_zero else initial_yaw, 0)
                    moving_at = time.monotonic()
                    minimum = measured.pitch_deg
                elif moving_at is not None:
                    minimum = min(minimum, measured.pitch_deg)
                    if measured.pitch_deg - minimum >= 2 and rebound_at is None:
                        rebound_at = time.monotonic()
                    if rebound_at is not None and time.monotonic() - rebound_at >= .5:
                        result["status"] = "rebound_captured"
                        break
                    if time.monotonic() - moving_at >= 4:
                        result["status"] = "motion_time_limit"
                        break
                time.sleep(max(0, .033 - (time.monotonic() - cycle)))
    except BaseException as exc:
        result.update(status="error", error=f"{type(exc).__name__}: {exc}")
    finally:
        if head is not None:
            try:
                if result["enable_attempted"]:
                    head.disable()
                    result["disable_requests_sent"] = True
            except Exception as exc:
                result["disable_error"] = str(exc)
            try:
                head.close()
                result["cleanup_completed"] = True
            except Exception as exc:
                result["close_error"] = str(exc)
        elif motor is not None:
            try:
                motor.close()
            except Exception as exc:
                result["close_error"] = str(exc)
        if fd is not None:
            try:
                fcntl.ioctl(fd, termios.TIOCNXCL)
            except OSError as exc:
                result["exclusive_release_error"] = str(exc)
            finally:
                os.close(fd)
        (directory / "run.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyACM0")
    parser.add_argument("--enable", action="store_true")
    parser.add_argument("--both-zero", action="store_true", help="return yaw to zero as well")
    args = parser.parse_args()
    if not args.enable:
        print(f"Preview: unchanged PI, yaw {'-> 0' if args.both_zero else 'held'}, pitch -> 0, "
              "0.5 s baseline, <=4 s motion; --enable to execute")
    else:
        def interrupted(signum, frame):
            raise KeyboardInterrupt(f"signal {signum}")
        signal.signal(signal.SIGTERM, interrupted)
        raise SystemExit(run(args.device, both_zero=args.both_zero)["status"] == "error")
