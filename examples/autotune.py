#!/usr/bin/env python3
"""Bounded PI search for the installed load; no persistence or SDK default changes."""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import asdict, replace
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import tempfile
import time

from tune import Config, Gimbal2Dof, PITCH_TEST_MAX, check_pitch_target, checked_position, quantized, summarize, wrap180


class MotionFault(RuntimeError):
    pass


def check_test_pose(center, axis, step):
    check_pitch_target(center[1])
    if axis == 1:
        check_pitch_target(center[1] - step)
        check_pitch_target(center[1] + step)


def candidate_values(value, integral=False):
    values = [value, 0, 2, 4, 8, 16, 32, 64] if integral else [value, round(value * .75), round(value * 1.25), round(value * 1.5)]
    return list(dict.fromkeys(max(0 if integral else 1, min(256, v)) for v in values))


def passed(metrics, args):
    return (metrics["hold_samples"] >= 20
            and metrics["hold_peak_to_peak_deg"] <= args.max_jitter + 1e-9
            and abs(metrics["hold_mean_error_deg"]) <= args.tolerance + 1e-9
            and abs(metrics["offset_mean_tracking_error_deg"]) <= args.tolerance + 1e-9
            and metrics["recovery_time_s"] is not None
            and metrics["recovery_time_s"] <= args.max_recovery)


def score(results, args):
    return max(2 * r["hold_peak_to_peak_deg"] + 2 * abs(r["hold_mean_error_deg"])
               + 2 * abs(r["offset_mean_tracking_error_deg"])
               + (r["recovery_time_s"] if r["recovery_time_s"] is not None else 10)
               + (0 if passed(r, args) else 10) for r in results)


def evaluate(config, axis, center, args, directory, label, session=None):
    """Hold, then small position steps in both directions; CSV is flushed before checks."""
    motor = session
    owned = session is None
    attempted = False
    results = []
    period = 1 / args.hz
    started = time.monotonic()
    record = {"label": label, "axis": axis, "requested_config": asdict(config),
              "center_deg": center, "started_at": datetime.now().astimezone().isoformat(),
              "serial_log": "serial.log" if owned else "../serial.log",
              "stimulus": "position_step_not_external_force", "status": "running"}
    def save():
        (directory / "result.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    save()
    try:
        check_test_pose(center, axis, args.step)
        with (directory / "samples.csv").open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_s", "phase", "yaw_target_deg", "pitch_target_deg",
                             "yaw_measured_deg", "pitch_measured_deg", "axis_center_error_deg"])
            if owned:
                motor = Gimbal2Dof(replace(config))
                motor.open()
                motor.start_log(directory / "serial.log")
                initial = checked_position(motor.read(), [None, None])
                attempted = True
                motor.enable_at(*initial)
            else:
                for name in ("yaw", "pitch"):
                    motor.set_pi(name, *(getattr(config, f"{name}_{gain}")
                                         for gain in ("speed_kp", "speed_ki", "pos_kp", "pos_ki")))
            commanded = [quantized(v) for v in checked_position(motor.read(), [None, None])]
            motor.set_deg(*commanded)
            sent = list(commanded)
            previous = time.monotonic()
            def collect(duration, desired, phase, ramp=False):
                nonlocal previous
                samples = []
                history = deque()
                deadline = time.monotonic() + duration
                while time.monotonic() < deadline:
                    tick = time.monotonic()
                    dt, previous = min(max(tick - previous, 0), .05), tick
                    for i in (0, 1):
                        delta = desired[i] - commanded[i]
                        if i == 0:
                            delta = wrap180(delta)
                        value = commanded[i] + (max(-args.move_speed * dt, min(args.move_speed * dt, delta)) if ramp else delta)
                        commanded[i] = wrap180(value)
                        value = quantized(commanded[i])
                        if value != sent[i]:
                            setter = motor.set_yaw_deg if i == 0 else motor.set_pitch_deg
                            setter(value)
                            sent[i] = value
                    measured = checked_position(motor.read(), [None, None])
                    elapsed = time.monotonic() - started
                    error = wrap180(measured[axis] - center[axis])
                    writer.writerow([elapsed, phase, *sent, *measured, error])
                    output.flush()
                    if any(abs(wrap180(measured[i] - sent[i])) > args.abort_error for i in (0, 1)):
                        raise MotionFault(f"tracking error exceeded {args.abort_error}°: {measured}")
                    history.append((elapsed, [wrap180(measured[i] - center[i]) for i in (0, 1)],
                                    [abs(wrap180(measured[i] - sent[i])) for i in (0, 1)]))
                    while len(history) > 1 and history[1][0] <= elapsed - args.stall_seconds:
                        history.popleft()
                    if elapsed - history[0][0] >= args.stall_seconds:
                        for i in (0, 1):
                            positions = [row[1][i] for row in history]
                            if (all(row[2][i] > args.stall_error for row in history)
                                    and max(positions) - min(positions) <= args.stall_motion + 1e-9):
                                raise MotionFault(f"{('yaw', 'pitch')[i]} no progress with sustained tracking error; check obstruction before retrying")
                    samples.append((elapsed, error))
                    time.sleep(max(0, period - (time.monotonic() - tick)))
                return samples
            distance = max(abs(wrap180(center[0] - commanded[0])), abs(center[1] - commanded[1]))
            collect(distance / args.move_speed + 1, center, "approach", ramp=True)
            if any(abs(wrap180(a - b)) > .11 for a, b in zip(center, commanded)):
                raise MotionFault("target ramp did not finish")
            hold = collect(args.hold_seconds, center, "hold")
            for direction in (1, -1):
                offset = list(center)
                offset[axis] = quantized(center[axis] + direction * args.step)
                collect(args.step / args.move_speed + .5, offset, "step_ramp", ramp=True)
                push = collect(args.step_hold, offset, "offset_hold")
                released = time.monotonic() - started
                recovery = collect(args.recovery_seconds, center, "return_step")
                trial = {"target": center[axis], "hold": hold, "push": push,
                         "release": recovery, "release_time": released}
                metrics = summarize(trial, args.tolerance, args.settle_dwell, max(3 * period, .1))
                metrics["direction"] = direction
                metrics["offset_mean_tracking_error_deg"] = sum(e - direction * args.step for _, e in push) / len(push)
                metrics["passed"] = passed(metrics, args)
                results.append(metrics)
            record["metrics"] = results
            record["status"] = "passed" if all(r["passed"] for r in results) else "failed_metrics"
    except BaseException as exc:
        record["status"], record["error"] = "fault", str(exc)
        raise
    finally:
        try:
            if motor is not None and owned:
                try:
                    if attempted:
                        motor.disable()
                        record["disable"] = "sent"
                finally:
                    motor.close()
            elif motor is not None:
                record["disable"] = "owned_by_session"
        except Exception as exc:
            record["status"], record["disable"] = "fault", str(exc)
            raise
        finally:
            save()
    return results


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device")
    parser.add_argument("--axis", choices=("yaw", "pitch", "both"), default="both")
    parser.add_argument("--center", type=float, nargs=2, metavar=("YAW", "PITCH"), help="test pose; omitted: initial measured pose")
    parser.add_argument("--enable", action="store_true", help="authorize automatic PI writes and small movements")
    parser.add_argument("--baseline-only", action="store_true", help="measure supplied PI without searching gains")
    parser.add_argument("--gains", nargs="+", choices=("speed_kp", "speed_ki", "pos_kp", "pos_ki"),
                        default=["speed_kp", "speed_ki", "pos_kp", "pos_ki"], help="gain stages to search")
    parser.add_argument("--hz", type=float, default=50)
    parser.add_argument("--move-speed", type=float, default=5)
    parser.add_argument("--step", type=float, default=3)
    parser.add_argument("--step-hold", type=float, default=2)
    parser.add_argument("--hold-seconds", type=float, default=3)
    parser.add_argument("--recovery-seconds", type=float, default=3)
    parser.add_argument("--tolerance", type=float, default=1)
    parser.add_argument("--settle-dwell", type=float, default=.5)
    parser.add_argument("--max-jitter", type=float, default=.6)
    parser.add_argument("--max-recovery", type=float, default=2)
    parser.add_argument("--abort-error", type=float, default=15)
    parser.add_argument("--stall-error", type=float, default=6, help="persistent tracking error threshold in degrees")
    parser.add_argument("--stall-seconds", type=float, default=1.5)
    parser.add_argument("--stall-motion", type=float, default=.3, help="maximum angle change indicating no progress")
    parser.add_argument("--validation-offsets", type=float, nargs="+", default=[0, -15, 15])
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "logs")
    config = Config()
    for axis in ("yaw", "pitch"):
        for gain in ("speed_kp", "speed_ki", "pos_kp", "pos_ki"):
            name = f"{axis}_{gain}"
            parser.add_argument("--" + name.replace("_", "-"), type=int, default=getattr(config, name))
    args = parser.parse_args(argv)
    for name in ("hz", "move_speed", "step", "step_hold", "hold_seconds", "recovery_seconds", "tolerance",
                 "settle_dwell", "max_jitter", "max_recovery", "abort_error",
                 "stall_error", "stall_seconds", "stall_motion"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"{name} must be finite and positive")
    if args.hz > 100 or args.move_speed > 60 or args.step > 10 or args.step <= args.tolerance:
        parser.error("hz <= 100, move-speed <= 60, tolerance < step <= 10 are required")
    if args.hold_seconds < 1 or args.recovery_seconds <= args.settle_dwell:
        parser.error("hold-seconds >= 1 and recovery-seconds > settle-dwell are required")
    if any(not math.isfinite(v) or abs(v) > 90 for v in args.validation_offsets):
        parser.error("validation offsets must be finite and within +/-90 degrees")
    if args.center is not None:
        if not all(math.isfinite(v) for v in args.center):
            parser.error("center angles must be finite")
        args.center = [quantized(v) for v in args.center]
    args.config = Config(device=args.device, **{name: getattr(args, name) for name in asdict(config) if name.endswith(("_kp", "_ki"))})
    try:
        args.config._validate()
    except ValueError as exc:
        parser.error(str(exc))
    return args


def run(args):
    print("Requested starting PI:", json.dumps(asdict(args.config)), flush=True)
    if not args.enable:
        print("Preview only. --enable runs both-direction steps and PI search; no save or zeroing.")
        return
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("autotune_%Y%m%d_%H%M%S_"), dir=args.output))
    manifest = {"status": "running", "settings": {k: v for k, v in vars(args).items() if k not in ("config", "output")},
                "initial_config": asdict(args.config), "experiments": [], "scope": "initial pose plus validation offsets; position steps, no external-force measurement"}
    def save():
        temporary = root / "run.json.tmp"
        temporary.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n")
        temporary.replace(root / "run.json")
    save()
    config = replace(args.config)
    motor = None
    attempted = False
    axes = (0, 1) if args.axis == "both" else (0 if args.axis == "yaw" else 1,)
    print(f"Recording to {root}", flush=True)
    try:
        motor = Gimbal2Dof(replace(config))
        motor.open()
        motor.start_log(root / "serial.log")
        center = [quantized(v) for v in checked_position(motor.read(), [None, None])]
        manifest["initial_pose_deg"] = list(center)
        if args.center is not None:
            center = list(args.center)
        manifest["test_center_deg"] = center
        for axis in axes:
            check_test_pose(center, axis, args.step)
            for offset in args.validation_offsets:
                position = list(center)
                position[axis] += offset
                check_test_pose(position, axis, args.step)
        attempted = True
        initial = manifest["initial_pose_deg"]
        motor.enable_at(*initial)
        def experiment(candidate, axis, position, label):
            directory = root / f"{len(manifest['experiments']):03d}_{label}"
            directory.mkdir()
            print(f"Testing {label}: {asdict(candidate)} at {position}", flush=True)
            result = evaluate(candidate, axis, position, args, directory, label, session=motor)
            manifest["experiments"].append({"path": directory.name, "config": asdict(candidate), "metrics": result})
            save()
            print(f"Result {label}: {json.dumps(result)}", flush=True)
            return result
        for axis in axes:
            name = ("yaw", "pitch")[axis]
            baseline = experiment(config, axis, center, f"{name}_baseline")
            best_score = score(baseline, args)
            best_passed = all(r["passed"] for r in baseline)
            if args.baseline_only:
                continue
            for gain in args.gains:
                field = f"{name}_{gain}"
                base = replace(config)
                values = candidate_values(getattr(base, field), gain.endswith("ki"))
                for value in values:
                    if value == getattr(base, field):
                        continue
                    candidate = replace(base, **{field: value})
                    result = experiment(candidate, axis, center, f"{field}_{value}")
                    candidate_score = score(result, args)
                    candidate_passed = all(r["passed"] for r in result)
                    better = candidate_score < best_score
                    if gain == "pos_kp" and candidate_passed:
                        better = not best_passed or value > getattr(config, field)
                    if better:
                        config, best_score = candidate, candidate_score
                        best_passed = candidate_passed
            manifest["best_so_far"] = asdict(config)
            save()
        valid = True
        for axis in axes:
            for offset in args.validation_offsets:
                position = list(center)
                position[axis] = quantized(position[axis] + offset)
                result = experiment(config, axis, position, f"validate_{axis}_{offset:g}")
                valid = all(r["passed"] for r in result) and valid
        manifest["status"] = "validated" if valid else "validation_failed"
        manifest["best_config"] = asdict(config)
        manifest["recommendation"] = asdict(config) if valid else None
        return valid
    except BaseException as exc:
        manifest["status"], manifest["error"] = "fault", str(exc)
        raise
    finally:
        try:
            if motor is not None:
                try:
                    if attempted:
                        motor.disable()
                        manifest["disable"] = "sent"
                finally:
                    motor.close()
        except Exception as exc:
            manifest["status"], manifest["disable"] = "fault", str(exc)
            raise
        finally:
            save()
            print(f"Result: {root / 'run.json'} ({manifest['status']})", flush=True)


def main(argv=None):
    args = parse_args(argv)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        return 2 if run(args) is False else 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"Autotuning stopped: {exc}", flush=True)
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
