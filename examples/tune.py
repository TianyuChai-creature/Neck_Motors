#!/usr/bin/env python3
"""Fixed-load PI experiments. Preview by default; --enable requires confirmation."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import json
import math
from pathlib import Path
import select
import signal
from statistics import mean, pstdev
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python"))
from f32c import Config, Gimbal2Dof

PITCH_TEST_MAX = 40.0  # Leave 3 degrees clear of the observed head-down limit (~43 degrees).


def check_pitch_target(value):
    if not math.isfinite(value) or not -180 <= value <= PITCH_TEST_MAX:
        raise ValueError(f"pitch test target must be within [-180, {PITCH_TEST_MAX}] degrees")


def wrap180(value):
    return (value + 180.0) % 360.0 - 180.0


def quantized(value):
    ticks = int(math.fmod(value, 360) * 10) % 3600
    return (ticks - 3600 if ticks >= 1800 else ticks) / 10


def checked_position(state, limits):
    values = [state.yaw_deg, state.pitch_deg]
    if not all(math.isfinite(v) for v in values):
        raise ValueError("non-finite feedback")
    values = [wrap180(v) for v in values]
    if any(bounds is not None and not bounds[0] <= v <= bounds[1]
           for v, bounds in zip(values, limits)):
        raise ValueError(f"feedback outside mechanical limits: {values}")
    return values


def summarize(trial, tolerance, dwell, max_gap):
    """Recovery requires a continuous sampled suffix inside the target band."""
    hold = [error for _, error in trial["hold"]]
    disturb = [error for _, error in trial["push"]]
    recovery = trial["release"]
    observed = bool(disturb) and max(map(abs, disturb)) > tolerance
    settled = None
    if observed and recovery:
        start = None
        previous = trial["release_time"]
        for timestamp, error in recovery:
            if abs(error) > tolerance or timestamp - previous > max_gap:
                start = None
            if abs(error) <= tolerance and start is None:
                start = timestamp
            previous = timestamp
        if start is not None and recovery[-1][0] - start >= dwell:
            settled = start - trial["release_time"]
    gaps = [b[0] - a[0] for phase in ("hold", "push", "release")
            for a, b in zip(trial[phase], trial[phase][1:])]
    return {
        "target_deg": trial["target"],
        "hold_samples": len(hold),
        "hold_duration_s": trial["hold"][-1][0] - trial["hold"][0][0] if hold else 0,
        "hold_peak_to_peak_deg": max(hold) - min(hold) if hold else None,
        "hold_mean_error_deg": mean(hold) if hold else None,
        "hold_stddev_deg": pstdev(hold) if hold else None,
        "disturbance_peak_error_deg": max(map(abs, disturb)) if disturb else None,
        "disturbance_observed": observed,
        "recovery_samples": len(recovery),
        "recovery_peak_error_deg": max(abs(e) for _, e in recovery) if recovery else None,
        "recovery_time_s": settled,
        "actual_sample_hz": len(gaps) / sum(gaps) if gaps and sum(gaps) > 0 else None,
        "max_sample_gap_s": max(gaps) if gaps else None,
        "tolerance_deg": tolerance,
        "settle_dwell_s": dwell,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--axis", choices=("yaw", "pitch"), required=True)
    for name in ("speed-kp", "speed-ki", "pos-kp", "pos-ki"):
        parser.add_argument(f"--{name}", type=int, required=True)
    parser.add_argument("--yaw-range", type=float, nargs=2, metavar=("MIN", "MAX"), help="optional position bounds")
    parser.add_argument("--pitch-range", type=float, nargs=2, metavar=("MIN", "MAX"), help="optional position bounds")
    parser.add_argument("--device", help="omitted: SDK auto-scan")
    parser.add_argument("--hz", type=float, default=50, help="requested feedback rate, not guaranteed")
    parser.add_argument("--move-speed", type=float, default=5, help="target ramp in degrees/s")
    parser.add_argument("--tolerance", type=float, default=1, help="recovery band around target in degrees")
    parser.add_argument("--settle-dwell", type=float, default=2, help="minimum continuous recovery observation in seconds")
    parser.add_argument("--output", type=Path, default=ROOT / "logs")
    parser.add_argument("--enable", action="store_true", help="open hardware and confirm before enabling both axes")
    args = parser.parse_args(argv)
    args.limits = [args.yaw_range, args.pitch_range]
    for bounds in args.limits:
        if bounds is None:
            continue
        lo, hi = bounds
        if not (math.isfinite(lo) and math.isfinite(hi) and -180 <= lo < hi <= 180):
            parser.error("mechanical ranges must be finite and within [-180, 180], MIN < MAX")
    if args.pitch_range is not None and args.pitch_range[1] > PITCH_TEST_MAX:
        parser.error(f"pitch test range must not exceed {PITCH_TEST_MAX} degrees")
    for name in ("hz", "move_speed", "tolerance", "settle_dwell"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    if args.hz > 100 or args.move_speed > 60:
        parser.error("--hz must be <= 100 and --move-speed <= 60")
    args.config = Config(device=args.device)
    for name in ("speed_kp", "speed_ki", "pos_kp", "pos_ki"):
        setattr(args.config, f"{args.axis}_{name}", getattr(args, name))
    try:
        args.config._validate()
    except ValueError as exc:
        parser.error(str(exc))
    return args


def run(args):
    print(json.dumps({"requested_config": asdict(args.config), "limits": args.limits}, indent=2))
    if not args.enable:
        print("Preview only. Add --enable to run; SDK defaults are unchanged.")
        return
    print("Both axes will be enabled in direct mode and receive the printed PI values.")
    print("No zeroing or parameter save. Type ENABLE to proceed.")
    if input("> ").strip() != "ENABLE":
        return
    args.output.mkdir(parents=True, exist_ok=True)
    directory = Path(tempfile.mkdtemp(prefix=datetime.now().strftime("tune_%Y%m%d_%H%M%S_"), dir=args.output))
    metadata = {"requested_config": asdict(args.config), "axis": args.axis,
                "limits": args.limits, "requested_hz": args.hz, "move_speed_deg_s": args.move_speed,
                "tolerance_deg": args.tolerance, "settle_dwell_s": args.settle_dwell,
                "status": "opening", "events": [], "trials": []}
    def save():
        temporary = directory / "run.json.tmp"
        temporary.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
        temporary.replace(directory / "run.json")
    save()
    print(f"Recording to {directory}")
    motor = None
    enable_attempted = False
    trials = []
    trial = None
    phase = "idle"
    start = time.monotonic()
    axis = 0 if args.axis == "yaw" else 1
    period = 1 / args.hz
    # shortcut: no force sensor; displacement compares stiffness only under the same applied load.
    try:
        with (directory / "samples.csv").open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(["elapsed_s", "trial", "phase", "yaw_target_deg", "pitch_target_deg",
                             "yaw_measured_deg", "pitch_measured_deg", "axis_error_deg"])
            output.flush()
            motor = Gimbal2Dof(args.config)
            motor.open()
            motor.start_log(directory / "serial.log")
            commanded = checked_position(motor.read(), args.limits)
            if commanded[1] > 43:
                raise ValueError("initial pitch feedback exceeds the reported ~43 degree limit")
            enable_attempted = True
            motor.enable()
            # enable() reads again while applying PI; use its resulting hold position.
            commanded = checked_position(motor.read(), args.limits)
            if commanded[1] > 43:
                raise ValueError("pitch feedback crossed the reported ~43 degree limit during enable")
            motor.set_deg(*commanded)
            sent = [quantized(v) for v in commanded]
            desired = commanded[axis]
            metadata["status"] = "running"
            save()
            print("t DEG: ramp target; hold: begin trial; push: mark disturbance; release: mark removal;")
            print("report: current metrics; q / Ctrl-C / EOF: disable and exit. PI changes require a new run.")
            last = time.monotonic()
            next_display = last
            while True:
                tick = time.monotonic()
                dt, last = min(tick - last, 0.05), tick
                ready, _, _ = select.select([sys.stdin], [], [], 0)
                if ready:
                    line = sys.stdin.readline()
                    if not line or line.strip() == "q":
                        break
                    command = line.split()
                    try:
                        if len(command) == 2 and command[0] == "t":
                            value = float(command[1])
                            if not math.isfinite(value):
                                raise ValueError("target must be finite")
                            value = wrap180(value)
                            if axis == 1:
                                check_pitch_target(value)
                            bounds = args.limits[axis]
                            if bounds is not None and not bounds[0] <= value <= bounds[1]:
                                raise ValueError("target outside configured limits")
                            desired = value
                            phase, trial = "moving", None
                        elif command == ["hold"]:
                            check_pitch_target(commanded[1])
                            if abs(wrap180(commanded[axis] - desired)) > 1e-9:
                                raise ValueError("wait for target ramp to finish")
                            trial = {"target": sent[axis], "hold": [], "push": [], "release": [], "release_time": None}
                            trials.append(trial)
                            phase = "hold"
                        elif command == ["push"] and phase == "hold":
                            phase = "push"
                        elif command == ["release"] and phase == "push":
                            phase = "release"
                            trial["release_time"] = time.monotonic() - start
                        elif command == ["report"] and trial is not None:
                            print(json.dumps(summarize(trial, args.tolerance, args.settle_dwell, max(3 * period, .1)), indent=2))
                        else:
                            raise ValueError("use t DEG, hold, push, release, report or q; markers must follow that order")
                        metadata["events"].append({"elapsed_s": time.monotonic() - start, "command": line.strip(), "trial": len(trials)})
                        save()
                    except ValueError as exc:
                        print(f"Command rejected: {exc}")
                distance = desired - commanded[axis]
                if axis == 0 and args.limits[axis] is None:
                    distance = wrap180(distance)
                step = max(-args.move_speed * dt, min(args.move_speed * dt, distance))
                if step:
                    commanded[axis] = wrap180(commanded[axis] + step)
                    setter = motor.set_yaw_deg if axis == 0 else motor.set_pitch_deg
                    setter(commanded[axis])
                    sent[axis] = quantized(commanded[axis])
                measured = checked_position(motor.read(), args.limits)
                elapsed = time.monotonic() - start
                error = wrap180(measured[axis] - sent[axis])
                writer.writerow([elapsed, len(trials), phase, *sent, *measured, error])
                output.flush()
                if measured[1] > 43:
                    raise ValueError("pitch feedback crossed the reported ~43 degree limit")
                if trial is not None:
                    trial[phase].append((elapsed, error))
                if tick >= next_display:
                    print(f"{elapsed:.1f}s {phase}: target={sent[axis]:.1f}°, measured={measured[axis]:.1f}°, error={error:+.1f}°")
                    next_display = tick + 1
                time.sleep(max(0, period - (time.monotonic() - tick)))
        metadata["status"] = "finished"
    except KeyboardInterrupt:
        metadata["status"] = "interrupted"
    except BaseException as exc:
        metadata["status"] = "fault"
        metadata["error"] = str(exc)
        raise
    finally:
        try:
            if motor is not None:
                try:
                    if enable_attempted:
                        motor.disable()
                        metadata["disable"] = "sent"
                finally:
                    motor.close()
        except Exception as exc:
            metadata["disable"] = f"failed: {exc}"
            metadata["status"] = "fault"
            raise
        finally:
            metadata["trials"] = [summarize(t, args.tolerance, args.settle_dwell, max(3 * period, .1)) for t in trials]
            save()
            print(json.dumps({"status": metadata["status"], "disable": metadata.get("disable"), "trials": metadata["trials"]}, indent=2))


def main(argv=None):
    args = parse_args(argv)
    def terminate(_signal, _frame):
        raise KeyboardInterrupt
    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        run(args)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"Tuning stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
