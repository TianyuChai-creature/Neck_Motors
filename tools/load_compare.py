#!/usr/bin/env python3
"""Fixed pitch comparison using the legacy two-layer Neck.set_target()/tick() path; preview by default."""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import asdict, replace
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from neck.control import Neck, ProtectionConfig, TrajectoryConfig
from neck.driver import Driver, DriverConfig

PERIOD = .033  # UI timer period; actual tick times are recorded.
PLAN = [("startup", 0., 8.)] + [
    (f"center_{center}_{index}", float(target), 6.)
    for center in (0, 20, 30)
    for index, target in enumerate((center, center + 3, center - 3, center))
] + [("return", 0., 6.)]


def check_snapshot(snapshot, window, now):
    if snapshot.level or snapshot.phase in ("fault", "degraded"):
        raise RuntimeError(f"protection stop: {snapshot.reason}")
    if snapshot.measured is None or snapshot.command is None:
        raise RuntimeError("missing feedback or command")
    window.append((now, snapshot.measured.pitch, abs(snapshot.command.pitch - snapshot.measured.pitch)))
    while len(window) > 1 and window[1][0] <= now - 1.5:
        window.popleft()
    if now - window[0][0] >= 1.5:
        angles = [row[1] for row in window]
        if all(row[2] > 1 for row in window) and max(angles) - min(angles) <= .3 + 1e-9:
            raise RuntimeError("no progress: command error >1 degree for 1.5s, motion <=0.3 degree")


def run(args):
    yaw_target = args.yaw_target
    if yaw_target is not None and (not math.isfinite(yaw_target) or not -60 <= yaw_target <= 60):
        raise ValueError("yaw target must be finite and within SDK limits [-60, 60]")
    config = DriverConfig(port=args.device)
    config = replace(config, pitch_gains=replace(config.pitch_gains, speed_kp=args.pitch_speed_kp))
    metadata = {"load": args.load,
                "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "requested_driver": asdict(config),
                "trajectory": asdict(TrajectoryConfig()), "protection": asdict(ProtectionConfig()),
                "plan": PLAN, "yaw_target_deg": yaw_target, "period_s": PERIOD,
                "unmeasured": ["current", "bus_voltage", "motor_speed_feedback", "parameter_acceptance"],
                "physical_stop_confirmed": False, "status": "preview"}
    print(json.dumps(metadata, indent=2))
    if not args.run:
        return None
    # An exclusive directory prevents overwriting either load state or historical runs.
    args.output.mkdir(parents=True, exist_ok=False)
    driver = neck = None
    metadata["status"] = "opening"
    def save():
        temporary = args.output / "run.json.tmp"
        temporary.write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
        temporary.replace(args.output / "run.json")
    save()
    try:
        with (args.output / "samples.jsonl").open("x") as output:
            driver = Driver.open(replace(config, log_path=str(args.output / "serial.log")))
            neck = Neck(driver)
            initial = driver.read()
            yaw_target = initial.yaw if yaw_target is None else yaw_target
            metadata["yaw_target_deg"] = yaw_target
            metadata["initial_pose"] = asdict(initial)
            save()
            if not -60 <= initial.yaw <= 60 or not -3 <= initial.pitch <= 45:
                raise RuntimeError("initial pose outside approved envelope: yaw -60..60, pitch -3..45")
            neck.enable()
            # Replace the queued zero move before the first tick; yaw must stay here.
            neck.set_target(yaw_target, PLAN[0][1])
            metadata["status"] = "running"
            save()
            for label, target, duration in PLAN:
                start = time.monotonic()
                window, tail = deque(), deque()
                while time.monotonic() - start < duration:
                    tick = time.monotonic()
                    try:
                        # Same live-input path as Panel._step; no Quest source is connected.
                        neck.set_target(yaw_target, target)
                        snapshot = neck.tick()
                    finally:
                        row = {"wall_time": datetime.now().astimezone().isoformat(),
                               "monotonic_s": time.monotonic(), "segment": label,
                               "segment_elapsed_s": time.monotonic() - start,
                               "input": {"yaw": yaw_target, "pitch": target},
                               "snapshot": asdict(neck.snapshot)}
                        output.write(json.dumps(row, allow_nan=False) + "\n")
                        output.flush()
                    now = time.monotonic()
                    check_snapshot(snapshot, window, now)
                    tail.append((now, snapshot.measured))
                    while tail and tail[0][0] < now - 1:
                        tail.popleft()
                    time.sleep(max(0, PERIOD - (time.monotonic() - tick)))
                if not tail or tail[-1][0] - tail[0][0] < .8:
                    raise RuntimeError(f"insufficient final feedback: {label}")
                if any(abs(p.pitch - target) > 1 or abs(p.yaw - yaw_target) > 1 for _, p in tail):
                    raise RuntimeError(f"not settled in final second: {label}")
                if max(p.pitch for _, p in tail) - min(p.pitch for _, p in tail) > 1 + 1e-9:
                    raise RuntimeError(f"hold swing >1 degree: {label}")
            metadata["status"] = "completed"
    except BaseException as exc:
        metadata.update(status="stopped", error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        # Two separately reported attempts, bounded by Driver's per-frame deadlines.
        # A successful request does not establish physical torque-off or stopping.
        if driver is not None:
            try:
                driver.disable()
                metadata["disable_request"] = "sent; hardware unconfirmed"
            except Exception as exc:
                metadata.update(status="cleanup_failed", disable_request=f"failed: {exc}")
            try:
                driver.close()
                metadata["close"] = "returned"
            except Exception as exc:
                metadata.update(status="cleanup_failed", close=f"failed: {exc}")
            metadata["driver_state"] = driver.state.value
        else:
            metadata.update(disable_request="not attempted: open did not return a driver",
                            close="not applicable: no driver handle")
        save()
        print(json.dumps(metadata, indent=2))
    if metadata["status"] != "completed":
        raise RuntimeError("cleanup failed; stop and inspect hardware, do not re-enable")
    return args.output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--load", choices=("A", "B"), required=True)
    parser.add_argument("--device", required=True, help="explicit serial path; no port scan")
    parser.add_argument("--output", type=Path, required=True, help="new, exclusive run directory")
    parser.add_argument("--yaw-target", type=float, help="omitted: hold initial measured yaw; candidate reuses baseline yaw")
    parser.add_argument("--pitch-speed-kp", type=int, choices=(10, 12), default=10,
                        help="baseline 10 or the single experiment-two candidate 12")
    parser.add_argument("--run", action="store_true", help="hardware motion: human approval required first")
    args = parser.parse_args(argv)
    def stop(_signal, _frame):
        raise KeyboardInterrupt("operator stop")
    previous = signal.signal(signal.SIGTERM, stop)
    try:
        run(args)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"Experiment stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    raise SystemExit(main())
