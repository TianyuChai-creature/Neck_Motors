#!/usr/bin/env python3
"""Simulated head input and generated commands; no motor or camera is opened."""

import argparse
import csv
import math
from pathlib import Path
import random
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from f32c import HeadTracker, State, TrackingConfig


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, help="write a new CSV; an existing file is never overwritten")
    args = parser.parse_args(argv)
    # These are simulation bounds, not a calibration of a physical device.
    tracker = HeadTracker(State(0, 0), TrackingConfig(pitch_limits=(-30, 40)), timestamp=0)
    rng = random.Random(52)
    rows = []
    for i in range(1, 1001):
        now = i * 0.02
        if now <= 2:
            yaw = 0.0
        elif now <= 2.5:
            yaw = 60 * (now - 2)
        elif now <= 5:
            yaw = 30.0
        elif now <= 10:
            yaw = 30 - 0.2 * (now - 5)
        else:
            yaw = 29 + 10 * math.sin((now - 10) * math.pi)
        raw_yaw, raw_pitch = yaw + rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06)
        tracker.set_target(raw_yaw, raw_pitch, timestamp=now)
        command = tracker.step(timestamp=now)
        filtered = tracker.filtered_target
        rows.append((now, raw_yaw, raw_pitch, filtered.yaw_deg, filtered.pitch_deg,
                     command.yaw_deg, command.pitch_deg))
    if args.csv:
        with args.csv.open("x", newline="") as output:
            writer = csv.writer(output)
            writer.writerow(("time_s", "input_yaw_deg", "input_pitch_deg", "filtered_yaw_deg",
                             "filtered_pitch_deg", "command_yaw_deg", "command_pitch_deg"))
            writer.writerows(rows)
    static = [r[5] for r in rows if r[0] <= 2]
    ramp = [(60 * (r[0] - 2) - r[5]) / 60 for r in rows if 2.2 <= r[0] <= 2.5]
    stopped = next(r for r in rows if abs(r[0] - 4) < 1e-9)
    print("Simulation only: these are generated commands, not measured motor feedback.")
    print(f"Static generated yaw span: {max(static) - min(static):.3f} deg")
    print(f"60 deg/s input: maximum generated lag after start: {max(ramp) * 1000:.1f} ms")
    print(f"Stopped input: generated error at 4 s: {abs(stopped[5] - 30):.3f} deg")


if __name__ == "__main__":
    main()
