#!/usr/bin/env python3
"""Offline Step 8 wheel-geometry sensitivity check.

Uses an already-recorded finish_nominal run. It does NOT move the robot and
does NOT use Isaac pose as an odometry input. Isaac ground truth is read only
for evaluation after wheel odometry is reconstructed.

The controlled perturbation is +5% wheel separation relative to the nominal
0.23993 m. This demonstrates how a plausible geometry error changes heading
estimation while replaying exactly the same wheel measurements.

Usage:
  python3 asn_step8_geometry_sensitivity.py
or
  python3 asn_step8_geometry_sensitivity.py --run /path/to/finish_nominal_...

Output:
  geometry_sensitivity.json in the selected run directory.
"""

import argparse
import csv
import json
import math
from pathlib import Path

NOMINAL_RADIUS = 0.03575
NOMINAL_SEPARATION = 0.23993
PERTURBED_SEPARATION = NOMINAL_SEPARATION * 1.05


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_xyzw(qx, qy, qz, qw):
    return math.atan2(
        2.0 * (qw*qz + qx*qy),
        1.0 - 2.0 * (qy*qy + qz*qz),
    )


def load_csv(path):
    with path.open(newline='') as f:
        return [
            {k: float(v) for k, v in row.items()}
            for row in csv.DictReader(f)
        ]


def integrate_joint_window(rows, start, end, radius, separation):
    pts = [r for r in rows if start <= r['t'] <= end]
    if len(pts) < 2:
        raise RuntimeError("Insufficient joint samples.")

    x = y = yaw = 0.0
    wraps_l = wraps_r = 0

    for a, b in zip(pts, pts[1:]):
        raw_l = b['left_q'] - a['left_q']
        raw_r = b['right_q'] - a['right_q']
        dl = wrap(raw_l)
        dr = wrap(raw_r)

        if abs(raw_l - dl) > math.pi:
            wraps_l += 1
        if abs(raw_r - dr) > math.pi:
            wraps_r += 1

        ds_l = radius * dl
        ds_r = radius * dr
        ds = 0.5 * (ds_l + ds_r)
        dtheta = (ds_r - ds_l) / separation

        mid = yaw + 0.5 * dtheta
        x += ds * math.cos(mid)
        y += ds * math.sin(mid)
        yaw += dtheta

    return {
        "x_m": x,
        "y_m": y,
        "distance_from_start_m": math.hypot(x, y),
        "yaw_change_rad": wrap(yaw),
        "yaw_change_deg": math.degrees(wrap(yaw)),
        "unwrapped_yaw_change_deg": math.degrees(yaw),
        "wrap_events_left": wraps_l,
        "wrap_events_right": wraps_r,
    }


def gt_window(isaac, start, end):
    pts = [r for r in isaac if start <= r['t'] <= end]
    if len(pts) < 2:
        raise RuntimeError("Insufficient Isaac samples.")

    a, b = pts[0], pts[-1]
    dx = b['x'] - a['x']
    dy = b['y'] - a['y']

    # New recorder files already contain yaw. Fall back to quaternion if needed.
    ya = a.get('yaw')
    yb = b.get('yaw')
    if ya is None:
        ya = yaw_from_xyzw(a['qx'], a['qy'], a['qz'], a['qw'])
        yb = yaw_from_xyzw(b['qx'], b['qy'], b['qz'], b['qw'])

    forward = math.cos(ya)*dx + math.sin(ya)*dy
    lateral = -math.sin(ya)*dx + math.cos(ya)*dy
    dyaw = wrap(yb - ya)

    return {
        "forward_m": forward,
        "lateral_m": lateral,
        "distance_from_start_m": math.hypot(dx, dy),
        "yaw_change_rad": dyaw,
        "yaw_change_deg": math.degrees(dyaw),
    }


def newest_run(project):
    candidates = []
    for p in (project / "results" / "step8").glob("finish_nominal_*"):
        if (p / "report.json").exists() and (p / "ros_joint_states.csv").exists() and (p / "isaac.csv").exists():
            candidates.append(p)
    if not candidates:
        raise RuntimeError("No complete finish_nominal Step 8 run found.")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path)
    args = parser.parse_args()

    project = Path.home() / "projects" / "autonomous-semantic-navigation"
    run = args.run.expanduser().resolve() if args.run else newest_run(project)

    report = json.loads((run / "report.json").read_text())
    joints = load_csv(run / "ros_joint_states.csv")
    isaac = load_csv(run / "isaac.csv")

    evals = {
        e["label"]: e
        for e in report.get("maneuver_evaluations", [])
    }

    labels = [
        "LEFT_TURN",
        "RIGHT_TURN",
        "SQUARE_TURN_1",
        "SQUARE_TURN_2",
        "SQUARE_TURN_3",
        "SQUARE_TURN_4",
        "SQUARE_WHOLE",
    ]

    results = []
    for label in labels:
        if label not in evals:
            continue

        window = evals[label]["window"]
        start = float(window["start"])
        end = float(window["end"])

        gt = gt_window(isaac, start, end)
        nominal = integrate_joint_window(
            joints, start, end, NOMINAL_RADIUS, NOMINAL_SEPARATION
        )
        perturbed = integrate_joint_window(
            joints, start, end, NOMINAL_RADIUS, PERTURBED_SEPARATION
        )

        nominal_error = math.degrees(
            wrap(math.radians(nominal["yaw_change_deg"] - gt["yaw_change_deg"]))
        )
        perturbed_error = math.degrees(
            wrap(math.radians(perturbed["yaw_change_deg"] - gt["yaw_change_deg"]))
        )

        results.append({
            "label": label,
            "ground_truth": gt,
            "nominal": nominal,
            "plus_5_percent_separation": perturbed,
            "nominal_heading_error_deg": nominal_error,
            "perturbed_heading_error_deg": perturbed_error,
            "error_change_deg": perturbed_error - nominal_error,
        })

    output = {
        "source_run": str(run),
        "test": "controlled wheel-separation geometry error",
        "nominal_wheel_radius_m": NOMINAL_RADIUS,
        "nominal_wheel_separation_m": NOMINAL_SEPARATION,
        "perturbed_wheel_separation_m": PERTURBED_SEPARATION,
        "perturbation_percent": 5.0,
        "note": (
            "Same recorded wheel measurements replayed offline. Isaac root pose "
            "is used only after reconstruction for evaluation."
        ),
        "results": results,
    }

    out = run / "geometry_sensitivity.json"
    out.write_text(json.dumps(output, indent=2))

    print("===== STEP 8 GEOMETRY SENSITIVITY =====")
    print("Run:", run)
    print(f"Nominal separation:   {NOMINAL_SEPARATION:.6f} m")
    print(f"Perturbed separation: {PERTURBED_SEPARATION:.6f} m (+5%)")
    print()
    for r in results:
        print(
            f"{r['label']}: "
            f"GT={r['ground_truth']['yaw_change_deg']:+.3f} deg | "
            f"nominal={r['nominal']['yaw_change_deg']:+.3f} "
            f"(err {r['nominal_heading_error_deg']:+.3f}) | "
            f"+5% b={r['plus_5_percent_separation']['yaw_change_deg']:+.3f} "
            f"(err {r['perturbed_heading_error_deg']:+.3f})"
        )

    print("\nSaved:", out)


if __name__ == "__main__":
    main()
