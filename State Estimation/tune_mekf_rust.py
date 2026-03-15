#!/usr/bin/env python3
"""
MEKF parameter tuning — sweeps over mekf_eval (Rust binary) RMSE output.

For each parameter combination the script:
  1. Calls `mekf_eval <log.csv> --r_flow <v> ...` for each flight log
  2. Parses the RMSE lines from stderr
  3. Records combined XY RMSE across circle + figure8 logs
  4. Saves a summary CSV to tune_results_rust.csv
  5. Prints a ranked summary table

Usage:
    python3 tune_mekf_rust.py
"""

import os
import re
import csv
import subprocess
import itertools
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────────────
# Paths
# ──────────────────────────────────────────────────────────────────────────────
WORKSPACE = Path(__file__).parent.parent
BINARY = WORKSPACE / "flying_drone_stack" / "target" / "release" / "mekf_eval"
RUNS_DIR = WORKSPACE / "flying_drone_stack" / "runs"
OUT_CSV = Path(__file__).parent / "tune_results_rust.csv"

# Use dynamic manoeuvres only — hover has little XY excitation for tuning flow
LOGS = {
    "circle": "circle_2026-03-15_11-46-16.csv",
    "figure8": "figure8_2026-03-15_11-47-55.csv",
}

# ──────────────────────────────────────────────────────────────────────────────
# Parameter grid
# ──────────────────────────────────────────────────────────────────────────────
GRID = {
    "r_flow": [0.5, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0],
    # keep q_pos/q_vel/q_att/r_height at default unless you want a full sweep
    # "q_vel":   [1e-4, 1e-3, 5e-3],
}

# ──────────────────────────────────────────────────────────────────────────────
# Parse RMSE from mekf_eval stderr
# ──────────────────────────────────────────────────────────────────────────────
# Expected lines (written to stderr):
#   z     : 0.49 cm  vs ToF range_z  (N samples)
#   fw_z  : 0.29 cm  firmware EKF pos_z vs ToF  (reference baseline)
#   x     : 5.3 cm  vs Lighthouse EKF pos_x  (N samples)
#   y     : 5.6 cm  vs Lighthouse EKF pos_y
#   roll  : 0.897°  ...
RMSE_RE = re.compile(
    r"^\s*(roll|pitch|yaw|x|y|z|fw_z)\s*:\s*([\d.]+)\s*(cm|°)",
    re.MULTILINE,
)


def parse_rmse(stderr: str) -> dict:
    result = {}
    for m in RMSE_RE.finditer(stderr):
        key = m.group(1)
        value = float(m.group(2))
        unit = m.group(3)
        result[key] = value
        result[key + "_unit"] = unit
    return result


# ──────────────────────────────────────────────────────────────────────────────
# Run mekf_eval for one log with given params
# ──────────────────────────────────────────────────────────────────────────────
def run_eval(log_name: str, params: dict) -> dict | None:
    log_path = RUNS_DIR / log_name
    if not log_path.exists():
        print(f"  WARN: log not found: {log_path}")
        return None

    cmd = [str(BINARY), str(log_path)]
    for key, val in params.items():
        cmd += [f"--{key}", str(val)]

    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"  ERROR running mekf_eval: {result.stderr[-200:]}")
        return None

    return parse_rmse(result.stderr)


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    if not BINARY.exists():
        print(f"Binary not found: {BINARY}")
        print("Run:  cd flying_drone_stack && cargo build --release --bin mekf_eval")
        return

    keys = list(GRID.keys())
    combos = list(itertools.product(*[GRID[k] for k in keys]))
    total = len(combos)
    print(f"Sweep: {total} combinations × {len(LOGS)} logs = {total*len(LOGS)} runs\n")

    all_rows = []

    for run_id, values in enumerate(combos):
        params = dict(zip(keys, values))
        param_str = "  ".join(f"{k}={v}" for k, v in params.items())

        xy_sq_sum = 0.0
        xy_count = 0
        per_log = {}

        for maneuver, log_name in LOGS.items():
            rmse = run_eval(log_name, params)
            if rmse is None:
                continue
            per_log[maneuver] = rmse
            # XY RMSE in cm — combine across logs
            x_cm = rmse.get("x", float("nan"))
            y_cm = rmse.get("y", float("nan"))
            if x_cm == x_cm and y_cm == y_cm:  # not nan
                xy_sq_sum += x_cm**2 + y_cm**2
                xy_count += 2

        combined_xy = (xy_sq_sum / xy_count) ** 0.5 if xy_count > 0 else float("nan")

        # Print progress
        log_parts = []
        for maneuver, rmse in per_log.items():
            log_parts.append(
                f"{maneuver}: x={rmse.get('x','?'):.1f} y={rmse.get('y','?'):.1f} cm"
            )
        print(
            f"[{run_id+1:2d}/{total}] {param_str:<30}  combined_xy={combined_xy:.2f} cm"
            f"  [{' | '.join(log_parts)}]"
        )

        row = {
            "run_id": run_id,
            "combined_xy_rmse_cm": round(combined_xy, 3),
            **params,
        }
        for maneuver, rmse in per_log.items():
            for field in ("x", "y", "z", "roll", "pitch", "yaw"):
                row[f"{maneuver}_{field}_cm"] = round(rmse.get(field, float("nan")), 3)

        all_rows.append(row)

    if not all_rows:
        print("No results collected.")
        return

    # Sort by combined XY RMSE
    all_rows.sort(key=lambda r: r["combined_xy_rmse_cm"])

    # Write CSV
    fieldnames = list(all_rows[0].keys())
    with open(OUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"\nResults saved → {OUT_CSV}")
    print("\n─── Top 5 by combined XY RMSE ───────────────────────────────────────")
    for row in all_rows[:5]:
        param_str = "  ".join(f"{k}={row[k]}" for k in keys)
        print(f"  combined_xy={row['combined_xy_rmse_cm']:.2f} cm   {param_str}")

    best = all_rows[0]
    print(
        f"\n✓ Best: r_flow={best.get('r_flow')}  combined XY RMSE = {best['combined_xy_rmse_cm']:.2f} cm"
    )
    print(f"  (default r_flow=8.0 for comparison)")


if __name__ == "__main__":
    main()
