#!/usr/bin/env python3
"""
MEKF parameter tuning script — uses the Rust mekf_eval binary.

For each parameter combination it:
  1. Calls `mekf_eval <log.csv> --r_flow <v> ...` and parses stderr RMSE lines
  2. Averages XY RMSE across circle + figure-8 logs (most sensitive to flow tuning)
  3. Logs all results to tune_results.csv
  4. Prints a sorted summary at the end

Usage:
    python3 tune_mekf.py
"""

import os
import re
import csv
import itertools
import subprocess

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
DRONE_DIR = os.path.join(HERE, "..", "flying_drone_stack")
BINARY = os.path.join(DRONE_DIR, "target", "release", "mekf_eval")
RUNS_DIR = os.path.join(DRONE_DIR, "runs")
LOG_PATH = os.path.join(HERE, "tune_results.csv")

# Logs to sweep — dynamic manoeuvres are most sensitive to flow quality
LOGS = {
    "hover": "hover_2026-03-15_11-44-06.csv",
    "circle": "circle_2026-03-15_11-46-16.csv",
    "figure8": "figure8_2026-03-15_11-47-55.csv",
}

# ---------------------------------------------------------------------------
# Parameter grid
# Focus sweep: r_flow (main knob).  Others held at default unless added here.
# ---------------------------------------------------------------------------
GRID = {
    "r_flow": [0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 12.0, 20.0],
    # Uncomment to do a fuller search:
    # "q_vel":  [1e-4, 1e-3, 5e-3],
    # "q_pos":  [1e-8, 1e-7, 1e-6],
}

# ---------------------------------------------------------------------------
# RMSE parsing
# ---------------------------------------------------------------------------
# mekf_eval stderr lines look like:
#   "  x     : 13.10 cm  vs Lighthouse EKF  (42 samples)"
#   "  y     : 15.30 cm  vs Lighthouse EKF  (42 samples)"
#   "  z     : 0.47 cm  vs ToF range_z  (42 samples)"
#   "  roll  : 0.795°  (42 samples)"
_RE_CM = re.compile(r"^\s+(\w+)\s*:\s*([\d.]+)\s*cm", re.MULTILINE)
_RE_DEG = re.compile(r"^\s+(\w+)\s*:\s*([\d.]+)\s*deg", re.MULTILINE)


def parse_rmse(stderr: str) -> dict | None:
    result = {}
    for m in _RE_CM.finditer(stderr):
        result[m.group(1)] = float(m.group(2))  # cm
    for m in _RE_DEG.finditer(stderr):
        result[m.group(1)] = float(m.group(2))  # degrees
    if not result:
        return None
    return result


def run_eval(log_name: str, params: dict) -> dict | None:
    log_path = os.path.join(RUNS_DIR, log_name)
    cmd = [BINARY, log_path]
    for k, v in params.items():
        cmd += [f"--{k}", str(v)]
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return None
    return parse_rmse(proc.stderr)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    keys = list(GRID.keys())
    combos = list(itertools.product(*[GRID[k] for k in keys]))
    total = len(combos)
    print(f"Tuning grid: {total} combinations × {len(LOGS)} logs\n")
    print(
        f"{'run':>4}  {'r_flow':>8}  {'hover_xy':>9}  {'circle_xy':>10}  {'fig8_xy':>9}  {'mean_xy':>9}  {'mean_att':>9}"
    )
    print("─" * 70)

    log_rows = []
    best_mean = float("inf")
    best_id = -1

    for run_id, values in enumerate(combos):
        params = dict(zip(keys, values))
        r_flow = params.get("r_flow", 8.0)

        per_log = {}
        for tag, fname in LOGS.items():
            m = run_eval(fname, params)
            per_log[tag] = m

        # XY RMSE = sqrt((x²+y²)/2) per log
        def xy(m):
            if m is None or "x" not in m or "y" not in m:
                return float("nan")
            return ((m["x"] ** 2 + m["y"] ** 2) / 2) ** 0.5

        hover_xy = xy(per_log.get("hover"))
        circle_xy = xy(per_log.get("circle"))
        fig8_xy = xy(per_log.get("figure8"))

        # Average over dynamic logs (circle + figure8); hover is less informative for flow
        valid_xy = [v for v in [circle_xy, fig8_xy] if v == v]
        mean_xy = sum(valid_xy) / len(valid_xy) if valid_xy else float("nan")

        # Attitude: average roll+pitch RMS over all logs
        att_vals = []
        for m in per_log.values():
            if m and "roll" in m and "pitch" in m:
                att_vals.append((m["roll"] ** 2 + m["pitch"] ** 2) ** 0.5)
        mean_att = sum(att_vals) / len(att_vals) if att_vals else float("nan")

        marker = ""
        if mean_xy < best_mean:
            best_mean = mean_xy
            best_id = run_id
            best_val = params.copy()
            marker = "  ◀ best"

        print(
            f"{run_id:4d}  {r_flow:8.3f}  {hover_xy:9.2f}  {circle_xy:10.2f}"
            f"  {fig8_xy:9.2f}  {mean_xy:9.2f}  {mean_att:9.3f}{marker}"
        )

        row = {
            "run_id": run_id,
            **{f"param_{k}": v for k, v in params.items()},
            "hover_xy_cm": round(hover_xy, 3),
            "circle_xy_cm": round(circle_xy, 3),
            "fig8_xy_cm": round(fig8_xy, 3),
            "mean_xy_cm": round(mean_xy, 3),
            "mean_att_deg": round(mean_att, 4),
        }
        # also store per-log details
        for tag, m in per_log.items():
            if m:
                for k2, v2 in m.items():
                    row[f"{tag}_{k2}"] = round(v2, 4)
        log_rows.append(row)

    # Write CSV
    if log_rows:
        fieldnames = list(log_rows[0].keys())
        with open(LOG_PATH, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(log_rows)

    print("\n" + "─" * 70)
    print(f"Best run #{best_id}: mean XY = {best_mean:.2f} cm  params = {best_val}")
    print(f"Results saved to: {LOG_PATH}")


if __name__ == "__main__":
    main()
