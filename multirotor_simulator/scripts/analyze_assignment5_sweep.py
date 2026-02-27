#!/usr/bin/env python3
"""
Analyze `assignment5` closed-loop CSV results and pick best actuator tuning combinations.

Outputs (into `results/data/`):
 - assignment5_metrics_summary.csv   : table of metrics per file
 - assignment5_metrics_summary.json  : same data as JSON

Prints recommendations for hover, circle and the figure8 sweep (Pareto front + recommended).
"""
import csv
import glob
import json
import math
import os
import re
from dataclasses import dataclass, asdict
from typing import Optional, List

import numpy as np

OUT_DIR = os.path.join("results", "data")
os.makedirs(OUT_DIR, exist_ok=True)


@dataclass
class Metrics:
    file: str
    mode: str
    tau: Optional[float]
    thrust_rate_limit: Optional[float]
    mean_acc: float
    max_acc: float
    mean_jerk: float
    max_jerk: float
    mean_pos_err: float
    max_pos_err: float
    n_samples: int


def compute_metrics(path: str) -> Optional[Metrics]:
    rows = []
    with open(path, "r") as f:
        r = csv.DictReader(f)
        for row in r:
            rows.append(row)
    if len(rows) < 2:
        return None
    dt = float(rows[1]["t"]) - float(rows[0]["t"])
    sim_vx = np.array([float(r["sim_vx"]) for r in rows])
    sim_vy = np.array([float(r["sim_vy"]) for r in rows])
    sim_vz = np.array([float(r["sim_vz"]) for r in rows])
    ax = np.concatenate(([0.0], np.diff(sim_vx) / dt))
    ay = np.concatenate(([0.0], np.diff(sim_vy) / dt))
    az = np.concatenate(([0.0], np.diff(sim_vz) / dt))
    acc = np.sqrt(ax * ax + ay * ay + az * az)
    jx = np.concatenate(([0.0], np.diff(ax) / dt))
    jy = np.concatenate(([0.0], np.diff(ay) / dt))
    jz = np.concatenate(([0.0], np.diff(az) / dt))
    jerk = np.sqrt(jx * jx + jy * jy + jz * jz)
    ref_x = np.array([float(r["ref_x"]) for r in rows])
    ref_y = np.array([float(r["ref_y"]) for r in rows])
    ref_z = np.array([float(r["ref_z"]) for r in rows])
    sim_x = np.array([float(r["sim_x"]) for r in rows])
    sim_y = np.array([float(r["sim_y"]) for r in rows])
    sim_z = np.array([float(r["sim_z"]) for r in rows])
    pos_err = np.sqrt(
        (ref_x - sim_x) ** 2 + (ref_y - sim_y) ** 2 + (ref_z - sim_z) ** 2
    )

    # parse filename to determine mode and optional params
    base = os.path.basename(path)
    mode = "unknown"
    tau = None
    R = None
    if "hover" in base:
        mode = "hover"
    elif "circle" in base:
        mode = "circle"
    elif "figure8" in base:
        mode = "figure8"
    # try find tau/R for figure8 sweep files
    m = re.search(r"tau([0-9\.]+)_R([0-9\.]+)", base)
    if m:
        try:
            tau = float(m.group(1))
            R = float(m.group(2))
        except Exception:
            tau = None
            R = None

    return Metrics(
        file=path,
        mode=mode,
        tau=tau,
        thrust_rate_limit=R,
        mean_acc=float(np.nanmean(acc)),
        max_acc=float(np.nanmax(acc)),
        mean_jerk=float(np.nanmean(jerk)),
        max_jerk=float(np.nanmax(jerk)),
        mean_pos_err=float(np.nanmean(pos_err)),
        max_pos_err=float(np.nanmax(pos_err)),
        n_samples=len(rows),
    )


def find_files():
    files = sorted(glob.glob(os.path.join(OUT_DIR, "assignment5_closedloop_*.csv")))
    return files


def pareto_front(metrics: List[Metrics], x_key: str, y_key: str) -> List[Metrics]:
    # minimize both x_key and y_key
    pts = sorted(metrics, key=lambda m: (getattr(m, x_key), getattr(m, y_key)))
    front = []
    best_y = math.inf
    for p in pts:
        y = getattr(p, y_key)
        if y < best_y - 1e-12:
            front.append(p)
            best_y = y
    return front


def main():
    files = find_files()
    if not files:
        print(
            "No closed-loop assignment5 CSVs found in results/data/. Run the sims first."
        )
        return

    metrics = []
    for f in files:
        m = compute_metrics(f)
        if m:
            metrics.append(m)

    # write a CSV summary table
    csv_out = os.path.join(OUT_DIR, "assignment5_metrics_summary.csv")
    json_out = os.path.join(OUT_DIR, "assignment5_metrics_summary.json")
    with open(csv_out, "w", newline="") as cf:
        w = csv.writer(cf)
        w.writerow(
            [
                "file",
                "mode",
                "tau",
                "thrust_rate_limit",
                "n_samples",
                "mean_acc",
                "max_acc",
                "mean_jerk",
                "max_jerk",
                "mean_pos_err",
                "max_pos_err",
            ]
        )
        for m in metrics:
            w.writerow(
                [
                    m.file,
                    m.mode,
                    m.tau if m.tau is not None else "",
                    m.thrust_rate_limit if m.thrust_rate_limit is not None else "",
                    m.n_samples,
                    f"{m.mean_acc:.12f}",
                    f"{m.max_acc:.12f}",
                    f"{m.mean_jerk:.12f}",
                    f"{m.max_jerk:.12f}",
                    f"{m.mean_pos_err:.12f}",
                    f"{m.max_pos_err:.12f}",
                ]
            )

    with open(json_out, "w") as jf:
        json.dump([asdict(m) for m in metrics], jf, indent=2)

    print(f"Wrote summary CSV: {csv_out}")
    print(f"Wrote summary JSON: {json_out}")

    # report hover & circle
    print("\n--- Hover / Circle ---")
    for mode in ("hover", "circle"):
        cand = [m for m in metrics if m.mode == mode]
        if not cand:
            print(f"{mode}: no data")
            continue
        m = cand[0]
        print(
            f"{mode}: mean_acc={m.mean_acc:.6f}, max_acc={m.max_acc:.6f}, mean_jerk={m.mean_jerk:.6f}, max_jerk={m.max_jerk:.6f}, mean_pos_err={m.mean_pos_err:.6f}, max_pos_err={m.max_pos_err:.6f}"
        )

    # figure8 analysis
    fig = [m for m in metrics if m.mode == "figure8"]
    if not fig:
        print("\nNo figure8 closed-loop files found")
        return

    # rank by max_jerk then mean_pos_err
    fig_sorted = sorted(fig, key=lambda x: (x.max_jerk, x.mean_pos_err))
    print("\n--- Figure8: top candidates (by max_jerk then mean_pos_err) ---")
    for i, m in enumerate(fig_sorted[:10]):
        print(
            f"{i+1}. tau={m.tau}, R={m.thrust_rate_limit}: max_jerk={m.max_jerk:.3f}, mean_jerk={m.mean_jerk:.3f}, mean_pos_err={m.mean_pos_err:.6f}"
        )

    # Pareto front (max_jerk vs mean_pos_err)
    front = pareto_front(fig, "max_jerk", "mean_pos_err")
    print("\nPareto front (max_jerk vs mean_pos_err):")
    for m in front:
        print(
            f"tau={m.tau}, R={m.thrust_rate_limit}: max_jerk={m.max_jerk:.3f}, mean_pos_err={m.mean_pos_err:.6f}"
        )

    # recommended: lowest max_jerk with mean_pos_err < 0.05, else lowest max_jerk overall
    candidates = [m for m in fig_sorted if m.mean_pos_err < 0.05]
    recommended = candidates[0] if candidates else fig_sorted[0]
    print("\nRecommended figure8 configuration:")
    print(
        f"tau={recommended.tau}, R={recommended.thrust_rate_limit}: max_jerk={recommended.max_jerk:.3f}, mean_pos_err={recommended.mean_pos_err:.6f}"
    )

    print("\nDone.")


if __name__ == "__main__":
    main()
