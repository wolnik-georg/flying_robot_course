#!/usr/bin/env python3
"""
MEKF parameter tuning script.

For each parameter combination it:
  1. Runs the MEKF
  2. Computes RMS error vs on-board EKF for roll, pitch, yaw
  3. Logs all results to tune_results.csv
  4. Saves a comparison plot to tune_plots/<run_id>.png

Usage:
    python3 tune_mekf.py
"""

import os
import csv
import itertools
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

from mekf_offline import run_mekf, NoiseParams

RAD2DEG = 180.0 / np.pi
CSV_PATH = os.path.join(os.path.dirname(__file__), "logging_ekf", "logging", "fr00.csv")
PLOT_DIR = os.path.join(os.path.dirname(__file__), "tune_plots")
LOG_PATH = os.path.join(os.path.dirname(__file__), "tune_results.csv")

os.makedirs(PLOT_DIR, exist_ok=True)

# ---------------------------------------------------------------------------
# Parameter grid — each list entry is one candidate value
# ---------------------------------------------------------------------------
GRID = {
    "q_pos": [1e-7, 1e-6, 1e-5],
    "q_vel": [1e-5, 1e-4, 1e-3],
    "q_att": [1e-6, 1e-5, 1e-4],
    "r_height": [1e-4, 1e-3, 1e-2],
    "r_flow": [0.5, 2.0, 8.0],
}


# ---------------------------------------------------------------------------
# Metric: interpolate MEKF onto EKF timestamps, compute RMS error
# ---------------------------------------------------------------------------
def rms_vs_ekf(mekf_t, mekf_euler, ekf_t, ekf_euler):
    """
    Interpolate MEKF angles onto ekf_t grid, return per-axis and total RMS [deg].
    """
    results = {}
    total_sq = 0.0
    for i, name in enumerate(["roll", "pitch", "yaw"]):
        mekf_deg = mekf_euler[:, i] * RAD2DEG
        ekf_deg = ekf_euler[:, i] * RAD2DEG
        # only evaluate where EKF times fall within MEKF time range
        mask = (ekf_t >= mekf_t[0]) & (ekf_t <= mekf_t[-1])
        if mask.sum() < 2:
            return None
        f = interp1d(mekf_t, mekf_deg, kind="linear")
        diff = f(ekf_t[mask]) - ekf_deg[mask]
        rms = float(np.sqrt(np.mean(diff**2)))
        results[name] = rms
        total_sq += rms**2
    results["total"] = float(np.sqrt(total_sq / 3))
    return results


# ---------------------------------------------------------------------------
# Plot a single run
# ---------------------------------------------------------------------------
def save_plot(run_id, params, mekf_t, mekf_euler, ekf_t, ekf_euler, metrics):
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
    fig.suptitle(
        f"Run {run_id} | RMS total={metrics['total']:.3f}° "
        f"r={metrics['roll']:.2f}° p={metrics['pitch']:.2f}° y={metrics['yaw']:.2f}°\n"
        f"{params}",
        fontsize=8,
    )
    labels = ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"]
    for ax, lbl, col in zip(axes, labels, range(3)):
        ax.plot(
            mekf_t,
            mekf_euler[:, col] * RAD2DEG,
            color="steelblue",
            lw=0.7,
            alpha=0.85,
            label="MEKF",
        )
        ax.plot(
            ekf_t,
            ekf_euler[:, col] * RAD2DEG,
            color="tomato",
            lw=1.2,
            alpha=0.90,
            label="On-board EKF",
        )
        ax.set_ylabel(lbl)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    path = os.path.join(PLOT_DIR, f"run_{run_id:04d}.png")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# Main tuning loop
# ---------------------------------------------------------------------------
def main():
    keys = list(GRID.keys())
    combos = list(itertools.product(*[GRID[k] for k in keys]))
    total = len(combos)
    print(f"Tuning grid: {total} combinations")

    log_rows = []
    best_rms = float("inf")
    best_id = -1
    best_params = None

    for run_id, values in enumerate(combos):
        params = NoiseParams(**dict(zip(keys, values)))

        mekf_t, mekf_euler, mekf_pos, ekf_t, ekf_euler, ekf_pos = run_mekf(
            CSV_PATH, params
        )

        metrics = rms_vs_ekf(mekf_t, mekf_euler, ekf_t, ekf_euler)
        if metrics is None:
            print(f"  [{run_id+1}/{total}] SKIP (insufficient overlap)")
            continue

        plot_path = save_plot(
            run_id, params, mekf_t, mekf_euler, ekf_t, ekf_euler, metrics
        )

        row = {
            "run_id": run_id,
            "rms_total": round(metrics["total"], 4),
            "rms_roll": round(metrics["roll"], 4),
            "rms_pitch": round(metrics["pitch"], 4),
            "rms_yaw": round(metrics["yaw"], 4),
            **dict(zip(keys, values)),
            "plot": os.path.basename(plot_path),
        }
        log_rows.append(row)

        marker = ""
        if metrics["total"] < best_rms:
            best_rms = metrics["total"]
            best_id = run_id
            best_params = params
            marker = "  *** NEW BEST ***"

        print(
            f"  [{run_id+1:3d}/{total}] RMS={metrics['total']:.3f}° "
            f"(r={metrics['roll']:.2f} p={metrics['pitch']:.2f} y={metrics['yaw']:.2f}) "
            f"{marker}"
        )

    # Write log CSV
    fieldnames = list(log_rows[0].keys())
    with open(LOG_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(log_rows)

    print(f"\nResults saved to: {LOG_PATH}")
    print(f"Best run: #{best_id}  RMS={best_rms:.4f}°")
    print(f"Best params: {best_params}")
    print(f"Best plot:   {PLOT_DIR}/run_{best_id:04d}.png")

    # Also save the best result as the main mekf_results.npz
    mekf_t, mekf_euler, mekf_pos, ekf_t, ekf_euler, ekf_pos = run_mekf(
        CSV_PATH, best_params
    )
    out_path = os.path.join(os.path.dirname(__file__), "mekf_results.npz")
    np.savez(
        out_path,
        mekf_t=mekf_t,
        mekf_roll=mekf_euler[:, 0],
        mekf_pitch=mekf_euler[:, 1],
        mekf_yaw=mekf_euler[:, 2],
        mekf_x=mekf_pos[:, 0],
        mekf_y=mekf_pos[:, 1],
        mekf_z=mekf_pos[:, 2],
        ekf_t=ekf_t,
        ekf_roll=ekf_euler[:, 0],
        ekf_pitch=ekf_euler[:, 1],
        ekf_yaw=ekf_euler[:, 2],
        ekf_x=ekf_pos[:, 0],
        ekf_y=ekf_pos[:, 1],
        ekf_z=ekf_pos[:, 2],
    )
    print(f"Best results saved to: {out_path}")


if __name__ == "__main__":
    main()
