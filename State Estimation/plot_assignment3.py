#!/usr/bin/env python3
"""
Plot Assignment 3: MEKF vs on-board EKF orientation comparison.

Loads mekf_results.npz (produced by mekf_offline.py) and produces:
  - assignment3_orientation.png  : roll / pitch / yaw vs time
  - assignment3_position.png     : x / y / z position MEKF vs on-board EKF
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

RAD2DEG = 180.0 / np.pi

# ---------------------------------------------------------------------------
# Load data
# ---------------------------------------------------------------------------
script_dir = os.path.dirname(__file__)
npz_path = os.path.join(script_dir, "mekf_results.npz")

if not os.path.exists(npz_path):
    raise FileNotFoundError(
        f"Results file not found: {npz_path}\n" "Run  python3 mekf_offline.py  first."
    )

d = np.load(npz_path)

mekf_t = d["mekf_t"]
mekf_roll = d["mekf_roll"] * RAD2DEG
mekf_pitch = d["mekf_pitch"] * RAD2DEG
mekf_yaw = d["mekf_yaw"] * RAD2DEG
mekf_x = d["mekf_x"]
mekf_y = d["mekf_y"]
mekf_z = d["mekf_z"]

ekf_t = d["ekf_t"]
ekf_roll = d["ekf_roll"] * RAD2DEG
ekf_pitch = d["ekf_pitch"] * RAD2DEG
ekf_yaw = d["ekf_yaw"] * RAD2DEG
ekf_x = d["ekf_x"]
ekf_y = d["ekf_y"]
ekf_z = d["ekf_z"]

# ---------------------------------------------------------------------------
# Figure 1: Orientation comparison
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle("MEKF vs On-board EKF — Orientation (figure-8 flight)", fontsize=13)

labels = ["Roll [deg]", "Pitch [deg]", "Yaw [deg]"]
mekf_ang = [mekf_roll, mekf_pitch, mekf_yaw]
ekf_ang = [ekf_roll, ekf_pitch, ekf_yaw]

for ax, ylabel, mekf_a, ekf_a in zip(axes, labels, mekf_ang, ekf_ang):
    ax.plot(
        mekf_t, mekf_a, color="steelblue", lw=0.8, alpha=0.85, label="MEKF (offline)"
    )
    ax.plot(ekf_t, ekf_a, color="tomato", lw=1.2, alpha=0.90, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes[-1].set_xlabel("Time [s]")
fig.tight_layout()
out1 = os.path.join(script_dir, "assignment3_orientation.png")
fig.savefig(out1, dpi=150)
print("Saved:", out1)

# ---------------------------------------------------------------------------
# Figure 2: Position comparison MEKF vs on-board EKF
# ---------------------------------------------------------------------------
fig2, axes2 = plt.subplots(3, 1, figsize=(12, 7), sharex=True)
fig2.suptitle("MEKF vs On-board EKF — Position (figure-8 flight)", fontsize=13)

mekf_pos_data = [mekf_x, mekf_y, mekf_z]
ekf_pos_data = [ekf_x, ekf_y, ekf_z]

for ax, ylabel, mp, ep in zip(
    axes2, ["x [m]", "y [m]", "z [m]"], mekf_pos_data, ekf_pos_data
):
    ax.plot(mekf_t, mp, color="steelblue", lw=0.8, alpha=0.85, label="MEKF (offline)")
    ax.plot(ekf_t, ep, color="tomato", lw=1.2, alpha=0.90, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes2[-1].set_xlabel("Time [s]")
fig2.tight_layout()
out2 = os.path.join(script_dir, "assignment3_position.png")
fig2.savefig(out2, dpi=150)
print("Saved:", out2)
