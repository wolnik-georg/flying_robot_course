#!/usr/bin/env python3
"""
Assignment 3 — MEKF vs On-board EKF (figure-8 flight, fr00.csv)
================================================================
Reads the two CSV files produced by `cargo run --bin assignment3`:
  results/data/assignment3_mekf.csv   — our Rust MEKF
  results/data/assignment3_ekf.csv    — on-board EKF baseline

Produces three PNG files in results/images/:
  assignment3_orientation.png  — roll / pitch / yaw vs time
  assignment3_position.png     — x / y / z vs time
  assignment3_xy.png           — XY top-down trajectory
"""

import os
import sys
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PROJ_ROOT = os.path.dirname(os.path.dirname(__file__))
DATA_DIR = os.path.join(PROJ_ROOT, "results", "data")
IMG_DIR = os.path.join(PROJ_ROOT, "results", "images")
os.makedirs(IMG_DIR, exist_ok=True)

RAD2DEG = 180.0 / np.pi


def load(name):
    path = os.path.join(DATA_DIR, name)
    if not os.path.exists(path):
        sys.exit(f"Missing: {path}\n" "Run:  cargo run --release --bin assignment3\n")
    return np.genfromtxt(path, delimiter=",", names=True)


mekf = load("assignment3_mekf.csv")
ekf = load("assignment3_ekf.csv")

mekf_t = mekf["time"]
mekf_roll = mekf["roll_rad"] * RAD2DEG
mekf_pitch = mekf["pitch_rad"] * RAD2DEG
mekf_yaw = mekf["yaw_rad"] * RAD2DEG
mekf_x, mekf_y, mekf_z = mekf["x"], mekf["y"], mekf["z"]

ekf_t = ekf["time"]
ekf_roll = ekf["roll_rad"] * RAD2DEG
ekf_pitch = ekf["pitch_rad"] * RAD2DEG
ekf_yaw = ekf["yaw_rad"] * RAD2DEG
ekf_x, ekf_y, ekf_z = ekf["x"], ekf["y"], ekf["z"]


# ---------------------------------------------------------------------------
# RMSE (interpolate EKF onto MEKF time grid for a fair comparison)
# ---------------------------------------------------------------------------
def rmse_interp(t_ref, v_ref, t_query, v_query):
    """RMSE of v_query interpolated onto t_ref grid vs v_ref."""
    v_interp = np.interp(t_ref, t_query, v_query)
    return float(np.sqrt(np.mean((v_ref - v_interp) ** 2)))


roll_rmse = rmse_interp(mekf_t, mekf_roll, ekf_t, ekf_roll)
pitch_rmse = rmse_interp(mekf_t, mekf_pitch, ekf_t, ekf_pitch)
yaw_rmse = rmse_interp(mekf_t, mekf_yaw, ekf_t, ekf_yaw)
x_rmse = rmse_interp(mekf_t, mekf_x, ekf_t, ekf_x)
y_rmse = rmse_interp(mekf_t, mekf_y, ekf_t, ekf_y)
z_rmse = rmse_interp(mekf_t, mekf_z, ekf_t, ekf_z)

print(f"Orientation RMSE (MEKF vs on-board EKF):")
print(f"  roll  = {roll_rmse:.3f}°")
print(f"  pitch = {pitch_rmse:.3f}°")
print(f"  yaw   = {yaw_rmse:.3f}°")
print(f"Position RMSE:")
print(f"  x = {x_rmse*100:.1f} cm")
print(f"  y = {y_rmse*100:.1f} cm")
print(f"  z = {z_rmse*100:.1f} cm")

# ---------------------------------------------------------------------------
# Figure 1: Orientation comparison
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle(
    "Assignment 3 — MEKF vs On-board EKF: Orientation (figure-8 flight)", fontsize=13
)

angle_data = [
    ("Roll [deg]", mekf_roll, ekf_roll, roll_rmse),
    ("Pitch [deg]", mekf_pitch, ekf_pitch, pitch_rmse),
    ("Yaw [deg]", mekf_yaw, ekf_yaw, yaw_rmse),
]

for ax, (ylabel, mv, ev, rmse) in zip(axes, angle_data):
    ax.plot(
        mekf_t, mv, color="steelblue", lw=0.8, alpha=0.9, label="MEKF (Rust, offline)"
    )
    ax.plot(ekf_t, ev, color="tomato", lw=1.2, alpha=0.85, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.set_title(f"RMSE = {rmse:.2f}°", fontsize=9, loc="right")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes[-1].set_xlabel("Time [s]")
fig.tight_layout()
out1 = os.path.join(IMG_DIR, "assignment3_orientation.png")
fig.savefig(out1, dpi=150)
print(f"Saved {out1}")
plt.close(fig)

# ---------------------------------------------------------------------------
# Figure 2: Position comparison
# ---------------------------------------------------------------------------
fig2, axes2 = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig2.suptitle(
    "Assignment 3 — MEKF vs On-board EKF: Position (figure-8 flight)", fontsize=13
)

pos_data = [
    ("x [m]", mekf_x, ekf_x, x_rmse),
    ("y [m]", mekf_y, ekf_y, y_rmse),
    ("z [m]", mekf_z, ekf_z, z_rmse),
]

for ax, (ylabel, mv, ev, rmse) in zip(axes2, pos_data):
    ax.plot(
        mekf_t, mv, color="steelblue", lw=0.8, alpha=0.9, label="MEKF (Rust, offline)"
    )
    ax.plot(ekf_t, ev, color="tomato", lw=1.2, alpha=0.85, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.set_title(f"RMSE = {rmse*100:.1f} cm", fontsize=9, loc="right")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes2[-1].set_xlabel("Time [s]")
fig2.tight_layout()
out2 = os.path.join(IMG_DIR, "assignment3_position.png")
fig2.savefig(out2, dpi=150)
print(f"Saved {out2}")
plt.close(fig2)

# ---------------------------------------------------------------------------
# Figure 3: XY top-down trajectory
# ---------------------------------------------------------------------------
fig3, ax3 = plt.subplots(figsize=(7, 7))
fig3.suptitle(
    "Assignment 3 — MEKF vs On-board EKF: XY Trajectory (figure-8 flight)", fontsize=12
)

ax3.plot(
    mekf_x, mekf_y, color="steelblue", lw=1.0, alpha=0.85, label="MEKF (Rust, offline)"
)
ax3.plot(ekf_x, ekf_y, color="tomato", lw=1.4, alpha=0.80, label="On-board EKF")
ax3.scatter([mekf_x[0]], [mekf_y[0]], color="steelblue", s=40, zorder=5)
ax3.scatter([ekf_x[0]], [ekf_y[0]], color="tomato", s=40, zorder=5)
ax3.set_xlabel("x [m]")
ax3.set_ylabel("y [m]")
ax3.set_aspect("equal")
ax3.grid(True, alpha=0.3)
ax3.legend(fontsize=9)
fig3.tight_layout()
out3 = os.path.join(IMG_DIR, "assignment3_xy.png")
fig3.savefig(out3, dpi=150)
print(f"Saved {out3}")
plt.close(fig3)
