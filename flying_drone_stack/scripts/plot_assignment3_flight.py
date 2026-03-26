#!/usr/bin/env python3
"""
Assignment 3 — MEKF: Real Flight Comparison vs Firmware EKF
============================================================
Reads the most recent 49-column run CSV from runs/ that contains mekf columns.
Compares the PC-side Rust MEKF (running live during flight) against the
Crazyflie's onboard Kalman EKF.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment3_flight.py
    # or pass a specific file:
    ~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment3_flight.py runs/figure8_2026-XX-XX.csv
"""

import os
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_ROOT = os.path.dirname(os.path.dirname(__file__))
IMG_DIR = os.path.join(_ROOT, "results", "assignment3", "images")
os.makedirs(IMG_DIR, exist_ok=True)

RAD2DEG = 180.0 / np.pi

# ── Load CSV ─────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    # Pick the most recent run that has mekf columns (49-col format)
    pattern = os.path.join(_ROOT, "runs", "*.csv")
    candidates = sorted(glob.glob(pattern))
    csv_path = None
    for p in reversed(candidates):
        with open(p) as f:
            header = f.readline()
        if "mekf_roll" in header:
            csv_path = p
            break
    if csv_path is None:
        sys.exit("No 49-col run CSV with mekf columns found in runs/.")

print(f"Loading: {csv_path}")
d = np.genfromtxt(csv_path, delimiter=",", names=True)

if "mekf_roll" not in d.dtype.names:
    sys.exit("CSV missing mekf_roll column. Need a 49-col run from --maneuver circle/figure8.")

t_ms = d["time_ms"]
t    = (t_ms - t_ms[0]) / 1000.0

# Firmware EKF (from stabilizer / stateEstimate)
fw_roll  =  d["roll"]          # degrees
fw_pitch =  d["pitch"]         # degrees
fw_yaw   =  d["yaw"]           # degrees
fw_x     =  d["pos_x"]         # metres
fw_y     =  d["pos_y"]
fw_z     =  d["pos_z"]

# Rust MEKF (PC-side, running live during flight)
mekf_roll  =  d["mekf_roll"]   # degrees
mekf_pitch = -d["mekf_pitch"]  # negate: sign convention mismatch (known issue)
mekf_yaw   =  d["mekf_yaw"]    # degrees
mekf_x     =  d["mekf_x"]
mekf_y     =  d["mekf_y"]
mekf_z     =  d["mekf_z"]

# ── Skip ground/pre-flight rows (range_z < 0.05 m) ───────────────────────────
airborne = d["range_z"] > 0.05
if airborne.any():
    start_idx = int(np.argmax(airborne))
else:
    start_idx = 0
t       = t[start_idx:]
fw_roll  = fw_roll[start_idx:];  mekf_roll  = mekf_roll[start_idx:]
fw_pitch = fw_pitch[start_idx:]; mekf_pitch = mekf_pitch[start_idx:]
fw_yaw   = fw_yaw[start_idx:];   mekf_yaw   = mekf_yaw[start_idx:]
fw_x     = fw_x[start_idx:];     mekf_x     = mekf_x[start_idx:]
fw_y     = fw_y[start_idx:];     mekf_y     = mekf_y[start_idx:]
fw_z     = fw_z[start_idx:];     mekf_z     = mekf_z[start_idx:]

# ── RMSE ─────────────────────────────────────────────────────────────────────
def rmse(a, b): return float(np.sqrt(np.mean((a - b)**2)))

roll_rmse  = rmse(fw_roll,  mekf_roll)
pitch_rmse = rmse(fw_pitch, mekf_pitch)
yaw_rmse   = rmse(fw_yaw,   mekf_yaw)
x_rmse     = rmse(fw_x,     mekf_x)
y_rmse     = rmse(fw_y,     mekf_y)
z_rmse     = rmse(fw_z,     mekf_z)

print(f"\nMEKF vs Firmware EKF RMSE (airborne only):")
print(f"  roll  = {roll_rmse:.2f}°")
print(f"  pitch = {pitch_rmse:.2f}°")
print(f"  yaw   = {yaw_rmse:.2f}°")
print(f"  x     = {x_rmse*100:.1f} cm")
print(f"  y     = {y_rmse*100:.1f} cm")
print(f"  z     = {z_rmse*100:.1f} cm")

maneuver = os.path.basename(csv_path).split("_")[0]

# ── Figure 1: Orientation ─────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle(f"Assignment 3 — MEKF vs Firmware EKF: Orientation  ({maneuver} flight)", fontsize=13)

angle_data = [
    ("Roll [deg]",  fw_roll,  mekf_roll,  roll_rmse),
    ("Pitch [deg]", fw_pitch, mekf_pitch, pitch_rmse),
    ("Yaw [deg]",   fw_yaw,   mekf_yaw,   yaw_rmse),
]
for ax, (ylabel, fw, mk, rmse_val) in zip(axes, angle_data):
    ax.plot(t, fw, color="tomato",    lw=1.2, alpha=0.85, label="Firmware EKF")
    ax.plot(t, mk, color="steelblue", lw=0.9, alpha=0.90, label="Rust MEKF (PC-side)")
    ax.set_ylabel(ylabel)
    ax.set_title(f"RMSE = {rmse_val:.2f}°", fontsize=9, loc="right")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
axes[-1].set_xlabel("Time [s]")
plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment3_flight_orientation.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()


# ── Figure 2: Position ────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
fig.suptitle(f"Assignment 3 — MEKF vs Firmware EKF: Position  ({maneuver} flight)", fontsize=13)

pos_data = [
    ("x [m]", fw_x, mekf_x, x_rmse),
    ("y [m]", fw_y, mekf_y, y_rmse),
    ("z [m]", fw_z, mekf_z, z_rmse),
]
for ax, (ylabel, fw, mk, rmse_val) in zip(axes, pos_data):
    ax.plot(t, fw, color="tomato",    lw=1.2, alpha=0.85, label="Firmware EKF")
    ax.plot(t, mk, color="steelblue", lw=0.9, alpha=0.90, label="Rust MEKF (PC-side)")
    ax.set_ylabel(ylabel)
    ax.set_title(f"RMSE = {rmse_val*100:.1f} cm", fontsize=9, loc="right")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)
axes[-1].set_xlabel("Time [s]")
plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment3_flight_position.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()


# ── Figure 3: XY trajectory comparison ───────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7))
ax.plot(fw_x,   fw_y,   color="tomato",    lw=1.4, alpha=0.85, label="Firmware EKF")
ax.plot(mekf_x, mekf_y, color="steelblue", lw=1.0, alpha=0.85, label="Rust MEKF (PC-side)")
ax.scatter([fw_x[0]],   [fw_y[0]],   c="tomato",    s=50, zorder=5)
ax.scatter([mekf_x[0]], [mekf_y[0]], c="steelblue",  s=50, zorder=5)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_aspect("equal")
ax.set_title(f"Assignment 3 — XY Trajectory: MEKF vs Firmware EKF  ({maneuver} flight)")
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment3_flight_xy.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()

print("\nDone.")
