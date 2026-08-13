#!/usr/bin/env python3
"""
plot_comparison.py — Python MEKF vs Rust MEKF vs On-board EKF

Loads:
  - mekf_results.npz            (Python MEKF + on-board EKF baseline)
  - ../multirotor_simulator/results/data/assignment3_mekf.csv   (Rust MEKF)
  - ../multirotor_simulator/results/data/assignment3_ekf.csv    (Rust-parsed on-board EKF)

Produces:
  - comparison_orientation.png  : roll / pitch / yaw, all three sources
  - comparison_position.png     : x / y / z, all three sources
  - comparison_residuals.png    : (Rust MEKF − Python MEKF) per axis — implementation diff
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

RAD2DEG = 180.0 / np.pi
HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "..", "multirotor_simulator", "results", "data")

# ---------------------------------------------------------------------------
# Load Python MEKF results
# ---------------------------------------------------------------------------
npz = np.load(os.path.join(HERE, "mekf_results.npz"))
py_t = npz["mekf_t"]
py_roll = npz["mekf_roll"] * RAD2DEG
py_pitch = npz["mekf_pitch"] * RAD2DEG
py_yaw = npz["mekf_yaw"] * RAD2DEG
py_x, py_y, py_z = npz["mekf_x"], npz["mekf_y"], npz["mekf_z"]

ekf_t = npz["ekf_t"]
ekf_roll = npz["ekf_roll"] * RAD2DEG
ekf_pitch = npz["ekf_pitch"] * RAD2DEG
ekf_yaw = npz["ekf_yaw"] * RAD2DEG
ekf_x, ekf_y, ekf_z = npz["ekf_x"], npz["ekf_y"], npz["ekf_z"]


# ---------------------------------------------------------------------------
# Load Rust MEKF CSV
# ---------------------------------------------------------------------------
def load_csv(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    return data


rust = load_csv(os.path.join(SIM, "assignment3_mekf.csv"))
rs_t = rust["time"]
rs_roll = rust["roll_rad"] * RAD2DEG
rs_pitch = rust["pitch_rad"] * RAD2DEG
rs_yaw = rust["yaw_rad"] * RAD2DEG
rs_x, rs_y, rs_z = rust["x"], rust["y"], rust["z"]


# ---------------------------------------------------------------------------
# RMS helper
# ---------------------------------------------------------------------------
def rms_interp(t_ref, vals_ref, t_query, vals_query):
    """Interpolate vals_ref onto t_query grid, return RMS of difference."""
    mask = (t_query >= t_ref[0]) & (t_query <= t_ref[-1])
    f = interp1d(t_ref, vals_ref, kind="linear")
    diff = f(t_query[mask]) - vals_query[mask]
    return float(np.sqrt(np.mean(diff**2)))


# Print RMS table
print("=== Orientation RMS vs on-board EKF ===")
print(f"{'':20s}  {'roll':>7}  {'pitch':>7}  {'yaw':>7}")
for label, t, r, p, y in [
    ("Python MEKF", py_t, py_roll, py_pitch, py_yaw),
    ("Rust MEKF", rs_t, rs_roll, rs_pitch, rs_yaw),
]:
    rr = rms_interp(t, r, ekf_t, ekf_roll)
    rp = rms_interp(t, p, ekf_t, ekf_pitch)
    ry = rms_interp(t, y, ekf_t, ekf_yaw)
    print(f"  {label:18s}  {rr:7.3f}°  {rp:7.3f}°  {ry:7.3f}°")

print()
print("=== Python vs Rust MEKF residuals (implementation diff) ===")
for label, ref_v, rs_v in [
    ("roll", py_roll, rs_roll),
    ("pitch", py_pitch, rs_pitch),
    ("yaw", py_yaw, rs_yaw),
]:
    rms = rms_interp(py_t, ref_v, rs_t, rs_v)
    print(f"  {label:6s}  RMS={rms:.4f}°")

# ---------------------------------------------------------------------------
# Figure 1: Orientation — all three on same axes
# ---------------------------------------------------------------------------
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
fig.suptitle("Orientation: Python MEKF vs Rust MEKF vs On-board EKF", fontsize=13)

rows = [
    ("Roll [deg]", py_roll, rs_roll, ekf_roll),
    ("Pitch [deg]", py_pitch, rs_pitch, ekf_pitch),
    ("Yaw [deg]", py_yaw, rs_yaw, ekf_yaw),
]
for ax, (ylabel, pv, rv, ev) in zip(axes, rows):
    ax.plot(py_t, pv, color="steelblue", lw=0.8, alpha=0.75, label="Python MEKF")
    ax.plot(
        rs_t,
        rv,
        color="seagreen",
        lw=0.8,
        alpha=0.75,
        label="Rust MEKF",
        linestyle="--",
    )
    ax.plot(ekf_t, ev, color="tomato", lw=1.4, alpha=0.90, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes[-1].set_xlabel("Time [s]")
fig.tight_layout()
out1 = os.path.join(HERE, "comparison_orientation.png")
fig.savefig(out1, dpi=150)
print(f"\nSaved: {out1}")

# ---------------------------------------------------------------------------
# Figure 2: Position — all three on same axes
# ---------------------------------------------------------------------------
fig2, axes2 = plt.subplots(3, 1, figsize=(13, 8), sharex=True)
fig2.suptitle("Position: Python MEKF vs Rust MEKF vs On-board EKF", fontsize=13)

pos_rows = [
    ("x [m]", py_x, rs_x, ekf_x),
    ("y [m]", py_y, rs_y, ekf_y),
    ("z [m]", py_z, rs_z, ekf_z),
]
for ax, (ylabel, pv, rv, ev) in zip(axes2, pos_rows):
    ax.plot(py_t, pv, color="steelblue", lw=0.8, alpha=0.75, label="Python MEKF")
    ax.plot(
        rs_t,
        rv,
        color="seagreen",
        lw=0.8,
        alpha=0.75,
        label="Rust MEKF",
        linestyle="--",
    )
    ax.plot(ekf_t, ev, color="tomato", lw=1.4, alpha=0.90, label="On-board EKF")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

axes2[-1].set_xlabel("Time [s]")
fig2.tight_layout()
out2 = os.path.join(HERE, "comparison_position.png")
fig2.savefig(out2, dpi=150)
print(f"Saved: {out2}")

# ---------------------------------------------------------------------------
# Figure 3: Residuals — Rust MEKF minus Python MEKF (implementation diff)
# ---------------------------------------------------------------------------
fig3, axes3 = plt.subplots(3, 1, figsize=(13, 7), sharex=True)
fig3.suptitle(
    "Rust MEKF − Python MEKF residuals (f32 vs f64 implementation diff)", fontsize=12
)

# Interpolate Python onto Rust timestamps for clean subtraction
res_labels = ["Roll residual [deg]", "Pitch residual [deg]", "Yaw residual [deg]"]
py_angs = [py_roll, py_pitch, py_yaw]
rs_angs = [rs_roll, rs_pitch, rs_yaw]

for ax, ylabel, pv, rv in zip(axes3, res_labels, py_angs, rs_angs):
    mask = (rs_t >= py_t[0]) & (rs_t <= py_t[-1])
    f = interp1d(py_t, pv, kind="linear")
    residual = rv[mask] - f(rs_t[mask])
    ax.plot(rs_t[mask], residual, color="darkorange", lw=0.7, alpha=0.85)
    ax.axhline(0, color="k", lw=0.5, linestyle="--")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    rms_val = float(np.sqrt(np.mean(residual**2)))
    ax.set_title(f"RMS = {rms_val:.4f}°", fontsize=8, loc="right")

axes3[-1].set_xlabel("Time [s]")
fig3.tight_layout()
out3 = os.path.join(HERE, "comparison_residuals.png")
fig3.savefig(out3, dpi=150)
print(f"Saved: {out3}")
