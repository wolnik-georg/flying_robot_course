#!/usr/bin/env python3
"""
Assignment 2 — Geometric Controller: Real Flight Tracking Error
===============================================================
Reads the most recent my_circle CSV from runs/ (21-column format with ref_x/y/z).
Produces plots and prints cumulative + RMS position error.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment2_flight.py
    # or pass a specific file:
    ~/.pyenv/versions/flying_robots/bin/python scripts/plot_assignment2_flight.py runs/my_circle_2026-XX-XX.csv
"""

import os
import sys
import glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

_ROOT = os.path.dirname(os.path.dirname(__file__))
IMG_DIR = os.path.join(_ROOT, "results", "assignment2", "images")
os.makedirs(IMG_DIR, exist_ok=True)

# ── Load CSV ─────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    # Pick most recent my_circle run
    pattern = os.path.join(_ROOT, "runs", "my_circle_*.csv")
    candidates = sorted(glob.glob(pattern))
    if not candidates:
        sys.exit("No my_circle_*.csv found in runs/. Run: cargo run --release --bin main -- --maneuver my_circle")
    csv_path = candidates[-1]

print(f"Loading: {csv_path}")
d = np.genfromtxt(csv_path, delimiter=",", names=True)

# Confirm this file has ref columns
if "ref_x" not in d.dtype.names:
    sys.exit("CSV missing ref_x/y/z columns. Rebuild with the updated main.rs and do a new flight.")

t_ms   = d["time_ms"]
t      = (t_ms - t_ms[0]) / 1000.0   # seconds from start of flight loop

# Dead-reckoned estimate (relative to circle start — zeros during hover phase)
est_x = d["est_x"]
est_y = d["est_y"]
est_z = d["range_z"]      # ToF height

# Reference (set to hover_ref during settle phase, then circle)
ref_x = d["ref_x"]
ref_y = d["ref_y"]
ref_z = d["ref_z"]

# ── Detect circle phase ──────────────────────────────────────────────────────
# Circle reference moves; hover reference is fixed at (0, 0, 0.3).
# Circle phase = rows where ref_x changes significantly over time.
ref_moving = np.abs(np.gradient(ref_x)) + np.abs(np.gradient(ref_y)) > 1e-4
circle_start_idx = int(np.argmax(ref_moving)) if ref_moving.any() else 0
t_circle_start = t[circle_start_idx]
print(f"Circle phase starts at t = {t_circle_start:.1f} s (index {circle_start_idx})")

# Slice to circle phase only for error stats
t_c    = t[circle_start_idx:]
ex     = (ref_x - est_x)[circle_start_idx:]
ey     = (ref_y - est_y)[circle_start_idx:]
ez     = (ref_z - est_z)[circle_start_idx:]
e3d    = np.sqrt(ex**2 + ey**2 + ez**2)

rms_x  = float(np.sqrt(np.mean(ex**2)))
rms_y  = float(np.sqrt(np.mean(ey**2)))
rms_z  = float(np.sqrt(np.mean(ez**2)))
rms_3d = float(np.sqrt(np.mean(e3d**2)))
cum_3d = float(np.trapezoid(e3d, t_c))

print(f"\nCircle-phase tracking error (Rust geometric controller on real Crazyflie):")
print(f"  RMS 3D  = {rms_3d*1000:.1f} mm")
print(f"  RMS x   = {rms_x*1000:.1f} mm")
print(f"  RMS y   = {rms_y*1000:.1f} mm")
print(f"  RMS z   = {rms_z*1000:.1f} mm")
print(f"  Cumulative 3D error = {cum_3d:.3f} m·s")


# ── Figure 1: XY top-down ─────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7))
ax.plot(ref_x[circle_start_idx:], ref_y[circle_start_idx:], "b-",  lw=2.5, label="Reference (circle)")
ax.plot(est_x[circle_start_idx:], est_y[circle_start_idx:], "g-",  lw=1.5, alpha=0.85, label="Estimated position")
ax.scatter([ref_x[circle_start_idx]], [ref_y[circle_start_idx]], c="blue",  s=60, zorder=5, label="Start")
ax.scatter([est_x[circle_start_idx]], [est_y[circle_start_idx]], c="green", s=60, zorder=5)
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_aspect("equal")
ax.set_title("Assignment 2 — Geometric Controller: XY Trajectory (Real Flight)")
ax.legend()
ax.grid(True, alpha=0.35)
plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment2_flight_xy.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()


# ── Figure 2: x, y, z vs time ────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
labels = ["x [m]", "y [m]", "z [m]"]
refs   = [ref_x, ref_y, ref_z]
ests   = [est_x, est_y, est_z]
for i, ax in enumerate(axes):
    ax.axvline(t_circle_start, color="gray", ls=":", lw=1, label="Circle start")
    ax.plot(t, refs[i], "b-",  lw=1.8, label="Reference")
    ax.plot(t, ests[i], "g-",  lw=1.2, alpha=0.85, label="Estimated")
    ax.set_ylabel(labels[i])
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
axes[-1].set_xlabel("Time [s]")
axes[0].set_title("Assignment 2 — Position Tracking (Real Flight, Geometric Controller)")
plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment2_flight_position.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()


# ── Figure 3: 3D tracking error ───────────────────────────────────────────────
e3d_full = np.sqrt((ref_x - est_x)**2 + (ref_y - est_y)**2 + (ref_z - est_z)**2)
fig, axes = plt.subplots(2, 1, figsize=(12, 7))

ax = axes[0]
ax.axvline(t_circle_start, color="gray", ls=":", lw=1, label="Circle start")
ax.plot(t, e3d_full * 1000, "g-", lw=1.5)
ax.set_ylabel("3D error [mm]")
ax.set_title(f"Position Tracking Error (Circle-phase RMS = {rms_3d*1000:.1f} mm, Cumulative = {cum_3d:.3f} m·s)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.35)

ax = axes[1]
ax.plot(t_c, ex * 1000, lw=1.2, label=f"ex (RMS={rms_x*1000:.1f} mm)")
ax.plot(t_c, ey * 1000, lw=1.2, label=f"ey (RMS={rms_y*1000:.1f} mm)")
ax.plot(t_c, ez * 1000, lw=1.2, label=f"ez (RMS={rms_z*1000:.1f} mm)")
ax.axhline(0, color="k", lw=0.5)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Error [mm]")
ax.set_title("Per-axis error (circle phase only)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.35)

plt.tight_layout()
out = os.path.join(IMG_DIR, "assignment2_flight_error.png")
plt.savefig(out, dpi=150)
print(f"Saved {out}")
plt.close()

print("\nDone.")
