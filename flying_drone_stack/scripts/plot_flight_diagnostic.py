#!/usr/bin/env python3
"""plot_flight_diagnostic.py — Phase 0 flight diagnostic plots.

Loads a run CSV (43-column format from run_firmware_mode) and produces
four diagnostic figures:

  1. EKF position x/y/z over time + XY ground track
  2. EKF vs MEKF orientation (roll, pitch, yaw) + RMS errors
  3. Raw sensors: flow_dx/dy, range_z, battery voltage
  4. Multi-ranger front/back/left/right/up (if non-zero)

Usage
-----
    # Most-recent CSV in runs/:
    python scripts/plot_flight_diagnostic.py

    # Specific file:
    python scripts/plot_flight_diagnostic.py runs/hover_2026-03-16_18-54-07.csv

Output: results/images/diag_<filename>.png (one per figure panel)
"""

import sys
import os
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RUNS_DIR = os.path.join(ROOT, "runs")
IMG_DIR  = os.path.join(ROOT, "results", "images")
os.makedirs(IMG_DIR, exist_ok=True)

# ── pick file ─────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    files = sorted(glob.glob(os.path.join(RUNS_DIR, "*.csv")))
    if not files:
        print("No CSV files found in runs/")
        sys.exit(1)
    csv_path = files[-1]

print(f"Loading: {csv_path}")
df = pd.read_csv(csv_path)
t  = df["time_ms"] / 1000.0  # → seconds
stem = os.path.splitext(os.path.basename(csv_path))[0]

# ── helpers ───────────────────────────────────────────────────────────────────
def rms(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))

def save(fig, tag):
    path = os.path.join(IMG_DIR, f"diag_{stem}_{tag}.png")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    print(f"  Saved: {path}")
    plt.close(fig)

# ── Figure 1: EKF position ────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle(f"EKF Position — {stem}", fontsize=11)

ax = axes[0, 0]
ax.plot(t, df["pos_x"], label="x")
ax.plot(t, df["pos_y"], label="y")
ax.set_ylabel("m"); ax.set_title("EKF x / y"); ax.legend(); ax.grid(True)

ax = axes[0, 1]
ax.plot(t, df["pos_z"], color="green")
ax.set_ylabel("m"); ax.set_title("EKF z (height)"); ax.grid(True)

ax = axes[1, 0]
ax.plot(df["pos_x"], df["pos_y"])
ax.plot(df["pos_x"].iloc[0], df["pos_y"].iloc[0], "go", label="start")
ax.plot(df["pos_x"].iloc[-1], df["pos_y"].iloc[-1], "rs", label="end")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("XY ground track")
ax.set_aspect("equal"); ax.legend(); ax.grid(True)

ax = axes[1, 1]
speed = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2 + df["vel_z"]**2)
ax.plot(t, speed, color="purple")
ax.set_ylabel("m/s"); ax.set_xlabel("time [s]"); ax.set_title("EKF speed"); ax.grid(True)

save(fig, "position")

# ── Figure 2: EKF vs MEKF orientation ────────────────────────────────────────
has_mekf = all(c in df.columns for c in ["mekf_roll", "mekf_pitch", "mekf_yaw"])

fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig.suptitle(f"EKF vs MEKF Orientation — {stem}", fontsize=11)

labels = ["roll", "pitch", "yaw"]
ekf_cols  = ["roll", "pitch", "yaw"]
mekf_cols = ["mekf_roll", "mekf_pitch", "mekf_yaw"]

for i, (ax, lbl, ec, mc) in enumerate(zip(axes, labels, ekf_cols, mekf_cols)):
    ax.plot(t, df[ec], label="firmware EKF", linewidth=1.2)
    if has_mekf:
        mekf_deg = df[mc]  # already in degrees (converted in fw_logging_step)
        # exclude pre-seeding rows (mekf_roll == 0 before range_z > 0.1)
        seeded = mekf_deg != 0.0
        ax.plot(t, mekf_deg, label="shadow MEKF", linewidth=1.0, linestyle="--")
        if seeded.sum() > 10:
            r = rms(df[ec][seeded].values, mekf_deg[seeded].values)
            ax.set_title(f"{lbl}  (RMSE seeded = {r:.2f}°)")
        else:
            ax.set_title(lbl + "  (MEKF not seeded)")
    else:
        ax.set_title(lbl)
    ax.set_ylabel("deg"); ax.legend(fontsize=8); ax.grid(True)

axes[-1].set_xlabel("time [s]")
save(fig, "orientation")

# ── Figure 3: raw sensors ─────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
fig.suptitle(f"Raw Sensors — {stem}", fontsize=11)

ax = axes[0]
ax.plot(t, df["flow_dx"], label="flow_dx [px]", linewidth=0.8)
ax.plot(t, df["flow_dy"], label="flow_dy [px]", linewidth=0.8)
ax.set_ylabel("pixels"); ax.set_title("Optical flow"); ax.legend(); ax.grid(True)

ax = axes[1]
ax.plot(t, df["range_z"], color="teal")
ax.set_ylabel("m"); ax.set_title("range_z (ToF)"); ax.grid(True)

ax = axes[2]
if "vbat" in df.columns:
    ax.plot(t, df["vbat"], color="orange")
    ax.set_ylabel("V"); ax.set_title("Battery voltage")
    ax.axhline(3.5, color="red", linestyle="--", linewidth=0.8, label="3.5V low")
    ax.legend(); ax.grid(True)
ax.set_xlabel("time [s]")

save(fig, "sensors")

# ── Figure 4: multi-ranger ────────────────────────────────────────────────────
multi_cols = ["multi_front", "multi_back", "multi_left", "multi_right", "multi_up"]
if all(c in df.columns for c in multi_cols):
    any_nonzero = any(df[c].abs().max() > 0.01 for c in multi_cols)
    if any_nonzero:
        fig, ax = plt.subplots(figsize=(12, 5))
        fig.suptitle(f"Multi-ranger Deck — {stem}", fontsize=11)
        for c in multi_cols:
            vals = df[c].replace(0, np.nan)  # 0 = out-of-range sentinel
            ax.plot(t, vals, label=c.replace("multi_", ""), linewidth=0.9)
        ax.set_ylabel("m"); ax.set_xlabel("time [s]")
        ax.legend(); ax.grid(True)
        save(fig, "multiranger")
    else:
        print("  Multi-ranger columns all zero — deck not attached or out of range, skipping.")

# ── Summary ───────────────────────────────────────────────────────────────────
print()
print(f"=== Diagnostic summary: {stem} ===")
print(f"  Duration:   {t.max():.1f} s   ({len(df)} rows @ ~{len(df)/t.max():.0f} Hz)")
print(f"  EKF pos:    x [{df.pos_x.min():.3f}, {df.pos_x.max():.3f}]  "
      f"y [{df.pos_y.min():.3f}, {df.pos_y.max():.3f}]  "
      f"z [{df.pos_z.min():.3f}, {df.pos_z.max():.3f}] m")
print(f"  EKF att:    roll max {df.roll.abs().max():.1f}°  pitch max {df.pitch.abs().max():.1f}°")
print(f"  range_z:    {df.range_z.min():.3f}–{df.range_z.max():.3f} m")
print(f"  flow_dx:    {df.flow_dx.min():.1f}–{df.flow_dx.max():.1f} px")
if has_mekf:
    for lbl, ec, mc in zip(labels, ekf_cols, mekf_cols):
        seeded = df[mc] != 0.0
        if seeded.sum() > 10:
            r = rms(df[ec][seeded].values, df[mc][seeded].values)
            print(f"  MEKF RMSE:  {lbl} = {r:.2f}° ({seeded.sum()} seeded rows)")
        else:
            print(f"  MEKF RMSE:  {lbl} = N/A (MEKF not seeded)")
if "vbat" in df.columns:
    print(f"  Battery:    {df.vbat.min():.2f}–{df.vbat.max():.2f} V")
crash = (df.roll.abs() > 90) | (df.pitch.abs() > 90)
if crash.any():
    t_crash = t[crash.idxmax()]
    print(f"  *** CRASH detected at t={t_crash:.1f} s "
          f"(roll={df.roll[crash.idxmax()]:.1f}°, pitch={df.pitch[crash.idxmax()]:.1f}°) ***")
else:
    print("  No crash detected (roll/pitch stayed < 90°)")
