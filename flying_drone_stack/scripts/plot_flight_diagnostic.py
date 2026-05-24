#!/usr/bin/env python3
"""plot_flight_diagnostic.py — Flight diagnostic plots.

Loads a run CSV (from run_firmware_mode) and produces five diagnostic figures:

  1. EKF position x/y/z over time + XY ground track
  2. EKF vs MEKF orientation (roll, pitch, yaw) + RMS errors
  3. Raw sensors: flow_dx/dy, range_z, battery voltage
  4. Multi-ranger front/back/left/right/up (if non-zero)
  5. Controller diagnostics: setpoint tracking, motors, thrust (if columns present)

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

# ── Figure 5: Controller diagnostics (firmware_app OOT SE(3) controller) ─────
# Only shown when the new block-6 columns are present (post-Apr-2026 CSVs).
motor_cols = ["motor_m1req", "motor_m2req", "motor_m3req", "motor_m4req"]
has_motors = all(c in df.columns for c in motor_cols)
has_ctrlxy  = "ctrltarget_x" in df.columns and "ctrltarget_y" in df.columns
has_thrust  = "our_thrust" in df.columns

if has_motors or has_ctrlxy or has_thrust:
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"OOT Controller Diagnostics — {stem}", fontsize=11)

    # --- Panel 1: Setpoint Z vs actual Z vs ToF ---
    ax = axes[0]
    if "ctrltarget_z" in df.columns:
        ax.plot(t, df["ctrltarget_z"], label="ctrltarget_z (setpoint seen by ctrl)", linewidth=1.2)
    ax.plot(t, df["pos_z"], label="pos_z (EKF)", linewidth=1.0)
    ax.plot(t, df["range_z"], label="range_z (ToF, truth)", linewidth=1.0, linestyle="--")
    ax.axhline(0.30, color="grey", linewidth=0.8, linestyle=":", label="target 0.30 m")
    ep_z = df["ctrltarget_z"] - df["pos_z"] if "ctrltarget_z" in df.columns else None
    title = "Z setpoint tracking"
    if ep_z is not None:
        rms_z = float(np.sqrt(np.mean(ep_z[df["range_z"] > 0.08] ** 2))) if (df["range_z"] > 0.08).any() else float("nan")
        title += f"  (Z error RMS in-flight = {rms_z:.3f} m)"
    ax.set_ylabel("m"); ax.set_title(title); ax.legend(fontsize=8); ax.grid(True)

    # --- Panel 2: Motor balance (all 4) ---
    ax = axes[1]
    if has_motors:
        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        for col, color in zip(motor_cols, colors):
            ax.plot(t, df[col], label=col, linewidth=0.9, color=color)
        hover_val = df.loc[df["range_z"] > 0.10, motor_cols].mean().mean() if (df["range_z"] > 0.10).any() else float("nan")
        ax.axhline(hover_val, color="grey", linewidth=0.8, linestyle=":",
                   label=f"mean in-flight = {hover_val:.0f}")
        ax.set_ylabel("raw PWM"); ax.set_title("Motor thrust requests (should be ~equal at hover)"); ax.legend(fontsize=8); ax.grid(True)
    else:
        ax.set_title("Motor data not available"); ax.grid(True)

    # --- Panel 3: Shadow thrust + XY setpoint confirmation ---
    ax = axes[2]
    if has_thrust:
        ax.plot(t, df["our_thrust"], label="our_thrust (shadow, N)", linewidth=1.0)
        hover_thr = 0.031 * 9.81  # ~0.304 N
        ax.axhline(hover_thr, color="grey", linewidth=0.8, linestyle=":", label=f"expected hover {hover_thr:.3f} N")
        ax.set_ylabel("N"); ax.set_title("Shadow controller thrust + XY setpoint confirmation")
    if has_ctrlxy:
        ax2 = ax.twinx()
        ax2.plot(t, df["ctrltarget_x"], label="ctrltgt_x", linewidth=0.8, linestyle="--", color="purple")
        ax2.plot(t, df["ctrltarget_y"], label="ctrltgt_y", linewidth=0.8, linestyle="--", color="brown")
        ax2.set_ylabel("m (XY setpoint)")
        ax2.legend(fontsize=8, loc="upper right")
    ax.legend(fontsize=8, loc="upper left"); ax.grid(True)
    ax.set_xlabel("time [s]")

    save(fig, "controller")

    # Print motor balance summary
    if has_motors and (df["range_z"] > 0.10).any():
        inflight = df[df["range_z"] > 0.10]
        means = [inflight[c].mean() for c in motor_cols]
        stds  = [inflight[c].std()  for c in motor_cols]
        print(f"\n  Motor balance (in-flight mean ± std):")
        for col, m, s in zip(motor_cols, means, stds):
            print(f"    {col}: {m:.0f} ± {s:.0f}")
        imbalance = max(means) - min(means)
        print(f"    Max imbalance: {imbalance:.0f} raw ({100*imbalance/max(means):.1f}%)")

# ── Figure 6: INDI diagnostics (only when CONTROLLER_MODE=1 data is present) ──
indi_tau_cols = ["indi_tau_x", "indi_tau_y", "indi_tau_z"]
indi_alp_cols = ["indi_alp_x", "indi_alp_y", "indi_alp_z"]
has_indi = all(c in df.columns for c in indi_tau_cols + indi_alp_cols)

if has_indi:
    # Check if any values are non-zero (INDI actually active in this flight)
    indi_active = any(df[c].abs().max() > 1e-6 for c in indi_tau_cols + indi_alp_cols)
    status_note = "(INDI active)" if indi_active else "(all zero — geometric mode or RPM deck absent)"

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig.suptitle(f"INDI Diagnostics {status_note} — {stem}", fontsize=11)

    ax = axes[0]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for col, lbl, color in zip(indi_tau_cols, ["τ_x", "τ_y", "τ_z"], colors):
        ax.plot(t, df[col] * 1000, label=f"{lbl} [mNm]", linewidth=0.9, color=color)
    ax.set_ylabel("τ_prev  [mNm]")
    ax.set_title("INDI torque base τ_prev — steady non-zero = attitude demand; oscillation = gain too high")
    ax.legend(fontsize=8); ax.grid(True)

    ax = axes[1]
    for col, lbl, color in zip(indi_alp_cols, ["α_x", "α_y", "α_z"], colors):
        ax.plot(t, df[col], label=f"{lbl} [rad/s²]", linewidth=0.9, color=color)
    ax.set_ylabel("α_meas  [rad/s²]")
    ax.set_title("INDI angular acceleration measurement α_meas — should track angular rate changes")
    ax.set_xlabel("time [s]")
    ax.legend(fontsize=8); ax.grid(True)

    save(fig, "indi")

    if indi_active:
        inflight = df[df["range_z"] > 0.08] if (df["range_z"] > 0.08).any() else df
        print(f"\n  INDI (in-flight):")
        for col in indi_tau_cols:
            print(f"    {col}: mean={inflight[col].mean()*1000:.3f} mNm  std={inflight[col].std()*1000:.3f} mNm")
        for col in indi_alp_cols:
            print(f"    {col}: mean={inflight[col].mean():.2f} rad/s²  std={inflight[col].std():.2f} rad/s²")
    else:
        print("  INDI columns present but all zero — geometric mode or RPM deck not fitted.")

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
