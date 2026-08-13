#!/usr/bin/env python3
"""
compute_kt_motor.py — Identify KT_MOTOR (per-motor) and diagnose RPM balance.

Reads rpm_m1..m4 columns logged by CS2 flight.py during hover.
Also computes per-motor torque bias — a consistent RPM imbalance causes
INDI to estimate a non-zero tau_current at hover, making the drone drift.

Two KT identification modes:
  --mode quick  (default): KT_i = (mass*g/4) / mean(RPM_i^2)
                Fast, assumes equal thrust per motor at hover.
  --mode nnls :            Solve min ||A @ kt - b||^2 s.t. kt >= 0
                where A[row,i] = RPM_i(row)^2, b[row] = mass*g.
                More robust when RPMs vary across rows (e.g. from figure-8).

Usage:
    python3 compute_kt_motor.py                               # auto-picks latest CSV
    python3 compute_kt_motor.py logs/hover_....csv
    python3 compute_kt_motor.py logs/hover_....csv --mass 0.031 --mode nnls
"""

import argparse
import sys
from pathlib import Path
import csv

import numpy as np
from scipy.optimize import nnls

GRAVITY      = 9.81    # m/s²
LOGS_DIR     = Path(__file__).resolve().parent / "logs"

# Crazyflie geometry (must match lib.rs)
ARM_M        = 0.032_526_9   # m  (√2/2 × 0.046)
TORQUE_RATIO = 0.005_964_552  # k_Q/k_T [m]

# Motor layout matching rpms_to_torque in lib.rs:
#   tau_x = ARM * (f2+f3 - f0-f1)   [roll]
#   tau_y = ARM * (f1+f2 - f0-f3)   [pitch]
#   tau_z = TR  * (f1+f3 - f0-f2)   [yaw]
# indices: 0=M1, 1=M2, 2=M3, 3=M4


def load_csv(path: Path) -> list[dict]:
    with open(path) as f:
        lines = f.readlines()
    header_idx = next(i for i, l in enumerate(lines) if not l.startswith("#"))
    header = lines[header_idx].strip().split(",")
    rows = []
    for line in lines[header_idx + 1:]:
        if not line.strip():
            continue
        vals = line.strip().split(",")
        if len(vals) != len(header):
            continue
        try:
            rows.append({k: float(v) for k, v in zip(header, vals)})
        except ValueError:
            continue
    return rows


def steady_rows(rows, skip_s=1.5):
    """Return rows after initial transient with all RPMs active."""
    t0 = rows[0].get("time_s", 0.0)
    return [r for r in rows
            if r.get("time_s", 0) >= t0 + skip_s
            and all(r.get(f"rpm_m{i}", 0) > 5000 for i in range(1, 5))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", help="Path to hover log CSV")
    parser.add_argument("--mass", type=float, default=0.027)
    parser.add_argument("--mode", choices=["quick", "nnls"], default="quick",
                        help="quick: KT_i=(mass*g/4)/mean(RPM_i^2); nnls: optimization fit")
    args = parser.parse_args()

    if args.csv:
        path = Path(args.csv)
    else:
        candidates = sorted(LOGS_DIR.glob("*.csv")) if LOGS_DIR.exists() else []
        if not candidates:
            print(f"No CSV found in {LOGS_DIR}. Pass a path explicitly.")
            sys.exit(1)
        path = candidates[-1]
        print(f"[auto] Using latest log: {path.name}")

    rows = load_csv(path)
    good = steady_rows(rows)
    if len(good) < 5:
        print("ERROR: fewer than 5 valid steady-hover rows. Check RPM columns / flight duration.")
        sys.exit(1)

    hover_thrust = args.mass * GRAVITY

    # ── Per-motor RPM stats ──────────────────────────────────────────────────
    rpms = np.array([[r[f"rpm_m{i}"] for i in range(1, 5)] for r in good])  # (N, 4)
    mean_rpm = rpms.mean(axis=0)   # shape (4,)
    std_rpm  = rpms.std(axis=0)
    overall_mean = mean_rpm.mean()

    # ── KT identification ────────────────────────────────────────────────────
    # Average KT (single value for backwards compat)
    rpm_sq_sum = (rpms**2).sum(axis=1)   # sum RPM_i^2 per row
    kt_vals    = hover_thrust / rpm_sq_sum
    kt         = float(kt_vals.mean())
    kt_std     = float(kt_vals.std())

    # Per-motor KT
    mean_rpm_sq = (rpms**2).mean(axis=0)   # shape (4,) — mean(RPM_i^2) per motor
    if args.mode == "nnls":
        # NNLS: min ||A @ kt_vec - b||^2, kt_vec >= 0
        # A[row, i] = RPM_i(row)^2,  b[row] = mass*g
        A = rpms.astype(float)**2          # (N, 4)
        b = np.full(len(good), hover_thrust)
        kt_per_motor, residual = nnls(A, b)
        kt_mode_label = "NNLS optimization"
    else:
        # Quick mode: equal-thrust assumption at hover
        kt_per_motor = hover_thrust / (4.0 * mean_rpm_sq)
        kt_mode_label = "quick (equal thrust)"

    # ── Torque bias at hover ──────────────────────────────────────────────────
    # tau_current = rpms_to_torque(RPMs, kt_per_motor)
    f = kt_per_motor * rpms**2   # (N, 4) per-motor forces using per-motor KT
    tau_x = ARM_M        * (f[:, 2] + f[:, 3] - f[:, 0] - f[:, 1])
    tau_y = ARM_M        * (f[:, 1] + f[:, 2] - f[:, 0] - f[:, 3])
    tau_z = TORQUE_RATIO * (f[:, 1] + f[:, 3] - f[:, 0] - f[:, 2])

    tau_x_mean = float(tau_x.mean());  tau_x_std = float(tau_x.std())
    tau_y_mean = float(tau_y.mean());  tau_y_std = float(tau_y.std())
    tau_z_mean = float(tau_z.mean());  tau_z_std = float(tau_z.std())

    # ── RPM imbalance: deviation of each motor from mean ─────────────────────
    rpm_dev = mean_rpm - overall_mean   # positive = faster than average

    # ── Print report ─────────────────────────────────────────────────────────
    W = 58
    print(f"\n{'='*W}")
    print(f"  KT_MOTOR IDENTIFICATION + MOTOR BALANCE REPORT")
    print(f"{'='*W}")
    print(f"  Log    : {path.name}")
    print(f"  Rows   : {len(good)} steady-hover  (of {len(rows)} total)")
    print(f"  Mass   : {args.mass} kg  →  hover thrust = {hover_thrust:.4f} N")
    print()

    print(f"  KT avg   = {kt:.4e} N/RPM^2  (std {kt_std:.2e})  [single-value, all motors]")
    print(f"  KT mode  : {kt_mode_label}")
    for i, k in enumerate(kt_per_motor):
        print(f"    KT_M{i+1} = {k:.4e} N/RPM^2")
    print()

    print(f"  Per-motor RPM (mean ± std):")
    for i, (m, s, d) in enumerate(zip(mean_rpm, std_rpm, rpm_dev)):
        flag = ""
        if abs(d) > 800:
            flag = "  ← LARGE IMBALANCE"
        elif abs(d) > 400:
            flag = "  ← moderate imbalance"
        print(f"    M{i+1}: {m:6.0f} ± {s:4.0f}  (dev {d:+.0f} RPM){flag}")
    print(f"    Overall mean: {overall_mean:.0f} RPM")
    print()

    print(f"  Torque bias at hover (should be ~0 for all axes):")
    for name, mean, std in [("tau_x (roll) ", tau_x_mean, tau_x_std),
                             ("tau_y (pitch)", tau_y_mean, tau_y_std),
                             ("tau_z (yaw)  ", tau_z_mean, tau_z_std)]:
        hover_torque_scale = ARM_M * kt * overall_mean**2
        rel = abs(mean) / hover_torque_scale * 100 if hover_torque_scale > 0 else 0
        flag = ""
        if rel > 5:
            flag = f"  ← BIAS {rel:.0f}% of hover torque — DRIFT RISK"
        elif rel > 2:
            flag = f"  ← mild bias {rel:.1f}%"
        print(f"    {name}: {mean:+.2e} Nm  (std {std:.2e}){flag}")
    print()

    print(f"  Paste into traj_iface.c:")
    for i, k in enumerate(kt_per_motor):
        print(f"    float g_indi_kt{i+1} = {k:.4e}f;")
    print(f"{'='*W}\n")

    # ── Drift diagnosis ───────────────────────────────────────────────────────
    drift_axes = []
    for name, mean, std in [("pitch (forward/back)", tau_y_mean, tau_y_std),
                             ("roll  (left/right)",   tau_x_mean, tau_x_std)]:
        hover_torque_scale = ARM_M * kt * overall_mean**2
        rel = abs(mean) / hover_torque_scale * 100 if hover_torque_scale > 0 else 0
        if rel > 3:
            direction = "forward" if (name.startswith("pitch") and mean > 0) else \
                        "backward" if name.startswith("pitch") else \
                        "right" if mean > 0 else "left"
            drift_axes.append(f"    {name}: bias {mean:+.2e} Nm ({rel:.0f}%) → expect drift {direction}")

    if drift_axes:
        print("  DRIFT PREDICTION (from RPM imbalance):")
        for d in drift_axes:
            print(d)
        print()
        print("  Fix options:")
        print("    1. Re-check KT_MOTOR on thrust stand (most reliable)")
        print("    2. Check for damaged/dirty motor on the outlier motor")
        print("    3. Add integral term on attitude (not in current INDI)")
        print()


if __name__ == "__main__":
    main()
