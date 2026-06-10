#!/usr/bin/env python3
"""
INDI tuning analysis — post-flight decision tool.

Auto-picks the latest CSV from Controls/logs/ (or use --csv).
Prints a decision table with pass/fail thresholds and saves one diagnostic plot.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py --csv logs/hover_....csv
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py --mass 0.0283  # if weighed
"""

import argparse
import csv
import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR    = os.path.join(SCRIPT_DIR, "logs")
GRAVITY     = 9.81

# ── Decision thresholds ─────────────────────────────────────────────────────
GYRO_STD_STABLE    = 12.0   # deg/s — clearly stable
GYRO_STD_MARGINAL  = 22.0   # deg/s — marginal, consider reducing gains
GYRO_STD_DANGER    = 35.0   # deg/s — oscillating, reduce gains immediately
ATT_RMSE_GOOD      = 3.0    # deg   — good attitude hold
ATT_RMSE_OK        = 8.0    # deg   — acceptable
POS_RMSE_GOOD      = 0.05   # m     — good position hold
POS_RMSE_OK        = 0.15   # m     — acceptable
RPM_MIN_ACTIVE     = 5000   # RPM   — below this = deck not reading


def find_latest_csv():
    files = glob.glob(os.path.join(LOGS_DIR, "*.csv"))
    if not files:
        print(f"[error] No CSVs found in {LOGS_DIR}")
        sys.exit(1)
    return max(files, key=os.path.getmtime)


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        for line in f:
            if line.startswith("#"):
                continue
            break
        reader = csv.DictReader(f, fieldnames=line.strip().split(","))
        for row in reader:
            try:
                rows.append({k: float(v) for k, v in row.items()})
            except (ValueError, TypeError):
                continue
    return rows


def arr(rows, key):
    return np.array([r[key] for r in rows if key in r and not np.isnan(r[key])])


def col(rows, key, default=np.nan):
    out = []
    for r in rows:
        v = r.get(key, default)
        out.append(float(v) if v == v else default)  # nan-safe
    return np.array(out)


def steady_slice(rows, skip_s=1.5):
    """Return indices after the initial transient (skip_s seconds from start)."""
    t = col(rows, "time_s")
    return np.where(t >= t[0] + skip_s)[0]


def status(val, good, ok, invert=False):
    """Return (symbol, label) for a metric."""
    if invert:
        if val <= good:   return "✓", "GOOD"
        if val <= ok:     return "~", "OK"
        return "✗", "BAD"
    else:
        if val >= good:   return "✓", "GOOD"
        if val >= ok:     return "~", "OK"
        return "✗", "BAD"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv",  default=None, help="Path to flight CSV")
    parser.add_argument("--mass", type=float, default=0.027, help="All-up mass [kg]")
    args = parser.parse_args()

    csv_path = args.csv or find_latest_csv()
    print(f"\nAnalysing: {os.path.basename(csv_path)}")

    # ── Load ────────────────────────────────────────────────────────────────
    # The CS2 flight.py CSV has comment lines starting with # then a header row.
    # Re-implement load to handle this correctly.
    rows = []
    with open(csv_path, newline="") as f:
        lines = f.readlines()

    header_idx = next(i for i, l in enumerate(lines) if not l.startswith("#"))
    header     = lines[header_idx].strip().split(",")
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

    if not rows:
        print("[error] No data rows found — logging likely failed.")
        sys.exit(1)

    print(f"  Rows: {len(rows)}  |  Duration: {rows[-1]['time_s']:.1f} s")

    idx = steady_slice(rows)
    if len(idx) < 5:
        print("[warn] Flight too short for steady-state analysis — using all rows.")
        idx = np.arange(len(rows))

    t_all  = col(rows, "time_s")
    t      = t_all[idx]

    # ── Position ─────────────────────────────────────────────────────────────
    x = col(rows, "x")[idx];  y = col(rows, "y")[idx];  z = col(rows, "z")[idx]
    x0, y0, z0 = np.nanmean(x), np.nanmean(y), np.nanmean(z)
    pos_rmse_xy = float(np.sqrt(np.nanmean((x - x0)**2 + (y - y0)**2)))
    pos_std_z   = float(np.nanstd(z))

    # ── Attitude ─────────────────────────────────────────────────────────────
    roll  = col(rows, "roll_deg")[idx]
    pitch = col(rows, "pitch_deg")[idx]
    att_rmse = float(np.sqrt(np.nanmean(roll**2 + pitch**2)))

    # ── Gyro ─────────────────────────────────────────────────────────────────
    gx = col(rows, "gyro_x")[idx]
    gy = col(rows, "gyro_y")[idx]
    gz = col(rows, "gyro_z")[idx]
    gyro_std_x = float(np.nanstd(gx))
    gyro_std_y = float(np.nanstd(gy))
    gyro_std_z = float(np.nanstd(gz))
    gyro_std   = float(np.sqrt(gyro_std_x**2 + gyro_std_y**2 + gyro_std_z**2) / np.sqrt(3))

    # ── RPM ──────────────────────────────────────────────────────────────────
    has_rpm = all(f"rpm_m{i}" in rows[0] for i in range(1, 5))
    if has_rpm:
        rpm_cols = np.stack([col(rows, f"rpm_m{i}")[idx] for i in range(1, 5)], axis=1)
        rpm_mean = float(np.nanmean(rpm_cols))
        rpm_std  = float(np.nanstd(rpm_cols))
        rpm_active = rpm_mean > RPM_MIN_ACTIVE
        kt_est   = args.mass * GRAVITY / (4.0 * rpm_mean**2) if rpm_mean > 0 else float("nan")
    else:
        rpm_mean = rpm_std = float("nan")
        rpm_active = False
        kt_est = float("nan")

    # ── Print report ────────────────────────────────────────────────────────
    print()
    print("=" * 58)
    print("  INDI TUNING REPORT")
    print("=" * 58)

    sym, lbl = status(gyro_std, GYRO_STD_STABLE, GYRO_STD_MARGINAL, invert=True)
    print(f"  {sym} Gyro std (rms xyz):  {gyro_std:6.1f} deg/s   [{lbl}]")
    print(f"       x:{gyro_std_x:.1f}  y:{gyro_std_y:.1f}  z:{gyro_std_z:.1f} deg/s")
    if gyro_std > GYRO_STD_DANGER:
        print("     → OSCILLATING: reduce KW/KR immediately")
    elif gyro_std > GYRO_STD_MARGINAL:
        print("     → Marginal: consider reducing KW slightly or checking FC_BW_HZ")

    sym, lbl = status(att_rmse, ATT_RMSE_GOOD, ATT_RMSE_OK, invert=True)
    print(f"  {sym} Attitude RMSE:        {att_rmse:6.2f} deg      [{lbl}]")
    print(f"       roll:{np.nanstd(roll):.2f}  pitch:{np.nanstd(pitch):.2f} deg std")

    sym, lbl = status(pos_rmse_xy, POS_RMSE_GOOD, POS_RMSE_OK, invert=True)
    print(f"  {sym} Position RMSE (XY):   {pos_rmse_xy*100:6.1f} cm       [{lbl}]")
    print(f"       Z std: {pos_std_z*100:.1f} cm")

    print()
    if has_rpm:
        rpm_sym = "✓" if rpm_active else "✗"
        print(f"  {rpm_sym} RPM deck:   mean={rpm_mean:.0f}  std={rpm_std:.0f}  "
              f"({'ACTIVE' if rpm_active else 'NOT READING — check deck'})")
        print(f"     KT estimate: {kt_est:.3e} N/RPM²  (mass={args.mass:.4f} kg)")
        print(f"     → Set indi_gains.kt = {kt_est:.3e} in yaml / cfclient")
    else:
        print("  ? RPM columns not found in CSV")

    print()
    print("  DECISION:")
    if gyro_std < GYRO_STD_STABLE and att_rmse < ATT_RMSE_GOOD:
        next_kr = None
        # Suggest next gain step (doubling toward target 603)
        print("  ✓ STABLE — ready to raise gains")
        print("  Suggested next step: indi_gains.kr → 2× current, keep ζ=KW/(2√KR)≈1.4")
    elif gyro_std > GYRO_STD_DANGER:
        print("  ✗ OSCILLATING — reduce indi_gains.kw by 30%, check FC_BW_HZ ≥ 80 Hz")
    elif gyro_std > GYRO_STD_MARGINAL:
        print("  ~ MARGINAL — hover a bit longer, or reduce indi_gains.kw by 15%")
    else:
        print("  ~ OK — stable but attitude error high, check position setpoint / mass")

    print("=" * 58)

    # ── Plot ─────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"INDI Tune — {os.path.basename(csv_path)}", fontsize=11)

    t_all_plot = t_all
    t_plot     = t

    # Panel 1: Gyro
    ax = axes[0]
    ax.plot(t_all_plot, col(rows, "gyro_x"), lw=0.7, label="gyro_x", alpha=0.8)
    ax.plot(t_all_plot, col(rows, "gyro_y"), lw=0.7, label="gyro_y", alpha=0.8)
    ax.plot(t_all_plot, col(rows, "gyro_z"), lw=0.7, label="gyro_z", alpha=0.8)
    for thresh, color, label in [(GYRO_STD_STABLE, "green", f"stable<{GYRO_STD_STABLE}"),
                                  (GYRO_STD_MARGINAL, "orange", f"marginal<{GYRO_STD_MARGINAL}"),
                                  (GYRO_STD_DANGER, "red", f"danger>{GYRO_STD_DANGER}")]:
        ax.axhline( thresh, color=color, lw=0.8, ls="--", alpha=0.6)
        ax.axhline(-thresh, color=color, lw=0.8, ls="--", alpha=0.6)
    ax.set_ylabel("Gyro [deg/s]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    # Panel 2: Attitude
    ax = axes[1]
    ax.plot(t_all_plot, col(rows, "roll_deg"),  lw=0.8, label="roll")
    ax.plot(t_all_plot, col(rows, "pitch_deg"), lw=0.8, label="pitch")
    ax.set_ylabel("Attitude [deg]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    # Panel 3: Position
    ax = axes[2]
    ax.plot(t_all_plot, col(rows, "x"), lw=0.8, label="x")
    ax.plot(t_all_plot, col(rows, "y"), lw=0.8, label="y")
    ax.plot(t_all_plot, col(rows, "z"), lw=0.8, label="z")
    ax.set_ylabel("Position [m]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    # Panel 4: RPM
    ax = axes[3]
    if has_rpm:
        for i in range(1, 5):
            ax.plot(t_all_plot, col(rows, f"rpm_m{i}"), lw=0.8, label=f"m{i}", alpha=0.8)
        ax.axhline(RPM_MIN_ACTIVE, color="red", lw=0.8, ls="--", alpha=0.6, label="min active")
        ax.set_ylabel("RPM")
        ax.legend(fontsize=7, loc="upper right")
    else:
        ax.text(0.5, 0.5, "RPM not logged", transform=ax.transAxes, ha="center")
        ax.set_ylabel("RPM")
    ax.set_xlabel("Time [s]"); ax.grid(alpha=0.3)

    plt.tight_layout()
    out = csv_path.replace(".csv", "_indi_tune.png")
    fig.savefig(out, dpi=120)
    plt.close()
    print(f"\n  Plot saved: {os.path.basename(out)}\n")


if __name__ == "__main__":
    main()
