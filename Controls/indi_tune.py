#!/usr/bin/env python3
"""
indi_tune.py — INDI tuning analysis + auto-update.

Reads the latest flight CSV, prints a decision report, and (by default)
writes the new indi_gains values directly into crazyflies.yaml.
Restart CS2 after running to apply the new params.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py --csv logs/hover_....csv
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py --dry-run   # show only, no write
    ~/.pyenv/versions/flying_robots/bin/python indi_tune.py --mass 0.0283
"""

import argparse
import csv
import glob
import os
import re
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRIPT_DIR   = Path(__file__).resolve().parent
LOGS_DIR     = SCRIPT_DIR / "logs"
YAML_PATH    = (SCRIPT_DIR.parents[1] / "crazyswarm2/crazyflie/config/crazyflies.yaml"
                if (SCRIPT_DIR.parents[1] / "crazyswarm2").exists()
                else Path("/home/flyingrobots/georg/ros2_ws/crazyswarm2/crazyflie/config/crazyflies.yaml"))
HISTORY_PATH = LOGS_DIR / "indi_tune_history.csv"

# Decision thresholds
GYRO_STD_STABLE   = 12.0   # deg/s — clearly stable, ready to raise gains
GYRO_STD_MARGINAL = 22.0   # deg/s — marginal, back off slightly
GYRO_STD_DANGER   = 35.0   # deg/s — oscillating, cut gains now
ATT_RMSE_GOOD     = 3.0    # deg
ATT_RMSE_OK       = 8.0
POS_RMSE_GOOD     = 0.05   # m
POS_RMSE_OK       = 0.15
RPM_MIN_ACTIVE    = 5000

# Firmware defaults (match traj_iface.c) — used if yaml has no indi_gains section
PARAM_DEFAULTS = {
    'kr': 100.0, 'kw': 30.0, 'kr_z': 100.0, 'kw_z': 30.0,
    'fc_bw': 60.0, 'fc_iir': 60.0,
}
KR_MAX = 603.0   # target ceiling (matches geometric Block N / JXX)
KW_MAX = 66.0    # target ceiling


# ── CSV helpers ───────────────────────────────────────────────────────────────

def find_latest_csv():
    files = glob.glob(str(LOGS_DIR / "*.csv"))
    if not files:
        print(f"[error] No CSVs found in {LOGS_DIR}")
        sys.exit(1)
    return max(files, key=os.path.getmtime)


def load_csv(path):
    with open(path, newline="") as f:
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


def col(rows, key, default=np.nan):
    return np.array([float(r.get(key, default))
                     if r.get(key) is not None else default for r in rows])


def steady_slice(rows, skip_s=1.5):
    t = col(rows, "time_s")
    idx = np.where(t >= t[0] + skip_s)[0]
    return idx if len(idx) >= 5 else np.arange(len(rows))


# ── YAML helpers ──────────────────────────────────────────────────────────────

def read_yaml_indi_gains():
    """Return dict of current indi_gains from yaml (empty if section absent)."""
    if not YAML_PATH.exists():
        return {}
    lines = YAML_PATH.read_text().split("\n")
    in_section, params = False, {}
    for line in lines:
        if re.match(r"\s+indi_gains:\s*$", line):
            in_section = True
            continue
        if in_section:
            if line.strip() and not line.startswith("      "):
                break
            m = re.match(r"\s+(\w+):\s*([\d.e+\-]+)", line)
            if m:
                params[m.group(1)] = float(m.group(2))
    return params


def _fmt(v):
    if v != 0 and abs(v) < 1e-6:
        return f"{v:.4e}"
    return f"{v:.1f}" if v == int(v) else f"{v:.4g}"


def write_yaml_indi_gains(merged_params, reason_str):
    """Overwrite the indi_gains section in crazyflies.yaml with merged_params."""
    if not YAML_PATH.exists():
        print(f"[warn] {YAML_PATH} not found — skipping write")
        return

    lines = YAML_PATH.read_text().split("\n")
    result = []
    in_section, section_written = False, False

    for line in lines:
        if re.match(r"\s+indi_gains:\s*$", line):
            in_section = True
            result.append(line)
            for k, v in sorted(merged_params.items()):
                result.append(f"      {k}: {_fmt(v)}")
            result.append(f"      # {reason_str}")
            section_written = True
            continue
        if in_section:
            if line.strip() and not line.startswith("      "):
                in_section = False
                result.append(line)
            continue   # drop old section lines (already rewritten above)
        result.append(line)

    if not section_written:
        # No indi_gains section yet — insert before locSrv
        final = []
        for line in result:
            if re.match(r"\s+locSrv:", line):
                final += ["    indi_gains:"] + \
                         [f"      {k}: {_fmt(v)}" for k, v in sorted(merged_params.items())] + \
                         [f"      # {reason_str}"]
            final.append(line)
        result = final

    YAML_PATH.write_text("\n".join(result))


def append_history(csv_name, old, new, metrics):
    HISTORY_PATH.parent.mkdir(parents=True, exist_ok=True)
    write_header = not HISTORY_PATH.exists()
    fieldnames = ["timestamp", "csv", "gyro_std", "att_rmse",
                  "kr_old", "kw_old", "fc_bw_old",
                  "kr_new", "kw_new", "fc_bw_new", "decision"]
    with open(HISTORY_PATH, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if write_header:
            w.writeheader()
        w.writerow({
            "timestamp": datetime.now().strftime("%Y-%m-%d_%H-%M-%S"),
            "csv":       Path(csv_name).name,
            "gyro_std":  f'{metrics["gyro_std"]:.1f}',
            "att_rmse":  f'{metrics["att_rmse"]:.2f}',
            "kr_old":    old.get("kr",    PARAM_DEFAULTS["kr"]),
            "kw_old":    old.get("kw",    PARAM_DEFAULTS["kw"]),
            "fc_bw_old": old.get("fc_bw", PARAM_DEFAULTS["fc_bw"]),
            "kr_new":    new.get("kr",    old.get("kr",    PARAM_DEFAULTS["kr"])),
            "kw_new":    new.get("kw",    old.get("kw",    PARAM_DEFAULTS["kw"])),
            "fc_bw_new": new.get("fc_bw", old.get("fc_bw", PARAM_DEFAULTS["fc_bw"])),
            "decision":  metrics["decision"],
        })


# ── Gain scheduling ───────────────────────────────────────────────────────────

def compute_new_params(current, gyro_std, att_rmse):
    """
    Returns (changes dict, decision str, reason str).
    'changes' only contains keys whose values differ from current.
    """
    kr   = current.get("kr",   PARAM_DEFAULTS["kr"])
    kw   = current.get("kw",   PARAM_DEFAULTS["kw"])
    kr_z = current.get("kr_z", kr)
    kw_z = current.get("kw_z", kw)
    fc_bw  = current.get("fc_bw",  PARAM_DEFAULTS["fc_bw"])
    fc_iir = current.get("fc_iir", PARAM_DEFAULTS["fc_iir"])

    changes, parts = {}, []

    ZETA_TARGET = 1.4   # target damping ratio — keep constant while scaling gains

    if gyro_std > GYRO_STD_DANGER:
        # Halve KR, then set KW to maintain ζ=ZETA_TARGET.
        # Reducing KW alone (old approach) destroys damping ratio and makes it worse.
        kr_new   = round(kr   * 0.50, 2)
        kw_new   = round(2.0 * ZETA_TARGET * kr_new**0.5, 2)
        kr_z_new = round(kr_z * 0.50, 2)
        kw_z_new = round(2.0 * ZETA_TARGET * kr_z_new**0.5, 2)
        changes.update(kr=kr_new, kw=kw_new, kr_z=kr_z_new, kw_z=kw_z_new)
        if fc_bw < 80.0:
            changes["fc_bw"]  = 80.0
            changes["fc_iir"] = 80.0
            parts.append(f"fc_bw {fc_bw:.0f}→80")
        zeta_old = kw / (2.0 * kr**0.5) if kr > 0 else 0.0
        parts.append(f"kr {kr:.1f}→{kr_new:.1f}, kw {kw:.2f}→{kw_new:.2f} "
                     f"(÷2 KR, ζ {zeta_old:.2f}→{ZETA_TARGET:.1f}, oscillating)")
        decision = "OSCILLATING"

    elif gyro_std > GYRO_STD_MARGINAL:
        # Reduce KR by 15%, recompute KW to hold ζ
        kr_new   = round(kr   * 0.85, 2)
        kw_new   = round(2.0 * ZETA_TARGET * kr_new**0.5, 2)
        kr_z_new = round(kr_z * 0.85, 2)
        kw_z_new = round(2.0 * ZETA_TARGET * kr_z_new**0.5, 2)
        changes.update(kr=kr_new, kw=kw_new, kr_z=kr_z_new, kw_z=kw_z_new)
        parts.append(f"kr {kr:.1f}→{kr_new:.1f}, kw {kw:.2f}→{kw_new:.2f} (−15% KR, ζ held)")
        decision = "MARGINAL"

    elif gyro_std < GYRO_STD_STABLE and att_rmse < ATT_RMSE_GOOD:
        kr_new  = min(kr   * 2.0, KR_MAX)
        kw_new  = min(kw   * 1.414, KW_MAX)   # keep ζ = KW/(2√KR) constant
        kr_z_new = min(kr_z * 2.0, KR_MAX)
        kw_z_new = min(kw_z * 1.414, KW_MAX)
        if kr_new > kr:
            changes.update(kr=round(kr_new, 1), kw=round(kw_new, 2),
                           kr_z=round(kr_z_new, 1), kw_z=round(kw_z_new, 2))
            parts.append(f"kr {kr:.0f}→{changes['kr']:.0f}, "
                         f"kw {kw:.2f}→{changes['kw']:.2f} (×2 KR, ζ held, stable)")
            decision = "RAISE"
        else:
            parts.append("at target gains (KR_MAX reached)")
            decision = "AT_TARGET"

    else:
        parts.append("hold — att RMSE high, check position setpoint or mass")
        decision = "HOLD"

    reason_str = "; ".join(parts) if parts else "no change"
    return changes, decision, reason_str


def status(val, good, ok, invert=False):
    if invert:
        if val <= good: return "✓", "GOOD"
        if val <= ok:   return "~", "OK"
        return "✗", "BAD"
    else:
        if val >= good: return "✓", "GOOD"
        if val >= ok:   return "~", "OK"
        return "✗", "BAD"


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv",     default=None)
    parser.add_argument("--mass",    type=float, default=0.027)
    parser.add_argument("--dry-run", action="store_true",
                        help="Print recommendations without writing yaml")
    parser.add_argument("--yaml",    default=None,
                        help="Path to crazyflies.yaml (overrides default)")
    args = parser.parse_args()

    global YAML_PATH
    if args.yaml:
        YAML_PATH = Path(args.yaml).expanduser().resolve()

    csv_path = args.csv or find_latest_csv()
    print(f"\nAnalysing: {Path(csv_path).name}")

    rows = load_csv(csv_path)
    if not rows:
        print("[error] No data rows found.")
        sys.exit(1)
    print(f"  Rows: {len(rows)}  |  Duration: {rows[-1]['time_s']:.1f} s")

    idx = steady_slice(rows)
    t_all = col(rows, "time_s")

    x = col(rows, "x")[idx];  y = col(rows, "y")[idx];  z = col(rows, "z")[idx]
    pos_rmse_xy = float(np.sqrt(np.nanmean((x - np.nanmean(x))**2 + (y - np.nanmean(y))**2)))
    pos_std_z   = float(np.nanstd(z))

    roll  = col(rows, "roll_deg")[idx]
    pitch = col(rows, "pitch_deg")[idx]
    att_rmse = float(np.sqrt(np.nanmean(roll**2 + pitch**2)))

    gx = col(rows, "gyro_x")[idx]
    gy = col(rows, "gyro_y")[idx]
    gz = col(rows, "gyro_z")[idx]
    gyro_std_x = float(np.nanstd(gx))
    gyro_std_y = float(np.nanstd(gy))
    gyro_std_z = float(np.nanstd(gz))
    gyro_std   = float(np.sqrt(gyro_std_x**2 + gyro_std_y**2 + gyro_std_z**2) / np.sqrt(3))

    has_rpm = all(f"rpm_m{i}" in rows[0] for i in range(1, 5))
    if has_rpm:
        rpm_cols  = np.stack([col(rows, f"rpm_m{i}")[idx] for i in range(1, 5)], axis=1)
        rpm_mean  = float(np.nanmean(rpm_cols))
        rpm_std   = float(np.nanstd(rpm_cols))
        rpm_active = rpm_mean > RPM_MIN_ACTIVE
        kt_est    = args.mass * 9.81 / (4.0 * rpm_mean**2) if rpm_mean > 0 else float("nan")
    else:
        rpm_mean = rpm_std = float("nan")
        rpm_active = False
        kt_est = float("nan")

    # ── Read current params, compute next step ────────────────────────────────
    yaml_gains = read_yaml_indi_gains()
    current    = {**PARAM_DEFAULTS, **yaml_gains}
    changes, decision, reason_str = compute_new_params(current, gyro_std, att_rmse)
    merged = {**current, **changes}

    # ── Print report ──────────────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("  INDI TUNING REPORT")
    print("=" * 60)
    print(f"  Current:  kr={current['kr']:.1f}  kw={current['kw']:.2f}  "
          f"fc_bw={current['fc_bw']:.0f} Hz  fc_iir={current['fc_iir']:.0f} Hz")
    print()

    sym, lbl = status(gyro_std, GYRO_STD_STABLE, GYRO_STD_MARGINAL, invert=True)
    print(f"  {sym} Gyro std (rms xyz):  {gyro_std:6.1f} deg/s   [{lbl}]")
    print(f"       x:{gyro_std_x:.1f}  y:{gyro_std_y:.1f}  z:{gyro_std_z:.1f} deg/s")

    sym, lbl = status(att_rmse, ATT_RMSE_GOOD, ATT_RMSE_OK, invert=True)
    print(f"  {sym} Attitude RMSE:        {att_rmse:6.2f} deg      [{lbl}]")
    print(f"       roll std:{np.nanstd(roll):.2f}  pitch std:{np.nanstd(pitch):.2f} deg")

    sym, lbl = status(pos_rmse_xy, POS_RMSE_GOOD, POS_RMSE_OK, invert=True)
    print(f"  {sym} Position RMSE (XY):   {pos_rmse_xy*100:6.1f} cm       [{lbl}]")
    print(f"       Z std: {pos_std_z*100:.1f} cm")

    print()
    if has_rpm:
        sym = "✓" if rpm_active else "✗"
        print(f"  {sym} RPM deck:  mean={rpm_mean:.0f}  std={rpm_std:.0f}  "
              f"({'ACTIVE' if rpm_active else 'NOT READING'})")
        print(f"     KT estimate: {kt_est:.3e} N/RPM²  (cross-check with compute_kt_motor.py)")

    print()
    print("  DECISION:")
    if changes:
        print(f"  → {reason_str}")
        print()
        print("  Changes to apply:")
        for k in sorted(changes):
            print(f"    indi_gains.{k}:  {current.get(k, '?')} → {changes[k]}")
    else:
        print(f"  → {reason_str}")

    print("=" * 60)

    metrics = {"gyro_std": gyro_std, "att_rmse": att_rmse,
               "pos_rmse_xy": pos_rmse_xy, "decision": decision}

    if changes:
        append_history(csv_path, current, merged, metrics)
        if not args.dry_run:
            write_yaml_indi_gains(merged, reason_str)
            print(f"\n  ✓ {YAML_PATH.name} updated — restart CS2 to apply")
        else:
            print(f"\n  [dry-run] yaml not updated")
    else:
        append_history(csv_path, current, current, metrics)

    print(f"  History: {HISTORY_PATH.name}")

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(f"INDI Tune — {Path(csv_path).name}", fontsize=11)

    ax = axes[0]
    for key, label in [("gyro_x", "x"), ("gyro_y", "y"), ("gyro_z", "z")]:
        ax.plot(t_all, col(rows, key), lw=0.7, label=label, alpha=0.8)
    for thresh, color in [(GYRO_STD_STABLE, "green"),
                          (GYRO_STD_MARGINAL, "orange"),
                          (GYRO_STD_DANGER, "red")]:
        ax.axhline( thresh, color=color, lw=0.8, ls="--", alpha=0.6)
        ax.axhline(-thresh, color=color, lw=0.8, ls="--", alpha=0.6)
    ax.set_ylabel("Gyro [deg/s]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    ax = axes[1]
    ax.plot(t_all, col(rows, "roll_deg"),  lw=0.8, label="roll")
    ax.plot(t_all, col(rows, "pitch_deg"), lw=0.8, label="pitch")
    ax.set_ylabel("Attitude [deg]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    ax = axes[2]
    for key in ("x", "y", "z"):
        ax.plot(t_all, col(rows, key), lw=0.8, label=key)
    ax.set_ylabel("Position [m]"); ax.legend(fontsize=7, loc="upper right"); ax.grid(alpha=0.3)

    ax = axes[3]
    if has_rpm:
        for i in range(1, 5):
            ax.plot(t_all, col(rows, f"rpm_m{i}"), lw=0.8, label=f"m{i}", alpha=0.8)
        ax.axhline(RPM_MIN_ACTIVE, color="red", lw=0.8, ls="--", alpha=0.6, label="min active")
        ax.set_ylabel("RPM"); ax.legend(fontsize=7, loc="upper right")
    else:
        ax.text(0.5, 0.5, "RPM not logged", transform=ax.transAxes, ha="center")
        ax.set_ylabel("RPM")
    ax.set_xlabel("Time [s]"); ax.grid(alpha=0.3)

    plt.tight_layout()
    out = str(csv_path).replace(".csv", "_indi_tune.png")
    fig.savefig(out, dpi=120)
    plt.close()
    print(f"  Plot: {Path(out).name}\n")


if __name__ == "__main__":
    main()
