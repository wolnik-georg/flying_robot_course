#!/usr/bin/env python3
"""
Diagnose which RPM log source is valid on the active drone.

Usage:
    python3 Controls/diagnose_rpm_source.py                  # auto-picks latest CSV
    python3 Controls/diagnose_rpm_source.py path/to/log.csv  # explicit file
"""
import sys
import glob
import os
import pandas as pd

# ── pick file ────────────────────────────────────────────────────────────────
if len(sys.argv) > 1:
    path = sys.argv[1]
else:
    pattern = os.path.join(os.path.dirname(__file__), "logs", "hover_mode0_*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        sys.exit("No hover_mode0_*.csv found in Controls/logs/. Run a geometric hover first.")
    path = files[-1]

print(f"Analysing: {path}\n")
df = pd.read_csv(path, comment='#')

# ── RPM columns to check ─────────────────────────────────────────────────────
sources = {
    "rpm.m1..m4   (optical deck / DShot-bridged)": ["rpm_m1",       "rpm_m2",       "rpm_m3",       "rpm_m4"],
    "motor.m*_rpm (DShot ESC telemetry direct)":   ["motor_m1_rpm", "motor_m2_rpm", "motor_m3_rpm", "motor_m4_rpm"],
}

# Only look at rows where drone is airborne (z > 0.2 m)
hover = df[df["z"] > 0.2] if "z" in df.columns else df
if len(hover) < 10:
    hover = df  # fallback if no height data

print(f"Rows airborne (z > 0.2 m): {len(hover)}\n")

verdict = {}
for label, cols in sources.items():
    present = [c for c in cols if c in df.columns]
    if not present:
        print(f"[MISSING] {label}")
        print(f"          Columns not in CSV: {cols}\n")
        verdict[label] = None
        continue

    data = hover[present]
    means = data.mean()
    stds  = data.std()
    zeros = (data == 0).sum()
    inv   = (data == 65535).sum()

    all_zero   = (means < 1).all()
    all_inv    = (inv > len(hover) * 0.5).any()
    looks_good = (means > 1000).all() and not all_inv

    print(f"{'[OK] ' if looks_good else '[BAD]'} {label}")
    for c in present:
        z = zeros[c]; i = inv[c]; n = len(hover)
        print(f"  {c:20s}  mean={means[c]:7.0f}  std={stds[c]:6.0f}  "
              f"zeros={z}/{n}  invalid(0xFFFF)={i}/{n}")
    if all_zero:
        print("  → ALL ZEROS — source not active (deck absent or not bridged)")
    elif all_inv:
        print("  → MOSTLY 0xFFFF — DShot telemetry invalid/not returned")
    elif looks_good:
        print("  → VALID — use this source")
    print()
    verdict[label] = looks_good

# ── conclusion ───────────────────────────────────────────────────────────────
good = [k for k, v in verdict.items() if v]
bad  = [k for k, v in verdict.items() if v is False]

print("=" * 60)
if len(good) == 1:
    if "rpm.m1" in good[0]:
        print("CONCLUSION: rpm.m1..m4 is the live source.")
        print("  rpm_get_all() should read group='rpm', vars m1..m4  (current code after fix)")
    else:
        print("CONCLUSION: motor.m*_rpm is the live source.")
        print("  rpm_get_all() should read group='motor', vars m1_rpm..m4_rpm  (revert traj_iface.c)")
elif len(good) == 2:
    print("CONCLUSION: BOTH sources have values — cross-check means match?")
    print("  Prefer rpm.m1..m4 (consistent with kt identification).")
elif len(good) == 0:
    print("CONCLUSION: NEITHER source has valid RPM values during hover.")
    print("  INDI cannot use RPM feedback. rpms_active will be false → tau integrates → crash.")
    print("  Options: fit optical RPM deck, or switch to att-only INDI without RPM feedback.")
