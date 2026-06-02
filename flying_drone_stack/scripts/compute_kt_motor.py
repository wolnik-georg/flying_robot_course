#!/usr/bin/env python3
"""
compute_kt_motor.py — Identify KT_MOTOR from a hover flight log CSV.

Reads rpm_m1..m4 columns logged by CS2 flight.py during hover.
Drops rows where any RPM is zero (deck not yet active / landed).
Computes k_T = hover_thrust / (m1² + m2² + m3² + m4²) per row, then averages.

Usage:
    python3 scripts/compute_kt_motor.py path/to/hover_log.csv [--mass 0.027]
    python3 scripts/compute_kt_motor.py          # auto-picks latest CSV in Controls/logs/
"""

import argparse
import sys
from pathlib import Path
import csv

GRAVITY   = 9.81   # m/s²
LOGS_DIR  = Path(__file__).resolve().parents[1] / "Controls" / "logs"


def load_csv(path: Path) -> list[dict]:
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            break
        # re-open to use DictReader from start of data
    with open(path) as f:
        reader = csv.DictReader(line for line in f if not line.startswith("#"))
        for row in reader:
            rows.append(row)
    return rows


def compute_kt(rows: list[dict], mass: float) -> float:
    hover_thrust = mass * GRAVITY
    kt_vals = []
    missing = 0
    for row in rows:
        try:
            m1 = float(row["rpm_m1"])
            m2 = float(row["rpm_m2"])
            m3 = float(row["rpm_m3"])
            m4 = float(row["rpm_m4"])
        except (KeyError, ValueError):
            missing += 1
            continue
        if any(r <= 0 or r != r for r in (m1, m2, m3, m4)):  # zero or NaN
            continue
        rpm_sq = m1**2 + m2**2 + m3**2 + m4**2
        kt_vals.append(hover_thrust / rpm_sq)

    if missing:
        print(f"[warn] {missing} rows had missing/invalid RPM columns")
    return sum(kt_vals) / len(kt_vals) if kt_vals else float("nan"), len(kt_vals)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", help="Path to hover log CSV")
    parser.add_argument("--mass", type=float, default=0.027,
                        help="Total drone mass in kg (default: 0.027)")
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
    kt, n = compute_kt(rows, args.mass)

    if kt != kt:
        print("ERROR: no valid RPM rows found. Does the CSV have rpm_m1..m4 columns?")
        sys.exit(1)

    # Per-motor mean RPM for sanity check
    def col_mean(key):
        vals = [float(r[key]) for r in rows if r.get(key) and float(r[key]) > 0]
        return sum(vals) / len(vals) if vals else 0.0

    print(f"\n{'='*52}")
    print(f"  Log file   : {path.name}")
    print(f"  Valid rows : {n}  (of {len(rows)} total)")
    print(f"  Mean RPM   : m1={col_mean('rpm_m1'):.0f}  m2={col_mean('rpm_m2'):.0f}"
          f"  m3={col_mean('rpm_m3'):.0f}  m4={col_mean('rpm_m4'):.0f}")
    print(f"  Mass       : {args.mass} kg  →  hover thrust = {args.mass*9.81:.4f} N")
    print(f"\n  KT_MOTOR = {kt:.4e}  [N/RPM²]")
    print(f"\n  Paste into firmware_app/src/lib.rs line ~164:")
    print(f"    const KT_MOTOR: f32 = {kt:.4e}_f32;")
    print(f"{'='*52}\n")


if __name__ == "__main__":
    main()
