#!/usr/bin/env python3
"""Identify per-motor thrust constants kt from a stable hover log.

Assumes equal thrust per motor at a level hover (each motor carries m*g/4), so:

    kt_i = (m * g / 4) / <rpm_i^2>     over a steady hover window

Works for both drones:
  - Brushless (CF21BL): RPM columns from ESC DShot telemetry (motor.mX_rpm).
  - CF2.1 std/upgraded : RPM columns from the optical deck (rpm.mX).
The 4 RPM columns are auto-detected by name.

Usage:
  ~/.pyenv/versions/flying_robots/bin/python measure_kt.py --csv logs/hover_....csv --mass 0.0393
"""
import argparse, csv, glob, math, os, re

G = 9.81


def find_rpm_columns(header):
    """Return the 4 column indices for motors 1..4 (order-preserving)."""
    idx = {}
    for i, name in enumerate(header):
        m = re.search(r"(?:motor[_.]?)?m?(\d)(?:_?rpm)?$", name.lower())
        if "rpm" in name.lower() and m:
            k = int(m.group(1))
            if 1 <= k <= 4 and k not in idx:
                idx[k] = i
    if len(idx) != 4:
        raise SystemExit(f"Could not find 4 RPM columns. Header had: "
                         f"{[h for h in header if 'rpm' in h.lower()]}")
    return [idx[1], idx[2], idx[3], idx[4]]


def find_col(header, *names):
    for n in names:
        if n in header:
            return header.index(n)
    return None


def main():
    ap = argparse.ArgumentParser(description="Identify kt1..kt4 from a hover log")
    ap.add_argument("--csv", help="hover CSV (default: newest logs/hover_*.csv)")
    ap.add_argument("--mass", type=float, required=True, help="drone mass m [kg]")
    ap.add_argument("--zmin", type=float, default=0.5, help="min z to count as airborne [m]")
    args = ap.parse_args()

    path = args.csv
    if not path:
        cands = sorted(glob.glob("logs/hover_*.csv"), key=os.path.getmtime)
        if not cands:
            raise SystemExit("No logs/hover_*.csv found — pass --csv.")
        path = cands[-1]
        print(f"[kt] using newest: {path}")

    with open(path) as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    header = rows[0]
    rpm_cols = find_rpm_columns(header)
    zc = find_col(header, "z", "pos_z", "pz")
    rc = find_col(header, "roll_deg", "roll")
    pc = find_col(header, "pitch_deg", "pitch")

    # steady hover window: airborne (z>zmin), roughly level, all 4 motors spinning
    sums = [0.0] * 4
    n = 0
    for r in rows[1:]:
        try:
            if zc is not None and float(r[zc]) < args.zmin:
                continue
            if rc is not None and abs(float(r[rc])) > 8:
                continue
            if pc is not None and abs(float(r[pc])) > 8:
                continue
            rpms = [float(r[c]) for c in rpm_cols]
        except (ValueError, IndexError):
            continue
        if min(rpms) < 100:      # a motor reading ~0 → skip (bad telemetry)
            continue
        for i in range(4):
            sums[i] += rpms[i] ** 2
        n += 1

    if n < 20:
        raise SystemExit(f"Only {n} steady rows — need a clean hover with valid RPM.")
    per = args.mass * G / 4.0
    kts = [per / (s / n) for s in sums]
    print(f"[kt] steady rows: {n}   per-motor thrust (m*g/4) = {per:.5f} N")
    for i, kt in enumerate(kts, 1):
        print(f"     kt{i} = {kt:.4e}   (<rpm{i}> = {math.sqrt(sums[i-1]/n):.0f})")
    print("[kt] Paste kt1..kt4 into crazyflies.yaml (active drone block).")


if __name__ == "__main__":
    main()
