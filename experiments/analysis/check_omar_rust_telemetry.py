#!/usr/bin/env python3
"""Triage merged uSD CSV for controller=10 (omar_indi_rust) flights.

Usage:
  python3 experiments/analysis/check_omar_rust_telemetry.py PATH/to/*_merged.csv

Expects merge_usd_logs.py column names (cf5.tau_*, cf5.a_res_*, cf5.e_r_*,
cf5.motor_m1..4, cf5.z, cf5.ctrltarget_z, ...).
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path


def fval(row: dict, key: str) -> float:
    v = row.get(key, "")
    if v is None or v == "":
        return float("nan")
    return float(v)


def motor_pattern(m: list[float]) -> tuple[bool, int, float]:
    """True if one motor > 2× all others (dominant index, ratio)."""
    if not m or any(math.isnan(x) for x in m):
        return False, -1, 0.0
    mx = max(m)
    if mx <= 1.0:
        return False, -1, 0.0
    others = sorted(m, reverse=True)
    if len(others) < 2 or others[1] < 1e-6:
        return True, m.index(mx), float("inf")
    ratio = mx / others[1]
    dom = m.index(mx)
    return ratio > 2.0, dom, ratio


def analyze(path: Path) -> int:
    lines = path.read_text().splitlines()
    hdr_idx = next(
        (i for i, ln in enumerate(lines) if ln.startswith("t,") or ln.startswith("time_s,")),
        None,
    )
    if hdr_idx is None:
        print(f"ERROR: no CSV header in {path}")
        return 1
    reader = csv.DictReader(lines[hdr_idx:])
    rows = list(reader)
    if not rows:
        print(f"ERROR: no data rows in {path}")
        return 1

    t_key = "t" if "t" in rows[0] else "time_s"
    need = [
        "cf5.tau_x", "cf5.tau_y", "cf5.tau_z",
        "cf5.a_res_x", "cf5.a_res_y", "cf5.a_res_z",
        "cf5.e_r_norm",
        "cf5.motor_m1", "cf5.motor_m2", "cf5.motor_m3", "cf5.motor_m4",
        "cf5.z", "cf5.ctrltarget_z",
    ]
    missing = [k for k in need if k not in rows[0]]
    if missing:
        print(f"ERROR: missing columns: {missing}")
        return 1

    tau_nonzero = 0
    a_res_nonzero = 0
    er_nonzero = 0
    peak_tau = 0.0
    peak_tau_t = 0.0
    peak_z_err = 0.0
    pattern_ticks = 0
    tau_spike_pattern = 0
    motor_dom_counts = [0, 0, 0, 0]

    for row in rows:
        t = fval(row, t_key)
        tx, ty, tz = fval(row, "cf5.tau_x"), fval(row, "cf5.tau_y"), fval(row, "cf5.tau_z")
        ax, ay, az = fval(row, "cf5.a_res_x"), fval(row, "cf5.a_res_y"), fval(row, "cf5.a_res_z")
        en = fval(row, "cf5.e_r_norm")
        motors = [
            fval(row, "cf5.motor_m1"), fval(row, "cf5.motor_m2"),
            fval(row, "cf5.motor_m3"), fval(row, "cf5.motor_m4"),
        ]
        z = fval(row, "cf5.z")
        zsp = fval(row, "cf5.ctrltarget_z")

        if any(abs(v) > 1e-9 for v in (tx, ty, tz) if not math.isnan(v)):
            tau_nonzero += 1
        if any(abs(v) > 1e-9 for v in (ax, ay, az) if not math.isnan(v)):
            a_res_nonzero += 1
        if not math.isnan(en) and abs(en) > 1e-9:
            er_nonzero += 1

        tm = max(abs(tx) if not math.isnan(tx) else 0.0,
                 abs(ty) if not math.isnan(ty) else 0.0,
                 abs(tz) if not math.isnan(tz) else 0.0)
        if tm > peak_tau:
            peak_tau = tm
            peak_tau_t = t
            peak_z_err = z - zsp if not (math.isnan(z) or math.isnan(zsp)) else float("nan")

        dom, idx, ratio = motor_pattern(motors)
        if dom:
            pattern_ticks += 1
            if idx >= 0:
                motor_dom_counts[idx] += 1
            if tm > 1e-4:
                tau_spike_pattern += 1

    n = len(rows)
    print(f"=== check_omar_rust_telemetry: {path.name} ===")
    print(f"rows={n}  duration_s≈{fval(rows[-1], t_key) - fval(rows[0], t_key):.2f}")

    print("\n--- INDI / attitude telemetry (cf5) ---")
    print(f"  tau_x/y/z nonzero rows:     {tau_nonzero}/{n}")
    print(f"  a_res_x/y/z nonzero rows:   {a_res_nonzero}/{n}")
    print(f"  e_r_norm nonzero rows:      {er_nonzero}/{n}")
    if tau_nonzero == 0 and a_res_nonzero == 0:
        print("  ** EXPECTED for pre-fix flights: tau/a_res dead-zero (telemetry gap, docs/41 §13).")
        print("  ** Next c=10 flight with indi_tau_write fix: these MUST be >0 or telemetry regressed.")
    elif tau_nonzero > 0:
        print("  OK: tau telemetry present (post-fix build or bridge active).")

    print(f"\n  peak |tau| = {peak_tau:.6f} N·m at t={peak_tau_t:.3f} s")

    # Re-scan for z at peak tau time
    best = min(rows, key=lambda r: abs(fval(r, t_key) - peak_tau_t))
    z_at = fval(best, "cf5.z")
    zsp_at = fval(best, "cf5.ctrltarget_z")
    m_at = [fval(best, f"cf5.motor_m{i}") for i in range(1, 5)]
    print(f"  nearest row: t={fval(best, t_key):.3f}  z={z_at:.3f}  target_z={zsp_at:.3f}  "
          f"dz={z_at - zsp_at:+.3f}  motors={[round(x, 0) for x in m_at]}")

    print("\n--- Single-motor-dominant PWM pattern (any motor >2× others) ---")
    print(f"  ticks with pattern: {pattern_ticks}/{n} ({100.0 * pattern_ticks / max(n, 1):.1f}%)")
    print(f"  dominant motor counts (m1..m4): {motor_dom_counts}")
    print(f"  ticks with |tau|>1e-4 AND pattern: {tau_spike_pattern}")
    if pattern_ticks > n * 0.05:
        print("  ** Pattern present (compare to 2026-09-29 c=10 ground failure signature).")
    if tau_spike_pattern > 0:
        print("  ** |tau| spike coincided with single-motor pattern — supports omega_des/B1 guard hypothesis.")
    elif peak_tau > 0.01 and pattern_ticks == 0:
        print("  note: tau spike without single-motor pattern on PWM columns.")

    print("\n--- Sample window (first 5 rows with motors) ---")
    shown = 0
    for row in rows:
        if shown >= 5:
            break
        motors = [fval(row, f"cf5.motor_m{i}") for i in range(1, 5)]
        if any(not math.isnan(x) and x > 100 for x in motors):
            t = fval(row, t_key)
            print(f"  t={t:.3f}  motors={motors}  tau=({fval(row,'cf5.tau_x'):.4f},"
                  f"{fval(row,'cf5.tau_y'):.4f},{fval(row,'cf5.tau_z'):.4f})  "
                  f"z={fval(row,'cf5.z'):.3f}")
            shown += 1

    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("merged_csv", type=Path)
    args = ap.parse_args()
    if not args.merged_csv.is_file():
        print(f"ERROR: not found: {args.merged_csv}")
        return 1
    return analyze(args.merged_csv)


if __name__ == "__main__":
    raise SystemExit(main())
