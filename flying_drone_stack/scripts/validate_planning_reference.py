#!/usr/bin/env python3
"""
Validate exported planning reference trajectories and write a graded report.

Reads, per folder under results/planning_sim/reference/mode*/*/:
  reference.csv, planning_meta.txt, feasibility_sampled.txt (flat),
  feasibility.txt (Richter), feasibility_se3.txt (Mode 2),
  motor_mixing_summary.txt

Run after:
  cargo run --release --bin planning_sim -- --export-reference-only
  ~/.pyenv/versions/flying_robots/bin/python scripts/plot_planning_reference.py   # optional

  ~/.pyenv/versions/flying_robots/bin/python scripts/validate_planning_reference.py
"""

from __future__ import annotations

import glob
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REF_ROOT = os.path.join(_ROOT, "results", "planning_sim", "reference")
REPORT_PATH = os.path.join(_ROOT, "results", "planning_sim", "validation_report.txt")

# Numerical consistency: max relative error (pos derivative vs reported vel).
# Denominator uses max(|v_csv|, |∇p|, VEL_REL_FLOOR_mps) so near-stationary crossings
# (figure-8) do not explode rel error. First / last samples are dropped (last two for
# periodic loops with duplicate endpoint samples) because np.gradient is inaccurate there.
WARN_REL_V = 1e-3
FAIL_REL_V = 5e-2
VEL_REL_FLOOR_MPS = 0.05
INTERIOR_LO = 1
INTERIOR_HI = 2
WARN_MOTOR_FRAC = 0.99
FAIL_MOTOR_FRAC = 0.90


def _read_kv(path: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not os.path.isfile(path):
        return out
    with open(path, "r", encoding="utf-8") as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("Note") or "=" not in ln:
                continue
            k, v = ln.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def _parse_bool(s: str) -> Optional[bool]:
    s = s.strip().lower()
    if s in ("true", "1", "yes"):
        return True
    if s in ("false", "0", "no"):
        return False
    return None


def _max_rel_velocity_error(d: np.ndarray, t: np.ndarray) -> float:
    px_t = np.gradient(d["px"], t, edge_order=2)
    py_t = np.gradient(d["py"], t, edge_order=2)
    pz_t = np.gradient(d["pz"], t, edge_order=2)
    err = np.sqrt((px_t - d["vx"]) ** 2 + (py_t - d["vy"]) ** 2 + (pz_t - d["vz"]) ** 2)
    sp_csv = np.sqrt(d["vx"] ** 2 + d["vy"] ** 2 + d["vz"] ** 2)
    sp_fd = np.sqrt(px_t**2 + py_t**2 + pz_t**2)
    denom = np.maximum(np.maximum(sp_csv, sp_fd), VEL_REL_FLOOR_MPS) + 1e-12
    rel = err / denom
    n = rel.shape[0]
    if n > INTERIOR_LO + INTERIOR_HI:
        rel = rel[INTERIOR_LO : n - INTERIOR_HI]
    return float(np.nanmax(rel))


def _grade_folder(folder: str) -> Tuple[str, List[str]]:
    """Return (PASS|WARN|FAIL, reasons)."""
    reasons: List[str] = []
    csv_p = os.path.join(folder, "reference.csv")
    if not os.path.isfile(csv_p):
        return "FAIL", ["missing reference.csv"]

    d = np.genfromtxt(csv_p, delimiter=",", names=True)
    t = d["t"]
    max_rel_v = _max_rel_velocity_error(d, t)
    if max_rel_v > FAIL_REL_V:
        reasons.append(f"numerical FAIL: max rel ‖∇p-v‖/scale={max_rel_v:.3e} (>{FAIL_REL_V})")
    elif max_rel_v > WARN_REL_V:
        reasons.append(f"numerical WARN: max rel ‖∇p-v‖/scale={max_rel_v:.3e}")

    meta = _read_kv(os.path.join(folder, "planning_meta.txt"))
    mode = int(meta.get("folder_mode", "-1"))
    pk = meta.get("planner_kind", "")

    motor = _read_kv(os.path.join(folder, "motor_mixing_summary.txt"))
    frac_s = motor.get("fraction_positive_motor_mix_feasible", "1")
    try:
        frac = float(frac_s)
    except ValueError:
        frac = 1.0
    if frac < FAIL_MOTOR_FRAC:
        reasons.append(f"motor mix FAIL: fraction_feasible={frac:.4f} (<{FAIL_MOTOR_FRAC})")
    elif frac < WARN_MOTOR_FRAC:
        reasons.append(f"motor mix WARN: fraction_feasible={frac:.4f}")

    # Feasibility flags
    fs = _read_kv(os.path.join(folder, "feasibility_sampled.txt"))
    f_se3 = _read_kv(os.path.join(folder, "feasibility_se3.txt"))
    f_r = _read_kv(os.path.join(folder, "feasibility.txt"))

    if mode == 2 or pk == "Se3Trajectory":
        b = _parse_bool(f_se3.get("feasible", "true"))
        if b is False:
            reasons.append("feasibility_se3: feasible=false (thrust/ω/torque limits)")
    elif pk == "RichterTrajectory":
        b1 = _parse_bool(f_r.get("feasible", "true"))
        b2 = _parse_bool(fs.get("feasible_thrust_omega_torque", "true"))
        if b1 is False:
            reasons.append("Richter feasibility.txt: feasible=false")
        if b2 is False:
            reasons.append("feasibility_sampled: feasible_thrust_omega_torque=false")
    else:
        b2 = _parse_bool(fs.get("feasible_thrust_omega_torque", "true"))
        if b2 is False:
            reasons.append("feasibility_sampled: feasible_thrust_omega_torque=false")

    # Aggregate grade
    if any("FAIL" in r for r in reasons):
        return "FAIL", reasons
    if reasons:
        return "WARN", reasons
    return "PASS", []


def main() -> None:
    pattern = os.path.join(REF_ROOT, "mode*", "*")
    folders = sorted([p for p in glob.glob(pattern) if os.path.isdir(p)])
    if not folders:
        print("No reference folders under", REF_ROOT, file=sys.stderr)
        sys.exit(1)

    lines: List[str] = []
    lines.append("Planning reference validation report")
    lines.append("=" * 60)
    lines.append(f"Root: {REF_ROOT}")
    lines.append("")
    counts = {"PASS": 0, "WARN": 0, "FAIL": 0}

    for folder in folders:
        rel = os.path.relpath(folder, REF_ROOT)
        grade, rs = _grade_folder(folder)
        counts[grade] = counts.get(grade, 0) + 1
        lines.append(f"{rel}: {grade}")
        for r in rs:
            lines.append(f"    — {r}")
        if not rs and grade == "PASS":
            lines.append("    — OK (feasibility + motor mix + numerical check)")

    lines.append("")
    lines.append("Summary")
    lines.append("-" * 40)
    lines.append(f"  PASS: {counts['PASS']}")
    lines.append(f"  WARN: {counts['WARN']}")
    lines.append(f"  FAIL: {counts['FAIL']}")

    if counts["FAIL"] > 0:
        lines.append("")
        lines.append("Overall: NOT SATISFACTORY (at least one FAIL).")
    elif counts["WARN"] > 0:
        lines.append("")
        lines.append("Overall: SATISFACTORY WITH WARNINGS (review listed items).")
    else:
        lines.append("")
        lines.append("Overall: SATISFACTORY for exported planning references.")

    lines.append("")
    lines.append("Criteria (brief):")
    lines.append(f"  — feasibility files (thrust / |ω| / per-axis torque caps in Rust export)")
    lines.append(f"  — motor X-mix: no negative ω² before clamp (fraction ≥ {WARN_MOTOR_FRAC} WARN, <{FAIL_MOTOR_FRAC} FAIL)")
    lines.append(
        f"  — ‖∇p-v‖ vs scale (floor {VEL_REL_FLOOR_MPS} m/s, interior t): WARN >{WARN_REL_V}, FAIL >{FAIL_REL_V}"
    )

    os.makedirs(os.path.dirname(REPORT_PATH), exist_ok=True)
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print(f"\nWrote {REPORT_PATH}")
    sys.exit(0 if counts["FAIL"] == 0 else 2)


if __name__ == "__main__":
    main()
