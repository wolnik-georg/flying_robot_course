#!/usr/bin/env python3
"""Detect when cf5 radio position locks onto cf_second (2-drone archived flights).

Uses the shared radio clock (no alignment). Pair CSVs by scenario prefix + timestamp.

Usage:
  python3 experiments/analysis/lock_on_scan.py
  python3 experiments/analysis/lock_on_scan.py --since 2026-09-19
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"

Y_TOL = 0.05
Z_TOL = 0.15
TILT_DEG = 20.0
LOCK_HOLD_S = 0.4  # require lock for this long at ~10 Hz (~4 samples)


def load_radio(path: Path):
    meta, header, rows = {}, None, []
    with open(path, newline="") as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif header is None and line and not line.startswith("#"):
                header = line.split(",")
            elif line and not line.startswith("#"):
                rows.append([float(x) for x in line.split(",")])
    if not header or not rows:
        return meta, None
    cols = {n: np.array([r[i] for r in rows], dtype=float) for i, n in enumerate(header)}
    return meta, cols


def tilt_deg(cols: dict) -> np.ndarray:
    r = cols.get("roll", np.zeros_like(cols["time_s"]))
    p = cols.get("pitch", np.zeros_like(cols["time_s"]))
    return np.sqrt(r * r + p * p)


def liftoff_time(cols: dict, z_thr: float = 0.12) -> float | None:
    t, z = cols["time_s"], cols["pos_z"]
    above = np.where(z > z_thr)[0]
    return float(t[above[0]]) if len(above) else None


def first_sustained_lock(t, dy, dz) -> float | None:
    """Earliest t such that |dy|<Y_TOL and |dz|<Z_TOL for all later samples (after hold)."""
    locked = (np.abs(dy) < Y_TOL) & (np.abs(dz) < Z_TOL)
    n = len(t)
    if n < 3:
        return None
    dt_med = float(np.median(np.diff(t)))
    need = max(2, int(round(LOCK_HOLD_S / max(dt_med, 0.05))))
    for i in range(n - need):
        if not locked[i]:
            continue
        if locked[i : i + need].all() and locked[i:].sum() >= need:
            return float(t[i])
    return None


def pair_csvs(root: Path):
    by_key: dict[str, dict[str, Path]] = {}
    pat = re.compile(r"^(A\d+)_(cf5|cf_second)_(.+)\.csv$")
    for p in root.rglob("*.csv"):
        m = pat.match(p.name)
        if not m:
            continue
        scen, role, ts = m.group(1), m.group(2), m.group(3)
        key = f"{scen}_{ts}"
        by_key.setdefault(key, {})[role] = p
    for key, mp in sorted(by_key.items()):
        if "cf5" in mp and "cf_second" in mp:
            yield key, mp["cf5"], mp["cf_second"]


def analyse_pair(key: str, p5: Path, p2: Path):
    m5, c5 = load_radio(p5)
    m2, c2 = load_radio(p2)
    if c5 is None or c2 is None:
        return None
    t0 = max(c5["time_s"][0], c2["time_s"][0])
    t1 = min(c5["time_s"][-1], c2["time_s"][-1])
    if t1 - t0 < 3.0:
        return None
    # Interpolate cf_second onto cf5 times (same clock, minor jitter)
    t = c5["time_s"]
    mask = (t >= t0) & (t <= t1)
    t = t[mask]
    y5, z5 = c5["pos_y"][mask], c5["pos_z"][mask]
    y2 = np.interp(t, c2["time_s"], c2["pos_y"])
    z2 = np.interp(t, c2["time_s"], c2["pos_z"])
    tilt = np.sqrt(c5["roll"][mask] ** 2 + c5["pitch"][mask] ** 2)

    dy, dz = y5 - y2, z5 - z2
    t_lift = liftoff_time({**c5, "time_s": t, "pos_z": z5})
    t_tilt20 = None
    idx20 = np.where(tilt > TILT_DEG)[0]
    if len(idx20):
        t_tilt20 = float(t[idx20[0]])
    t_lock = first_sustained_lock(t, dy, dz)
    max_tilt = float(np.max(tilt)) if len(tilt) else 0.0
    date = re.search(r"(\d{4}-\d{2}-\d{2})", p5.name)
    return {
        "key": key,
        "date": date.group(1) if date else "?",
        "scenario": p5.name.split("_")[0],
        "n": len(t),
        "liftoff_s": t_lift,
        "tilt20_s": t_tilt20,
        "lock_s": t_lock,
        "max_tilt_deg": max_tilt,
        "lock_after_tilt20": (
            t_lock is not None and t_tilt20 is not None and t_lock >= t_tilt20 - 0.05
        ),
        "lock_without_tilt20": t_lock is not None and (t_tilt20 is None or t_lock < t_tilt20 - 0.05),
        "cf5_file": p5.name,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="2026-09-19")
    ap.add_argument("--root", type=Path, default=LOGS)
    args = ap.parse_args()

    rows = []
    for key, p5, p2 in pair_csvs(args.root):
        if args.since:
            dm = re.search(r"(\d{4}-\d{2}-\d{2})", p5.name)
            if not dm or dm.group(1) < args.since:
                continue
        r = analyse_pair(key, p5, p2)
        if r:
            rows.append(r)

    if not rows:
        print("No paired 2-drone radio CSVs found.")
        return 1

    print(
        f"{'date':<12} {'scen':<4} {'lift':>6} {'tilt20':>7} {'lock':>7} {'maxT':>6} "
        f"{'after flip?':<12} {'lock w/o >20°':<14} file"
    )
    for r in rows:
        def ftime(x):
            return f"{x:6.1f}" if x is not None else "   —  "

        aft = "yes" if r["lock_after_tilt20"] else ("no" if r["lock_s"] else "—")
        wo = "YES" if r["lock_without_tilt20"] else "no"
        print(
            f"{r['date']:<12} {r['scenario']:<4} {ftime(r['liftoff_s'])} {ftime(r['tilt20_s'])} "
            f"{ftime(r['lock_s'])} {r['max_tilt_deg']:6.1f} {aft:<12} {wo:<14} {r['cf5_file'][:36]}"
        )

    locks = [r for r in rows if r["lock_s"] is not None]
    wo = [r for r in locks if r["lock_without_tilt20"]]
    print()
    print(f"Pairs with sustained lock: {len(locks)} / {len(rows)}")
    print(f"Lock without preceding tilt>20°: {len(wo)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
