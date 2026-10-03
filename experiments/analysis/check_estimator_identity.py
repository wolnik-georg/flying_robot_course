#!/usr/bin/env python3
"""Compare stateEstimate between cf5 and cf_second on the same flight (uSD archives).

Pairs logs by in-file ``run_tag`` (``usd.runTag``), not thesis index or filename order.
Aligns both drones on a common time grid via linear interpolation on overlap.

Usage:
  python3 experiments/analysis/check_estimator_identity.py --since 2026-09-19
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
USD_RAW = REPO / "experiments" / "logs" / "usd_raw"
LOGS = REPO / "experiments" / "logs"
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
from decode_usd_log import load  # noqa: E402

DATE_RE = re.compile(r"(\d{4}-\d{2}-\d{2})")


def drone_role(name: str) -> str | None:
    n = name.lower()
    if n.startswith("cf5") or "/cf5" in n or "cf5__" in n:
        return "cf5"
    if "cf_second" in n:
        return "cf_second"
    return None


def flight_run_tag(path: Path) -> int | None:
    try:
        d = load(str(path))
    except Exception:
        return None
    if "run_tag" not in d or len(d["run_tag"]) == 0:
        return None
    tag = int(round(float(d["run_tag"][0])))
    return tag if tag != 0 else None


def align_interp(t_a, y_a, t_b, y_b, rate_hz: float = 500.0):
    """Overlap window, common grid, interpolate both onto it."""
    t0 = max(float(t_a[0]), float(t_b[0]))
    t1 = min(float(t_a[-1]), float(t_b[-1]))
    if t1 - t0 < 0.5:
        return None
    dt = 1.0 / rate_hz
    grid = np.arange(t0, t1, dt)
    if len(grid) < 50:
        return None
    ya = np.interp(grid, t_a, y_a)
    yb = np.interp(grid, t_b, y_b)
    return grid, ya, yb, t1 - t0


def verdict(cy: float, cz: float, rms_y_mm: float, rms_z_mm: float) -> str:
    # Lab criterion for "cf5 follows cf_second": y tracks (horizontal swap), z may stay offset.
    if cy > 0.99 and rms_y_mm < 50.0:
        return "y-identical (cf5→cf_second)"
    if cy < -0.5 and cz < -0.5:
        return "anti-correlated"
    if abs(cy) < 0.35 and abs(cz) < 0.35:
        return "independent"
    return "partial"


def scenario_from_meta(run_tag: int) -> str:
    for p in LOGS.rglob("*.meta.json"):
        try:
            m = json.loads(p.read_text())
        except Exception:
            continue
        if int(m.get("usd_run_tag", -1)) == run_tag:
            return str(m.get("scenario", m.get("meta:scenario", "?")))
    return "?"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", type=Path, default=USD_RAW)
    ap.add_argument("--since", default="2026-09-19")
    ap.add_argument("--rate", type=float, default=500.0)
    args = ap.parse_args()

    by_tag: dict[int, dict[str, Path]] = {}
    for p in sorted(args.root.rglob("*.bin")):
        role = drone_role(p.name)
        if role is None:
            continue
        dm = DATE_RE.search(p.name)
        if not dm or dm.group(1) < args.since:
            continue
        tag = flight_run_tag(p)
        if tag is None:
            continue
        prev = by_tag.get(tag, {}).get(role)
        if prev is None or p.stat().st_mtime > prev.stat().st_mtime:
            by_tag.setdefault(tag, {})[role] = p

    rows = []
    for tag in sorted(by_tag):
        mp = by_tag[tag]
        if "cf5" not in mp or "cf_second" not in mp:
            continue
        p5, p2 = mp["cf5"], mp["cf_second"]
        d5, d2 = load(str(p5)), load(str(p2))
        date = DATE_RE.search(p5.name)
        date_s = date.group(1) if date else "?"
        scen = scenario_from_meta(tag)
        for axis, key in (("y", "y"), ("z", "z")):
            pass
        ay = align_interp(d5["t"], d5["y"], d2["t"], d2["y"], args.rate)
        az = align_interp(d5["t"], d5["z"], d2["t"], d2["z"], args.rate)
        if ay is None or az is None:
            rows.append(
                dict(
                    date=date_s,
                    scenario=scen,
                    run_tag=tag,
                    status="no_overlap",
                    overlap_s=0.0,
                    cy=np.nan,
                    cz=np.nan,
                    rms_y_mm=np.nan,
                    rms_z_mm=np.nan,
                    mean_z_cf5=np.nan,
                    mean_z_cs=np.nan,
                    verdict="—",
                    cf5=p5.name,
                    cf_second=p2.name,
                )
            )
            continue
        _, ya, yb, ov = ay
        _, za, zb, _ = az
        cy = float(np.corrcoef(ya, yb)[0, 1])
        cz = float(np.corrcoef(za, zb)[0, 1])
        rms_y = float(np.sqrt(np.mean((ya - yb) ** 2)) * 1000)
        rms_z = float(np.sqrt(np.mean((za - zb) ** 2)) * 1000)
        v = verdict(cy, cz, rms_y, rms_z)
        rows.append(
            dict(
                date=date_s,
                scenario=scen,
                run_tag=tag,
                status="ok",
                overlap_s=ov,
                cy=cy,
                cz=cz,
                rms_y_mm=rms_y,
                rms_z_mm=rms_z,
                mean_z_cf5=float(np.mean(za)),
                mean_z_cs=float(np.mean(zb)),
                verdict=v,
                cf5=p5.name,
                cf_second=p2.name,
            )
        )

    if not rows:
        print("No cf5/cf_second pairs with matching run_tag found.")
        return 1

    print(
        f"{'date':<12} {'scen':<4} {'tag':>6} {'ov_s':>6} {'cy':>7} {'cz':>7} "
        f"{'rms_y':>7} {'rms_z':>7} {'mean_z 5/cs':>14} {'verdict':<16} files"
    )
    for r in rows:
        if r["status"] != "ok":
            print(f"{r['date']:<12} {r['scenario']:<4} {r['run_tag']:6d} {'—':>6}  no overlap")
            continue
        mz = f"{r['mean_z_cf5']:.2f}/{r['mean_z_cs']:.2f}"
        print(
            f"{r['date']:<12} {r['scenario']:<4} {r['run_tag']:6d} {r['overlap_s']:6.1f} "
            f"{r['cy']:7.3f} {r['cz']:7.3f} {r['rms_y_mm']:7.1f} {r['rms_z_mm']:7.1f} "
            f"{mz:>14} {r['verdict']:<16} {r['cf5'][:30]}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
