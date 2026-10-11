#!/usr/bin/env python3
"""Pose-bag clock for the relative metrics (protocol section 5): mocap poses of both drones from the rosbag2/mcap bag of a flight.

The bag (/cf5/pose, /cf_second/pose, ~100 Hz PoseStamped, one host clock) is the common clock by construction and the mocap is the
position truth, so the separation metrics are computed from it directly (no per-drone uSD clock needed).
Requires `pip install mcap mcap-ros2-support` (not in the base env). Bags live in experiments/logs/rosbags/<name>/*.mcap.
  python3 experiments/analysis/pose_bag.py --validate      # compare bag vs uSD-pair separation on all matchable A1/A8 flights
"""
from __future__ import annotations

import argparse
import datetime as dt
import glob
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
BAGS = REPO / "experiments" / "logs" / "rosbags"
LOGS = REPO / "experiments" / "logs"
DRONES = ("cf5", "cf_second")


def _reader_imports():
    try:
        from mcap.reader import make_reader
        from mcap_ros2.decoder import DecoderFactory
    except ImportError as exc:  # pragma: no cover
        raise ImportError("pose_bag needs `pip install mcap mcap-ros2-support`") from exc
    return make_reader, DecoderFactory


def bag_start_local(bag_dir: Path) -> dt.datetime:
    import yaml
    meta = yaml.safe_load(open(bag_dir / "metadata.yaml"))["rosbag2_bagfile_information"]
    return dt.datetime.fromtimestamp(meta["starting_time"]["nanoseconds_since_epoch"] / 1e9)


def bag_duration_s(bag_dir: Path) -> float:
    import yaml
    return yaml.safe_load(open(bag_dir / "metadata.yaml"))["rosbag2_bagfile_information"]["duration"]["nanoseconds"] / 1e9


def find_bag(date: str, stamp: str, tol_s: float = 150.0) -> Path | None:
    """Bag whose start time (local) is within tol_s before the flight stamp (flight stamp = start of run_formation; the bag is
    started by hand around it)."""
    try:
        t_flight = dt.datetime.strptime(f"{date} {stamp}", "%Y-%m-%d %H-%M-%S")
    except ValueError:
        return None
    best, best_d = None, tol_s
    for d in sorted(BAGS.glob("*/metadata.yaml")):
        try:
            s = bag_start_local(d.parent)
        except Exception:
            continue
        # the flight stamp lies inside the bag (or shortly after its start)
        delta = (t_flight - s).total_seconds()
        if -tol_s <= delta <= bag_duration_s(d.parent) + tol_s and abs(delta) < best_d + bag_duration_s(d.parent):
            if best is None or abs(delta) < best_d:
                best, best_d = d.parent, abs(delta)
    return best


def load_pose_bag(bag_dir: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """-> {drone: (t [s since bag start], pos [N,3])} from the PoseStamped header stamps."""
    make_reader, DecoderFactory = _reader_imports()
    out: dict[str, tuple[list, list]] = {d: ([], []) for d in DRONES}
    for f in sorted(glob.glob(str(bag_dir / "*.mcap"))):
        with open(f, "rb") as fh:
            rd = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, ch, msg, dec_ in rd.iter_decoded_messages(topics=[f"/{d}/pose" for d in DRONES]):
                name = ch.topic.split("/")[1]
                out[name][0].append(dec_.header.stamp.sec + dec_.header.stamp.nanosec * 1e-9)
                out[name][1].append([dec_.pose.position.x, dec_.pose.position.y, dec_.pose.position.z])
    res = {}
    t0 = min(min(v[0]) for v in out.values() if v[0])
    for k, (t, p) in out.items():
        if t:
            res[k] = (np.asarray(t) - t0, np.asarray(p))
    return res


def relative_metrics(bag: dict, dz_cmd: float, w_after: float = 6.0, w_before: float = 4.0) -> dict:
    """Separation metrics from the mocap poses on a common 50 Hz grid, over the airborne window (both drones above 0.3 m), minus
    w_after s at the start and w_before s at the end (same window logic as the uSD metrics)."""
    a, b = bag["cf5"], bag["cf_second"]
    grid = np.arange(max(a[0][0], b[0][0]), min(a[0][-1], b[0][-1]), 0.02)
    pa = np.column_stack([np.interp(grid, a[0], a[1][:, k]) for k in range(3)])
    pb = np.column_stack([np.interp(grid, b[0], b[1][:, k]) for k in range(3)])
    up = (pa[:, 2] > 0.3) & (pb[:, 2] > 0.3)
    if up.sum() < 100:
        return {}
    t0, t1 = grid[up][0], grid[up][-1]
    w = up & (grid >= t0 + w_after) & (grid <= t1 - w_before)
    dz = np.abs(pb[:, 2] - pa[:, 2])
    horiz = np.hypot(pb[:, 0] - pa[:, 0], pb[:, 1] - pa[:, 1])
    sag = dz_cmd - dz
    sep3 = np.sqrt(dz ** 2 + horiz ** 2)
    return {"bag_sep_err_mean_m": float(np.mean(sag[w])), "bag_sep_err_rms_m": float(np.sqrt(np.mean(sag[w] ** 2))),
            "bag_min_separation_m": float(np.min(sep3[up])), "bag_airborne_s": float(t1 - t0)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--validate", action="store_true")
    a = ap.parse_args()
    if not a.validate:
        ap.error("--validate")
    sys.path.insert(0, str(HERE))
    import pandas as pd
    rows = pd.read_csv(HERE / "out" / "study_metrics" / "study_rows.csv")
    rows = rows[rows["vehicle_id"] == "cf5"].drop_duplicates(["date", "stamp"])
    print("flight                  bag                  dz_cmd | bag sep_err mean / rms / min | uSD pair sep_err mean / rms / min")
    n = 0
    for _, r in rows.iterrows():
        bag_dir = find_bag(r["date"], r["stamp"])
        if bag_dir is None:
            continue
        meta = json.loads((LOGS / f"{r['scenario']}_{r['date']}_{r['stamp']}.meta.json").read_text())
        dz_cmd = float((meta.get("params") or {}).get("dz", meta.get("param_dz", 0.5)) or 0.5)
        m = relative_metrics(load_pose_bag(bag_dir), dz_cmd)
        if not m:
            continue
        n += 1
        print(f"{r['scenario']} {r['date']} {r['stamp']}  {bag_dir.name:22s} {dz_cmd:4.2f} | {m['bag_sep_err_mean_m']:+.3f} {m['bag_sep_err_rms_m']:.3f} {m['bag_min_separation_m']:.3f} | "
              f"{r['sep_err_mean_m']:+.3f} {r['sep_err_rms_m']:.3f} {r['min_separation_m']:.3f}")
    print(f"{n} flights compared")


if __name__ == "__main__":
    main()
