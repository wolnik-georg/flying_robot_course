#!/usr/bin/env python3
"""Compare training-style rel.v (500 Hz stateEstimate diff) vs packet-diff peer dv."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
from decode_usd_log import load  # noqa: E402

GATE_DVX = 1.5


def pair_by_tag(root: Path, tag: int):
    cf5 = cf2 = None
    for p in root.rglob("*.bin"):
        if "cf5" not in p.name.lower() and not p.name.startswith("cf5"):
            continue
        try:
            d = load(str(p))
            if int(round(d["run_tag"][0])) != tag:
                continue
        except Exception:
            continue
        if p.name.startswith("cf5") or "/cf5" in str(p):
            cf5 = (p, d)
        if "cf_second" in p.name:
            cf2 = (p, d)
    return cf5, cf2


def resample(t, *arrays, rate=500.0):
    t0, t1 = float(t[0]), float(t[-1])
    grid = np.arange(t0, t1, 1.0 / rate)
    out = [grid]
    for a in arrays:
        out.append(np.interp(grid, t, a))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", type=int, default=1791025846, help="usd.runTag (10-03 A8)")
    ap.add_argument("--root", type=Path, default=REPO / "experiments/logs/usd_raw")
    args = ap.parse_args()

    cf5_path = cf2_path = None
    d5 = d2 = None
    for p in args.root.rglob("*.bin"):
        try:
            d = load(str(p))
            if int(round(d["run_tag"][0])) != args.tag:
                continue
        except Exception:
            continue
        if p.name.startswith("cf5"):
            cf5_path, d5 = p, d
        elif "cf_second" in p.name:
            cf2_path, d2 = p, d
    if d5 is None or d2 is None:
        print("Could not find pair for tag", args.tag)
        return 1

    t0 = max(d5["t"][0], d2["t"][0])
    t1 = min(d5["t"][-1], d2["t"][-1])
    grid = np.arange(t0, t1, 1.0 / 500.0)
    vx5 = np.interp(grid, d5["t"], d5["vx"])
    vx2 = np.interp(grid, d2["t"], d2["vx"])
    t = grid
    # Training merge: rel vx = vx_cf5 - vx_cf_second at 500 Hz
    dv_train = vx5 - vx2
    # Packet-style: finite diff of cf_second position at ~100 Hz (downsample peer to 10 ms steps)
    t2 = d2["t"]
    x2 = d2["x"] if "x" in d2 else np.zeros_like(t2)
    # simulate peer packets every 10 ms on cf_second x
    peer_t = np.arange(t2[0], t2[-1], 0.01)
    peer_x = np.interp(peer_t, t2, x2)
    peer_vx = np.zeros_like(peer_t)
    for i in range(1, len(peer_t)):
        dt = peer_t[i] - peer_t[i - 1]
        peer_vx[i] = (peer_x[i] - peer_x[i - 1]) / dt
    rel_vx_packet = np.interp(t, peer_t, peer_vx) - vx5  # i relative to j, x only proxy

    diff = dv_train - rel_vx_packet
    frac_gate = float(np.mean(np.abs(dv_train) >= GATE_DVX))
    frac_exceed_pair = float(np.mean(np.abs(rel_vx_packet) >= GATE_DVX))
    print(f"Pair {cf5_path.name} + {cf2_path.name} tag={args.tag}")
    print(f"RMS dv_x diff (train vs packet-proxy) {np.sqrt(np.mean(diff**2)):.3f} m/s")
    print(f"|dvx|>={GATE_DVX}: train {100*frac_gate:.1f}%  packet-proxy {100*frac_exceed_pair:.1f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
