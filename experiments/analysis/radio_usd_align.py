#!/usr/bin/env python3
"""Align radio CSV vs uSD decode for one drone; attitude-based lag + position overlay."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
from decode_usd_log import load  # noqa: E402


def load_radio_csv(path: Path):
    meta, header, rows = {}, None, []
    with open(path) as f:
        for line in f:
            line = line.rstrip("\n")
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif header is None and line and not line.startswith("#"):
                header = line.split(",")
            elif line and not line.startswith("#"):
                rows.append([float(x) for x in line.split(",")])
    a = np.array(rows) if rows else np.zeros((0, 1))
    cols = {n: a[:, i] for i, n in enumerate(header)} if header and len(rows) else {}
    return meta, cols


def xcorr_lag(t_a, sig_a, t_b, sig_b, max_shift_s=5.0, rate=100.0):
    t0 = max(t_a[0], t_b[0])
    t1 = min(t_a[-1], t_b[-1])
    if t1 - t0 < 2.0:
        return None
    dt = 1.0 / rate
    grid = np.arange(t0, t1, dt)
    sa = np.interp(grid, t_a, sig_a)
    sb = np.interp(grid, t_b, sig_b)
    sa = sa - sa.mean()
    sb = sb - sb.mean()
    max_k = int(max_shift_s * rate)
    best_k, best_c = 0, -1.0
    for k in range(-max_k, max_k + 1):
        if k >= 0:
            a, b = sa[k:], sb[: len(sa) - k]
        else:
            a, b = sa[: len(sa) + k], sb[-k:]
        if len(a) < 50:
            continue
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))
        if c > best_c:
            best_c, best_k = c, k
    lag_s = best_k / rate
    return lag_s, best_c, grid, sa, sb


def summarize(name, t, z):
    return dict(
        name=name,
        n=len(z),
        mean_z=float(np.mean(z)),
        std_z=float(np.std(z)),
        min_z=float(np.min(z)),
        max_z=float(np.max(z)),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radio", type=Path, required=True)
    ap.add_argument("--usd", type=Path, required=True)
    ap.add_argument("--label", default="drone")
    args = ap.parse_args()

    meta, radio = load_radio_csv(args.radio)
    usd = load(str(args.usd))
    tr = radio["time_s"]
    tu = usd["t"]
    lag_roll, c_roll, _, _, _ = xcorr_lag(tr, radio["roll"], tu, usd["roll_deg"])
    lag_pitch, c_pitch, _, _, _ = xcorr_lag(tr, radio["pitch"], tu, usd["pitch_deg"])
    lag = lag_roll if c_roll >= c_pitch else lag_pitch
    corr = max(c_roll or 0, c_pitch or 0)
    print(f"=== {args.label} radio={args.radio.name} usd={args.usd.name} ===")
    print(f"attitude xcorr lag_s={lag:+.3f} quality={corr:.3f} (roll={c_roll:.3f} pitch={c_pitch:.3f})")

    tu_shift = tu + lag
    t0 = max(tr[0], tu_shift[0])
    t1 = min(tr[-1], tu_shift[-1])
    dt = 0.02
    grid = np.arange(t0, t1, dt)
    rz = np.interp(grid, tr, radio["pos_z"])
    uz = np.interp(grid, tu_shift, usd["z"])
    diff = rz - uz
    print(
        f"aligned pos_z: radio mean={rz.mean():.3f} usd={uz.mean():.3f} "
        f"RMS diff={np.sqrt(np.mean(diff**2))*1000:.1f} mm corr={np.corrcoef(rz,uz)[0,1]:.3f}"
    )
    print(f"radio only: {summarize('radio', tr, radio['pos_z'])}")
    print(f"usd only:   {summarize('usd', tu, usd['z'])}")


if __name__ == "__main__":
    main()
