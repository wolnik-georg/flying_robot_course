#!/usr/bin/env python3
"""Replay firmware_forward on SIL residual CSV states; compare to logged rnn_pred_z."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flying_drone_stack/tools/residual"))
from model import OUT_CLAMP, firmware_forward  # noqa: E402


def replay(csv_path: Path, weights: Path, ego: str = "cf231_active", peer: str = "cf_second") -> dict:
    w = np.load(weights)["weights"]
    h = open(csv_path).readline().strip().split(",")
    a = np.loadtxt(csv_path, delimiter=",", skiprows=1, ndmin=2)
    col = {n: a[:, i] for i, n in enumerate(h)}
    m = (col[f"{ego}.z"] > 0.05) & (col[f"{peer}.z"] > 0.05)
    logged = col[f"{ego}.rnn_pred_z"][m]
    offline, gated = [], []
    for i in np.where(m)[0]:
        ez = col[f"{ego}.z"][i]
        ev = np.array([col[f"{ego}.vx"][i], col[f"{ego}.vy"][i], col[f"{ego}.vz"][i]])
        dp = np.array(
            [
                col[f"{peer}.x"][i] - col[f"{ego}.x"][i],
                col[f"{peer}.y"][i] - col[f"{ego}.y"][i],
                col[f"{peer}.z"][i] - col[f"{ego}.z"][i],
            ]
        )
        dv = np.array(
            [
                col[f"{peer}.vx"][i] - col[f"{ego}.vx"][i],
                col[f"{peer}.vy"][i] - col[f"{ego}.vy"][i],
                col[f"{peer}.vz"][i] - col[f"{ego}.vz"][i],
            ]
        )
        g = abs(dp[0]) < 0.2 and abs(dp[1]) < 0.2 and abs(dv[0]) < 1.5
        rel = [(dp, dv)] if g else []
        offline.append(firmware_forward(w, rel, ez, ev, apply_clamp=True)[0])
        gated.append(g)
    offline = np.array(offline)
    gated = np.array(gated)
    a_res = col[f"{ego}.a_res_z"][m]

    def corr(x, y):
        if len(x) < 50 or np.std(x) < 1e-12 or np.std(y) < 1e-12:
            return float("nan")
        return float(np.corrcoef(x, y)[0, 1])

    return {
        "csv": str(csv_path),
        "weights": str(weights),
        "n_in_air": int(m.sum()),
        "logged_clamp_fraction": float(np.mean(np.abs(logged) >= OUT_CLAMP - 1e-6)),
        "offline_clamp_fraction": float(np.mean(np.abs(offline) >= OUT_CLAMP - 1e-6)),
        "corr_logged_vs_offline": corr(logged, offline),
        "corr_a_res_vs_offline_gated_true": corr(a_res[gated], offline[gated]),
        "corr_a_res_vs_logged": corr(a_res, logged),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", type=Path)
    ap.add_argument(
        "--weights",
        type=Path,
        default=ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz",
    )
    ap.add_argument("-o", type=Path, default=None)
    args = ap.parse_args()
    report = replay(args.csv, args.weights)
    text = json.dumps(report, indent=2) + "\n"
    if args.o:
        args.o.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
