#!/usr/bin/env python3
"""Summarise c2_fullbank_predict.csv → c2_fullbank_sim_inference.json."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

OUT_CLAMP = 8.0


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open() as f:
        header = f.readline().strip().split(",")
    a = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return {n: a[:, i] for i, n in enumerate(header)}


def drone_summary(cols: dict, name: str, mask: np.ndarray) -> dict:
    pred = cols[f"{name}.rnn_pred_z"][mask]
    a_res = cols[f"{name}.a_res_z"][mask]
    finite = np.isfinite(pred) & np.isfinite(a_res)
    pred, a_res = pred[finite], a_res[finite]
    if len(pred) == 0:
        return {"n": 0}
    c = float(np.corrcoef(a_res, pred)[0, 1]) if len(pred) > 2 else float("nan")
    return {
        "n": int(len(pred)),
        "rnn_pred_z_rms": float(np.sqrt(np.mean(pred**2))),
        "rnn_pred_z_min": float(np.min(pred)),
        "rnn_pred_z_max": float(np.max(pred)),
        "rnn_pred_z_clamp_fraction": float(np.mean(np.abs(pred) >= OUT_CLAMP - 1e-6)),
        "rnn_pred_z_zero_fraction": float(np.mean(np.abs(pred) < 1e-9)),
        "corr_a_res_z_vs_rnn_pred_z": c,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("-o", type=Path, required=True)
    ap.add_argument("--weights", type=str, default="")
    ap.add_argument("--config", type=str, default="")
    args = ap.parse_args()

    cols = load(args.csv)
    t = cols["t"]
    m = (cols["cf231_active.z"] > 0.05) & (cols["cf_second.z"] > 0.05)

    out = {
        "date": "2026-09-26",
        "weights": args.weights,
        "config": args.config,
        "scenario": "A3 dz=0.30 predict-only rnn.en=0 backend=neuralswarm",
        "rows": int(len(t)),
        "rows_in_air": int(m.sum()),
        "csv": str(args.csv),
        "cf231_active": drone_summary(cols, "cf231_active", m),
        "cf_second": drone_summary(cols, "cf_second", m),
        "notes": (
            "rnn_pred_z_clamp_fraction = fraction at OUT_CLAMP (±8 m/s²). "
            "X/Y predictions are architecture-always-zero."
        ),
    }
    args.o.write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {args.o}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
