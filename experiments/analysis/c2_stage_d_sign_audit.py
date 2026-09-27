#!/usr/bin/env python3
"""Audit Stage D synthetic dz sweep vs training-data geometry convention."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
WEIGHTS = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz"
STAGE_A = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json"

sys.path.insert(0, str(RES))
import dataset  # noqa: E402
import model as M  # noqa: E402
from model import firmware_forward  # noqa: E402


def sweep(dp_z_fn, dz_grid: np.ndarray) -> np.ndarray:
    w = np.load(WEIGHTS)["weights"].astype(np.float64)
    own_z, own_v, mass = 0.8, np.zeros(3), M.DEFAULT_MASS
    preds = []
    for dz in dz_grid:
        dp = np.array([0.0, 0.0, dp_z_fn(dz)], np.float32)
        dv = np.zeros(3, np.float32)
        a, _ = firmware_forward(w, [(dp, dv)], own_z, own_v, mass=mass, apply_clamp=False)
        preds.append(float(a))
    return np.array(preds)


def training_conditional_means(paths: list[Path]) -> dict:
    rel, mask, ground, y, _ = dataset.build([str(p) for p in paths], verbose=False)
    dz_vals, y_vals = [], []
    for i in range(len(y)):
        for k in range(rel.shape[1]):
            if mask[i, k] < 0.5:
                continue
            dz_vals.append(float(rel[i, k, 2]))
            y_vals.append(float(y[i]))
            break
    dz_vals = np.array(dz_vals)
    y_vals = np.array(y_vals)
    # overhead only (peer above ego)
    oh = dz_vals > 0.02
    edges = np.linspace(0.05, 1.2, 13)
    bins = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = oh & (dz_vals >= lo) & (dz_vals < hi)
        if sel.sum() < 20:
            continue
        bins.append(
            {
                "dz_lo": float(lo),
                "dz_hi": float(hi),
                "n": int(sel.sum()),
                "y_mean": float(np.mean(y_vals[sel])),
                "y_p50": float(np.median(y_vals[sel])),
                "abs_y_mean": float(np.mean(np.abs(y_vals[sel]))),
            }
        )
    return {
        "n_gated_overhead": int(oh.sum()),
        "corr_dz_vs_abs_y_overhead": float(
            np.corrcoef(dz_vals[oh], np.abs(y_vals[oh]))[0, 1]
        )
        if oh.sum() > 50
        else float("nan"),
        "corr_dz_vs_y_overhead": float(np.corrcoef(dz_vals[oh], y_vals[oh])[0, 1])
        if oh.sum() > 50
        else float("nan"),
        "binned_overhead": bins,
    }


def main() -> int:
    sa = json.loads(STAGE_A.read_text())
    paths = [
        ROOT / v["path"]
        for v in sa["per_flight"].values()
        if v.get("rows_kept", 0) > 0
    ]
    dz_grid = np.linspace(0.05, 1.2, 25)

    # Convention A: dp = peer - own, dz positive overhead (matches dataset.py / firmware)
    pred_a = sweep(lambda dz: dz, dz_grid)
    # Convention B: sign flip (own - peer) — would be a script bug if this matched training better
    pred_b = sweep(lambda dz: -dz, dz_grid)

    def metrics(preds: np.ndarray) -> dict:
        return {
            "a_at_dz_min": float(preds[0]),
            "a_at_dz_max": float(preds[-1]),
            "abs_larger_at_small_dz": bool(abs(preds[0]) >= abs(preds[-1])),
            "correlation_abs_a_vs_dz": float(np.corrcoef(dz_grid, np.abs(preds))[0, 1]),
            "correlation_a_vs_dz": float(np.corrcoef(dz_grid, preds)[0, 1]),
        }

    report = {
        "synthetic_sweep_peer_minus_own": {
            "metrics": metrics(pred_a),
            "a_res_z_unclamped": pred_a.tolist(),
        },
        "synthetic_sweep_own_minus_peer_bug_hypothesis": {
            "metrics": metrics(pred_b),
            "a_res_z_unclamped": pred_b.tolist(),
        },
        "training_gated_overhead_labels": training_conditional_means(paths),
    }
    out = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/stage_d_sign_audit.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
