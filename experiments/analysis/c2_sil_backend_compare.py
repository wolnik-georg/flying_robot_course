#!/usr/bin/env python3
"""Compare SIL predict logs (neuralswarm vs np) — same metrics as c2_sil_predict_diagnosis."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flying_drone_stack/tools/residual"))
import dataset  # noqa: E402

from c2_sil_predict_diagnosis import (  # noqa: E402
    OUT_CLAMP,
    corr,
    lag_sweep,
    load_residual_csv,
    quantiles,
    sil_relative_features,
    training_bank_stats,
)

STAGE_A = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json"


def diagnose_csv(csv_path: Path, label: str) -> dict:
    cols = load_residual_csv(csv_path)
    ego, peer = "cf231_active", "cf_second"
    a_res = cols[f"{ego}.a_res_z"]
    pred = cols[f"{ego}.rnn_pred_z"]
    m = (cols[f"{ego}.z"] > 0.05) & (cols[f"{peer}.z"] > 0.05)
    m &= np.isfinite(a_res) & np.isfinite(pred)
    rel = sil_relative_features(cols, ego, peer)
    sil_gated = rel["gated"] & m
    phases = []
    for name, sel in (
        ("gated_true", sil_gated),
        ("gated_false", m & ~rel["gated"]),
    ):
        phases.append(
            {
                "phase": name,
                "n": int(sel.sum()),
                "corr": corr(a_res[sel], pred[sel]),
                "clamp_fraction": float(
                    np.mean(np.abs(pred[sel]) >= OUT_CLAMP - 1e-6)
                ),
            }
        )
    return {
        "label": label,
        "csv": str(csv_path),
        "n_in_air": int(m.sum()),
        "corr_lag0_in_air": corr(a_res[m], pred[m]),
        "a_res_z_in_air": quantiles(a_res[m]),
        "rnn_pred_z_clamp_fraction_in_air": float(
            np.mean(np.abs(pred[m]) >= OUT_CLAMP - 1e-6)
        ),
        "rnn_pred_z_in_air": quantiles(pred[m]),
        "per_phase": phases,
        "lag_sweep_in_air": lag_sweep(a_res[m], pred[m], max_lag=5),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--neuralswarm-csv",
        type=Path,
        default=ROOT / "experiments/sim_validation/c2_fullbank_predict.csv",
    )
    ap.add_argument(
        "--np-csv",
        type=Path,
        default=ROOT / "experiments/sim_validation/c2_fullbank_predict_np.csv",
    )
    ap.add_argument(
        "-o",
        type=Path,
        default=ROOT
        / "experiments/analysis/out/c2_e2e_2026-09-26/sil_predict_backend_compare.json",
    )
    args = ap.parse_args()

    report = {
        "training_bank_y_gated": training_bank_stats(STAGE_A).get("y", {}),
        "neuralswarm": diagnose_csv(args.neuralswarm_csv, "backend=neuralswarm"),
        "np": diagnose_csv(args.np_csv, "backend=np"),
    }
    args.o.parent.mkdir(parents=True, exist_ok=True)
    args.o.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
