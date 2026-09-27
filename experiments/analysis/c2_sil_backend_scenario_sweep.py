#!/usr/bin/env python3
"""Aggregate multi-scenario SIL backend comparison (a_res_z vs training bank)."""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "flying_drone_stack/tools/residual"))
from c2_sil_predict_diagnosis import load_residual_csv, quantiles, training_bank_stats  # noqa: E402

STAGE_A = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/stage_a_full_bank.json"
TRAIN_P50 = -1.32
TRAIN_P99 = 0.21


def _json_sanitize(obj):
    if isinstance(obj, dict):
        return {k: _json_sanitize(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_json_sanitize(v) for v in obj]
    if isinstance(obj, float) and not np.isfinite(obj):
        return None
    return obj


def a_res_in_air(cols: dict) -> np.ndarray:
    ego, peer = "cf231_active", "cf_second"
    m = (cols[f"{ego}.z"] > 0.05) & (cols[f"{peer}.z"] > 0.05)
    a = cols[f"{ego}.a_res_z"]
    return a[m & np.isfinite(a)]


def summarize_csv(path: Path) -> dict:
    cols = load_residual_csv(path)
    a = a_res_in_air(cols)
    q = quantiles(a) if len(a) else {}
    p50 = float(q.get("p50", float("nan")))
    p99 = float(q.get("p99", float("nan")))
    return {
        "csv": str(path),
        "n_in_air": int(len(a)),
        "a_res_z_in_air": q,
        "ratio_p50_vs_train": (p50 / TRAIN_P50 if TRAIN_P50 and np.isfinite(p50) else float("nan")),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep-dir", type=Path, required=True)
    ap.add_argument(
        "-o",
        type=Path,
        default=ROOT / "experiments/analysis/out/c2_e2e_2026-09-27/sil_backend_scenario_sweep.json",
    )
    ap.add_argument(
        "--extra",
        action="append",
        default=[],
        help="scenario_id,neuralswarm_csv,np_csv (repeatable)",
    )
    args = ap.parse_args()

    scenarios: dict[str, dict] = {}
    for p in sorted(args.sweep_dir.glob("*_neuralswarm.csv")):
        m = re.match(r"(.+)_neuralswarm\.csv$", p.name)
        if not m:
            continue
        sid = m.group(1)
        np_path = args.sweep_dir / f"{sid}_np.csv"
        if not np_path.is_file():
            continue
        scenarios[sid] = {
            "neuralswarm": summarize_csv(p),
            "np": summarize_csv(np_path),
        }

    for item in args.extra:
        sid, ns, npf = item.split(",", 2)
        scenarios[sid] = {
            "neuralswarm": summarize_csv(Path(ns)),
            "np": summarize_csv(Path(npf)),
        }

    train = training_bank_stats(STAGE_A).get("y", {})
    report = {
        "training_bank_a_res_z_gated": train,
        "training_reference_p50_mps2": TRAIN_P50,
        "training_reference_p99_mps2": TRAIN_P99,
        "scenarios": scenarios,
    }
    args.o.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(_json_sanitize(report), indent=2) + "\n"
    args.o.write_text(text)
    print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
