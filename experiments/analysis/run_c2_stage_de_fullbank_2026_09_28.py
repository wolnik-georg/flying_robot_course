#!/usr/bin/env python3
"""Stage D/E for 2026-09-28 full C.1 bank weights."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
OUT = ROOT / "experiments/analysis/out/c2_e2e_2026-09-28"
WEIGHTS = OUT / "full_bank_c1_complete.npz"
STAGE_A = OUT / "stage_a_full_bank.json"

sys.path.insert(0, str(RES))
import dataset  # noqa: E402
import model as M  # noqa: E402
from model import firmware_forward  # noqa: E402


def training_paths() -> list[Path]:
    sa = json.loads(STAGE_A.read_text())
    return [
        ROOT / v["path"]
        for v in sa["per_flight"].values()
        if v.get("rows_kept", 0) > 0
    ]


def stage_d_synthetic(w_path: Path) -> dict:
    w = np.load(w_path)["weights"].astype(np.float64)
    dz_grid = np.linspace(0.05, 1.2, 25)
    own_z, own_v = 0.8, np.zeros(3)
    preds = []
    for dz in dz_grid:
        dp = np.array([0.0, 0.0, dz], np.float32)
        dv = np.zeros(3, np.float32)
        a, _ = firmware_forward(w, [(dp, dv)], own_z, own_v, mass=M.DEFAULT_MASS, apply_clamp=False)
        preds.append(float(a))
    preds = np.array(preds)
    return {
        "correlation_abs_a_vs_dz": float(np.corrcoef(dz_grid, np.abs(preds))[0, 1]),
        "abs_larger_at_small_dz": bool(np.abs(preds[0]) >= np.abs(preds[-1])),
    }


def stage_d_binned(w_path: Path, paths: list[Path]) -> dict:
    from eval_model import binned_error, predict_batch, neighbour_geometry  # noqa: E402

    w = np.load(w_path)["weights"].astype(np.float64)
    rel, mask, ground, y, _ = dataset.build([str(p) for p in paths], verbose=False)
    pred = predict_batch(w, rel, mask, ground, M.DEFAULT_MASS)
    err = pred - y
    dy, dz = neighbour_geometry(rel, mask)
    return {
        "n_samples": int(len(y)),
        "rmse_all": float(np.sqrt(np.mean(err**2))),
        "binned_error_dz": binned_error(dy, dz, err, "dz", np.linspace(0.05, 1.0, 8)),
    }


def main() -> int:
    paths = training_paths()
    report = {
        "date": "2026-09-28",
        "weights": str(WEIGHTS),
        "n_training_files": len(paths),
        "stage_d_synthetic": stage_d_synthetic(WEIGHTS),
        "stage_d_binned": stage_d_binned(WEIGHTS, paths),
    }
    out = OUT / "stage_de_fullbank_2026_09_28.json"
    sh = ROOT / "experiments/analysis/c2_stage_e_fullbank_dryrun.sh"
    if sh.is_file():
        r = subprocess.run(["bash", str(sh)], cwd=str(ROOT), capture_output=True, text=True, timeout=900)
        report["stage_e"] = {"exit_code": r.returncode, "stdout_tail": (r.stdout or "")[-4000:]}
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
