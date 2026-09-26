#!/usr/bin/env python3
"""18-fold LOO on the full C.1 training bank (18 files with rows). docs/40 extension."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
OUT = ROOT / "experiments/analysis/out/c2_e2e_2026-09-26/loo_weights_18fold"
PY = sys.executable
EPOCHS = 40

sys.path.insert(0, str(RES))
import dataset  # noqa: E402

from c2_e2e_validation import eval_holdout  # noqa: E402


def training_paths() -> dict[str, Path]:
    m23 = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json").read_text()
    )
    paths23 = [
        ROOT / e["path"]
        for e in m23
        if e.get("training_eligible") and e.get("merge_status") == "merged"
    ]
    m21 = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json").read_text()
    )
    cw = json.loads(
        (
            ROOT / "experiments/logs/c1_2026-09-21_merged/training_eligible_crosswalk.json"
        ).read_text()
    )
    te = {k for k, v in cw["entries"].items() if v.get("training_eligible")}
    paths21 = [ROOT / e["path"] for e in m21 if e["stamp"] in te]
    flights: dict[str, Path] = {}
    for p in paths23 + paths21:
        key = p.parent.name  # e.g. A3_2026-09-23_17-45-03
        flights[key] = p
    return flights


def usable(flights: dict[str, Path]) -> list[str]:
    ok = []
    for name, path in sorted(flights.items()):
        try:
            _, _, _, y, _ = dataset.build([str(path)], verbose=False)
            if len(y) > 0:
                ok.append(name)
        except SystemExit:
            pass
    return ok


def main() -> int:
    flights = training_paths()
    use = usable(flights)
    print(f"LOO: {len(use)} usable flights (of {len(flights)} paths)")
    OUT.mkdir(parents=True, exist_ok=True)
    folds = {}
    for hold in use:
        train = [str(flights[k]) for k in use if k != hold]
        # Match 21-Sep naming: train_without_<scenario>_<stamp> from folder name
        short = hold.replace("_2026-09-23_", "_").replace("_2026-09-21_", "_")
        w_path = OUT / f"train_without_{short}.npz"
        subprocess.run(
            [PY, str(RES / "train.py"), *train, "-o", str(w_path), "--epochs", str(EPOCHS)],
            cwd=str(ROOT),
            check=True,
        )
        metrics = eval_holdout(w_path, flights[hold])
        folds[hold] = {"weights": str(w_path), "held_out": hold, "metrics": metrics}
        m = metrics
        print(
            f"  {hold}: n={m.get('n')} RMSE={m.get('rmse_mps2', float('nan')):.4f} "
            f"zero={m.get('baseline_zero_rmse', float('nan')):.4f} "
            f"pct={m.get('pct_reduction_vs_zero', float('nan')):.1f}%"
        )
    summary_path = OUT.parent / "loo_18fold_results.json"
    summary_path.write_text(json.dumps({"usable": use, "folds": folds}, indent=2) + "\n")
    print(f"wrote {summary_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
