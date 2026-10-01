#!/usr/bin/env python3
"""Lab retrain: manifest → Stage A → full train → LOO → eval_report (docs/40 pattern).

Does not fly. Intended for same-day use after A5/A6/C4 merges land on disk.

  python3 experiments/analysis/run_c2_lab_retrain.py \\
    --manifest experiments/analysis/out/c2_e2e_2026-09-30/training_manifest_lab.json

Writes under ``experiments/analysis/out/c2_e2e_<date>/`` next to the manifest unless
``--out-dir`` is set.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
PY = sys.executable
EPOCHS_FULL = 40
EPOCHS_LOO = 40

sys.path.insert(0, str(RES))
import dataset  # noqa: E402

from c2_e2e_validation import eval_holdout, stage_a  # noqa: E402


def flights_from_manifest(manifest_path: Path) -> dict[str, Path]:
    m = json.loads(manifest_path.read_text())
    out: dict[str, Path] = {}
    for f in m["flights"]:
        p = ROOT / f["path"]
        out[f["folder"]] = p
    return out


def short_name(folder: str) -> str:
    return (
        folder.replace("_2026-09-30_", "_")
        .replace("_2026-09-28_", "_")
        .replace("_2026-09-23_", "_")
        .replace("_2026-09-21_", "_")
    )


def print_loo_gate(loo_path: Path, new_scenarios: tuple[str, ...]) -> None:
    """Operator-facing GO/NO-GO for rnn.en=1 (see lab checklist decision gate)."""
    data = json.loads(loo_path.read_text())
    folds = data.get("folds", {})
    print("\n=== LOO decision gate (new scenarios only) ===")
    print(f"{'held_out':<40} {'R²':>8} {'%vs0':>8} {'n':>8}  gate")
    any_new = False
    hard_fail = False
    soft_fail = False
    for hold, rec in sorted(folds.items()):
        scen = hold.split("_")[0]
        if scen not in new_scenarios:
            continue
        any_new = True
        m = rec.get("metrics") or {}
        r2 = float(m.get("r2", float("nan")))
        pct = float(m.get("pct_reduction_vs_zero", float("nan")))
        n = int(m.get("n", 0))
        if n == 0 or r2 < 0.20 or pct < 20.0:
            gate = "NO-GO (hard)"
            hard_fail = True
        elif r2 < 0.35 or pct < 50.0:
            gate = "NO-GO (soft)"
            soft_fail = True
        else:
            gate = "OK"
        print(f"{hold:<40} {r2:8.3f} {pct:8.1f} {n:8d}  {gate}")
    if not any_new:
        print(f"(no LOO folds for scenarios {new_scenarios} — manifest may lack new merges)")
        return
    if hard_fail:
        print("\nVERDICT: NO-GO for rnn.en=1 — at least one new-scenario LOO fold failed hard threshold.")
    elif soft_fail:
        print("\nVERDICT: NO-GO for rnn.en=1 — new-scenario LOO below A1–A7-like band (R²≥0.35, ≥50% vs zero).")
    else:
        print("\nVERDICT: LOO gate PASSED for new scenarios — operator may consider Checklist G (still not auto-GO).")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, default=None)
    ap.add_argument("--new-scenarios", nargs="+", default=["A5", "A6", "C4"])
    ap.add_argument("--skip-loo", action="store_true", help="full train + eval only (faster)")
    args = ap.parse_args()

    manifest_path = args.manifest if args.manifest.is_absolute() else ROOT / args.manifest
    out_dir = args.out_dir or manifest_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    flights = flights_from_manifest(manifest_path)
    sa = stage_a(flights)
    sa["manifest"] = str(manifest_path)
    (out_dir / "stage_a_full_bank.json").write_text(json.dumps(sa, indent=2) + "\n")
    usable = sa["usable_flights"]
    print(f"Stage A: {sa['n_usable']} usable / {len(flights)} paths")

    train_paths = [str(flights[k]) for k in usable]
    weights = out_dir / "full_bank_c1_complete.npz"
    log = out_dir / "full_bank_train.log"
    with log.open("w") as lf:
        subprocess.run(
            [PY, str(RES / "train.py"), *train_paths, "-o", str(weights), "--epochs", str(EPOCHS_FULL)],
            cwd=str(ROOT),
            check=True,
            stdout=lf,
            stderr=subprocess.STDOUT,
        )
    print(f"wrote {weights}")

    loo_path = out_dir / "loo_results.json"
    if not args.skip_loo:
        wdir = out_dir / "loo_weights"
        wdir.mkdir(exist_ok=True)
        folds = {}
        for hold in usable:
            train = [str(flights[k]) for k in usable if k != hold]
            w_path = wdir / f"train_without_{short_name(hold)}.npz"
            subprocess.run(
                [PY, str(RES / "train.py"), *train, "-o", str(w_path), "--epochs", str(EPOCHS_LOO)],
                cwd=str(ROOT),
                check=True,
            )
            metrics = eval_holdout(w_path, flights[hold])
            folds[hold] = {"weights": str(w_path), "held_out": hold, "metrics": metrics}
            m = metrics
            print(
                f"LOO {hold}: R²={m.get('r2', float('nan')):.3f} "
                f"RMSE={m.get('rmse_mps2', float('nan')):.4f} "
                f"pct_vs_zero={m.get('pct_reduction_vs_zero', float('nan')):.1f}%"
            )
        loo_path.write_text(json.dumps({"usable": usable, "folds": folds}, indent=2) + "\n")
        print(f"wrote {loo_path}")
        print_loo_gate(loo_path, tuple(args.new_scenarios))

    eval_dir = out_dir / "eval_model"
    subprocess.run(
        [
            PY,
            str(RES / "eval_model.py"),
            str(weights),
            *train_paths,
            "-o",
            str(eval_dir),
            "--no-plots",
        ],
        cwd=str(ROOT),
        check=True,
    )
    print(f"wrote {eval_dir}/eval_report.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
