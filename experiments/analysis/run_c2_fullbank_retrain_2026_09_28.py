#!/usr/bin/env python3
"""Full C.1 bank retrain (21+23+28 Sep) — manifest, Stage A, train, LOO. docs/40."""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RES = ROOT / "flying_drone_stack/tools/residual"
OUT = ROOT / "experiments/analysis/out/c2_e2e_2026-09-28"
PY = sys.executable
EPOCHS_FULL = 40
EPOCHS_LOO = 40
LEGACY_A2 = {("A2", "19-21-05"), ("A2", "19-27-03")}

sys.path.insert(0, str(RES))
import dataset  # noqa: E402

from c2_e2e_validation import eval_holdout, stage_a  # noqa: E402


def build_flight_paths() -> dict[str, Path]:
    """Return {folder_name: path} for the post-28-Sep C.1 interaction bank."""
    flights: dict[str, Path] = {}

    cw = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-21_merged/training_eligible_crosswalk.json").read_text()
    )
    m21 = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json").read_text()
    )
    by_stamp = {e["stamp"]: e for e in m21}
    for stamp, meta in cw["entries"].items():
        if not meta.get("training_eligible"):
            continue
        if stamp == "13-25-10":
            # Superseded by 28-Sep A1_17-47-50 (A-2 rep 2/2); keep out of bank.
            continue
        e = by_stamp[stamp]
        p = ROOT / e["path"]
        flights[p.parent.name] = p

    m23 = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json").read_text()
    )
    for e in m23:
        if not e.get("training_eligible") or e.get("merge_status") != "merged":
            continue
        if e["scenario"] == "C5":
            continue
        if (e["scenario"], e["stamp"]) in LEGACY_A2:
            continue
        p = ROOT / e["path"]
        flights[p.parent.name] = p

    m28 = json.loads(
        (ROOT / "experiments/logs/c1_2026-09-28_merged/manifest_2026-09-28_c1.json").read_text()
    )
    for e in m28:
        if not e.get("training_eligible"):
            continue
        p = ROOT / e["path"]
        flights[p.parent.name] = p

    return dict(sorted(flights.items()))


def manifest_record(name: str, path: Path) -> dict:
    parts = name.split("_")
    return {
        "folder": name,
        "scenario": parts[0],
        "date": parts[1] if len(parts) > 1 else "",
        "stamp": "_".join(parts[2:]) if len(parts) > 2 else "",
        "path": str(path.relative_to(ROOT)),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    flights = build_flight_paths()
    manifest = {
        "date": "2026-09-28",
        "note": "C.1 interaction bank: 21-Sep (minus superseded A1_13-25-10), "
                "23-Sep (minus legacy A2 + solo C5), 28-Sep eligible only.",
        "n_paths": len(flights),
        "flights": [manifest_record(k, v) for k, v in flights.items()],
    }
    (OUT / "training_manifest_2026-09-28.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )

    sa = stage_a(flights)
    sa["manifest"] = str(OUT / "training_manifest_2026-09-28.json")
    (OUT / "stage_a_full_bank.json").write_text(json.dumps(sa, indent=2) + "\n")
    usable = sa["usable_flights"]
    print(f"Stage A: {sa['n_usable']} usable / {len(flights)} paths")
    for k in usable:
        pf = sa["per_flight"][k]
        st = pf.get("stats") or {}
        gated = st.get("gated_fraction", st.get("fraction_gated", "?"))
        print(f"  {k}: rows={pf['rows_kept']} gated_frac={gated}")

    train_paths = [str(flights[k]) for k in usable]
    weights = OUT / "full_bank_c1_complete.npz"
    log = OUT / "full_bank_train.log"
    with log.open("w") as lf:
        subprocess.run(
            [PY, str(RES / "train.py"), *train_paths, "-o", str(weights),
             "--epochs", str(EPOCHS_FULL)],
            cwd=str(ROOT), check=True, stdout=lf, stderr=subprocess.STDOUT,
        )
    print(f"wrote {weights}")

    wdir = OUT / "loo_weights"
    wdir.mkdir(exist_ok=True)
    folds = {}
    for hold in usable:
        train = [str(flights[k]) for k in usable if k != hold]
        short = hold.replace("_2026-09-28_", "_").replace("_2026-09-23_", "_").replace(
            "_2026-09-21_", "_")
        w_path = wdir / f"train_without_{short}.npz"
        subprocess.run(
            [PY, str(RES / "train.py"), *train, "-o", str(w_path),
             "--epochs", str(EPOCHS_LOO)],
            cwd=str(ROOT), check=True,
        )
        metrics = eval_holdout(w_path, flights[hold])
        folds[hold] = {"weights": str(w_path), "held_out": hold, "metrics": metrics}
        m = metrics
        print(
            f"LOO {hold}: RMSE={m.get('rmse_mps2', float('nan')):.4f} "
            f"pct_vs_zero={m.get('pct_reduction_vs_zero', float('nan')):.1f}%"
        )
    loo_path = OUT / "loo_results.json"
    loo_path.write_text(json.dumps({"usable": usable, "folds": folds}, indent=2) + "\n")
    print(f"wrote {loo_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
