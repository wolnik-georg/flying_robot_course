#!/usr/bin/env python3
"""Held-out-SETTING test for the NS2 retrain: train on every flight except one scenario family or one nominal dz, score the left-out flights.

Leave-one-flight-out keeps near-identical repeats in training; this test leaves a whole setting out and asks whether the network
generalises to it. v1 is scored on the same flights (it saw the old ones, so only the new 2026-10-16 flights are an unbiased comparison).
  python3 experiments/analysis/run_heldout_setting.py --manifest <manifest.json> --out-dir <NEW dir> --scenario A3
  python3 experiments/analysis/run_heldout_setting.py --manifest <manifest.json> --out-dir <NEW dir> --dz 0.75
Writes heldout_results.json, MODEL_CARD.json and the trained weights (register them in weights/registry.json under a new id)."""
import argparse, hashlib, json, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]; RES = ROOT / "flying_drone_stack/tools/residual"
sys.path.insert(0, str(Path(__file__).parent)); sys.path.insert(0, str(RES))
import run_c2_lab_retrain as R  # noqa: E402
from c2_e2e_validation import eval_holdout, stage_a  # noqa: E402

def nominal_dz(f):
    try: return json.load(open(ROOT / f"experiments/logs/{f['scenario']}_{f['date']}_{f['stamp']}.meta.json")).get("params", {}).get("dz")
    except Exception: return None

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manifest", type=Path, required=True); ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--scenario", default=None); ap.add_argument("--dz", type=float, default=None)
    ap.add_argument("--v1-weights", type=Path, default=RES / "weights" / "ns2_v1.npz")
    a = ap.parse_args()
    if bool(a.scenario) == (a.dz is not None): sys.exit("give exactly one of --scenario / --dz")
    if (a.out_dir / "heldout_results.json").exists() or (a.out_dir / "held_out_model.npz").exists(): sys.exit(f"refusing to overwrite {a.out_dir}")
    mp = a.manifest if a.manifest.is_absolute() else ROOT / a.manifest
    man = json.load(open(mp)); flights = R.flights_from_manifest(mp)
    usable = stage_a(flights)["usable_flights"]; meta = {f["folder"]: f for f in man["flights"]}
    held = [k for k in usable if (meta[k]["scenario"] == a.scenario if a.scenario else nominal_dz(meta[k]) is not None and abs(nominal_dz(meta[k]) - a.dz) < 1e-6)]
    train = [k for k in usable if k not in held]
    if not held: sys.exit("no flights matched")
    a.out_dir.mkdir(parents=True, exist_ok=True); w = a.out_dir / "held_out_model.npz"
    print(f"train {len(train)} flights, held out {len(held)}: {', '.join(held)}")
    with (a.out_dir / "train.log").open("w") as lf:
        subprocess.run([R.PY, str(RES / "train.py"), *[str(flights[k]) for k in train], "-o", str(w), "--epochs", str(R.EPOCHS_FULL)], cwd=str(ROOT), check=True, stdout=lf, stderr=subprocess.STDOUT)
    res = {}
    for k in held:
        m = eval_holdout(w, flights[k]); v = eval_holdout(a.v1_weights, flights[k]); res[k] = {"new": m, "v1": v, "new_flight_of_2026-10-16": "2026-10-16" in k}
        print(f"{k:<28} R2 {m.get('r2', float('nan')):6.3f}  vs0 {m.get('pct_reduction_vs_zero', float('nan')):5.1f}%  RMSE {m.get('rmse_mps2', float('nan')):.3f} | v1 R2 {v.get('r2', float('nan')):6.3f} RMSE {v.get('rmse_mps2', float('nan')):.3f}")
    sha = lambda q: hashlib.sha256(Path(q).read_bytes()).hexdigest()
    json.dump({"held_out_rule": {"scenario": a.scenario, "dz": a.dz}, "train": train, "held_out": held, "results": res}, open(a.out_dir / "heldout_results.json", "w"), indent=1)
    json.dump({"command": " ".join(sys.argv), "git_commit": subprocess.run(["git", "rev-parse", "HEAD"], cwd=str(ROOT), capture_output=True, text=True).stdout.strip(),
               "weights_sha256": sha(w), "manifest": str(mp), "manifest_sha256": sha(mp), "n_train": len(train), "n_held_out": len(held), "v1_sha256": sha(a.v1_weights),
               "note": "held-out-setting experiment model, not for flashing"}, open(a.out_dir / "MODEL_CARD.json", "w"), indent=1)
if __name__ == "__main__": main()
