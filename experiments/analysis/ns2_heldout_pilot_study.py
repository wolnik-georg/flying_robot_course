#!/usr/bin/env python3
"""NS2 quality, Task 2 steps 2-3 (supervisor feedback 2026-10-22): does the network generalise to the settings the study tests, and does down-weighting
near-zero rows help?  Trains ns2 variants on the 73-flight v2 bank with (a) the pilot settings held out, (b) boring rows down-weighted, scores them
and v2 on the pilot flights where the NN was NOT applied (S0, S1; label = measured residual, 43 flights, a different bottom vehicle than in training).

Held-out rules (applied to the 73 training flights):
  H1 pilot settings: A8 dz 0.5 at 0.5 m/s, A1 dz 0.5, all A4, all A9 (chained A4 settings)                       -> 14 flights
  H2 neighbourhood too: A8 dz 0.5 at every speed, A1 dz 0.375-0.625, all A4, all A9, A2/A3/R1 untouched          -> more
Variants: plain loss (as v2) or --boring-weight 0.2 (rows with |a_res| < 0.3 m/s^2).
  python3 experiments/analysis/ns2_heldout_pilot_study.py --out-dir experiments/results/ns2_heldout_2026-10-22          (system python3: torch + bindings)"""
import argparse, json, subprocess, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]; RES = ROOT / "flying_drone_stack/tools/residual"
sys.path.insert(0, str(RES)); sys.path.insert(0, str(Path(__file__).parent))
import dataset, model as M                          # noqa: E402
from eval_model import predict_batch                # noqa: E402

EPOCHS = 40


def meta(f):
    try: return json.load(open(ROOT / f"experiments/logs/{f['scenario']}_{f['date']}_{f['stamp']}.meta.json")).get("params", {})
    except Exception: return {}


def rule(f, level):
    p = meta(f); sc = f["scenario"]; dz = p.get("dz")
    if sc in ("A4", "A9"): return True
    if sc == "A1": return dz is not None and (abs(dz - 0.5) < 1e-6 if level == 1 else 0.37 <= dz <= 0.63)
    if sc == "A8": return dz is not None and abs(dz - 0.5) < 1e-6 and (abs(p.get("duration", 0) - 4.375) < 1e-3 if level == 1 else True)
    return False


def pilot_eval_files():
    """merged CSVs of the pilot flights without the NN applied (cache of nn_vs_indi_raw.py has the strategy flag)."""
    cache = ROOT / "experiments/results/nn_vs_indi_2026-10-22/cache"; man = {}
    for b in sorted(ROOT.glob("experiments/logs/bank_*2026-10-19")) + sorted(ROOT.glob("experiments/logs/bank_*2026-10-20")):
        for fl in json.load(open(b / "training_manifest.json"))["flights"]: man[fl["folder"]] = ROOT / fl["path"]
    out = {}
    for z in sorted(cache.glob("*.npz")):
        d = np.load(z, allow_pickle=True)
        if float(d["coef_pred"]) > -0.3 and str(d["folder"]) in man: out[str(d["folder"])] = man[str(d["folder"])]
    return out


def score(w, files, mass=M.DEFAULT_MASS):
    """pooled over flights: rmse / R2 / reduction vs zero for all rows, in-gate rows and big-residual rows (|y| > 0.3), plus per-scenario RMSE in the gate."""
    Y, P, G, S = [], [], [], []
    for name, path in files.items():
        try: rel, mask, ground, y, _ = dataset.build([str(path)], verbose=False)
        except SystemExit: continue
        if len(y) == 0: continue
        Y.append(np.asarray(y).reshape(len(y), -1)[:, -1]); P.append(np.asarray(predict_batch(w, rel, mask, ground, mass)).reshape(len(y), -1)[:, -1])
        G.append(mask.sum(1) > 0); S += [name.split("_")[0]] * len(y)
    Y, P, G, S = np.concatenate(Y), np.concatenate(P), np.concatenate(G), np.array(S)
    def m(sel):
        e = P[sel] - Y[sel]; v = ((Y[sel] - Y[sel].mean()) ** 2).mean()
        return dict(n=int(sel.sum()), rmse=float(np.sqrt((e ** 2).mean())), r2=float(1 - (e ** 2).mean() / v), vs_zero_pct=float(100 * (1 - np.sqrt((e ** 2).mean()) / np.sqrt((Y[sel] ** 2).mean()))))
    out = {"all": m(np.ones(len(Y), bool)), "gate": m(G), "big": m(np.abs(Y) > 0.3)}
    for sc in ("A1", "A4", "A8"):
        if (G & (S == sc)).sum() > 100: out[f"gate_{sc}"] = m(G & (S == sc))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--out-dir", type=Path, required=True); ap.add_argument("--variants", nargs="*", default=None); ap.add_argument("--seed", type=int, default=0); a = ap.parse_args()
    out = a.out_dir if a.out_dir.is_absolute() else ROOT / a.out_dir; out.mkdir(parents=True, exist_ok=True)
    man = json.load(open(ROOT / "experiments/results/ns2_pilot2_2026-10-17/training_manifest_pilot2.json"))["flights"]
    usable = list(man)                                                                           # all 73 flights of v2 (outliers are not in the manifest)
    paths = {f["folder"]: ROOT / f["path"] for f in usable}
    ev = pilot_eval_files(); print(len(ev), "pilot evaluation flights (NN not applied)")
    held1 = [f["folder"] for f in usable if rule(f, 1)]; held2 = [f["folder"] for f in usable if rule(f, 2)]
    V = {"all_plain": ([], 1.0), "all_w0.2": ([], 0.2), "H1_plain": (held1, 1.0), "H1_w0.2": (held1, 0.2), "H2_plain": (held2, 1.0), "H2_w0.2": (held2, 0.2), "all_w0.05": ([], 0.05), "H1_w0.05": (held1, 0.05)}
    res = json.load(open(out / "results.json")) if (out / "results.json").exists() else {}
    if "v2" not in res: res["v2"] = score(np.load(RES / "weights/ns2_v2.npz")["weights"].astype(np.float64), ev)
    for name, (held, bw) in V.items():
        if a.variants and name not in a.variants: continue
        tag = name if a.seed == 0 else f"{name}_s{a.seed}"; w = out / f"{tag}.npz"; train = [k for k in paths if k not in held]
        if not w.exists():
            print(f"train {name}: {len(train)} flights (held out {len(held)}), boring weight {bw}", flush=True)
            with (out / f"{tag}.train.log").open("w") as lf:
                subprocess.run([sys.executable, str(RES / "train.py"), *[str(paths[k]) for k in train], "-o", str(w), "--epochs", str(EPOCHS), "--boring-weight", str(bw), "--seed", str(a.seed)], cwd=str(ROOT), check=True, stdout=lf, stderr=subprocess.STDOUT)
        res[tag] = score(np.load(w)["weights"].astype(np.float64), ev); res[tag]["n_train_flights"] = len(train); res[tag]["held_out"] = held
        json.dump(res, open(out / "results.json", "w"), indent=1)
        print(tag, {k: round(v["rmse"], 3) for k, v in res[tag].items() if isinstance(v, dict) and "rmse" in v}, flush=True)
    print("v2", {k: round(v["rmse"], 3) for k, v in res["v2"].items() if isinstance(v, dict)})


if __name__ == "__main__":
    main()
