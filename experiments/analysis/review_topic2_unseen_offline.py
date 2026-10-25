#!/usr/bin/env python3
"""Independent review (Topic 2): score the existing scratch networks of the 10-22 held-out / boring-weight study (no training) and the released v2
on the 2026-10-23 unseen-setting flights without the NN applied (S0/S1), next to the 37 pilot flights. Same scoring code as ns2_heldout_pilot_study.py
(in-gate RMSE vs the causal label of dataset.build); adds the in-gate mean of prediction and label.
  python3 experiments/analysis/review_topic2_unseen_offline.py        (system python3: torch)"""
import json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[2]; RES = ROOT / "flying_drone_stack/tools/residual"; sys.path.insert(0, str(RES)); sys.path.insert(0, str(ROOT / "experiments/analysis"))
import dataset, model as M
from eval_model import predict_batch
H = ROOT / "experiments/results/ns2_heldout_2026-10-22"; CACHE = ROOT / "experiments/results/nn_vs_indi_2026-10-22/cache"
man = {}
for b in sorted(ROOT.glob("experiments/logs/bank_*2026-10-19")) + sorted(ROOT.glob("experiments/logs/bank_*2026-10-20")) + sorted(ROOT.glob("experiments/logs/bank_unseen_*2026-10-23")):
    for fl in json.load(open(b / "training_manifest.json"))["flights"]: man[fl["folder"]] = ROOT / fl["path"]
sets = {"seen": {}, "unseen": {}}
for z in sorted(CACHE.glob("*.npz")):
    d = np.load(z, allow_pickle=True); n = str(d["folder"])
    if float(d["coef_pred"]) > -0.3 and n in man: sets["unseen" if "2026-10-23" in n else "seen"][n] = man[n]
data = {}
for k, files in sets.items():
    L = []
    for name, p in files.items():
        try: rel, mask, ground, y, _ = dataset.build([str(p)], verbose=False)
        except SystemExit: continue
        if len(y): L.append((name.split("_")[0], rel, mask, ground, np.asarray(y).reshape(len(y), -1)[:, -1]))
    data[k] = L; print(k, len(L), "flights")
nets = {"v2": RES / "weights/ns2_v2.npz"}
for v in ("all_plain", "all_w0.05", "H1_plain", "H1_w0.05"):
    for s in ("", "_s1", "_s2"): nets[v + s] = H / f"{v}{s}.npz"
out = {}
for nn, p in nets.items():
    w = np.load(p)["weights"].astype(np.float64)
    for k, L in data.items():
        for sc in ("all", "A1", "A4", "A8"):
            Y, P = [], []
            for s_, rel, mask, ground, y in L:
                if sc != "all" and s_ != sc: continue
                g = mask.sum(1) > 0; pr = np.asarray(predict_batch(w, rel, mask, ground, M.DEFAULT_MASS)).reshape(len(y), -1)[:, -1]; Y.append(y[g]); P.append(pr[g])
            Y, P = np.concatenate(Y), np.concatenate(P); out[(nn, k, sc)] = (float(np.sqrt(((P - Y) ** 2).mean())), float(P.mean()), float(Y.mean()))
def agg(v, k, sc, i):
    xs = [out[(v + s, k, sc)][i] for s in ("", "_s1", "_s2")]; return np.mean(xs), np.std(xs, ddof=1)
print("\nin-gate RMSE [m/s2] (mean +- std over 3 seeds; v2 = released network)  |  mean pred / mean label")
for k in ("seen", "unseen"):
    for sc in ("all", "A1", "A4", "A8"):
        r = out[("v2", k, sc)]; line = f"{k:6s} {sc:3s} v2 {r[0]:.3f} ({r[1]:+.2f}/{r[2]:+.2f})"
        for v in ("all_plain", "all_w0.05", "H1_plain", "H1_w0.05"):
            m, s = agg(v, k, sc, 0); line += f" | {v} {m:.3f}+-{s:.3f} ({agg(v, k, sc, 1)[0]:+.2f})"
        print(line)
for sc in ("all", "A1", "A4", "A8"):
    for v in ("all_plain", "H1_plain"):
        pass
print("\nratio held-out / all-data RMSE (seed-paired mean):")
for k in ("seen", "unseen"):
    print(k, {sc: round(float(np.mean([out[("H1_plain" + s, k, sc)][0] / out[("all_plain" + s, k, sc)][0] for s in ("", "_s1", "_s2")])), 3) for sc in ("all", "A1", "A4", "A8")})
print("ratio boring-0.05 / plain RMSE (all data, seed-paired mean):")
for k in ("seen", "unseen"):
    print(k, {sc: round(float(np.mean([out[("all_w0.05" + s, k, sc)][0] / out[("all_plain" + s, k, sc)][0] for s in ("", "_s1", "_s2")])), 3) for sc in ("all", "A1", "A4", "A8")})
