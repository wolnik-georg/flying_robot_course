#!/usr/bin/env python3
"""A4 offset 0.125 (bottom drone cf5): roll/pitch torque the controller supplies against the horizontal offset to the top drone, FBL (all 4 flights, gain block identified
from the logs) against S0 / S1 / S3 of the unseen block. Shows the downwash edge torque that the FBL port has no term for (torque residual 0, P-only rate loop).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_a4_roll_torque.py --out experiments/results/fbl_replay_2026-10-24"""
import argparse, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); out = a.out if a.out.is_absolute() else REPO / a.out
BLK = {"14-29-07": "preset 1 TRIP", "14-44-24": "preset 1 TRIP", "14-50-08": "_kr", "15-01-21": "_kp"}
COL = {"preset 1 TRIP": "#9467bd", "_kr": "#ff7f0e", "_kp": "#2ca02c", "S0": "#7f7f7f", "S1": "#1f77b4", "S3": "#d62728"}
REF = pd.read_csv(REPO / "experiments/results/unseen_block_2026-10-23/per_flight.csv"); strat = dict(zip(REF.folder, REF.strategy))
def load(path, fbl):
    df = pd.read_csv(REPO / path, comment="#"); pb = [c.split(".")[0] for c in df.columns if c.startswith("cf5")][0]; pt = [c.split(".")[0] for c in df.columns if c.startswith("cf_second")][0]
    g = lambda p, k: df[f"{p}.{k}"].values
    m = g(pb, "z") > 0.3
    if fbl: m &= (g(pb, "fbl_active") > 0) & (g(pb, "fbl_tripped") == 0)
    return dict(t=df.t.values[m], dx=(g(pt, "x") - g(pb, "x"))[m], dy=(g(pt, "y") - g(pb, "y"))[m], tx=1e6 * g(pb, "omar_tau_x")[m], ty=1e6 * g(pb, "omar_tau_y")[m], roll=g(pb, "roll_deg")[m])
fl = []
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"] + sorted(REF.bank.unique()):
    for f in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        if f["scenario"] != "A4": continue
        meta = json.load(open(REPO / "experiments/logs" / f"A4_{f['date']}_{f['stamp']}.meta.json"))
        if abs(meta["params"].get("offset", 0) - 0.125) > 1e-6: continue
        lab = BLK.get(f["stamp"]) if "fbl" in b else strat.get(f["folder"])
        if lab in COL: fl.append((lab, f["stamp"], load(f["path"], "fbl" in b)))
fig, axs = plt.subplots(1, 2, figsize=(13, 4.5)); seen = set()
for lab, st, d in sorted(fl, key=lambda x: x[0] in ("S0", "S1", "S3"), reverse=True):
    ref = lab in ("S0", "S1", "S3"); kw = dict(c=COL[lab], s=2 if ref else 3, alpha=0.25 if ref else 0.6)
    axs[0].scatter(d["dx"], d["tx"], **kw); axs[1].plot(d["t"], d["tx"], c=COL[lab], lw=0.6 if ref else 1.2, alpha=0.5 if ref else 1)
axs[0].set_xlabel("x offset top - bottom [m]  (plan: 0.125)", fontsize=8); axs[0].set_ylabel("roll torque supplied [µN m]", fontsize=8); axs[0].set_xlim(-0.4, 0.4); axs[0].set_ylim(-4000, 2500)
axs[0].legend(handles=[plt.Line2D([], [], marker="o", ls="", c=c) for c in COL.values()], labels=["FBL preset 1 (both tripped)", "FBL _kr", "FBL _kp", "S0", "S1", "S3"], fontsize=7); axs[0].grid(alpha=0.3); axs[0].set_title("A4 offset 0.125: roll torque vs side offset to the top drone", fontsize=9)
axs[1].set_xlabel("time since scenario start [s] (FBL up to the trip)", fontsize=8); axs[1].set_ylabel("roll torque supplied [µN m]", fontsize=8); axs[1].set_ylim(-4000, 2500); axs[1].grid(alpha=0.3)
axs[1].set_title("same, over time", fontsize=9)
fig.tight_layout(); fig.savefig(out / "a4_0125_roll_torque.png", dpi=120)
for lab, st, d in fl: print(f"{lab:14s} {st} mean tau_x {d['tx'].mean():8.1f}  p5 {np.percentile(d['tx'],5):8.1f}  mean |dx| {np.abs(d['dx']).mean():.3f}  mean dy {d['dy'].mean():.3f}")
