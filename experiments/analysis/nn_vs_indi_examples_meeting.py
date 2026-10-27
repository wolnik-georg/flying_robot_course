#!/usr/bin/env python3
"""Meeting-doc version of the NN-vs-INDI example figure (same four flights and signals as examples.png of nn_vs_indi_check.py, clearer labels).
Reads the cache of nn_vs_indi_raw.py and the pilot banks of 2026-10-19/20 (gate only). Real flights, bottom drone, NN logged but not applied (S0, S1).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/nn_vs_indi_examples_meeting.py"""
import glob, json, sys
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
CACHE = REPO / "experiments/results/nn_vs_indi_2026-10-22/cache"; OUT = REPO / "experiments/results/nn_vs_indi_2026-10-22/examples_meeting.png"
FC = 5.0
man = {}
for b in sorted(glob.glob(str(REPO / "experiments/logs/bank_*2026-10-19")) + glob.glob(str(REPO / "experiments/logs/bank_*2026-10-20"))):
    for fl in json.load(open(Path(b) / "training_manifest.json"))["flights"]: man[fl["folder"]] = REPO / fl["path"]
# reuse the helpers of the analysis script without running it
from scipy.signal import butter, filtfilt
import pandas as pd
src = (REPO / "experiments/analysis/nn_vs_indi_check.py").read_text(); ns = {"man": man, "np": np, "pd": pd, "butter": butter, "filtfilt": filtfilt, "FS": 500.0, "MAXLAG": 0.4}
exec(src[src.index("def lp("):src.index("rows, ex = [], {}")], ns)
lp, gate_mask = ns["lp"], ns["gate_mask"]
ex = {}
for f in sorted(CACHE.glob("*.npz")):
    z = np.load(f, allow_pickle=True); name = str(z["folder"])
    if name not in man or float(z["coef_pred"]) < -0.3: continue
    strat = "S1" if bool(z["indi"]) else "S0"; key = (str(z["scenario"]), strat)
    if key not in [("A8", "S0"), ("A8", "S1"), ("A1", "S0"), ("A4", "S1")] or key in ex: continue
    t = z["t"]; air = z["z"] > 0.2
    if air.sum() < 1500: continue
    i0 = np.argmax(air); w = air & (t > t[i0] + 1.0) & (t < t[np.where(air)[0][-1]] - 0.5)
    raw = z["a_imu_raw"] - z["a_rpm_raw"]; caus = z["a_imu_f"] - z["a_rpm_f"]; gate, _ = gate_mask(name, t, z["z"])
    ex[key] = dict(name=name, t=t[w] - t[w][0], P=lp(z["pred"][:, 2], FC)[w], L=lp(raw[:, 2], FC)[w], C=caus[w, 2], G=gate[w])
NAMES = {"A8": "A8 swap", "A1": "A1 stacked hold", "A4": "A4 offset line"}; CTRL = {"S0": "geometric (S0)", "S1": "INDI (S1)"}
fig, axs = plt.subplots(2, 2, figsize=(13, 7.4))
for a_, key in zip(axs.ravel(), [("A8", "S0"), ("A8", "S1"), ("A1", "S0"), ("A4", "S1")]):
    v = ex[key]
    a_.plot(v["t"], v["C"], c="tab:orange", lw=0.9, label="INDI signal, computed onboard in real time (filtered, slight delay)")
    a_.plot(v["t"], v["L"], c="k", lw=1.6, label="INDI signal, smoothed offline without delay (reference)")
    a_.plot(v["t"], v["P"], c="tab:blue", lw=1.6, label="NS2 network prediction")
    a_.fill_between(v["t"], a_.get_ylim()[0], a_.get_ylim()[1], where=v["G"], color="0.88", zorder=0, label="NS2 active (neighbour above)")
    a_.set_title(f"{NAMES[key[0]]}, {CTRL[key[1]]} controller ({v['name']})", fontsize=9); a_.set_xlabel("flight time [s]"); a_.set_ylabel("extra z acceleration [m/s²]\n(measured − rotor model; < 0 = pushed down)")
h, l = axs[0, 0].get_legend_handles_labels(); fig.legend(h, l, loc="lower center", ncol=2, fontsize=9)
fig.suptitle("Does the NS2 network predict the extra acceleration (mostly downwash) that INDI measures? Real flights, bottom drone, network logged but not applied", fontsize=11)
fig.tight_layout(rect=[0, 0.08, 1, 0.95]); fig.savefig(OUT, dpi=130); print(OUT, {k: v["name"] for k, v in ex.items()})
