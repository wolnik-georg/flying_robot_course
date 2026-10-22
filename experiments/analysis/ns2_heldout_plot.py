#!/usr/bin/env python3
"""Plot of ns2_heldout_pilot_study.py results: in-gate RMSE on the 37 pilot flights (NN not applied), mean over 3 seeds, dots = seeds.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_heldout_plot.py"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT = Path(__file__).resolve().parents[2] / "experiments/results/ns2_heldout_2026-10-22"; r = json.load(open(OUT / "results.json"))
M = [("all_plain", "trained with the\npilot settings", "#1f77b4"), ("H1_plain", "pilot settings\nheld out", "#d62728"), ("all_w0.2", "all data,\nboring weight 0.2", "#9ecae1"), ("all_w0.05", "all data,\nboring weight 0.05", "#c6dbef"),
     ("H1_w0.2", "held out,\nweight 0.2", "#ff9896"), ("H1_w0.05", "held out,\nweight 0.05", "#fcbba1")]
M = [m for m in M if m[0] in r]
cells = [("gate", "all in-gate rows"), ("gate_A1", "A1 hold"), ("gate_A4", "A4 offset line"), ("gate_A8", "A8 swap")]
fig, ax = plt.subplots(figsize=(12, 4.2)); wd = 0.8 / len(M)
for j, (k, lab, c) in enumerate(M):
    ks = [x for x in (k, k + "_s1", k + "_s2") if x in r]
    for i, (cell, _) in enumerate(cells):
        v = np.array([r[x][cell]["rmse"] for x in ks]); xx = i - 0.4 + wd * (j + 0.5)
        ax.bar(xx, v.mean(), wd * 0.92, color=c, label=lab.replace("\n", " ") if i == 0 else None); ax.scatter(np.full(len(v), xx), v, c="k", s=8, zorder=3)
ax.axhline(0.115, c="0.4", ls=":", lw=1); ax.text(3.45, 0.12, "label noise floor 0.115", ha="right", fontsize=8, color="0.3")
ax.set_xticks(range(len(cells))); ax.set_xticklabels([c[1] for c in cells]); ax.set_ylabel("RMSE of NN vs measured residual [m/s²]"); ax.set_ylim(0, 0.8)
ax.legend(fontsize=7, ncol=3, loc="upper left"); ax.set_title("NS2 v2-type network on pilot flights (S0/S1, other bottom vehicle): effect of holding out the pilot settings and of down-weighting near-zero rows", fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "heldout_rmse.png", dpi=130)
