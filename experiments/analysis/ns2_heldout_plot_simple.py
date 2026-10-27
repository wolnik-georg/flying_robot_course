#!/usr/bin/env python3
"""Simple version of ns2_heldout_plot.py for the meeting doc: four networks (v2 recipe, pilot settings held out, boring weight 0.05, both), in-gate RMSE on the 37 pilot flights, mean over 3 seeds, dots = seeds.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_heldout_plot_simple.py"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT = Path(__file__).resolve().parents[2] / "experiments/results/ns2_heldout_2026-10-22"; r = json.load(open(OUT / "results.json"))
M = [("all_plain", "trained on all 73 flights (ns2_v2 recipe)", "#1f77b4"), ("H1_plain", "trained without the tested settings", "#d62728"), ("all_w0.05", "all flights, near-zero rows at 5 % weight", "#9ecae1"), ("H1_w0.05", "without the tested settings + near-zero rows at 5 %", "#fcbba1")]
cells = [("gate", "all cells"), ("gate_A1", "A1 hold"), ("gate_A4", "A4 offset line"), ("gate_A8", "A8 swap")]
fig, ax = plt.subplots(figsize=(10, 4.2)); wd = 0.8 / len(M)
for j, (k, lab, c) in enumerate(M):
    ks = [x for x in (k, k + "_s1", k + "_s2") if x in r]
    for i, (cell, _) in enumerate(cells):
        v = np.array([r[x][cell]["rmse"] for x in ks]); xx = i - 0.4 + wd * (j + 0.5)
        ax.bar(xx, v.mean(), wd * 0.92, color=c, label=lab if i == 0 else None); ax.scatter(np.full(len(v), xx), v, c="k", s=9, zorder=3)
ax.set_xticks(range(len(cells))); ax.set_xticklabels([c[1] for c in cells]); ax.set_ylabel("network error vs INDI signal\n(RMSE, NS2 active) [m/s²]"); ax.set_ylim(0, 0.8); ax.legend(fontsize=8, ncol=2, loc="upper left")
ax.set_title("Network prediction error on the 37 test flights (real flights): trained without the tested settings, and with near-zero rows down-weighted\n(bars = mean of 3 retrainings, dots = single retrainings)", fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "heldout_rmse_simple.png", dpi=140)
