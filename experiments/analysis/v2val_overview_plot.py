#!/usr/bin/env python3
"""One simple overview figure of the unified-tree comparison (2026-10-19): A8 crossing dip and A1 mean z error per strategy, dots = flights, uSD logs only.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_overview_plot.py"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
A8 = json.load(open(REPO / "experiments/results/v2val_2026-10-19/v2val_summary.json")); A1 = json.load(open(REPO / "experiments/results/a1_2026-10-19/a1_summary.json"))
ORDER = ["S0", "S1", "S2", "S3"]; LAB = {"S0": "S0\ngeometric", "S1": "S1\nINDI", "S2": "S2\ngeometric\n+ NS2", "S3": "S3\nINDI\n+ NS2"}; COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}
M8, M1 = [], []
DRAIN = {"17-55-03", "17-56-35"}
fig, ax = plt.subplots(1, 2, figsize=(10, 4.3))
for i, n in enumerate(ORDER):
    fl = [r for r in A8 if r["name"] == n and r["file"].split("_")[-1] not in DRAIN]; v = [r["dip_mean_cm"] for r in fl]
    ax[0].bar(i, np.mean(v), color=COL[n], alpha=.4, width=.6); ax[0].scatter([i + (j - (len(v) - 1) / 2) * .08 for j in range(len(v))], v, color=COL[n], s=22, zorder=3)
    rr = [r for r in A1 if r["strategy"] == n]; z = [r["z_mean_cm"] for r in rr]
    M8.append(np.mean(v)); M1.append(np.mean(z))
    ax[1].bar(i, np.mean(z), color=COL[n], alpha=.4, width=.6); ax[1].scatter([i + (j - (len(z) - 1) / 2) * .08 for j in range(len(z))], z, color=COL[n], s=22, zorder=3)
for a, M, t, y in ((ax[0], M8, "A8 swap: height dip at the crossing", "worst z error at the crossing [cm]\n(per flight: mean of 4 crossings)"), (ax[1], M1, "A1 stacked hold: mean height error", "mean z error in the 15 s hold [cm]")):
    a.set_xticks(range(4)); a.set_xticklabels([f"{LAB[n]}\n{m:+.1f} cm" for n, m in zip(ORDER, M)], fontsize=8); a.axhline(0, color="k", lw=.6); a.set_title(t, fontsize=10); a.set_ylabel(y, fontsize=8)
ax[0].set_ylim(-16, 1.5); ax[1].set_ylim(-23, 5)
fig.suptitle("Unified controller tree, NS2 v2, bottom vehicle (bar = mean of the flights, dots = flights, uSD logs only; 2026-10-19)", fontsize=9)
fig.tight_layout(); out = REPO / "docs/meetings/assets/2026-10-19/overview.png"; fig.savefig(out, dpi=130); print(out)
