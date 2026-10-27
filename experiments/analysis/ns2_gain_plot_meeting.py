#!/usr/bin/env python3
"""Meeting-doc figure: NS2 gain (error reduction) at the trained vs the new settings, geometric pair S0 -> S2 and INDI pair S1 -> S3.
Values from experiments/results/unseen_block_2026-10-23/verdict.json (pre-registered retention rule). Real flights, bottom drone, uSD logs.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_gain_plot_meeting.py"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; RES = REPO / "experiments/results/unseen_block_2026-10-23"; v = json.load(open(RES / "verdict.json"))
import csv, collections
PILOT = json.load(open(REPO / "experiments/results/unified_overview_2026-10-20/summary.json"))
NEW = collections.Counter((r["cell"], r["strategy"]) for r in csv.DictReader(open(RES / "per_flight.csv")))
A4REF = {"A4 dz0.4 off0.075 v0.35": ("0.05", "0.1"), "A4 dz0.4 off0.125 v0.35": ("0.1", "0.15")}  # trained reference = interpolation between these offsets


def n_trained(cell, s):
    if cell in A4REF: return "+".join(str(PILOT["A4"][o][s]["n"]) for o in A4REF[cell])
    return str(PILOT[cell[:2]][s]["n"])
names = {"A1 dz0.45": "A1 hold", "A4 dz0.4 off0.075 v0.35": "A4 0.075", "A4 dz0.4 off0.125 v0.35": "A4 0.125", "A8 dz0.4 dur6.25": "A8 swap"}
fig, axs = plt.subplots(1, 2, figsize=(15, 5.6))
for ax, pair, ttl in ((axs[0], "geometric", "S0 → S2: NS2 added to the geometric controller (pre-registered test)"), (axs[1], "INDI", "S1 → S3: NS2 added to INDI")):
    x = np.arange(len(v["cells"])); b_, n_ = ("S0", "S2") if pair == "geometric" else ("S1", "S3")
    for k, (key, lab, c) in enumerate(((f"RI_pilot_{pair}", "trained settings (in the NS2 training data)", "#7f7f7f"), (f"RI_unseen_{pair}", "new settings (not in the training data)", "#d62728"))):
        y = np.array([100 * cl[key] for cl in v["cells"]]); yc = np.clip(y, -100, 100); xs = x + (k - 0.5) * 0.38
        ax.bar(xs, yc, 0.38, color=c, label=lab)
        for xi, yi, yv, cl in zip(xs, yc, y, v["cells"]):
            ax.text(xi, max(yi, 0) + 3, f"{yv:.0f} %", ha="center", fontsize=9, fontweight="bold")
            nb, nn = (n_trained(cl["cell"], b_), n_trained(cl["cell"], n_)) if k == 0 else (NEW[(cl["cell"], b_)], NEW[(cl["cell"], n_)])
            txt = f"{b_}: {nb}\n{n_}: {nn}\nflights"
            if abs(yi) >= 35: ax.text(xi, yi / 2, txt, ha="center", va="center", fontsize=7.5, color="white")
            elif yi >= 0: ax.text(xi, yi + 13, txt, ha="center", va="bottom", fontsize=7.5, color="0.25")
            else: ax.text(xi, yi - 3, txt, ha="center", va="top", fontsize=7.5, color="0.25")
    ax.set_xticks(x); ax.set_xticklabels([names[cl["cell"]] for cl in v["cells"]], fontsize=10); ax.set_ylim(-110, 110); ax.axhline(0, c="k", lw=0.8)
    ax.set_ylabel("NS2 gain: error reduction [%]"); ax.set_title(ttl, fontsize=10)
axs[0].legend(fontsize=9, loc="lower left")
axs[1].text(0.02, 0.98, "A1 bar clipped at −100 %. Negative = S3 worse than S1: in A1 S3 sits 1–5 cm too high,\nS1 0–2 cm low; % is large because S1 is close to zero", transform=axs[1].transAxes, fontsize=8, va="top")
fig.suptitle(f"Does NS2 help as much on settings it never saw? Real flights, bottom drone; retention (geometric, all cells) = {v['R_geometric_pooled']:.2f}, pass ≥ 0.7\n"
             "in / on each bar: flights per strategy behind that gain; A4 trained 3+3 = no flight at exactly 0.075 / 0.125 m, interpolated between the two neighbouring offsets", fontsize=10)
fig.tight_layout(rect=[0, 0, 1, 0.90]); fig.savefig(RES / "ns2_gain_meeting.png", dpi=130)
