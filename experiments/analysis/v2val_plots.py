#!/usr/bin/env python3
"""Figures of the 2026-10-19 A8 validation for the meeting doc (uSD logs only). Writes docs/meetings/assets/2026-10-19/A8_comparison.png and A8_crossing_profile.png
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_plots.py"""
import glob, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(REPO / "experiments/analysis"))
BANK = REPO / "experiments/logs/bank_v2val_2026-10-19"; OUT = REPO / "docs/meetings/assets/2026-10-19"; OUT.mkdir(parents=True, exist_ok=True)
S = json.load(open(BANK / "v2val_summary.json"))
COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}; LAB = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geometric + NS2", "S3": "S3 INDI + NS2"}
DRAIN = {"17-55-03", "17-56-35"}
fig, ax = plt.subplots(figsize=(7.5, 4.2))
for i, n in enumerate(COL):
    rr = [r for r in S if r["name"] == n]
    good = [r for r in rr if r["file"].split("_")[-1] not in DRAIN]
    ax.bar(i, np.mean([x for r in good for x in r["dip_cm"]]), color=COL[n], alpha=.35, width=.6)
    for j, r in enumerate(rr):
        dr = r["file"].split("_")[-1] in DRAIN
        ax.scatter([i + (j - (len(rr) - 1) / 2) * .08] * 4, r["dip_cm"], s=14, color=COL[n], facecolors="none" if dr else COL[n], zorder=3)
ax.set_xticks(range(4)); ax.set_xticklabels([LAB[n] for n in COL], fontsize=8); ax.set_ylabel("z error at the crossing [cm]"); ax.axhline(0, color="k", lw=.5)
ax.set_title("A8 crossing dip, bottom vehicle (bar = mean, dots = the 4 crossings of each flight;\nhollow = S3 flights on a draining battery, not in the bar)", fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "A8_comparison.png", dpi=130)
fig, ax = plt.subplots(figsize=(7.5, 4.2)); tau = np.linspace(-2, 2, 201)
for n in COL:
    prof = []
    for r in S:
        if r["name"] != n or r["file"].split("_")[-1] in DRAIN: continue
        d = pd.read_csv(glob.glob(f"{BANK}/{r['file']}/*_merged_usd.csv")[0], comment="#")
        pre = sorted({c.split(".")[0] for c in d.columns if c.startswith("cf5_")})[0]; t = d.t.values
        ez = (d[pre + ".z"] - d[pre + ".ctrltarget_z"]).values
        for c in r["crossings"]: prof.append(100 * (np.interp(c + tau, t, ez) - ez[(t > 3) & (d[pre + ".z"].values > .3)].mean() * 0 - r["z_offset_cm"] / 100))
    prof = np.array(prof); m = prof.mean(0); s = prof.std(0)
    ax.plot(tau, m, color=COL[n], label=f"{LAB[n]} (n={len(prof)} crossings)"); ax.fill_between(tau, m - s, m + s, color=COL[n], alpha=.12)
ax.axvline(0, color="k", lw=.5); ax.axhline(0, color="k", lw=.5); ax.set_xlabel("time from the crossing [s]"); ax.set_ylabel("z error, offset removed [cm]"); ax.legend(fontsize=8)
ax.set_title("A8 height error around the crossing (mean ± std over crossings; draining-battery flights excluded)", fontsize=9)
fig.tight_layout(); fig.savefig(OUT / "A8_crossing_profile.png", dpi=130); print("ok")
