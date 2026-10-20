#!/usr/bin/env python3
"""Time-profile figures of the unified comparison with ALL pilot flights (2026-10-19 and 2026-10-20), uSD only:
 A8_crossing_profile.png (z error around the crossing, mean ± std over the crossings per strategy) and A1_comparison.png (z error over the hold, mean and min..max band per strategy, plus mean z error per strategy).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unified_plots.py"""
import glob, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; RES = REPO / "experiments/results"; LOGS = REPO / "experiments/logs"; FIG = REPO / "docs/meetings/assets/2026-10-19"
ORDER = ["S0", "S1", "S2", "S3"]; LAB = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geometric + NS2", "S3": "S3 INDI + NS2"}; COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}
DRAIN = {"17-55-03", "17-56-35"}
def bottom_ez(path):
    d = pd.read_csv(glob.glob(path + "/*_merged_usd.csv")[0], comment="#"); pb = sorted({c.split(".")[0] for c in d.columns if c.startswith("cf5_")})[0]
    return d.t.values, (d[pb + ".z"] - d[pb + ".ctrltarget_z"]).values, d[pb + ".z"].values
# ---- A8
a8 = [(r, str(LOGS / "bank_v2val_2026-10-19" / r["file"])) for r in json.load(open(RES / "v2val_2026-10-19/v2val_summary.json"))]
for b in ("pilot2", "pilot3", "pilot5"): a8 += [(r, str(LOGS / f"bank_{b}_2026-10-20" / r["file"])) for r in json.load(open(RES / f"{b}_2026-10-20/a8_summary.json"))]
a8 = [(r, p) for r, p in a8 if r["file"].split("_")[-1] not in DRAIN]
tau = np.linspace(-2, 2, 201); fig, ax = plt.subplots(figsize=(7.8, 4.4))
for s in ORDER:
    prof = []
    for r, p in a8:
        if r["name"] != s: continue
        t, ez, _ = bottom_ez(p)
        for c in r["crossings"]: prof.append(100 * np.interp(c + tau, t, ez) - r["z_offset_cm"])
    prof = np.array(prof); m, sd = prof.mean(0), prof.std(0); n = sum(1 for r, _ in a8 if r["name"] == s)
    ax.plot(tau, m, color=COL[s], lw=2, label=f"{LAB[s]} (n={n} flights, {len(prof)} crossings)"); ax.fill_between(tau, m - sd, m + sd, color=COL[s], alpha=.12)
ax.axvline(0, color="k", lw=.5); ax.axhline(0, color="k", lw=.5); ax.set_xlabel("time from the crossing [s]"); ax.set_ylabel("bottom z error, steady offset removed [cm]"); ax.legend(fontsize=8, loc="lower left")
ax.set_title("A8 swap, dz 0.5 m, 0.5 m/s: height error around the crossing (mean ± std over the crossings)", fontsize=9); fig.tight_layout(); fig.savefig(FIG / "A8_crossing_profile.png", dpi=130)
# ---- A1
a1 = []
for bank, res in (("bank_a1clean_2026-10-19", "a1_2026-10-19"), ("bank_pilot2_2026-10-20", "pilot2_2026-10-20"), ("bank_pilot3_2026-10-20", "pilot3_2026-10-20"), ("bank_pilot5_2026-10-20", "pilot5_2026-10-20"), ("bank_video_2026-10-20", "video_2026-10-20")):
    a1 += [(r, str(LOGS / bank / r["file"])) for r in json.load(open(RES / res / "a1_summary.json"))]
tg = np.linspace(0, 15, 301); fig, ax = plt.subplots(1, 2, figsize=(12.5, 4.3), gridspec_kw={"width_ratios": [1.5, 1]})
for s in ORDER:
    prof = []
    for r, p in a1:
        if r["strategy"] != s: continue
        t, ez, _ = bottom_ez(p); prof.append(100 * np.interp(tg, t, ez))
    prof = np.array(prof); ax[0].plot(tg, prof.mean(0), color=COL[s], lw=2, label=f"{LAB[s]} (n={len(prof)})"); ax[0].fill_between(tg, prof.min(0), prof.max(0), color=COL[s], alpha=.12)
ax[0].axhline(0, color="k", lw=.5); ax[0].set_xlabel("time in the hold [s]"); ax[0].set_ylabel("bottom z error [cm] (mean, band = min..max over flights)"); ax[0].legend(fontsize=8, loc="upper right"); ax[0].set_title("A1 stacked hold: z error over time", fontsize=10)
for i, s in enumerate(ORDER):
    v = [r["z_mean_cm"] for r, _ in a1 if r["strategy"] == s]; ax[1].bar(i, np.mean(v), color=COL[s], alpha=.4, width=.6)
    ax[1].scatter([i + (j - (len(v) - 1) / 2) * .07 for j in range(len(v))], v, color=COL[s], s=18, zorder=3)
ax[1].set_xticks(range(4)); ax[1].set_xticklabels([f"{LAB[s].replace(' + ', chr(10) + '+ ').replace('geometric', 'geom.')}\n{np.mean([r['z_mean_cm'] for r, _ in a1 if r['strategy'] == s]):+.1f} cm" for s in ORDER], fontsize=7.5)
ax[1].axhline(0, color="k", lw=.5); ax[1].set_ylabel("mean z error in the hold [cm] (dots = flights)"); ax[1].set_title("mean per strategy", fontsize=10)
fig.suptitle("A1 stacked hold, dz 0.5 m: all pilot flights (uSD only)", fontsize=9); fig.tight_layout(); fig.savefig(FIG / "A1_comparison.png", dpi=130); print("ok", {s: sum(1 for r, _ in a1 if r["strategy"] == s) for s in ORDER}, {s: sum(1 for r, _ in a8 if r["name"] == s) for s in ORDER})
