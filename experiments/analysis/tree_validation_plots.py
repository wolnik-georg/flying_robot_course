#!/usr/bin/env python3
"""Plots for the 2026-10-15 tree validation from the plot-ready extract (experiments/results/tree_validation_2026-10-15).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/tree_validation_plots.py
Bottom vehicle (cf5) carries the strategy; top vehicle always S0. z error = measured - commanded (cm)."""
import glob, json, re
from pathlib import Path
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
D = Path("experiments/results/tree_validation_2026-10-15"); O = D / "plots"; O.mkdir(exist_ok=True)
S = ["S0", "S1", "S2", "S3"]
NAME = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geometric + NS2", "S3": "S3 INDI + NS2"}
COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#e08a00", "S3": "#2ca02c"}
M = json.load(open(D / "metrics.json"))
def load(f):
    a = np.genfromtxt(f, delimiter=",", names=True); return a
F = {s: {k: load(D / f"{k}.csv") for k in M if f"_{s}_" in k} for s in S}
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .3, "axes.spines.top": False, "axes.spines.right": False})

def crossings(a):
    ry = a["bottom_y"] - a["top_y"]; s = np.sign(ry); air = a["bottom_z"] > 0.3
    i = np.where((s[:-1] * s[1:] < 0) & air[:-1])[0]; return a["t"][i]

# 1) A8 crossing-aligned z error, all strategies in one panel
fig, ax = plt.subplots(1, 2, figsize=(12, 4.3), gridspec_kw={"width_ratios": [2, 1]})
tt = np.linspace(-1.5, 1.5, 301)
for s in S:
    segs = []
    for k, a in F[s].items():
        if "A8" not in k: continue
        ez = (a["bottom_z"] - a["bottom_ctrltarget_z"]) * 100; off = M[k]["offset"]
        for tc in crossings(a):
            if tc - 1.5 > a["t"][0] and tc + 1.5 < a["t"][-1] and (a["bottom_z"][np.searchsorted(a["t"], tc)] > 0.3):
                segs.append(np.interp(tt + tc, a["t"], ez) - off)
    if not segs: continue
    segs = np.array(segs); m = segs.mean(0); sd = segs.std(0)
    ax[0].plot(tt, m, color=COL[s], lw=2, label=f"{NAME[s]} (n={len(segs)} crossings)")
    ax[0].fill_between(tt, m - sd, m + sd, color=COL[s], alpha=.15)
ax[0].axvline(0, color="k", lw=.8); ax[0].axhline(0, color="k", lw=.8)
ax[0].set_xlabel("time relative to crossing (s)"); ax[0].set_ylabel("bottom z error rel. to steady offset (cm)")
ax[0].set_title("A8: height error of the lower vehicle while the other passes over it"); ax[0].legend(loc="lower left", frameon=False)
for j, s in enumerate(S):
    d = [M[k]["dip_mean"] for k in F[s] if "A8" in k and "dip_mean" in M[k]]
    if not d: continue
    ax[1].bar(j, np.mean(d), color=COL[s], alpha=.8); ax[1].plot([j] * len(d), d, "ko", ms=4)
    ax[1].text(j, np.mean(d) - 1.3, f"{np.mean(d):.1f}", ha="center", va="top", fontsize=9)
ax[1].set_xticks(range(4)); ax[1].set_xticklabels(S); ax[1].set_ylabel("crossing dip (cm)"); ax[1].set_title("dip per strategy (dots = flights)")
ax[1].set_ylim(-20, 1); fig.tight_layout(); fig.savefig(O / "A8_comparison.png", dpi=150); plt.close(fig)

# 2) A8 time series per strategy (one flight each, z error and separation)
fig, axs = plt.subplots(4, 1, figsize=(11, 8.5), sharex=True, sharey=True)
for ax, s in zip(axs, S):
    ks = [k for k in F[s] if "A8" in k]
    if not ks: ax.set_title(NAME[s] + ": no valid flight"); continue
    k = ks[0]; a = F[s][k]; air = a["bottom_z"] > 0.3
    ez = (a["bottom_z"] - a["bottom_ctrltarget_z"]) * 100; ez[~air] = np.nan
    ax.plot(a["t"], ez, color=COL[s], lw=1.2); ax.axhline(0, color="k", lw=.6)
    for tc in crossings(a):
        if np.interp(tc, a["t"], a["bottom_z"]) > .3: ax.axvline(tc, color="r", lw=.7, ls="--", alpha=.6)
    ax.set_title(f"{NAME[s]}  ({k}; red dashed = crossings)  dip {M[k].get('dip_mean', np.nan):+.1f} cm", loc="left", fontsize=10)
    ax.set_ylabel("z error (cm)")
axs[-1].set_xlabel("time (s)"); axs[0].set_xlim(0, None); fig.tight_layout(); fig.savefig(O / "A8_timeseries.png", dpi=150); plt.close(fig)

# 3) A1 stack hold: z error over time + bars
fig, ax = plt.subplots(1, 2, figsize=(12, 4.3), gridspec_kw={"width_ratios": [2, 1]})
for s in S:
    for k, a in F[s].items():
        if "A1" not in k: continue
        w = (a["t"] >= 1) & (a["t"] <= 15); ez = (a["bottom_z"] - a["bottom_ctrltarget_z"]) * 100
        ax[0].plot(a["t"][w], ez[w], color=COL[s], lw=1.4, label=NAME[s])
ax[0].axhline(0, color="k", lw=.8); ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("bottom z error (cm)")
ax[0].set_title("A1: stack hold, dz 0.5 m (commanded hold window)"); ax[0].legend(frameon=False, loc="upper right", ncol=2, fontsize=8)
for j, s in enumerate(S):
    for k in F[s]:
        if "A1" in k:
            r = M[k]; ax[1].bar(j, r["z_err_mean"], yerr=r["z_err_std"], color=COL[s], alpha=.8, capsize=4)
            ax[1].text(j, r["z_err_mean"] + (r["z_err_std"] + 1 if r["z_err_mean"] >= 0 else -r["z_err_std"] - 1), f"{r['z_err_mean']:+.1f}", ha="center", va="bottom" if r["z_err_mean"] >= 0 else "top", fontsize=9)
ax[1].set_xticks(range(4)); ax[1].set_xticklabels(S); ax[1].axhline(0, color="k", lw=.8); ax[1].set_ylabel("mean z error ± std (cm)"); ax[1].set_title("A1 z error per strategy (n=1 each)")
ax[1].set_ylim(-28, 12); ax[0].set_ylim(-30, 16); fig.tight_layout(); fig.savefig(O / "A1_comparison.png", dpi=150); plt.close(fig)

# 4) overview: steady offset far apart, A8 dip, A1 z error side by side
fig, ax = plt.subplots(1, 3, figsize=(13, 3.8))
off = {s: [M[k]["offset"] for k in F[s] if "A8" in k and "offset" in M[k]] for s in S}
dip = {s: [M[k]["dip_mean"] for k in F[s] if "A8" in k and "dip_mean" in M[k]] for s in S}
a1 = {s: [M[k]["z_err_mean"] for k in F[s] if "A1" in k] for s in S}
for a, d, t in zip(ax, (off, dip, a1), ("A8 steady offset away from the crossing (cm)", "A8 crossing dip (cm)", "A1 stack-hold z error (cm)")):
    for j, s in enumerate(S):
        if d[s]: a.bar(j, np.mean(d[s]), color=COL[s], alpha=.85); a.plot([j] * len(d[s]), d[s], "ko", ms=3)
    a.axhline(0, color="k", lw=.8); a.set_xticks(range(4)); a.set_xticklabels(S); a.set_title(t, fontsize=10)
fig.suptitle("Tree validation 2026-10-15: S0 geometric, S1 INDI, S2 geometric + NS2, S3 INDI + NS2 (dots = flights)", fontsize=10)
fig.tight_layout(); fig.savefig(O / "overview.png", dpi=150); plt.close(fig)
print("plots ->", O)
