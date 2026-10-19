#!/usr/bin/env python3
"""A1 overview (per flight + per strategy) and figure for the meeting doc, uSD logs only.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/v2val_a1_overview.py"""
import glob, json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; BANK = REPO / "experiments/logs/bank_a1clean_2026-10-19"; OUT = REPO / "experiments/results/a1_2026-10-19"; FIG = REPO / "docs/meetings/assets/2026-10-19"
S = json.load(open(BANK / "a1_summary.json")); ORDER = ["S0", "S1", "S2", "S3"]
LAB = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geometric + NS2", "S3": "S3 INDI + NS2"}; COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}
L = ["# A1 stacked hold, 2026-10-19 — all merged flights (bottom vehicle; dz 0.5 m, hold 15 s, window 2–15 s; uSD logs only)", "",
     "z error = measured z minus commanded z (bottom vehicle). Battery = loaded cell voltage logged while airborne. Flights lost to the full cards (5) are not in the bank.", "",
     "| strat | flight | z mean ± std [cm] | z RMS | drift over the hold | xy RMS | max roll/pitch | loaded vbat start / mean / min [V] | motors max | top z RMS | stack gap [m] |", "|---|---|---|---|---|---|---|---|---|---|---|"]
rows = sorted(S, key=lambda r: (r["strategy"], r["flight"]))
for r in rows:
    L.append(f"| {r['strategy']} | {r['flight']} | {r['z_mean_cm']:+.1f} ± {r['z_std_cm']:.1f} | {r['z_rms_cm']:.1f} | {r['z_drift_cm']:+.1f} | {r['xy_rms_cm']:.1f} | {r['roll_pitch_max_deg']:.0f}° | {r['vbat_start']:.2f} / {r['vbat_mean']:.2f} / {r['vbat_min']:.2f} | {r['motor_max_pct']:.0f} % | {r['top_z_rms_cm']:.2f} | {r['gap_mean_m']:.2f} |")
L += ["", "## Per strategy", "| strat | flights | z mean [cm] | spread of the flight means | z RMS [cm] | xy RMS [cm] | max roll/pitch | vbat mean / min [V] | motors max |", "|---|---|---|---|---|---|---|---|---|"]
for n in ORDER:
    rr = [r for r in S if r["strategy"] == n]; m = np.array([r["z_mean_cm"] for r in rr])
    L.append(f"| {LAB[n]} | {len(rr)} ({', '.join(str(r['flight']) for r in rr)}) | **{m.mean():+.1f}** | {m.min():+.1f} … {m.max():+.1f} | {np.mean([r['z_rms_cm'] for r in rr]):.1f} | {np.mean([r['xy_rms_cm'] for r in rr]):.1f} | {max(r['roll_pitch_max_deg'] for r in rr):.0f}° | {np.mean([r['vbat_mean'] for r in rr]):.2f} / {min(r['vbat_min'] for r in rr):.2f} | {min(r['motor_max_pct'] for r in rr):.0f}–{max(r['motor_max_pct'] for r in rr):.0f} % |")
(OUT / "a1_overview.md").write_text("\n".join(L) + "\n"); print("\n".join(L))
fig, ax = plt.subplots(1, 2, figsize=(12, 4.2)); tg = np.linspace(0, 15, 301)
for n in ORDER:
    prof = []
    for r in [r for r in S if r["strategy"] == n]:
        d = pd.read_csv(glob.glob(f"{BANK}/{r['file']}/*_merged_usd.csv")[0], comment="#"); pre = sorted({c.split(".")[0] for c in d.columns if c.startswith("cf5_")})[0]
        prof.append(100 * np.interp(tg, d.t.values, (d[pre + ".z"] - d[pre + ".ctrltarget_z"]).values))
    prof = np.array(prof); ax[0].plot(tg, prof.mean(0), color=COL[n], label=f"{LAB[n]} (n={len(prof)})"); ax[0].fill_between(tg, prof.min(0), prof.max(0), color=COL[n], alpha=.12)
ax[0].axhline(0, color="k", lw=.5); ax[0].set_xlabel("time in the hold [s]"); ax[0].set_ylabel("bottom z error [cm] (mean, band = min..max)"); ax[0].legend(fontsize=8); ax[0].set_title("A1 stack hold: z error over time", fontsize=9)
for i, n in enumerate(ORDER):
    rr = [r for r in S if r["strategy"] == n]; ax[1].bar(i, np.mean([r["z_mean_cm"] for r in rr]), color=COL[n], alpha=.35, width=.6)
    ax[1].scatter([i + (j - (len(rr) - 1) / 2) * .08 for j in range(len(rr))], [r["z_mean_cm"] for r in rr], color=COL[n], s=18, zorder=3)
ax[1].set_xticks(range(4)); ax[1].set_xticklabels([LAB[n] for n in ORDER], fontsize=8); ax[1].axhline(0, color="k", lw=.5); ax[1].set_ylabel("mean z error in the hold [cm] (dots = flights)"); ax[1].set_title("per strategy", fontsize=9)
fig.tight_layout(); fig.savefig(FIG / "A1_comparison.png", dpi=130)
