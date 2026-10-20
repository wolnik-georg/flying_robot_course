#!/usr/bin/env python3
"""Unified pilot overview S0-S3 (A8, A1, A4 per offset), all pilot flights of 2026-10-19 and 2026-10-20, uSD logs only. Writes experiments/results/unified_overview_2026-10-20/{overview.md,summary.json}
and docs/meetings/assets/2026-10-19/overview.png.   ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/unified_overview.py"""
import json
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; RES = REPO / "experiments/results"; OUT = RES / "unified_overview_2026-10-20"; OUT.mkdir(exist_ok=True)
ORDER = ["S0", "S1", "S2", "S3"]; LAB = {"S0": "S0 geometric", "S1": "S1 INDI", "S2": "S2 geometric + NS2", "S3": "S3 INDI + NS2"}; COL = {"S0": "#7f7f7f", "S1": "#1f77b4", "S2": "#2ca02c", "S3": "#d62728"}
DRAIN = {"17-55-03", "17-56-35"}                       # S3 A8 flights of 2026-10-19 on a draining battery
a8 = json.load(open(RES / "v2val_2026-10-19/v2val_summary.json"))
for b in ("pilot2", "pilot3", "pilot5"): a8 += json.load(open(RES / f"{b}_2026-10-20/a8_summary.json"))
a8 = [r for r in a8 if r["file"].split("_")[-1] not in DRAIN]
a1 = json.load(open(RES / "a1_2026-10-19/a1_summary.json"))
for b in ("pilot2", "pilot3", "pilot5", "video"): a1 += json.load(open(RES / f"{b}_2026-10-20/a1_summary.json"))   # "video": A1 with a 20 s hold (same dz, window 2-15 s)
A4 = pd.concat([pd.DataFrame(json.load(open(RES / "pilot_all_2026-10-20/cell_summary_all.json"))),
                pd.DataFrame(json.load(open(RES / "pilot5_2026-10-20/cell_summary.json"))).assign(strategy="S1"),
                pd.DataFrame(json.load(open(RES / "pilot6_2026-10-20/cell_summary.json"))).assign(strategy="S1")], ignore_index=True)
A4 = A4[(A4.scenario == "A4") & (A4.offset > 0.01)]
A4.to_json(RES / "pilot_all_2026-10-20/cell_summary_all_with_S1.json", orient="records", indent=1)
rows, S = [], {}
L = ["### A8 swap (dz 0.5, 0.5 m/s, 4 crossings per flight)", "| strategy | flights | mean dip [cm] | std of the flight means | worst crossing | z RMS [cm] | motors max |", "|---|---|---|---|---|---|---|"]
for s in ORDER:
    rr = [r for r in a8 if r["name"] == s]; m = np.array([r["dip_mean_cm"] for r in rr]); dips = np.array([r["dip_cm"] for r in rr])
    S.setdefault("A8", {})[s] = dict(n=len(rr), mean=float(m.mean()), std=float(m.std(ddof=1)) if len(rr) > 1 else 0, worst=float(dips.min()), zrms=float(np.mean([r["z_rms_cm"] for r in rr])), per_flight=[round(float(x), 1) for x in m])
    L.append(f"| {LAB[s]} | {len(rr)} | **{m.mean():.1f}** | {S['A8'][s]['std']:.1f} | {dips.min():.1f} | {S['A8'][s]['zrms']:.1f} | {min(r['motor_max_pct'] for r in rr):.0f}–{max(r['motor_max_pct'] for r in rr):.0f} % |")
L += ["", "### A1 stacked hold (dz 0.5, hold 15 s, window 2–15 s)", "| strategy | flights | z error mean [cm] | std of the flight means | z RMS [cm] | max roll/pitch | motors max |", "|---|---|---|---|---|---|---|"]
for s in ORDER:
    rr = [r for r in a1 if r["strategy"] == s]; m = np.array([r["z_mean_cm"] for r in rr])
    S.setdefault("A1", {})[s] = dict(n=len(rr), mean=float(m.mean()), std=float(m.std(ddof=1)), zrms=float(np.mean([r["z_rms_cm"] for r in rr])), per_flight=[round(float(x), 1) for x in m])
    L.append(f"| {LAB[s]} | {len(rr)} | **{m.mean():+.1f}** | {S['A1'][s]['std']:.1f} | {S['A1'][s]['zrms']:.1f} | {max(r['roll_pitch_max_deg'] for r in rr):.0f}° | {min(r['motor_max_pct'] for r in rr):.0f}–{max(r['motor_max_pct'] for r in rr):.0f} % |")
L += ["", "### A4 offset line (dz 0.5, 0.5 m/s): z RMS [cm], mean ± std over flights (n)", "| offset | " + " | ".join(LAB[s] for s in ORDER) + " |", "|---|" + "---|" * 4]
for o in (0.05, 0.10, 0.15):
    cells = []
    for s in ORDER:
        g = A4[(A4.strategy == s) & np.isclose(A4.offset, o)]
        S.setdefault("A4", {}).setdefault(str(o), {})[s] = dict(n=len(g), mean=float(g.z_rms_cm.mean()) if len(g) else None, std=float(g.z_rms_cm.std()) if len(g) > 1 else 0.0, worst_low=float(g.z_min_cm.mean()) if len(g) else None)
        cells.append(f"{g.z_rms_cm.mean():.1f} ± {g.z_rms_cm.std():.1f} (n={len(g)})" if len(g) > 1 else (f"{g.z_rms_cm.mean():.1f} (n=1)" if len(g) == 1 else "–"))
    L.append(f"| {o:.2f} m | " + " | ".join(cells) + " |")
(OUT / "overview.md").write_text("\n".join(L) + "\n"); json.dump(S, open(OUT / "summary.json", "w"), indent=1); print("\n".join(L))
fig, ax = plt.subplots(1, 3, figsize=(14, 4.2), gridspec_kw={"width_ratios": [1, 1, 1.4]})
for i, s in enumerate(ORDER):
    for k, key, sgn in ((0, "A8", 1), (1, "A1", 1)):
        d = S[key][s]; ax[k].bar(i, d["mean"], color=COL[s], alpha=.4, width=.6, yerr=d["std"], capsize=4, error_kw=dict(ecolor="k", lw=1.2))
        pts = d["per_flight"]; ax[k].scatter([i + (j - (len(pts) - 1) / 2) * .07 for j in range(len(pts))], pts, color=COL[s], s=18, zorder=3)
for k, (key, t, y) in enumerate((("A8", "A8 swap: height dip at the crossing", "dip per flight [cm]"), ("A1", "A1 stacked hold: mean height error", "mean z error in the hold [cm]"))):
    M = [S[key][s]["mean"] for s in ORDER]; N = [S[key][s]["n"] for s in ORDER]; SD = [S[key][s]["std"] for s in ORDER]
    ax[k].set_xticks(range(4)); ax[k].set_xticklabels([f"{LAB[s].replace(' + ', chr(10) + '+ ').replace('geometric', 'geom.')}\n{m:+.1f} ± {sd:.1f} cm\n(n={n} flights)" for s, m, sd, n in zip(ORDER, M, SD, N)], fontsize=7.5); ax[k].axhline(0, color="k", lw=.6); ax[k].set_title(t, fontsize=10); ax[k].set_ylabel(y + "\nbar = mean, black whisker = ± std between flights, dots = flights", fontsize=7.5)
w = .2
for j, s in enumerate(ORDER):
    xs, ys, es = [], [], []
    for oi, o in enumerate((0.05, 0.10, 0.15)):
        d = S["A4"][str(o)][s]
        if d["n"]: xs.append(oi + (j - 1.5) * w); ys.append(d["mean"]); es.append(d["std"])
    ax[2].bar(xs, ys, width=w * .9, color=COL[s], alpha=.7, yerr=es, capsize=2, label=LAB[s])
ax[2].set_xticks(range(3)); ax[2].set_xticklabels(["offset 0.05 m", "offset 0.10 m", "offset 0.15 m"]); ax[2].set_ylabel("z RMS [cm]"); ax[2].set_title("A4 offset line: z RMS (mean ± std over flights)", fontsize=10); ax[2].legend(fontsize=7.5)
fig.suptitle("Unified controller tree, NS2 v2, bottom vehicle (pilot flights 2026-10-19/20, uSD logs only, bars = means, dots = flights)", fontsize=9); fig.tight_layout(); fig.savefig(REPO / "docs/meetings/assets/2026-10-19/overview.png", dpi=130)
