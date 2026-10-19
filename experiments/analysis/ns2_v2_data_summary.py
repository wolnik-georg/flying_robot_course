#!/usr/bin/env python3
"""Compact overview of the data behind ns2_v2 (73 flights): per scenario flights, flight time, time inside the NS2 gate, coverage of vertical gap and lateral offset; table + figure for the meeting doc.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/ns2_v2_data_summary.py"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]; sys.path.insert(0, str(REPO / "experiments/fbl"))
from common import data as D  # noqa: E402
man = json.load(open(REPO / "experiments/results/ns2_pilot2_2026-10-17/training_manifest_pilot2.json"))["flights"]
STEP = 10; DT = STEP / 500.0; rows = []; allg = []
for e in man:
    d = D.load_flight(e, STEP); gate = (d.dx.abs() < 0.2) & (d.dy.abs() < 0.2) & (d.dz > 0) & ((d.n_vx - d.e_vx).abs() < 1.5)
    new = e["source"] != "v1_relabelled"
    rows.append(dict(scenario=e["scenario"], new=new, flight_s=len(d) * DT, gate_s=gate.sum() * DT, folder=e["folder"]))
    g = d[gate]; allg.append(pd.DataFrame(dict(scenario=e["scenario"], new=new, z0=g.z0.values, r0=g.r0.values)))
R = pd.DataFrame(rows); G = pd.concat(allg)
WHAT = {"A8": ("two drones swap sides (crossing)", "dz 0.25 / 0.5 / 0.75 m × peak speed 0.1–0.5 m/s"), "A2": ("stacked circle, radius 0.40 m", "dz 0.5 m × speed 0.1–0.5 m/s"),
        "A3": ("static top drone, bottom shuttles under it", "dz 0.5 m × speed 0.1–0.5 m/s (+ earlier flights)"), "A1": ("stacked hold", "dz 0.25 / 0.375 / 0.5 / 0.625 / 0.75 m"),
        "R1": ("random waypoints, stack-biased", "9 seeds, bias 0.7, dz 0.25–0.75 m"), "A4": ("bottom offset sideways, shuttles along a line", "offset 0–0.15 m, dz 0.25–0.75 m, speed 0.3–0.5 m/s"),
        "A9": ("nine A4 settings chained in one flight", "dz 0.25 / 0.5 / 0.75 × offset 0 / 0.1 / 0.15 m"), "A5": ("reverse circle (earlier flights)", "earlier data only"),
        "A6": ("(earlier flight)", "earlier data only"), "A7": ("descend through the lower drone (earlier flights)", "earlier data only")}
L = ["| Scenario | What it is | Settings covered | Flights (new + earlier) | Flight time | Time in the NS2 gate |", "|---|---|---|---|---|---|"]
for sc in ["A8", "A2", "A3", "A1", "R1", "A4", "A9", "A5", "A6", "A7"]:
    r = R[R.scenario == sc]; 
    if r.empty: continue
    n_new, n_old = int(r.new.sum()), int((~r.new).sum())
    L.append(f"| {sc} | {WHAT[sc][0]} | {WHAT[sc][1]} | {n_new + n_old} ({n_new} + {n_old}) | {r.flight_s.sum() / 60:.1f} min | {r.gate_s.sum():.0f} s |")
L.append(f"| **all** | | | **{len(R)}** ({int(R.new.sum())} + {int((~R.new).sum())}) | **{R.flight_s.sum() / 60:.1f} min** | **{R.gate_s.sum():.0f} s** |")
print("\n".join(L)); print("gap in the gate: median %.2f m, 5-95 %% %.2f–%.2f m; lateral offset in the gate: median %.3f m" % (G.z0.median(), G.z0.quantile(.05), G.z0.quantile(.95), G.r0.median()))
Path(REPO / "experiments/results/ns2_v2_data_summary.md").write_text("\n".join(L) + "\n")
fig, ax = plt.subplots(1, 2, figsize=(11, 4.2)); scs = [s for s in ["A8", "A2", "A3", "A1", "R1", "A4", "A9", "A5", "A6", "A7"] if s in set(R.scenario)]
new = [R[(R.scenario == s) & R.new].gate_s.sum() for s in scs]; old = [R[(R.scenario == s) & ~R.new].gate_s.sum() for s in scs]
ax[0].bar(scs, old, color="#9ecae1", label="earlier flights (26)"); ax[0].bar(scs, new, bottom=old, color="#08519c", label="new flights, 2026-10-16/17 (47)")
for i, (o, n) in enumerate(zip(old, new)): ax[0].text(i, o + n + 5, f"{o + n:.0f}", ha="center", fontsize=8)
ax[0].set_ylabel("time inside the NS2 gate [s]"); ax[0].set_title("training data per scenario (73 flights)", fontsize=10); ax[0].legend(fontsize=8)
h = ax[1].hist2d(G.r0, G.z0, bins=[np.linspace(0, 0.28, 29), np.linspace(0, 1.3, 27)], weights=np.full(len(G), DT), cmap="viridis", cmin=0.01)
fig.colorbar(h[3], ax=ax[1], label="seconds"); ax[1].set_xlabel("horizontal offset to the upper drone [m]"); ax[1].set_ylabel("vertical gap [m]"); ax[1].set_title("coverage inside the gate", fontsize=10)
fig.tight_layout(); fig.savefig(REPO / "docs/meetings/assets/2026-10-19/ns2_v2_data.png", dpi=130)
