#!/usr/bin/env python3
"""Two simple overview plots for docs/53a (INDI loop rates and delay budget).

Run:  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/indi_loop_overview_plots.py
Reads experiments/analysis/out/indi_loop_rates/delay_budget_6p3hz.json (from indi_loop_rates_delay_budget.py).
"""
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent / "out" / "indi_loop_rates"
data = json.loads((OUT / "delay_budget_6p3hz.json").read_text())

# ---- Fig 1: attitude-law update rate per variant (code facts, see docs/53 s.2) -------------------
variants = ["Ours\n(ctrl 6, mode 3)", "Omar C\n(ctrl 9)", "Omar Rust\n(ctrl 10)", "NA-INDI\n(ctrl 7/8)", "Stock INDI\n(ctrl 3)"]
rate_hz = [1000, 500, 500, 500, 500]
colors = ["#D1495B", "#2E6DA4", "#2E6DA4", "#9A6B12", "#6E7773"]
fig, ax = plt.subplots(figsize=(8, 3.6))
ax.barh(variants[::-1], rate_hz[::-1], color=colors[::-1])
for y, r in enumerate(rate_hz[::-1]):
    ax.text(r + 15, y, f"{r} Hz", va="center", fontsize=10)
ax.axvline(1000, color="black", lw=0.8, ls="--")
ax.text(1000, 4.45, "stabilizer / motors: 1000 Hz (all variants)", ha="right", fontsize=9)
ax.set_xlim(0, 1250)
ax.set_xlabel("update rate of the attitude INDI law (Hz)")
ax.set_title("How often each variant runs its attitude law")
fig.tight_layout()
fig.savefig(OUT / "fig_overview_attitude_rate.png", dpi=150)
plt.close(fig)

# ---- Fig 2: phase at 6.3 Hz per variant, stacked by term ----------------------------------------
names = list(data["variants"].keys())
short = {n: n.split(" (")[0].replace("Stock Bitcraze INDI", "Stock INDI") for n in names}
def group(name):
    if name.startswith("INDI Butterworth") or name.startswith("Filter rate mismatch") or name.startswith("Control output sample"):
        return "INDI filters + output hold (differs per variant)"
    return name[:46]

term_names = []
for v in data["variants"].values():
    for t in v["terms"]:
        g = group(t["name"])
        if g not in term_names:
            term_names.append(g)
cmap = plt.get_cmap("tab10")
fig, ax = plt.subplots(figsize=(9, 4.4))
x = np.arange(len(names))
bottom = np.zeros(len(names))
for i, tn in enumerate(term_names):
    vals = []
    for n in names:
        vals.append(sum(t["phase_deg"] for t in data["variants"][n]["terms"] if group(t["name"]) == tn))
    ax.bar(x, vals, bottom=bottom, color=cmap(i), label=tn)
    bottom += np.array(vals)
for xi, tot in zip(x, bottom):
    ax.text(xi, tot + 2, f"{tot:.0f}°", ha="center", fontsize=10)
ax.set_xticks(x)
ax.set_xticklabels([short[n] for n in names], fontsize=9)
ax.set_ylabel("phase lag at 6.3 Hz (degrees)")
ax.set_ylim(0, max(bottom) * 1.12)
ax.set_title("Where the delay comes from (sum of terms, not a stability margin)")
ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.0, 1.0))
fig.tight_layout()
fig.savefig(OUT / "fig_overview_delay_budget.png", dpi=150, bbox_inches="tight")
plt.close(fig)
print("wrote", OUT / "fig_overview_attitude_rate.png", OUT / "fig_overview_delay_budget.png")
