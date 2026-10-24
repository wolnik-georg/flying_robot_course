#!/usr/bin/env python3
"""Plots for docs/fbl/STOCK_INNER_LOOP_2026-10-24.md from stock_inner_sim.py (out/stage3/stock_inner_sim.json + stock_inner_traces/).
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage3_port/stock_inner_plots.py --out docs/fbl/assets/stock_inner_2026-10-24"""
import argparse, json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT3 = Path(__file__).parents[1] / "out/stage3"
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); a.out.mkdir(parents=True, exist_ok=True)
R = json.load(open(OUT3 / "stock_inner_sim.json"))
C_IN = {0: "#eb6834", 1: "#2a78d6"}; L_IN = {0: "inner 0: Eq. 19 P-only (flown 10-24)", 1: "inner 1: stock rate PID (paper hardware)"}
C_PR = {1: "#1baf7a", 0: "#e87ba4"}; L_PR = {1: "preset 1 (ours)", 0: "preset 0 (bundle 'Deployed')"}
plt.rcParams.update({"axes.spines.top": False, "axes.spines.right": False, "axes.grid": True, "grid.alpha": 0.25, "font.size": 8})

# 1. traces: constant torque, preset 1, 44 ms, inner 0 vs 1, per scenario: x, y error and yaw
fig, axs = plt.subplots(3, 3, figsize=(12, 7), sharex="col")
for j, sc in enumerate(("A1", "A4", "A8")):
    for inner in (0, 1):
        r = np.load(OUT3 / "stock_inner_traces" / f"{sc}_i{inner}_p1_const_lag44_fbl.npy"); t = r[:, 0] - 4.5
        for i, (col, sc_, yl) in enumerate(((1, 100, "x error [cm]"), (2, 100, "y error [cm]"), (4, 1, "yaw [deg] (cmd 0)"))):
            axs[i, j].plot(t, sc_ * r[:, col], c=C_IN[inner], lw=1.5, label=L_IN[inner]); axs[i, 0].set_ylabel(yl)
    for i in range(3): axs[i, j].axvline(0, c="0.6", lw=0.8); axs[i, j].axhline(0, c="0.3", lw=0.6)
    axs[0, j].set_title(f"{sc}: constant body torque +80 / -300 / -30 uN m", fontsize=8); axs[2, j].set_xlabel("time from scenario start [s] (takeoff + FBL switch-on before 0)")
axs[0, 0].legend(fontsize=7, frameon=False)
fig.suptitle("Simulation (real firmware via host bindings, 44 ms motor lag, preset 1): the stock rate PID removes the lateral offset and holds yaw", fontsize=9)
fig.tight_layout(); fig.savefig(a.out / "traces_const_torque.png", dpi=130); plt.close(fig)

# 2. summary: steady lateral offset and max |yaw| per scenario and disturbance, preset 1, 44 ms, J fbl, no state hold
sel = lambda **k: [r for r in R if all(r.get(x) == v for x, v in k.items()) and not r.get("hold")]
fig, axs = plt.subplots(1, 2, figsize=(11, 3.6)); cats = [(s, d) for s in ("A1", "A4", "A8") for d in ("none", "const", "var")]; xs = np.arange(len(cats))
for ax, key, yl, lim in ((axs[0], "xy_steady_cm", "steady lateral offset [cm] (mean, 2nd half)", 2.0), (axs[1], "yaw_absmax_deg", "max |yaw| [deg]", 2.0)):
    for k, inner in enumerate((0, 1)):
        v = [sel(scen=s, dist=d, inner=inner, preset=1, lag=44, J="fbl")[0][key] for s, d in cats]
        ax.bar(xs + (k - 0.5) * 0.38, v, 0.36, color=C_IN[inner], label=L_IN[inner])
    ax.axhline(lim, c="0.3", lw=0.8, ls="--"); ax.text(len(cats) - 0.5, lim, " Gate 2 limit", va="bottom", ha="right", fontsize=7, color="0.3")
    ax.set_xticks(xs); ax.set_xticklabels([f"{s}\n{d}" for s, d in cats]); ax.set_ylabel(yl)
fig.legend(*axs[0].get_legend_handles_labels(), loc="upper center", ncol=2, fontsize=7, frameon=False, bbox_to_anchor=(0.5, 0.93))
fig.suptitle("Simulation, preset 1, 44 ms: disturbance none / constant / slowly varying torque (sign flips)", fontsize=9)
fig.tight_layout(rect=(0, 0, 1, 0.88)); fig.savefig(a.out / "summary_offset_yaw.png", dpi=130); plt.close(fig)

# 3. lag margin: max tilt vs motor lag, inner 1, constant torque, presets 1 and 0
fig, axs = plt.subplots(1, 3, figsize=(12, 3.4), sharey=True)
for ax, sc in zip(axs, ("A1", "A4", "A8")):
    for pr in (1, 0):
        pts = sorted({(r["lag"], r["tilt_max_deg"] if np.isfinite(r["tilt_max_deg"]) else 120.0, r["stable"]) for r in sel(scen=sc, inner=1, preset=pr, dist="const", J="fbl")})
        if not pts: continue
        ax.plot([p[0] for p in pts], [min(p[1], 120) for p in pts], c=C_PR[pr], lw=1.5, marker="o", ms=5, label=L_PR[pr])
        for p in pts:
            if not p[2]: ax.plot(p[0], min(p[1], 120), "x", c="0.1", ms=7, mew=1.5)
    ax.axvline(44, c="0.6", lw=0.8, ls="--"); ax.text(44, 100, " measured\n 44 ms", fontsize=7, color="0.4"); ax.set_yscale("log"); ax.set_ylim(0.1, 150)
    ax.set_title(f"{sc}, inner 1, constant torque", fontsize=8); ax.set_xlabel("motor lag [ms]")
axs[0].set_ylabel("max tilt during the scenario [deg] (x = unstable / tripped)"); axs[0].legend(fontsize=7, frameon=False)
fig.suptitle("Lag margin with the stock inner loop: preset 1 stable 15-60 ms; bundle gains (preset 0) only up to 27 ms", fontsize=9)
fig.tight_layout(); fig.savefig(a.out / "lag_margin.png", dpi=130); plt.close(fig)
print("written", sorted(p.name for p in a.out.glob("*.png")))
