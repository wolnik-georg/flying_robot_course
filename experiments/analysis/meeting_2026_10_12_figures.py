#!/usr/bin/env python3
"""Figures/numbers for docs/meetings/2026-10-12.md (items 1, 3+7). uSD logs only, A8 cf5 bottom drone.
Reuses the validated pipeline of a8_variant_comparison.py (zero-crossing detector, steady window)."""
from __future__ import annotations

import json
import os
from pathlib import Path

os.environ["PYTHONWARNINGS"] = "ignore"
import matplotlib.pyplot as plt
import numpy as np

import a8_variant_comparison as a8

OUT = Path(__file__).resolve().parent / "out" / "meeting_2026-10-12"
OUT.mkdir(parents=True, exist_ok=True)


def band(ax):
    ax.axhspan(-a8.Z_BAND_CM, a8.Z_BAND_CM, color="#4FB39A", alpha=0.15, zorder=0)
    ax.axhline(0, color="k", lw=0.6, alpha=0.4)


def stats(fl):
    dips = np.array([d for r in fl for d in r.dips_cm])
    return dict(n_flights=len(fl), n_crossings=int(len(dips)),
                steady_mean_cm=float(np.mean([r.steady_mean_cm for r in fl])),
                steady_mean_sd_cm=float(np.std([r.steady_mean_cm for r in fl])),
                crossing_mean_cm=float(dips.mean()), crossing_sd_cm=float(dips.std()),
                crossing_max_abs_cm=float(np.abs(dips).max()),
                per_flight=[dict(stamp=f"{r.date} {r.stamp}", steady_cm=r.steady_mean_cm,
                                 crossing_mean_cm=float(np.mean(r.dips_cm)),
                                 crossing_max_abs_cm=float(np.max(np.abs(r.dips_cm)))) for r in fl])


def main():
    a8._build_usd_index()
    rows, _, _ = a8.process_all()
    rows = [r for r in rows if r.source == "usd"]
    by = {}
    for r in rows:
        by.setdefault(r.variant, []).append(r)
    a8.apply_meeting_style()
    res = {}

    # ---- item 1: NS2 res_sign -1 (uSD only) vs geometric (= NS2 off) ----
    sets1 = [("Geometric (= NS2 off)", by["Geometric baseline"], "#4D4D4D"),
             ("NS2 100 Hz, res_sign −1", by["NS2 res_sign −1"], "#009E73")]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    for lab, fl, col in sets1:
        for i, r in enumerate(fl):
            tt, z, ez = a8.plot_trace(r)
            axes[0].plot(tt, z, color=col, lw=0.9, alpha=0.8, label=f"{lab} (n={len(fl)})" if i == 0 else None)
            axes[1].plot(tt, ez, color=col, lw=0.9, alpha=0.8)
        res[lab] = stats(fl)
    axes[0].axhline(a8.Z_CMD_M, ls="--", color="k", lw=0.8, label="commanded z")
    axes[0].set_ylabel("z (m)"); axes[0].legend(fontsize=8)
    axes[0].set_title("A8, bottom drone — NS2 at 100 Hz, correct sign (uSD logs, one line per flight)")
    band(axes[1]); axes[1].set_ylim(-10, 6); axes[1].set_ylabel("z error (cm)"); axes[1].set_xlabel("time since scenario start (s)")
    fig.tight_layout(); fig.savefig(OUT / "fig1_ns2_100hz_correct_sign.png"); plt.close(fig)

    # ---- items 3+7: best result of each controller ----
    best = [("Ours INDI (filter on)", by["Ours INDI (10-09 filter ON)"], "#D55E00"),
            ("Omar C + Iz 1.5", by["Omar C + Iz 1.5"] + by["Omar C + Iz 1.5 (10-09)"], "#005AB5"),
            ("Omar Rust + Iz 1.5", by["Omar Rust + Iz 1.5"], "#A50F15")]
    fig = plt.figure(figsize=(15, 6.5))
    gs = fig.add_gridspec(2, 3, width_ratios=[2.4, 1, 1])
    axz, axe = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0], sharex=None)
    axs, axd = fig.add_subplot(gs[:, 1]), fig.add_subplot(gs[:, 2])
    for i, (lab, fl, col) in enumerate(best):
        for j, r in enumerate(fl):
            tt, z, ez = a8.plot_trace(r)
            axz.plot(tt, z, color=col, lw=0.9, alpha=0.8, label=f"{lab} (n={len(fl)})" if j == 0 else None)
            axe.plot(tt, ez, color=col, lw=0.9, alpha=0.8)
        s = stats(fl); res[lab] = s
        axs.bar(i, s["steady_mean_cm"], yerr=s["steady_mean_sd_cm"], color=col, capsize=3)
        axs.text(i, 0.08, f"{s['steady_mean_cm']:+.1f}", ha="center", va="bottom", fontsize=10, color="white", weight="bold")
        axd.bar(i, s["crossing_mean_cm"], yerr=s["crossing_sd_cm"], color=col, capsize=3)
        axd.text(i, -0.15, f"{s['crossing_mean_cm']:+.1f}", ha="center", va="top", fontsize=10, color="white", weight="bold")
    axz.axhline(a8.Z_CMD_M, ls="--", color="k", lw=0.8, label="commanded z"); axz.set_ylabel("z (m)"); axz.legend(fontsize=8)
    axz.set_title("A8, bottom drone — best result per controller (uSD, one line per flight)")
    band(axe); axe.set_ylim(-18, 10); axe.set_ylabel("z error (cm)"); axe.set_xlabel("time since scenario start (s)")
    for ax, t in ((axs, "steady z error\nmean of e_z in steady flight (crossings ±1.5 s excluded)\nbar = mean over flights, whisker = sd over flights"),
                  (axd, "crossing dip\nlowest e_z within ±1 s of a crossing\nbar = mean, whisker = sd over all crossings")):
        ax.set_xticks(range(3)); ax.set_xticklabels(["ours", "Omar C", "Omar Rust"], fontsize=9); ax.axhline(0, color="k", lw=0.5); ax.set_title(t, fontsize=8)
        ax.set_ylabel("cm")
    fig.tight_layout(); fig.savefig(OUT / "fig3_best_per_controller.png"); plt.close(fig)

    (OUT / "numbers.json").write_text(json.dumps(res, indent=2))
    for k, v in res.items():
        print(f"{k}: flights={v['n_flights']} crossings={v['n_crossings']} steady={v['steady_mean_cm']:+.2f}±{v['steady_mean_sd_cm']:.2f} "
              f"crossing mean={v['crossing_mean_cm']:+.2f} (sd {v['crossing_sd_cm']:.2f}) max|.|={v['crossing_max_abs_cm']:.2f}")


if __name__ == "__main__":
    main()
