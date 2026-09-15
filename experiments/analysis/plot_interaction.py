#!/usr/bin/env python3
"""Interaction-focused plots for a 2-drone uSD dataset -- the questions plot_flight.py's
per-vehicle dashboard does not answer.

plot_flight.py asks "did this drone track its own trajectory". These three ask the actual
thesis question: what does the residual look like as a function of the OTHER drone's
position, and have we sampled that relationship broadly enough to train on.

  1. Residual vs separation   -- |a_res| against dz and against full 3D separation |d|.
     The core physical relationship (a_res_z 2026-09-15: -3.36 m/s^2 at |d|=0.198m).
  2. Input-space coverage     -- scatter of the relative position actually sampled (dy, dz).
     This is how you know whether a dataset is enough to train on, not just whether a flight
     was clean -- and it is what would have visually caught A3 never exciting relative y.
  3. Crossing-aligned overlay -- every close-approach event in the flight, stacked on one
     relative-time axis, to see repeatability across passes/takes rather than one flight's
     own timeline.

Usage
-----
    python3 plot_interaction.py <merged.csv> --bottom cf231_active --top cf_second \\
        --out interaction.png
"""

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import metrics as M  # noqa: E402


def find_crossings(d: np.ndarray, min_sep_s: float, rate: float, rel_thresh: float = 0.5):
    """Indices of local minima of |d| that are actual close-approach events, not flight noise.

    Every LOCAL minimum of a noisy signal is a candidate, including ones near the flight's
    median separation that are just sensor/attitude jitter -- on the real A8 dataset this used
    to also "find" two spurious crossings near the end of the flight at |d| = 1.00-1.02m
    against a genuine pair at 0.20m, i.e. essentially at the median separation, not a close
    approach at all. `rel_thresh` requires a candidate to sit below `rel_thresh` * median(|d|)
    -- 0.5 means "at least twice as close as typical" -- before it counts. `min_sep_s` then
    collapses noisy multi-sample dips at the SAME real crossing into one.
    """
    min_gap = int(min_sep_s * rate)
    thresh = rel_thresh * np.median(d)
    cand = np.where((d[1:-1] < d[:-2]) & (d[1:-1] < d[2:]) & (d[1:-1] < thresh))[0] + 1
    out = []
    for i in cand:
        if not out or i - out[-1] >= min_gap:
            out.append(i)
        elif d[i] < d[out[-1]]:
            out[-1] = i
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("merged_csv")
    ap.add_argument("--bottom", required=True, help="name of the lower/residual-bearing drone")
    ap.add_argument("--top", required=True, help="name of the upper drone")
    ap.add_argument("--window", type=float, default=3.0,
                    help="seconds either side of a crossing to show in the overlay (default 3)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    vehicles = M.load_merged_csv(Path(args.merged_csv))
    if args.bottom not in vehicles or args.top not in vehicles:
        sys.exit(f"{args.merged_csv} has vehicles {list(vehicles)}, "
                 f"need --bottom {args.bottom!r} and --top {args.top!r}")
    bot, top = vehicles[args.bottom], vehicles[args.top]
    if bot.pos is None or top.pos is None:
        sys.exit("both vehicles need measured position (pos) -- check the merged CSV's columns")

    t = bot.t
    rate = len(t) / (t[-1] - t[0]) if len(t) > 1 else 500.0
    rel = top.pos - bot.pos                 # top relative to bottom, world frame
    dz = rel[:, 2]
    d = np.linalg.norm(rel, axis=1)
    a_res_bot = np.linalg.norm(bot.a_res, axis=1) if bot.a_res is not None else None
    a_res_z_bot = bot.a_res[:, 2] if bot.a_res is not None else None

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    fig.suptitle(f"Interaction — {args.bottom} (bottom) / {args.top} (top)")

    # 1. residual vs separation
    ax = axes[0]
    if a_res_bot is not None:
        sc = ax.scatter(d, a_res_bot, c=t, cmap="viridis", s=4, alpha=0.6)
        fig.colorbar(sc, ax=ax, label="t [s]")
    ax.set_xlabel("|d| separation [m]"); ax.set_ylabel("|a_res| bottom [m/s^2]")
    ax.set_title("Residual vs separation")

    # 2. input-space coverage: relative (dy, dz) actually sampled, coloured by |a_res|
    ax = axes[1]
    if a_res_bot is not None:
        sc = ax.scatter(rel[:, 1], dz, c=a_res_bot, cmap="magma", s=4)
        fig.colorbar(sc, ax=ax, label="|a_res| bottom [m/s^2]")
    else:
        ax.scatter(rel[:, 1], dz, s=4, color="tab:blue", alpha=0.5)
    ax.set_xlabel("relative y (top - bottom) [m]"); ax.set_ylabel("relative z [m]")
    ax.set_title("Input-space coverage (dy, dz)")

    # 3. crossing-aligned overlay
    ax = axes[2]
    crossings = find_crossings(d, min_sep_s=2.0, rate=rate)
    win = int(args.window * rate)
    for k, c in enumerate(crossings):
        lo, hi = max(0, c - win), min(len(t), c + win)
        tt = (t[lo:hi] - t[c])
        if a_res_z_bot is not None:
            ax.plot(tt, a_res_z_bot[lo:hi], lw=1.2, label=f"crossing {k+1} (t={t[c]:.1f}s)")
    ax.axvline(0, color="k", lw=0.7, ls=":")
    ax.set_xlabel("t - t_crossing [s]"); ax.set_ylabel("a_res_z bottom [m/s^2]")
    ax.set_title(f"Crossing-aligned overlay ({len(crossings)} crossing(s) found)")
    ax.legend(fontsize=8)

    fig.tight_layout()
    out = args.out or (Path(args.merged_csv).stem + "_interaction.png")
    fig.savefig(out, dpi=140)
    print(f"wrote {out}  ({len(crossings)} crossing(s) at t={[round(t[c],2) for c in crossings]})")


if __name__ == "__main__":
    main()
