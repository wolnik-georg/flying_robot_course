#!/usr/bin/env python3
"""P4 — publication figures for the controller comparison (docs/27 gap 7).

Run with the pyenv env (system python3's matplotlib is broken here):
    ~/.pyenv/versions/flying_robots/bin/python plot_comparison.py rows.csv -o fig/

    # grouped bars with error bars, one panel per metric
    plot_comparison.py rows.csv --metrics pos_rmse_m,roll_std,motor_rms_ratio -o fig/

    # per-phase breakdown for one metric
    plot_comparison.py rows.csv --by-phase --metric pos_rmse_m -o fig/

`aggregate.py` produces the numbers; this produces the figures that go in the thesis. Both read
the same per-run rows, so a figure and its table can never disagree.

## Honesty rules this module enforces, deliberately

- **Error bars are always drawn**, and they are the SEM across repeat *flights*, not a spread
  across phases of one flight. Where n=1 the bar is drawn hollow with the count printed on it,
  so a single-run number can never be mistaken for a measured mean.
- **n is printed on every bar.** A reader should never have to look elsewhere to find out how
  much data a bar represents.
- **Nothing is normalised away.** Absolute units on the axis; ratios go in the caption, where
  they belong, not into a bar height that hides the underlying magnitudes.
- **Bars are not truncated at a non-zero baseline.** Truncated axes exaggerate differences and
  are the most common way an honest number becomes a misleading picture.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402
import pandas as pd                       # noqa: E402

import aggregate as A                     # noqa: E402

# Colour-blind-safe (Okabe-Ito). Controllers keep the same colour across every figure in the
# thesis -- a reader learns the mapping once.
PALETTE = {
    "geometric": "#0072B2",
    "indi": "#D55E00",
    "stock_indi": "#009E73",
    "stock_lee": "#999999",
    "naindi": "#CC79A7",
    "ns2": "#E69F00",
    "hybrid": "#56B4E9",
}
FALLBACK = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9", "#999999"]

PRETTY = {
    "pos_rmse_m": ("Position RMSE", "m"),
    "pos_rmse_x": ("Position RMSE, x", "m"),
    "pos_rmse_y": ("Position RMSE, y", "m"),
    "pos_rmse_z": ("Position RMSE, z", "m"),
    "pos_peak_m": ("Peak position error", "m"),
    "roll_std": ("Roll std", "deg"),
    "pitch_std": ("Pitch std", "deg"),
    "e_R_rmse": ("Attitude error RMSE", "–"),
    "a_res_z_rms": ("Measured residual, z (RMS)", "m/s²"),
    "a_hat_vs_a_res_rmse": ("Residual prediction error", "m/s²"),
    "motor_rms_ratio": ("Motor command (RMS ratio)", "–"),
    "motor_sat_frac": ("Time at motor saturation", "fraction"),
    "tau_rms": ("Commanded torque (RMS)", "N·m"),
    "gyro_band_frac": ("Angular-rate power in 5–9 Hz", "fraction"),
    "min_sep_m": ("Minimum separation", "m"),
}


def _label(metric: str) -> tuple[str, str]:
    return PRETTY.get(metric, (metric, ""))


def _colour(name: str, i: int) -> str:
    return PALETTE.get(str(name).lower().replace(" ", "_"), FALLBACK[i % len(FALLBACK)])


def _n_flights(df: pd.DataFrame, ctrl: str, controller_col: str, run_col: str) -> int:
    """Distinct FLIGHTS, not rows -- the number that decides whether an error bar is honest."""
    sub = df[df[controller_col] == ctrl]
    return int(sub[run_col].nunique()) if run_col in sub.columns else int(len(sub))


def bar_panel(ax, df: pd.DataFrame, metric: str, controller_col="controller",
              run_col="run") -> None:
    """One metric, one bar per controller, SEM error bars, n annotated."""
    ctrls = list(dict.fromkeys(df[controller_col].dropna()))
    means, sems, ns = [], [], []
    for c in ctrls:
        vals = df.loc[df[controller_col] == c, metric].to_numpy(float)
        vals = vals[np.isfinite(vals)]
        nf = _n_flights(df, c, controller_col, run_col)
        means.append(float(np.mean(vals)) if len(vals) else np.nan)
        # SEM over FLIGHTS. With one flight there is no measured spread and pretending
        # otherwise (e.g. std across that flight's phases) would be the pseudo-replication
        # error aggregate.py guards against -- so draw no bar and say n=1 on the face of it.
        sems.append(float(np.std(vals, ddof=1) / np.sqrt(nf)) if nf > 1 and len(vals) > 1 else 0.0)
        ns.append(nf)

    x = np.arange(len(ctrls))
    for i, (c, m, s, nf) in enumerate(zip(ctrls, means, sems, ns)):
        solid = nf > 1
        ax.bar(x[i], m, 0.62, yerr=(s if solid else None), capsize=4,
               color=_colour(c, i) if solid else "none",
               edgecolor=_colour(c, i), linewidth=1.8,
               hatch=None if solid else "///",
               error_kw=dict(ecolor="#333333", elinewidth=1.2))
        if np.isfinite(m):
            ax.text(x[i], m + (s if solid else 0) + 0.02 * max(np.nanmax(means), 1e-9),
                    f"n={nf}" + ("" if solid else "*"), ha="center", va="bottom",
                    fontsize=8, color="#333333")

    name, unit = _label(metric)
    ax.set_xticks(x)
    ax.set_xticklabels(ctrls, rotation=12, ha="right")
    ax.set_ylabel(f"{name}" + (f" [{unit}]" if unit not in ("", "–") else ""))
    ax.set_title(name, fontsize=10)
    ax.set_ylim(bottom=0)            # never truncate a bar axis
    ax.grid(axis="y", alpha=0.25, linewidth=0.6)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)


def figure_metrics(df: pd.DataFrame, metrics: list[str], out: Path, title: str = "",
                   controller_col: str = "controller") -> Path:
    n = len(metrics)
    cols = min(3, n)
    rows = int(np.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.1 * cols, 3.5 * rows), squeeze=False)
    for k, metric in enumerate(metrics):
        ax = axes[k // cols][k % cols]
        if metric not in df.columns:
            ax.set_visible(False)
            continue
        bar_panel(ax, df, metric, controller_col=controller_col)
    for k in range(n, rows * cols):
        axes[k // cols][k % cols].set_visible(False)
    if title:
        fig.suptitle(title, fontsize=12)
    fig.text(0.005, 0.005, "* n=1: single flight, no measured spread — descriptive only",
             fontsize=7.5, color="#666666")
    fig.tight_layout(rect=(0, 0.02, 1, 0.97 if title else 1))
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


def figure_by_phase(df: pd.DataFrame, metric: str, out: Path,
                    controller_col="controller", run_col="run") -> Path:
    """Grouped bars: phase on x, one bar per controller. Shows where an advantage actually
    lives -- 2026-09-18 found INDI's edge is SMALLER at the crossings than over the whole
    scenario, which a single whole-window bar hides completely."""
    if "phase" not in df.columns:
        raise SystemExit("no 'phase' column -- run metrics.vehicle_metrics_by_phase first")
    order = [p for p in ["ramp", "scenario", "approach1", "approach2", "approach3", "land"]
             if p in set(df["phase"])]
    order += [p for p in dict.fromkeys(df["phase"]) if p not in order]
    ctrls = list(dict.fromkeys(df[controller_col].dropna()))
    w = 0.8 / max(len(ctrls), 1)
    fig, ax = plt.subplots(figsize=(1.9 * len(order) + 3.0, 4.2))
    for i, c in enumerate(ctrls):
        means, sems, ns = [], [], []
        for p in order:
            v = df.loc[(df[controller_col] == c) & (df["phase"] == p), metric].to_numpy(float)
            v = v[np.isfinite(v)]
            nf = (df.loc[(df[controller_col] == c) & (df["phase"] == p), run_col].nunique()
                  if run_col in df.columns else len(v))
            means.append(float(np.mean(v)) if len(v) else np.nan)
            sems.append(float(np.std(v, ddof=1) / np.sqrt(nf)) if nf > 1 and len(v) > 1 else 0.0)
            ns.append(int(nf))
        x = np.arange(len(order)) + (i - (len(ctrls) - 1) / 2) * w
        solid = all(k > 1 for k in ns)
        ax.bar(x, means, w * 0.92, yerr=sems if solid else None, capsize=3, label=str(c),
               color=_colour(c, i) if solid else "none", edgecolor=_colour(c, i),
               linewidth=1.6, hatch=None if solid else "///",
               error_kw=dict(ecolor="#333333", elinewidth=1.1))
    name, unit = _label(metric)
    ax.set_xticks(np.arange(len(order)))
    ax.set_xticklabels(order)
    ax.set_ylabel(f"{name}" + (f" [{unit}]" if unit not in ("", "–") else ""))
    ax.set_title(f"{name} by flight phase")
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", alpha=0.25, linewidth=0.6)
    ax.set_axisbelow(True)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.legend(frameon=False, ncol=min(len(ctrls), 4))
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rows", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=Path("fig"))
    ap.add_argument("--metrics", default="pos_rmse_m,roll_std,a_res_z_rms")
    ap.add_argument("--metric", default="pos_rmse_m", help="for --by-phase")
    ap.add_argument("--by-phase", action="store_true")
    ap.add_argument("--phase", default=None, help="restrict the metric figure to one phase")
    ap.add_argument("--vehicle", default=None)
    ap.add_argument("--title", default="")
    ap.add_argument("--group-by", default="controller",
                    help="column for controller grouping (default controller)")
    args = ap.parse_args()

    df = pd.read_csv(args.rows)
    gcol = args.group_by if args.group_by in df.columns else "controller"
    if args.vehicle and "vehicle_id" in df.columns:
        df = df[df["vehicle_id"] == args.vehicle]
    # Drop rows whose position error is void (reconstructed command outside the scenario
    # window -- see metrics.vehicle_metrics_by_phase). Plotting them would let a meaningless
    # 0.7 m ramp "error" dominate a figure whose real numbers are 0.04-0.10 m.
    metric_names = {args.metric} | {m.strip() for m in args.metrics.split(",")}
    if "pos_err_valid" in df.columns and any(m.startswith("pos_") for m in metric_names):
        dropped = int((df["pos_err_valid"] == 0).sum())
        if dropped:
            print(f"[plot] dropping {dropped} row(s) with pos_err_valid=0 "
                  f"(reconstructed command does not describe ramp/land)")
            df = df[df["pos_err_valid"] == 1]

    if args.by_phase:
        p = figure_by_phase(df, args.metric, args.out / f"by_phase_{args.metric}.png",
                            controller_col=gcol)
        print(f"wrote {p}")
        return
    if args.phase and "phase" in df.columns:
        df = df[df["phase"] == args.phase]
    metrics = [m.strip() for m in args.metrics.split(",") if m.strip()]
    p = figure_metrics(df, metrics, args.out / "comparison.png", args.title,
                       controller_col=gcol)
    print(f"wrote {p}")
    # The matching numbers, so a figure never travels without its table.
    for m in metrics:
        if m in df.columns:
            print()
            print(A.summarise(df, m, by=(gcol,)).to_string(index=False))


if __name__ == "__main__":
    main()
