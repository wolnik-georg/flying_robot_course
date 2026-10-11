#!/usr/bin/env python3
"""Study report generator (protocol section 8): per scenario, z error vs time per strategy (median + 10-90 % band over flights, one panel
per setting) and metric-vs-setting curves (mean +- sd over flights, one dot per flight), plus the cell tables.

  python3 experiments/analysis/study_report.py --rows out/study_metrics/study_rows.csv [--metric primary_metric] [--include-other]
Unit of analysis = one flight (cf5 row); clean flights only when the decisions file has an entry for the cell, else all flights (marked).
Time axis: A8 aligned on the first crossing (= 5.0 s), all other scenarios on the log start (the uSD log starts with the scenario).
Output: out/study_report/<scenario>_z_error_vs_time.png, <scenario>_metric_vs_setting.png, cells.md."""
import argparse, sys
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools")); sys.path.insert(0, str(HERE))
import decode_usd_log as dec  # noqa: E402
from a8_crossings import zero_crossings  # noqa: E402
import aggregate as ag  # noqa: E402

OUT = HERE / "out" / "study_report"
COL = {"S0": "#4D4D4D", "S1": "#005AB5", "S2": "#009E73", "ours": "#D55E00"}
GRID = np.arange(0, 26, 0.05)


def trace(path: str, scenario: str):
    r = dec.load(path)
    t = np.asarray(r["t"], float); t = t - t[0]
    z, sp = np.asarray(r["z"], float), np.asarray(r["ctrltarget_z"], float)
    ez = (z - sp) * 100
    up = z > 0.3
    if scenario == "A8":
        tc = zero_crossings(t[up], np.asarray(r["y"], float)[up])
        if tc:
            t = t - (tc[0] - 5.0)
    return np.interp(GRID, t[up], ez[up], left=np.nan, right=np.nan)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--rows", type=Path, default=HERE / "out" / "study_metrics" / "study_rows.csv")
    ap.add_argument("--metric", default="primary_metric")
    ap.add_argument("--include-other", action="store_true")
    ap.add_argument("--baseline-strategy", default="S0")
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(a.rows)
    df = df[df["vehicle_id"] == "cf5"].copy()
    df["strategy"] = df["strategy"].fillna("")
    df = df[df["strategy"] != ""]
    if not a.include_other:
        df = df[df["strategy"].isin(COL)]
    md = [f"# Study report (generated from `{a.rows.name}`, metric `{a.metric}`)", ""]
    for scen, g in df.groupby("scenario"):
        settings = sorted(g["setting_key"].dropna().unique())
        decided = bool((g["decision"].fillna("") != "").any())
        gg = g[g["decision"].fillna("").eq("clean")] if decided else g
        fig, axes = plt.subplots(1, len(settings), figsize=(6.5 * len(settings), 4.2), squeeze=False)
        for ax, st in zip(axes[0], settings):
            for strat, gs in gg[gg["setting_key"] == st].groupby("strategy"):
                mats = np.vstack([trace(p, scen) for p in gs["usd_path"]])
                c = COL.get(strat, "#999999")
                if len(mats) >= 3:
                    ax.fill_between(GRID, np.nanpercentile(mats, 10, axis=0), np.nanpercentile(mats, 90, axis=0), color=c, alpha=0.15)
                    ax.plot(GRID, np.nanmedian(mats, axis=0), color=c, lw=1.8, label=f"{strat} (n={len(mats)}, median, 10–90 %)")
                else:
                    for m in mats:
                        ax.plot(GRID, m, color=c, lw=1.0, alpha=0.8)
                    ax.plot([], [], color=c, label=f"{strat} (n={len(mats)}, single flights)")
            ax.axhspan(-2, 2, color="#4FB39A", alpha=0.15); ax.axhline(0, color="k", lw=0.5)
            ax.set_title(f"{scen} {st}", fontsize=10); ax.set_xlabel("time (s)"); ax.set_ylabel("z error (cm)"); ax.legend(fontsize=7); ax.grid(alpha=0.3)
        fig.suptitle(f"{scen}: z error vs time per strategy (cf5, uSD)" + ("" if decided else " — decisions file has no entry: all flights shown"), fontsize=10)
        fig.tight_layout(); fig.savefig(OUT / f"{scen}_z_error_vs_time.png", dpi=140); plt.close(fig)

        # metric vs setting
        xs = {s: i for i, s in enumerate(settings)}
        num = {}
        for s in settings:
            try:
                num[s] = float(str(s).split("=")[1])
            except Exception:
                num[s] = float(xs[s])
        fig, ax = plt.subplots(figsize=(6, 4.2))
        for strat, gs in gg.groupby("strategy"):
            c = COL.get(strat, "#999999")
            pts = [(num[s], v) for s, v in zip(gs["setting_key"], gs[a.metric]) if s in num and v == v]
            if not pts:
                continue
            ax.scatter(*zip(*pts), color=c, s=18, alpha=0.5)
            m = pd.DataFrame(pts, columns=["x", "y"]).groupby("x")["y"].agg(["mean", "std"]).reset_index()
            ax.errorbar(m["x"], m["mean"], yerr=m["std"].fillna(0), color=c, marker="o", capsize=3, label=strat)
        name = g["primary_metric_name"].dropna().unique()
        ax.set_xlabel(("speed (m/s)" if settings and str(settings[0]).startswith("speed") else "dz (m)" if settings else "setting"))
        ax.set_ylabel(f"{a.metric}" + (f" ({name[0]})" if len(name) and a.metric == "primary_metric" else ""))
        ax.legend(fontsize=8); ax.grid(alpha=0.3); ax.set_title(f"{scen}: metric vs setting (dots = flights, line = mean ± sd)", fontsize=10)
        fig.tight_layout(); fig.savefig(OUT / f"{scen}_metric_vs_setting.png", dpi=140); plt.close(fig)

        cells, vs, _ = ag.study_table(g, a.metric, a.baseline_strategy)
        md += [f"## {scen}", "", "Cells (statistics over flights):", "", "```\n" + cells.round(3).to_string(index=False) + "\n```" if len(cells) else "(none)", "",
               "Strategy vs baseline per setting (unpaired bootstrap 95 % CI):", "", "```\n" + vs.round(3).to_string(index=False) + "\n```" if len(vs) else "(none)", ""]
    (OUT / "cells.md").write_text("\n".join(md) + "\n")
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
