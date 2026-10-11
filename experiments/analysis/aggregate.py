#!/usr/bin/env python3
"""P3 — turn many per-run metric rows into a defensible comparison (docs/27 gap 1).

    # aggregate rows produced by run_analysis.py / metrics.vehicle_metrics_by_phase
    python3 aggregate.py rows.csv --metric pos_rmse_m --phase approach1

    # compare two controllers, paired by scenario
    python3 aggregate.py rows.csv --metric pos_rmse_m --baseline geometric --treatment indi

Everything upstream of this reports SINGLE runs. A single run cannot support a claim like
"INDI reduces tracking error 2.9x" -- that is why the 2026-09-18 comparison is labelled
*indicative, not statistical*. This module is what turns a pile of runs into a number with an
uncertainty attached and, where the design allows it, a significance test.

## Why these specific choices

**Wilcoxon signed-rank, not a t-test.** n is small (a handful of repeats per controller), the
metrics are RMSEs and so are bounded below and right-skewed, and normality is neither plausible
nor checkable at this n. Wilcoxon assumes only symmetry of the paired differences.

**Paired, not independent.** Runs are matched by (scenario, phase): the same commanded
trajectory flown by each controller. Pairing removes scenario-to-scenario variance, which is
large here and would otherwise swamp the controller effect.

**Effect size always, p-value only when n justifies it.** At n=3-5 a p-value is nearly
meaningless on its own and inviting a reader to read significance into it would be misleading.
The ratio and its CI say more than a star does. `n_pairs` is reported so nobody has to guess.

**Bootstrap CI on the ratio, not error propagation.** The quantity of interest is a RATIO of
RMSEs; its distribution is not symmetric and Gaussian propagation understates the upper tail.
A percentile bootstrap over pairs makes no distributional assumption at all.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

try:
    from scipy import stats as _st
except ImportError:                                  # pragma: no cover
    _st = None

# Below this, report the effect size and CI but suppress the p-value: at n<6 the Wilcoxon
# statistic cannot reach conventional significance regardless of effect, so printing one
# invites a reader to conclude "not significant" from what is really "not enough runs".
MIN_N_FOR_P = 6
N_BOOT = 10000


def summarise(df: pd.DataFrame, metric: str, by=("controller", "scenario", "phase")) -> pd.DataFrame:
    """mean ± std ± sem and n, grouped. The `n` column is not decoration -- every downstream
    claim should be read against it."""
    by = [b for b in by if b in df.columns]
    g = df.groupby(by, dropna=False)[metric]
    out = g.agg(n="count", mean="mean", std="std", median="median",
                q25=lambda s: s.quantile(0.25), q75=lambda s: s.quantile(0.75)).reset_index()
    out["sem"] = out["std"] / np.sqrt(out["n"].clip(lower=1))
    return out


def _bootstrap_ratio_ci(a: np.ndarray, b: np.ndarray, n_boot: int = N_BOOT,
                         seed: int = 0) -> tuple[float, float]:
    """Percentile CI for mean(a)/mean(b), resampling PAIRS (so the pairing is preserved)."""
    rng = np.random.default_rng(seed)
    k = len(a)
    if k < 2:
        return float("nan"), float("nan")
    idx = rng.integers(0, k, size=(n_boot, k))
    ra = a[idx].mean(axis=1)
    rb = b[idx].mean(axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.where(rb != 0, ra / rb, np.nan)
    r = r[np.isfinite(r)]
    if not len(r):
        return float("nan"), float("nan")
    return float(np.percentile(r, 2.5)), float(np.percentile(r, 97.5))


def paired_compare(df: pd.DataFrame, metric: str, baseline: str, treatment: str,
                    pair_on=("scenario", "phase"), controller_col: str = "controller",
                    run_col: str = "run") -> dict:
    """Paired comparison of two controllers on one metric.

    Rows are matched on `pair_on`; where a key has several runs per controller (repeats), each
    controller's runs are averaged within the key first, so one key contributes one pair rather
    than silently weighting well-sampled scenarios more heavily.

    ⚠️ **Pseudo-replication guard.** Pairing on `phase` means that with only ONE flight per
    controller, the "pairs" are phases of the same flight -- not independent replicates. The
    arithmetic still produces a confident-looking CI and, past n=6, a p-value, but it would be
    measuring within-flight variation and calling it between-flight evidence. If `run_col` is
    present and either controller has only one distinct run, that is flagged loudly in `note`
    and the p-value is suppressed regardless of n. Repeats mean repeated FLIGHTS.
    """
    pair_on = [c for c in pair_on if c in df.columns]
    sub = df[df[controller_col].isin([baseline, treatment])]
    sub = sub[np.isfinite(sub[metric])]
    if sub.empty:
        return dict(metric=metric, baseline=baseline, treatment=treatment, n_pairs=0,
                    note="no finite rows for either controller")

    wide = (sub.groupby(pair_on + [controller_col])[metric].mean()
               .unstack(controller_col))
    if baseline not in wide.columns or treatment not in wide.columns:
        return dict(metric=metric, baseline=baseline, treatment=treatment, n_pairs=0,
                    note=f"one controller absent after pairing on {pair_on}")
    wide = wide.dropna(subset=[baseline, treatment])
    b = wide[baseline].to_numpy(float)
    t = wide[treatment].to_numpy(float)
    n = len(b)
    res = dict(metric=metric, baseline=baseline, treatment=treatment, n_pairs=n,
               pair_on="+".join(pair_on),
               baseline_mean=float(np.mean(b)) if n else float("nan"),
               treatment_mean=float(np.mean(t)) if n else float("nan"))
    if n == 0:
        res["note"] = "no overlapping keys -- nothing was flown by both controllers"
        return res

    res["ratio"] = float(np.mean(b) / np.mean(t)) if np.mean(t) else float("nan")
    lo, hi = _bootstrap_ratio_ci(b, t)
    res["ratio_ci_lo"], res["ratio_ci_hi"] = lo, hi
    d = b - t
    res["mean_diff"] = float(np.mean(d))
    # Matched-pairs rank-biserial: effect size on [-1,1], reads as "share of pairs favouring
    # the treatment". Defined at any n, unlike a p-value, and not inflated by outliers.
    nz = d[d != 0]
    if len(nz):
        r = _st.rankdata(np.abs(nz)) if _st is not None else np.argsort(np.argsort(np.abs(nz))) + 1.0
        tot = r.sum()
        res["effect_rank_biserial"] = float((r[nz > 0].sum() - r[nz < 0].sum()) / tot) if tot else float("nan")
    else:
        res["effect_rank_biserial"] = 0.0
    res["n_favouring_treatment"] = int(np.sum(d > 0))

    # Pseudo-replication check: are these pairs really independent flights?
    pseudo = False
    if run_col in sub.columns:
        per_ctrl = sub.groupby(controller_col)[run_col].nunique()
        if (per_ctrl <= 1).any():
            pseudo = True
            thin = ", ".join(f"{c}={int(k)}" for c, k in per_ctrl.items())
            res["note"] = (f"PSEUDO-REPLICATION: only one distinct run for a controller "
                           f"({thin}). The pairs here are phases of the same flight, not "
                           f"independent repeats -- treat the ratio as descriptive only. "
                           f"p-value suppressed.")
    res["n_runs_ok"] = int(not pseudo)

    if not pseudo and n >= MIN_N_FOR_P and _st is not None and len(nz):
        try:
            res["wilcoxon_p"] = float(_st.wilcoxon(b, t, zero_method="wilcox").pvalue)
        except ValueError as exc:
            res["wilcoxon_p"] = float("nan")
            res["note"] = f"wilcoxon undefined: {exc}"
    else:
        res["wilcoxon_p"] = float("nan")
        res.setdefault("note", (f"p-value withheld: n_pairs={n} < {MIN_N_FOR_P}. Report the "
                                f"ratio and its CI; do not read non-significance into this."))
    return res


def parse_cell(spec: str) -> tuple[str, str, str]:
    parts = [p.strip() for p in spec.split(",")]
    if len(parts) != 3:
        raise ValueError("cell must be scenario,setting,strategy")
    return parts[0], parts[1], parts[2]


def _flight_values(df: pd.DataFrame, metric: str) -> pd.DataFrame:
    """One value per FLIGHT (unit of analysis): the subject drone (cf5) row of every flight, with its decision."""
    sub = df.copy()
    if "vehicle_id" in sub.columns:
        sub = sub[sub["vehicle_id"] == "cf5"]
    keys = [c for c in ("date", "stamp") if c in sub.columns]
    sub = sub.drop_duplicates(subset=keys) if keys else sub
    sub["_decision"] = sub["decision"].fillna("") if "decision" in sub.columns else ""
    return sub


def summarise_study_cell(df: pd.DataFrame, metric: str) -> dict:
    """One cell: statistics over FLIGHTS. clean = decision 'clean' (all flights count as clean, with decision_unknown set,
    when the decisions file has no entry at all). 'not_reached' = 8 attempts and fewer than 5 clean flights (protocol section 4)."""
    sub = _flight_values(df, metric)
    n_attempts = len(sub)
    decided = bool((sub["_decision"] != "").any())
    clean = sub[sub["_decision"].eq("clean")] if decided else sub
    vals = clean[metric].dropna().astype(float) if metric in clean.columns else pd.Series(dtype=float)
    n_clean = int(len(clean))
    return {
        "n_clean": n_clean,
        "n_attempts": n_attempts,
        "failure_rate": float(1 - n_clean / n_attempts) if n_attempts else float("nan"),
        "mean": float(vals.mean()) if len(vals) else float("nan"),
        "std": float(vals.std(ddof=1)) if len(vals) > 1 else float("nan"),
        "median": float(vals.median()) if len(vals) else float("nan"),
        "incomplete": n_clean < 5,
        "not_reached": bool(n_attempts >= 8 and n_clean < 5),
        "decision_unknown": not decided,
    }


def _boot_unpaired(a: np.ndarray, b: np.ndarray, n_boot: int = N_BOOT, seed: int = 0):
    """Unpaired bootstrap (flights of two strategies are independent): 95 % CI of mean(a)-mean(b) and mean(a)/mean(b)."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    if len(a) < 2 or len(b) < 2:
        return (float("nan"),) * 4
    rng = np.random.default_rng(seed)
    ra = a[rng.integers(0, len(a), size=(n_boot, len(a)))].mean(axis=1)
    rb = b[rng.integers(0, len(b), size=(n_boot, len(b)))].mean(axis=1)
    diff = ra - rb
    with np.errstate(divide="ignore", invalid="ignore"):
        ratio = np.where(rb != 0, ra / rb, np.nan)
    ratio = ratio[np.isfinite(ratio)]
    rlo, rhi = (np.percentile(ratio, 2.5), np.percentile(ratio, 97.5)) if len(ratio) else (float("nan"),) * 2
    return float(np.percentile(diff, 2.5)), float(np.percentile(diff, 97.5)), float(rlo), float(rhi)


def study_table(df: pd.DataFrame, metric: str, baseline: str = "S0") -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """(cells, vs_baseline, metric_vs_setting). Cell = scenario x setting x strategy; statistics over flights."""
    d = df.copy()
    d["setting"] = d["setting_key"].fillna("") if "setting_key" in d.columns else ""
    d["strategy"] = d["strategy"].fillna("") if "strategy" in d.columns else ""
    d = d[d["strategy"] != ""]
    cells, vs, per_setting = [], [], []
    for (scen, setting), g in d.groupby(["scenario", "setting"]):
        per_strat = {}
        for strat, gg in g.groupby("strategy"):
            st = summarise_study_cell(gg, metric)
            cells.append({"scenario": scen, "setting": setting, "strategy": strat, **st})
            fv = _flight_values(gg, metric)
            if not st["decision_unknown"]:
                fv = fv[fv["_decision"].eq("clean")]
            per_strat[strat] = fv[metric].dropna().astype(float).values if metric in fv.columns else np.array([])
            per_setting.append({"scenario": scen, "setting": setting, "strategy": strat, "mean": st["mean"], "std": st["std"], "n_clean": st["n_clean"]})
        base = per_strat.get(baseline)
        if base is None or not len(base):
            continue
        for strat, vals in per_strat.items():
            if strat == baseline or not len(vals):
                continue
            dlo, dhi, rlo, rhi = _boot_unpaired(vals, base)
            vs.append({"scenario": scen, "setting": setting, "strategy": strat, "baseline": baseline,
                       "n": len(vals), "n_baseline": len(base), "mean_diff": float(vals.mean() - base.mean()),
                       "diff_ci_lo": dlo, "diff_ci_hi": dhi, "ratio": float(vals.mean() / base.mean()) if base.mean() != 0 else float("nan"),
                       "ratio_ci_lo": rlo, "ratio_ci_hi": rhi,
                       "note": "n<5 in a group: descriptive only" if min(len(vals), len(base)) < 5 else ""})
    cells_df = pd.DataFrame(cells)
    vs_df = pd.DataFrame(vs)
    ps = pd.DataFrame(per_setting)
    mvs = ps.pivot_table(index=["scenario", "strategy"], columns="setting", values="mean") if len(ps) else pd.DataFrame()
    return cells_df, vs_df, mvs



def format_compare(r: dict) -> str:
    if not r.get("n_pairs"):
        return f"{r['metric']}: {r.get('note', 'no pairs')}"
    lines = [
        f"{r['metric']}  —  {r['baseline']} vs {r['treatment']}   (paired on {r['pair_on']}, "
        f"n_pairs={r['n_pairs']})",
        f"   {r['baseline']:>14s}: {r['baseline_mean']:.5g}",
        f"   {r['treatment']:>14s}: {r['treatment_mean']:.5g}",
        f"   ratio (base/treat): {r['ratio']:.3g}  95% CI [{r['ratio_ci_lo']:.3g}, "
        f"{r['ratio_ci_hi']:.3g}]",
        f"   effect (rank-biserial): {r['effect_rank_biserial']:+.3f}   "
        f"pairs favouring {r['treatment']}: {r['n_favouring_treatment']}/{r['n_pairs']}",
    ]
    if np.isfinite(r.get("wilcoxon_p", float("nan"))):
        lines.append(f"   Wilcoxon signed-rank p = {r['wilcoxon_p']:.4g}")
    if r.get("note"):
        lines.append(f"   note: {r['note']}")
    # A CI straddling 1.0 means the data do not establish a difference in either direction.
    if np.isfinite(r.get("ratio_ci_lo", float("nan"))) and r["ratio_ci_lo"] <= 1.0 <= r["ratio_ci_hi"]:
        lines.append("   ⚠️  the ratio's 95% CI includes 1.0 — this does not establish a difference")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("rows", type=Path, help="CSV of per-run metric rows")
    ap.add_argument("--metric", default="pos_rmse_m")
    ap.add_argument("--phase", default=None, help="restrict to one phase (e.g. approach1)")
    ap.add_argument("--baseline", default=None)
    ap.add_argument("--treatment", default=None)
    ap.add_argument("--vehicle", default=None, help="restrict to one vehicle_id")
    ap.add_argument("--group-by", default="controller",
                    help="column for summarise / paired_compare grouping (e.g. study_controller)")
    ap.add_argument("--cell", default=None,
                    help="study cell filter: scenario,setting_key,strategy (e.g. A8,speed=0.4,S0)")
    ap.add_argument("--study-metric", default="primary_metric",
                    help="metric column when using --cell (default primary_metric)")
    ap.add_argument("--cell-csv", type=Path, default=None,
                    help="write the per-cell summary table (with --study-table) for plotting")
    ap.add_argument("--study-table", action="store_true",
                    help="study mode: per-cell table (scenario x setting x strategy, stats over flights), strategy-vs-baseline per setting "
                         "(unpaired bootstrap CI), metric-vs-setting table; uses study_rows.csv with a `strategy` column")
    ap.add_argument("--baseline-strategy", default="S0")
    args = ap.parse_args()

    df = pd.read_csv(args.rows)
    if args.study_table:
        metric = args.study_metric if args.study_metric in df.columns else args.metric
        cells, vs, mvs = study_table(df, metric, args.baseline_strategy)
        pd.set_option("display.width", 250); pd.set_option("display.max_columns", 30)
        print(f"metric = {metric}; unit = one flight (cf5); baseline = {args.baseline_strategy}\n")
        print("== cells ==\n" + (cells.round(3).to_string(index=False) if len(cells) else "(no rows with a strategy)"))
        print("\n== strategy vs baseline per setting (unpaired bootstrap 95 % CI) ==\n" + (vs.round(3).to_string(index=False) if len(vs) else "(none)"))
        print("\n== metric vs setting (mean over flights) ==\n" + (mvs.round(3).to_string() if len(mvs) else "(none)"))
        if args.cell_csv:
            cells.to_csv(args.cell_csv, index=False)
            vs.to_csv(args.cell_csv.with_name(args.cell_csv.stem + "_vs_baseline.csv"), index=False)
            if len(mvs):
                mvs.to_csv(args.cell_csv.with_name(args.cell_csv.stem + "_metric_vs_setting.csv"))
        return
    if args.cell:
        scen, setting, strategy = parse_cell(args.cell)
        if "scenario" in df.columns:
            df = df[df["scenario"] == scen]
        if "setting_key" in df.columns:
            df = df[df["setting_key"] == setting]
        if "strategy" in df.columns:
            df = df[df["strategy"] == strategy]
        metric = args.study_metric if args.study_metric in df.columns else args.metric
        stats = summarise_study_cell(df, metric)
        print(f"cell {args.cell} metric={metric}: {stats}")
        if args.baseline and args.treatment and "strategy" in df.columns:
            wide = df[df["strategy"].isin([args.baseline, args.treatment])]
            print(format_compare(paired_compare(wide, metric, args.baseline, args.treatment,
                                                pair_on=("date", "stamp"), controller_col="strategy")))
        return
    gcol = args.group_by if args.group_by in df.columns else "controller"
    if args.phase and "phase" in df.columns:
        df = df[df["phase"] == args.phase]
    if args.vehicle and "vehicle_id" in df.columns:
        df = df[df["vehicle_id"] == args.vehicle]
    if args.metric not in df.columns:
        sys.exit(f"no column {args.metric!r}; have: {', '.join(df.columns)}")

    print(summarise(df, args.metric, by=(gcol,)).to_string(index=False))
    if args.baseline and args.treatment:
        print()
        print(format_compare(paired_compare(df, args.metric, args.baseline, args.treatment,
                                            controller_col=gcol)))


if __name__ == "__main__":
    main()
