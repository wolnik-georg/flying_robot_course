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
    args = ap.parse_args()

    df = pd.read_csv(args.rows)
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
