#!/usr/bin/env python3
"""NS2 sign-test cohort analysis (crossing dips, stats, pass/fail vs reference)."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy import stats

REPO = Path(__file__).resolve().parents[2]
LOG = REPO / "experiments/logs"
USD = LOG / "usd_raw"
ANALYSIS = Path(__file__).resolve().parent
sys.path.insert(0, str(ANALYSIS))
import ns2_2026_10_05_crossing_dip as ns2  # noqa: E402

REF_EN0_MEAN_CM = -5.9
MIN_IMPROVEMENT_CM = 1.0  # test mean must be at least this much shallower than the baseline mean
REF_EN1_RS_PLUS_MEAN_CM = -10.9
PASS_CRITERION_TEXT = (
    "Pass: the crossing dip (cf5 z error, a negative number) is clearly SHALLOWER than the "
    "network-off baseline of about −5.9 cm (mean at least MIN_IMPROVEMENT_CM less negative), "
    "ranges not overlapping, no over-compensation. "
    "Current network-on, res_sign=+1: −10.9 cm (DEEPER = worse)."
)


def parse_cohorts(specs: list[str]) -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}
    for s in specs:
        if ":" not in s:
            raise ValueError(f"cohort must be name:stamp,stamp got {s!r}")
        name, stamps = s.split(":", 1)
        out[name.strip()] = {x.strip() for x in stamps.split(",") if x.strip()}
    return out


def _thesis_key(name: str) -> str:
    m = re.search(r"(thesis\d+|A8rnn1_thesis\d+)", name)
    return m.group(1) if m else name


def _pick_bins_for_stamp(stamp: str, paths: list[Path]) -> list[Path]:
    """Multiple flights can share a wall-clock stamp on uSD cards; keep valid thesis slots."""
    a8 = sorted(p for p in paths if "_A8_thesis" in p.name and f"_{stamp}.bin" in p.name)
    if a8:
        return a8
    rnn = sorted(p for p in paths if stamp in p.name)
    if not rnn:
        return []
    # Same card time, duplicate/crash copy (thesis03) — keep thesis02 when present.
    t2 = [p for p in rnn if "thesis02" in p.name]
    if t2:
        return t2[:1]
    return rnn[:1]


def bins_for_cohorts(date: str, cohorts: dict[str, set[str]]) -> dict[str, list[Path]]:
    all_stamps = set().union(*cohorts.values())
    by_stamp: dict[str, list[Path]] = {}
    for p in sorted(USD.glob(f"cf5_A8*_{date}_*.bin")):
        if "A1" in p.name:
            continue
        st = ns2.stamp_from_name(p.name)
        if st not in all_stamps:
            continue
        by_stamp.setdefault(st, []).append(p)
    by_cohort: dict[str, list[Path]] = {k: [] for k in cohorts}
    for cname, stamps in cohorts.items():
        for st in sorted(stamps):
            by_cohort[cname].extend(_pick_bins_for_stamp(st, by_stamp.get(st, [])))
    return by_cohort


def cohort_stats(dips: list[dict]) -> dict:
    if not dips:
        return {"n": 0}
    arr = np.array([r["dip_cm"] for r in dips])
    return {
        "n": int(len(arr)),
        "mean_cm": float(np.mean(arr)),
        "std_cm": float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
        "min_cm": float(np.min(arr)),
        "max_cm": float(np.max(arr)),
    }


def per_flight_pred_residual_corr(path: Path, dips: list[dict]) -> dict:
    """At crossing windows: corr(pred_z_mean, dip_cm - mean dip for that flight)."""
    file_dips = [r for r in dips if r["file"] == path.name]
    if len(file_dips) < 2:
        return {"file": path.name, "n_crossings": len(file_dips), "corr_pred_dip_residual": float("nan")}
    sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
    from decode_usd_log import load as load_usd  # noqa: E402

    d = load_usd(str(path))
    m = np.isfinite(d.get("rnn_pred_z", np.nan)) & np.isfinite(d.get("a_res_z", np.nan))
    if np.sum(m) < 50:
        return {"file": path.name, "n_crossings": len(file_dips), "corr_pred_dip_residual": float("nan")}
    pred = d["rnn_pred_z"][m]
    a_res = d["a_res_z"][m]
    c_pa = float(np.corrcoef(pred, a_res)[0, 1])
    mean_dip = float(np.mean([r["dip_cm"] for r in file_dips]))
    pred_at = np.array([r["pred_z_mean"] for r in file_dips])
    resid = np.array([r["dip_cm"] for r in file_dips]) - mean_dip
    c_dr = float(np.corrcoef(pred_at, resid)[0, 1]) if len(file_dips) > 2 else float("nan")
    return {
        "file": path.name,
        "n_crossings": len(file_dips),
        "corr_pred_a_res_full_log": c_pa,
        "corr_pred_at_crossing_vs_dip_residual": c_dr,
    }


def compare_reference(stats_a: dict, stats_b: dict) -> dict:
    d0 = stats_a.get("mean_cm", float("nan"))
    d1 = stats_b.get("mean_cm", float("nan"))
    overlap = False
    if stats_a.get("n", 0) and stats_b.get("n", 0):
        overlap = bool(stats_a["max_cm"] >= stats_b["min_cm"] and stats_b["max_cm"] >= stats_a["min_cm"])
    out = {"ranges_overlap": overlap}
    if stats_a.get("n", 0) >= 2 and stats_b.get("n", 0) >= 2:
        a = [r["dip_cm"] for r in stats_a.get("_rows", [])]
        b = [r["dip_cm"] for r in stats_b.get("_rows", [])]
        t, pw = stats.ttest_ind(a, b, equal_var=False)
        u, pm = stats.mannwhitneyu(a, b, alternative="two-sided")
        out.update(welch_t=float(t), welch_p=float(pw), mannwhitney_u=float(u), mannwhitney_p=float(pm))
    out["ref_en0_mean_cm"] = REF_EN0_MEAN_CM
    out["ref_en1_rs_plus_mean_cm"] = REF_EN1_RS_PLUS_MEAN_CM
    return out


def write_svg(groups: list[tuple[str, list[float], str]], path: Path, title: str) -> None:
    w, h = 480, 240
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">',
        '<rect width="100%" height="100%" fill="#fafafa"/>',
        f'<text x="10" y="18" font-size="11">{title}</text>',
        '<line x1="90" y1="35" x2="90" y2="210" stroke="#999"/>',
    ]
    x_centers = [180, 340][: len(groups)]

    def ymap(v):
        return 45 + (-v / 14.0) * 150

    for i, (lab, vals, col) in enumerate(groups):
        if not vals:
            continue
        xc = x_centers[i]
        arr = np.array(vals)
        mean = float(np.mean(arr))
        lines.append(f'<text x="{xc - 60}" y="32" font-size="10">{lab} n={len(vals)} mean={mean:.1f} cm</text>')
        for j, v in enumerate(vals):
            jitter = ((j % 5) - 2) * 5
            lines.append(f'<circle cx="{xc + jitter}" cy="{ymap(v):.1f}" r="3.5" fill="{col}" opacity="0.75"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines))


def pass_fail(
    baseline_stats: dict,
    test_stats: dict,
    *,
    baseline_label: str,
    test_label: str,
) -> dict:
    """Gate: test dip clearly SHALLOWER than the baseline (less negative), ranges not overlapping, no over-compensation.

    Review fix 2026-10-06: the first version tested te_mean < bl_mean, i.e. numerically below = DEEPER = worse,
    and so passed the wrong-sign +1 data itself."""
    bl_mean = baseline_stats.get("mean_cm", float("nan"))
    te_mean = test_stats.get("mean_cm", float("nan"))
    overlap = False
    if baseline_stats.get("n") and test_stats.get("n"):
        overlap = bool(
            baseline_stats["max_cm"] >= test_stats["min_cm"]
            and test_stats["max_cm"] >= baseline_stats["min_cm"]
        )
    clearly_shallower = bool(np.isfinite(te_mean) and np.isfinite(bl_mean) and (te_mean - bl_mean) >= MIN_IMPROVEMENT_CM)
    worse = bool(np.isfinite(te_mean) and np.isfinite(bl_mean) and (bl_mean - te_mean) >= MIN_IMPROVEMENT_CM)
    over_comp = bool(np.isfinite(te_mean) and te_mean > 0.0)
    passed = clearly_shallower and (not overlap) and (not over_comp)
    return {
        "criterion": PASS_CRITERION_TEXT,
        "baseline_label": baseline_label,
        "test_label": test_label,
        "baseline_mean_cm": bl_mean,
        "test_mean_cm": te_mean,
        "ranges_overlap": overlap,
        "clearly_shallower_than_baseline": clearly_shallower,
        "worse_than_baseline": worse,
        "over_compensation_mean_dip_positive": over_comp,
        "pass": passed,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--date", required=True, help="session date YYYY-MM-DD")
    ap.add_argument(
        "--cohort",
        action="append",
        required=True,
        help="name:HH-MM-SS,HH-MM-SS (stamps in cf5 uSD filenames)",
    )
    ap.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="default experiments/analysis/out/ns2_signtest_<date>",
    )
    ap.add_argument(
        "--reference-baseline",
        default="en0",
        help="cohort name treated as network-off baseline for pass/fail",
    )
    ap.add_argument(
        "--reference-test",
        default="en1",
        help="cohort name treated as network-on test for pass/fail",
    )
    args = ap.parse_args()
    cohorts = parse_cohorts(args.cohort)
    out_dir = args.out_dir or (ANALYSIS / "out" / f"ns2_signtest_{args.date}")
    out_dir.mkdir(parents=True, exist_ok=True)

    by_cohort = bins_for_cohorts(args.date, cohorts)
    all_dips: list[dict] = []
    cohort_rows: dict[str, list[dict]] = {}
    for cname, paths in by_cohort.items():
        rows = []
        for p in paths:
            rows.extend(ns2.crossing_dips_usd(p))
        for r in rows:
            r["cohort"] = cname
        cohort_rows[cname] = rows
        all_dips.extend(rows)

    stats_by: dict[str, dict] = {}
    for cname, rows in cohort_rows.items():
        st = cohort_stats(rows)
        st["_rows"] = rows
        stats_by[cname] = st

    bl = args.reference_baseline
    te = args.reference_test
    cmp_stats = compare_reference(stats_by.get(bl, {}), stats_by.get(te, {}))
    pf = pass_fail(stats_by.get(bl, {}), stats_by.get(te, {}), baseline_label=bl, test_label=te)

    flight_corrs = []
    seen = set()
    for r in all_dips:
        if r["file"] in seen:
            continue
        seen.add(r["file"])
        p = USD / r["file"]
        if p.is_file():
            flight_corrs.append(per_flight_pred_residual_corr(p, all_dips))

    summary = {
        "date": args.date,
        "cohorts": {k: [str(p.name) for p in v] for k, v in by_cohort.items()},
        "stats_by_cohort": {k: {kk: vv for kk, vv in v.items() if kk != "_rows"} for k, v in stats_by.items()},
        "comparison": cmp_stats,
        "pass_fail_vs_baseline": pf,
        "per_flight_correlations": flight_corrs,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    md = [
        f"# NS2 sign-test analysis — {args.date}",
        "",
        PASS_CRITERION_TEXT,
        "",
        f"**Pass/fail ({te} vs {bl}):** {'PASS' if pf['pass'] else 'FAIL'}",
        "",
        "## Cohort statistics (crossing dip min e_z, ±1 s)",
        "",
        "| Cohort | n | mean [cm] | std | min | max |",
        "|--------|---|-----------|-----|-----|-----|",
    ]
    for cname in sorted(stats_by.keys()):
        s = stats_by[cname]
        if not s.get("n"):
            md.append(f"| {cname} | 0 | — | — | — | — |")
        else:
            md.append(
                f"| {cname} | {s['n']} | {s['mean_cm']:.2f} | {s['std_cm']:.2f} | "
                f"{s['min_cm']:.2f} | {s['max_cm']:.2f} |"
            )
    md.extend(
        [
            "",
            "## vs 2026-10-05 reference",
            f"- Reference rnn.en=0 mean: **{REF_EN0_MEAN_CM} cm** (8 crossings)",
            f"- Reference rnn.en=1 res_sign=+1 mean: **{REF_EN1_RS_PLUS_MEAN_CM} cm** (16 crossings)",
            "",
            f"- Welch p (this session {bl} vs {te}): {cmp_stats.get('welch_p', 'n/a')}",
            f"- Ranges overlap: {cmp_stats.get('ranges_overlap', 'n/a')}",
            "",
        ]
    )
    (out_dir / "summary.md").write_text("\n".join(md) + "\n")

    groups = []
    colors = ["#48a", "#c44", "#6a4", "#a64"]
    for i, cname in enumerate(sorted(stats_by.keys())):
        rows = cohort_rows.get(cname, [])
        groups.append((cname, [r["dip_cm"] for r in rows], colors[i % len(colors)]))
    write_svg(groups, out_dir / "fig_crossing_dip.svg", f"NS2 sign-test {args.date}")

    print(json.dumps({"out": str(out_dir), "pass_fail": pf, "stats": summary["stats_by_cohort"]}, indent=2))


if __name__ == "__main__":
    main()
