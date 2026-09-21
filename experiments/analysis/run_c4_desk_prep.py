#!/usr/bin/env python3
"""Desk prep for C.4 — run aggregation + comparison figures on existing phase-metric CSVs.

Uses P3/P4 from docs/27 (aggregate.py, plot_comparison.py). Does not need the lab.

Default input: 2026-09-19 A8 suite output (cf5, scenario phase).

Grouping policy (2026-09-21 audit): headline summarise/plots use
``phase_metrics_c4_compare.csv`` — geometric + full INDI only. ``study_controller`` is the
source of truth; ``controller`` must not alias stock Lee to geometric (see suite CTRL_LABEL).

    python3 experiments/analysis/run_c4_desk_prep.py
    python3 experiments/analysis/run_c4_desk_prep.py --rows path/to_phase_metrics.csv --out path/to/out
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
AN = Path(__file__).resolve().parent
sys.path.insert(0, str(AN))

import aggregate as agg_mod  # noqa: E402

DEFAULT_ROWS = AN / "out/a8_2026-09-19/a8_2026-09-19_phase_metrics_all.csv"
DEFAULT_OUT = AN / "out/c4_desk_prep"
PYENV_PY = Path.home() / ".pyenv/versions/flying_robots/bin/python"


def _plot_python() -> str:
    """matplotlib needs the pyenv env on this machine (see experiments/analysis/README.md)."""
    return str(PYENV_PY) if PYENV_PY.is_file() else sys.executable


def _summarise_csv(csv: Path, metric: str, group_col: str) -> None:
    print(agg_mod.summarise(
        __import__("pandas").read_csv(csv), metric, by=(group_col,),
    ).to_string(index=False))


def _paired_csv(csv: Path, metric: str, group_col: str) -> None:
    df = __import__("pandas").read_csv(csv)
    print(agg_mod.format_compare(
        agg_mod.paired_compare(df, metric, "geometric", "indi", controller_col=group_col),
    ))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rows", type=Path, default=DEFAULT_ROWS,
                    help="phase metrics CSV (vehicle_metrics_by_phase rows)")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--vehicle", default="cf5")
    ap.add_argument("--phase", default="scenario",
                    help="phase column filter (scenario = main interaction window)")
    args = ap.parse_args()

    if not args.rows.is_file():
        sys.exit(f"missing rows file: {args.rows}\n"
                 f"Run: python3 experiments/analysis/run_a8_2026_09_19_suite.py")

    args.out.mkdir(parents=True, exist_ok=True)
    py = sys.executable
    plot = AN / "plot_comparison.py"

    import pandas as pd

    df = pd.read_csv(args.rows)
    if "vehicle_id" in df.columns:
        df = df[df["vehicle_id"] == args.vehicle]
    if "phase" in df.columns:
        df = df[df["phase"] == args.phase]
    if df.empty:
        sys.exit(f"no rows after filter vehicle={args.vehicle!r} phase={args.phase!r}")

    filtered = args.out / "phase_metrics_filtered.csv"
    df.to_csv(filtered, index=False)
    print(f"wrote {filtered} ({len(df)} rows)")

    if "study_controller" in df.columns:
        c4 = df[df["study_controller"].isin(("geometric", "full_indi"))].copy()
        c4["controller"] = c4["study_controller"].map(
            {"geometric": "geometric", "full_indi": "indi"})
        group_col = "controller"
    else:
        c4 = df[~df["controller"].isin(("stock_lee",))].copy()
        group_col = "controller"
    if c4.empty:
        sys.exit("no geometric/full_indi rows for C.4 compare (check study_controller column)")

    compare_csv = args.out / "phase_metrics_c4_compare.csv"
    c4.to_csv(compare_csv, index=False)
    print(f"wrote {compare_csv} ({len(c4)} rows) — used for summarise/plot")

    metrics = [
        "pos_rmse_m", "pos_rmse_z", "e_R_rmse", "a_res_z_rms",
        "effort_proxy", "motor_rms_ratio", "min_sep_m",
    ]
    for m in metrics:
        if m not in c4.columns:
            continue
        print(f"\n=== summarise {m} ===")
        _summarise_csv(compare_csv, m, group_col)

    if "pos_rmse_m" in c4.columns and len(c4) >= 2:
        print("\n=== paired geometric vs full INDI (pos_rmse_m) ===")
        _paired_csv(compare_csv, "pos_rmse_m", group_col)

    plot_py = _plot_python()
    if plot_py != py:
        print(f"(figures via {plot_py})")
    print("\n=== figures (C.4 compare subset only) ===")
    plot_args_base = ["--group-by", group_col]
    for m in ("pos_rmse_m", "pos_rmse_z", "e_R_rmse", "effort_proxy"):
        if m not in c4.columns:
            continue
        subprocess.run(
            [plot_py, str(plot), str(compare_csv), "--metric", m, "--phase", args.phase,
             "-o", str(args.out), *plot_args_base],
            check=False,
        )
    subprocess.run(
        [plot_py, str(plot), str(compare_csv), "--by-phase", "--metric", "pos_rmse_m",
         "-o", str(args.out), *plot_args_base],
        check=False,
    )

    manifest = args.out / "README.txt"
    manifest.write_text(
        "C.4 desk prep output (desk parallel track §1, audit 2026-09-21)\n"
        f"source: {args.rows}\n"
        f"filter: vehicle={args.vehicle}, phase={args.phase}\n"
        f"figures/summarise: {compare_csv.name} (geometric n=2 vs indi n=2 on 19 Sep A8 — indicative)\n"
        "Indicative only until 3–5 repeat FLIGHTS per controller per scenario (docs/27).\n",
        encoding="utf-8",
    )
    print(f"\nDone. See {args.out}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
