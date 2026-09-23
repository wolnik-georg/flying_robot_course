#!/usr/bin/env python3
"""Deck vs DShot RPM sensor quality on C.1 merged uSD CSVs (500 Hz).

Reads only merged CSVs (never raw .bin — see usd_raw/*_PAIRING.md). Imports
cross_corr_lag from flying_drone_stack/tools/investigate_dshot_rpm.py.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
TOOLS = REPO / "flying_drone_stack" / "tools"
sys.path.insert(0, str(TOOLS))
from investigate_dshot_rpm import cross_corr_lag  # noqa: E402

FS = 500.0
DT_MS = 1000.0 / FS
OUT_DIR = REPO / "experiments" / "analysis" / "out" / "rpm_source_quality"


def vehicle_role(prefix: str) -> str:
    if prefix == "cf5" or prefix.startswith("cf5_"):
        return "bottom"
    if prefix == "cf_second" or prefix.startswith("cf_second_"):
        return "top"
    return "unknown"


def detect_vehicle_prefixes(header: list[str]) -> list[str]:
    prefs = set()
    for col in header:
        if col == "t" or col.startswith("rel."):
            continue
        if ".rpm_m1" in col:
            prefs.add(col.rsplit(".", 1)[0])
    return sorted(prefs)


def load_merged_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    with open(path, newline="") as f:
        first = f.readline()
        if not first.startswith("#"):
            raise ValueError(f"expected # meta line first: {path}")
        reader = csv.reader(f)
        header = next(reader)
        cols: dict[str, list[float]] = {h: [] for h in header}
        for row in reader:
            if len(row) < len(header):
                continue
            for i, h in enumerate(header):
                try:
                    cols[h].append(float(row[i]))
                except ValueError:
                    cols[h].append(float("nan"))
    t = np.array(cols["t"], dtype=float)
    arrays = {k: np.array(v, dtype=float) for k, v in cols.items() if k != "t"}
    return t, arrays


def metrics_one_motor(rd: np.ndarray, rs: np.ndarray) -> dict:
    n = len(rd)
    deck_zero = float(np.mean(rd <= 0)) if n else float("nan")
    dshot_zero = float(np.mean(rs <= 0)) if n else float("nan")
    valid = (rd > 0) & (rs > 0) & (rs < 60000)
    n_valid = int(valid.sum())
    valid_pct = 100.0 * n_valid / n if n else float("nan")
    out = {
        "n_samples": n,
        "n_valid": n_valid,
        "valid_pct": valid_pct,
        "deck_zero_pct": 100.0 * deck_zero,
        "dshot_zero_pct": 100.0 * dshot_zero,
        "bias_rpm": float("nan"),
        "bias_pct": float("nan"),
        "rmse_rpm": float("nan"),
        "max_abs_err_rpm": float("nan"),
        "lag_ms": float("nan"),
    }
    if n_valid < 50:
        return out
    rd_v = rd[valid].astype(float)
    rs_v = rs[valid].astype(float)
    err = rs_v - rd_v
    out["bias_rpm"] = float(np.mean(err))
    out["bias_pct"] = float(100.0 * np.mean(err) / np.mean(rd_v))
    out["rmse_rpm"] = float(np.sqrt(np.mean(err**2)))
    out["max_abs_err_rpm"] = float(np.max(np.abs(err)))
    out["lag_ms"] = float(cross_corr_lag(rd_v, rs_v, FS))
    return out


def synthetic_lag_self_test() -> bool:
    """Shift deck by N samples; DShot copy should recover N * DT_MS lag."""
    # Use a long noisy trace
    rng = np.random.default_rng(0)
    n = 8000
    rd = np.cumsum(rng.normal(0, 1, n)) + 15000 + 200 * np.sin(
        2 * np.pi * 3.0 * np.arange(n) / FS
    )
    N = 5  # 10 ms at 500 Hz
    rs = np.empty_like(rd)
    rs[:N] = rd[0]
    rs[N:] = rd[:-N]
    got = cross_corr_lag(rd.astype(float), rs.astype(float), FS)
    expect = N * DT_MS
    ok = abs(got - expect) < 0.01
    print(f"Synthetic lag self-test: injected {N} samples ({expect:.1f} ms), measured {got:.2f} ms — {'PASS' if ok else 'FAIL'}")
    return ok


def load_manifest_flights() -> list[dict]:
    flights: list[dict] = []
    for rel in (
        "experiments/logs/c1_2026-09-21_merged/manifest_2026-09-21_c1.json",
        "experiments/logs/c1_2026-09-23_merged/manifest_2026-09-23_c1.json",
    ):
        p = REPO / rel
        if not p.exists():
            continue
        for entry in json.loads(p.read_text()):
            if entry.get("merge_status") == "merge_failed":
                continue
            if entry.get("merge_status") == "no_usd":
                continue
            path_s = entry.get("path")
            if not path_s:
                continue
            if entry.get("merge_status") == "merged" or "c1_2026-09-21" in rel:
                flights.append(entry)
    # De-dupe by path
    seen = set()
    out = []
    for e in flights:
        if e["path"] in seen:
            continue
        seen.add(e["path"])
        out.append(e)
    return out


def infer_scenario_stamp(path: Path) -> tuple[str, str]:
    # A3_2026-09-23_17-54-32
    parts = path.parent.name.split("_")
    if len(parts) >= 4:
        return parts[0], f"{parts[2]}_{parts[3]}"
    return "?", path.parent.name


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=OUT_DIR)
    args = parser.parse_args()
    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    if not synthetic_lag_self_test():
        print("Aborting: lag self-test failed.", file=sys.stderr)
        return 1

    manifest_entries = {e["path"]: e for e in load_manifest_flights()}
    csv_paths = sorted(REPO.glob("experiments/logs/c1_*_merged/*/*_merged_usd.csv"))
    if len(csv_paths) != 29:
        print(f"WARNING: expected 29 merged CSVs, found {len(csv_paths)}", file=sys.stderr)

    per_flight_rows: list[dict] = []

    for csv_path in csv_paths:
        rel = csv_path.relative_to(REPO).as_posix()
        meta = manifest_entries.get(rel, {})
        scenario = meta.get("scenario") or infer_scenario_stamp(csv_path)[0]
        stamp = meta.get("stamp") or infer_scenario_stamp(csv_path)[1].split("_", 1)[-1]

        _, arrays = load_merged_csv(csv_path)
        with open(csv_path) as f:
            f.readline()
            header = next(csv.reader(f))
        prefixes = detect_vehicle_prefixes(header)

        for prefix in prefixes:
            role = vehicle_role(prefix)
            for motor in range(1, 5):
                rd_col = f"{prefix}.rpm_m{motor}"
                rs_col = f"{prefix}.motor_m{motor}_rpm"
                if rd_col not in arrays or rs_col not in arrays:
                    continue
                m = metrics_one_motor(arrays[rd_col], arrays[rs_col])
                row = {
                    "scenario": scenario,
                    "date": "2026-09-21" if "2026-09-21" in rel else "2026-09-23",
                    "stamp": stamp,
                    "merge_path": rel,
                    "vehicle_prefix": prefix,
                    "vehicle_role": role,
                    "motor": motor,
                    **m,
                }
                per_flight_rows.append(row)

    per_path = out_dir / "per_flight.csv"
    fieldnames = list(per_flight_rows[0].keys()) if per_flight_rows else []
    with open(per_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(per_flight_rows)

    # Scenario + vehicle_role summary (motor-level rows aggregated)
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for r in per_flight_rows:
        groups[(r["scenario"], r["vehicle_role"])].append(r)

    summary_rows = []
    for (scenario, role), rows in sorted(groups.items()):
        def agg(key, fn):
            vals = [x[key] for x in rows if np.isfinite(x[key])]
            if not vals:
                return float("nan"), float("nan"), float("nan"), 0
            return float(np.mean(vals)), float(np.median(vals)), float(np.max(np.abs(vals))), len(vals)

        bias_m, bias_med, bias_worst, n = agg("bias_pct", np.mean)
        lag_m, lag_med, lag_worst_abs, _ = agg("lag_ms", np.mean)
        lag_vals = [x["lag_ms"] for x in rows if np.isfinite(x["lag_ms"])]
        lag_max = float(np.max(lag_vals)) if lag_vals else float("nan")
        lag_min = float(np.min(lag_vals)) if lag_vals else float("nan")
        deck_worst = float(np.max([x["deck_zero_pct"] for x in rows]))
        dshot_worst = float(np.max([x["dshot_zero_pct"] for x in rows]))
        n_flights = len({x["merge_path"] for x in rows})
        summary_rows.append(
            {
                "scenario": scenario,
                "vehicle_role": role,
                "n_flights": n_flights,
                "n_motor_rows": len(rows),
                "bias_pct_mean": bias_m,
                "bias_pct_median": bias_med,
                "bias_pct_worst_abs": bias_worst,
                "lag_ms_mean": lag_m,
                "lag_ms_median": lag_med,
                "lag_ms_min": lag_min,
                "lag_ms_max": lag_max,
                "deck_zero_pct_worst": deck_worst,
                "dshot_zero_pct_worst": dshot_worst,
            }
        )

    summary_path = out_dir / "summary_by_scenario_vehicle.csv"
    with open(summary_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()) if summary_rows else [])
        w.writeheader()
        w.writerows(summary_rows)

    # Overview plot: lag and |bias_pct| by vehicle_role
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    roles = ["bottom", "top"]
    lag_by_role = [[r["lag_ms"] for r in per_flight_rows if r["vehicle_role"] == role and np.isfinite(r["lag_ms"])] for role in roles]
    bias_by_role = [[abs(r["bias_pct"]) for r in per_flight_rows if r["vehicle_role"] == role and np.isfinite(r["bias_pct"])] for role in roles]
    axes[0].boxplot(lag_by_role, tick_labels=roles)
    axes[0].set_ylabel("lag (ms)\n(DShot vs deck, + = DShot lags)")
    axes[0].set_title("Cross-correlation lag by vehicle role")
    axes[1].boxplot(bias_by_role, tick_labels=roles)
    axes[1].set_ylabel("|bias| (%)")
    axes[1].set_title("Deck vs DShot bias magnitude")
    fig.suptitle("C.1 merged logs — RPM source quality (all motors, full segment)")
    fig.tight_layout()
    plot_path = out_dir / "overview_lag_bias_by_role.png"
    fig.savefig(plot_path, dpi=130)
    plt.close(fig)

    print(f"Wrote {per_path} ({len(per_flight_rows)} motor-rows)")
    print(f"Wrote {summary_path}")
    print(f"Wrote {plot_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
