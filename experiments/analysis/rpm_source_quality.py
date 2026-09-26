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


def _injected_shift_lag(rd: np.ndarray, N: int) -> tuple[float, float]:
    """Build DShot trace delayed by N samples vs deck; return (expected_ms, measured_ms)."""
    rs = np.empty_like(rd)
    rs[:N] = rd[0]
    rs[N:] = rd[:-N]
    expect = N * DT_MS
    got = float(cross_corr_lag(rd.astype(float), rs.astype(float), FS))
    return expect, got


def synthetic_lag_self_test() -> bool:
    """Shift deck by N samples; DShot copy should recover N * DT_MS lag."""
    rng = np.random.default_rng(0)
    n = 8000
    rd = np.cumsum(rng.normal(0, 1, n)) + 15000 + 200 * np.sin(
        2 * np.pi * 3.0 * np.arange(n) / FS
    )
    N = 5  # 10 ms at 500 Hz
    expect, got = _injected_shift_lag(rd, N)
    ok = abs(got - expect) < 0.01
    print(
        f"Synthetic lag self-test (noise): injected {N} samples ({expect:.1f} ms), "
        f"measured {got:.2f} ms — {'PASS' if ok else 'FAIL'}"
    )
    return ok


def synthetic_lag_self_test_real_deck() -> bool:
    """Same injected shift on cf5.rpm_m1 from a real 23-Sep A1 merged CSV."""
    csv_path = (
        REPO
        / "experiments/logs/c1_2026-09-23_merged/A1_2026-09-23_17-17-26/A1_2026-09-23_17-17-26_merged_usd.csv"
    )
    if not csv_path.exists():
        print(f"Real-deck lag self-test: missing {csv_path} — FAIL")
        return False
    _, arrays = load_merged_csv(csv_path)
    col = "cf5.rpm_m1"
    if col not in arrays:
        print(f"Real-deck lag self-test: no {col} in merge — FAIL")
        return False
    rd = arrays[col].astype(float)
    valid = rd > 0
    if valid.sum() < 2000:
        print(f"Real-deck lag self-test: too few valid deck samples ({valid.sum()}) — FAIL")
        return False
    rd = rd[valid]
    N = 5
    expect, got = _injected_shift_lag(rd, N)
    ok = abs(got - expect) < 0.01
    print(
        f"Synthetic lag self-test (real cf5.rpm_m1, A1 17-17-26): injected {N} samples "
        f"({expect:.1f} ms), measured {got:.2f} ms — {'PASS' if ok else 'FAIL'}"
    )
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


def rolling_cross_corr_lag(
    deck: np.ndarray,
    dshot: np.ndarray,
    t: np.ndarray,
    fs: float = FS,
    window_s: float = 2.0,
    step_s: float = 0.5,
) -> np.ndarray:
    """Windowed cross-correlation lags; each row is (window_center_time_s, lag_ms).

    Uses the same valid mask and cross_corr_lag() as metrics_one_motor(), applied
    independently per window (default 2.0 s window, 0.5 s step at 500 Hz).
    """
    n = len(deck)
    win_n = max(int(round(window_s * fs)), 1)
    step_n = max(int(round(step_s * fs)), 1)
    t0 = float(t[0]) if n else 0.0
    rows: list[tuple[float, float]] = []
    for start in range(0, max(n - win_n + 1, 0), step_n):
        end = start + win_n
        rd = deck[start:end]
        rs = dshot[start:end]
        valid = (rd > 0) & (rs > 0) & (rs < 60000)
        center = float(t[start + win_n // 2] - t0)
        if int(valid.sum()) < 50:
            rows.append((center, float("nan")))
            continue
        lag = float(cross_corr_lag(rd[valid].astype(float), rs[valid].astype(float), fs))
        rows.append((center, lag))
    if not rows:
        return np.zeros((0, 2))
    return np.array(rows, dtype=float)


def plot_rpm_overlay(
    t: np.ndarray,
    deck: np.ndarray,
    dshot: np.ndarray,
    out_path: Path,
    *,
    title: str,
    motor_label: str,
) -> None:
    """Deck vs DShot RPM on shared time base; highlight deck<=0 samples."""
    t = t - t[0]
    fig, ax = plt.subplots(figsize=(12, 4))
    ymax = float(np.nanmax(dshot[dshot > 0])) if np.any(dshot > 0) else 1.0
    ymax = max(ymax, float(np.nanmax(deck[deck > 0])) if np.any(deck > 0) else ymax)
    deck_zero = deck <= 0
    if np.any(deck_zero):
        ax.fill_between(
            t,
            0,
            ymax * 1.05,
            where=deck_zero,
            color="0.85",
            alpha=0.55,
            label="deck ≤ 0 (shaded)",
            zorder=0,
        )
        ax.plot(
            t[deck_zero],
            np.zeros(deck_zero.sum()),
            "x",
            color="0.45",
            ms=2,
            alpha=0.35,
            label="deck zero samples",
            zorder=1,
        )
    ax.plot(t, deck, lw=0.7, color="C0", alpha=0.9, label="deck RPM")
    ax.plot(t, dshot, lw=0.7, color="C1", alpha=0.85, label="DShot RPM")
    ax.set_xlabel("time since flight start (s)")
    ax.set_ylabel("RPM")
    ax.set_title(f"{title} — {motor_label}")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(t[0], t[-1])
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_rolling_lag(
    t: np.ndarray,
    deck: np.ndarray,
    dshot: np.ndarray,
    out_path: Path,
    *,
    title: str,
    motor_label: str,
    global_lag_ms: float | None = None,
    window_s: float = 2.0,
    step_s: float = 0.5,
) -> None:
    roll = rolling_cross_corr_lag(deck, dshot, t, FS, window_s, step_s)
    fig, ax = plt.subplots(figsize=(12, 3.5))
    if roll.size:
        ax.plot(roll[:, 0], roll[:, 1], "o-", ms=3, lw=1, color="C2", label="rolling lag")
    if global_lag_ms is not None and np.isfinite(global_lag_ms):
        ax.axhline(
            global_lag_ms,
            color="C3",
            ls="--",
            lw=1,
            label=f"flight-wide lag_ms = {global_lag_ms:.1f}",
        )
    ax.axhline(0, color="0.7", lw=0.5)
    ax.set_xlabel("window center time (s)")
    ax.set_ylabel("lag (ms)\n(+ = DShot lags deck)")
    ax.set_title(f"Rolling lag ({window_s}s window, {step_s}s step) — {title} — {motor_label}")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


# Top-3 |lag_ms| flights excluding A2 (from per_flight.csv); grid uses top vehicle.
GRID4_LAG_FLIGHTS = (
    ("A1", "17-17-26", "experiments/logs/c1_2026-09-23_merged/A1_2026-09-23_17-17-26/A1_2026-09-23_17-17-26_merged_usd.csv", "cf_second"),
    ("A7", "19-11-19", "experiments/logs/c1_2026-09-23_merged/A7_2026-09-23_19-11-19/A7_2026-09-23_19-11-19_merged_usd.csv", "cf_second"),
    ("A3", "13-04-34", "experiments/logs/c1_2026-09-21_merged/A3_2026-09-21_13-04-34/A3_2026-09-21_13-04-34_merged_usd.csv", "cf_second_A3_13-04-34"),
)


def _plot_rpm_pair_on_ax(
    ax,
    t: np.ndarray,
    deck: np.ndarray,
    dshot: np.ndarray,
    *,
    subplot_title: str,
) -> tuple:
    """Draw deck/DShot on ax; return line artists for figure legend."""
    t = t - t[0]
    ymax = float(np.nanmax(dshot[dshot > 0])) if np.any(dshot > 0) else 1.0
    ymax = max(ymax, float(np.nanmax(deck[deck > 0])) if np.any(deck > 0) else ymax)
    deck_zero = deck <= 0
    if np.any(deck_zero):
        ax.fill_between(
            t, 0, ymax * 1.05, where=deck_zero, color="0.85", alpha=0.45, zorder=0,
        )
    line_deck, = ax.plot(t, deck, lw=0.6, color="C0", alpha=0.9)
    line_dshot, = ax.plot(t, dshot, lw=0.6, color="C1", alpha=0.85)
    ax.set_title(subplot_title, fontsize=9)
    ax.set_xlim(t[0], t[-1])
    ax.tick_params(labelsize=7)
    return line_deck, line_dshot


def plot_rpm_grid4(
    merge_rel: str,
    prefix: str,
    out_path: Path,
    per_flight_rows: list[dict],
    scenario: str,
    stamp: str,
    vehicle_role_name: str,
) -> None:
    """2×2 deck vs DShot for m1–m4 on one vehicle."""
    csv_path = REPO / merge_rel
    t, arrays = load_merged_csv(csv_path)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    legend_lines = None
    for motor, ax in zip(range(1, 5), axes.flat):
        rd = arrays[f"{prefix}.rpm_m{motor}"]
        rs = arrays[f"{prefix}.motor_m{motor}_rpm"]
        meta = next(
            (
                r
                for r in per_flight_rows
                if r["scenario"] == scenario
                and r["stamp"] == stamp
                and r["vehicle_role"] == vehicle_role_name
                and int(r["motor"]) == motor
            ),
            None,
        )
        if meta:
            st = (
                f"m{motor}: bias {meta['bias_pct']:+.2f}%  "
                f"lag {meta['lag_ms']:+.1f} ms"
            )
        else:
            st = f"m{motor}"
        lines = _plot_rpm_pair_on_ax(ax, t, rd, rs, subplot_title=st)
        if legend_lines is None:
            legend_lines = lines
        if motor in (3, 4):
            ax.set_xlabel("time since flight start (s)", fontsize=8)
        if motor in (1, 3):
            ax.set_ylabel("RPM", fontsize=8)
    fig.suptitle(f"{scenario} {stamp} — {prefix} (deck vs DShot, 500 Hz)", fontsize=11)
    fig.legend(
        legend_lines,
        ["deck RPM", "DShot RPM"],
        loc="upper center",
        ncol=2,
        fontsize=9,
        bbox_to_anchor=(0.5, 1.02),
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def flight_summary_table_md(per_flight_rows: list[dict]) -> str:
    """Markdown table: one row per flight (all motors), A2 excluded."""
    from statistics import median

    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in per_flight_rows:
        if r["scenario"] == "A2":
            continue
        groups[(r["scenario"], r["stamp"])].append(r)

    rows_out = []
    for (scenario, stamp), motors in sorted(groups.items()):
        abs_bias = [abs(float(m["bias_pct"])) for m in motors if np.isfinite(m["bias_pct"])]
        lags = [float(m["lag_ms"]) for m in motors if np.isfinite(m["lag_ms"])]
        abs_lags = [abs(x) for x in lags]
        rows_out.append(
            {
                "scenario": scenario,
                "flight": stamp,
                "mean_abs_bias": float(np.mean(abs_bias)) if abs_bias else float("nan"),
                "mean_lag": float(np.mean(abs_lags)) if abs_lags else float("nan"),
                "max_abs_lag": float(np.max(abs_lags)) if abs_lags else float("nan"),
                "deck_drop": float(np.max([m["deck_zero_pct"] for m in motors])),
                "dshot_drop": float(np.max([m["dshot_zero_pct"] for m in motors])),
            }
        )

    def fmt(x, nd=2):
        return f"{x:.{nd}f}" if np.isfinite(x) else "—"

    lines = [
        "| Scenario | Flight | Mean |bias| % | Mean |lag| (ms) | Max |lag| (ms) | Deck dropout % | DShot dropout % |",
        "|----------|--------|---------------|---------------|----------------|-----------------|-----------------|",
    ]
    for r in rows_out:
        lines.append(
            f"| {r['scenario']} | {r['flight']} | {fmt(r['mean_abs_bias'], 3)} | "
            f"{fmt(r['mean_lag'], 1)} | {fmt(r['max_abs_lag'], 1)} | "
            f"{fmt(r['deck_drop'], 1)} | {fmt(r['dshot_drop'], 1)} |"
        )
    mb = [r["mean_abs_bias"] for r in rows_out]
    ml = [r["mean_lag"] for r in rows_out]
    mal = [r["max_abs_lag"] for r in rows_out]
    lines.append(
        f"| **All flights (mean)** | — | **{fmt(np.mean(mb), 3)}** | "
        f"**{fmt(np.mean(ml), 1)}** | **{fmt(np.mean(mal), 1)}** | — | — |"
    )
    lines.append(
        f"| **All flights (median)** | — | **{fmt(median(mb), 3)}** | "
        f"**{fmt(median(ml), 1)}** | **{fmt(median(mal), 1)}** | — | — |"
    )
    return "\n".join(lines)


def write_grid_and_flight_summary(out_dir: Path, per_flight_rows: list[dict]) -> str:
    for scenario, stamp, merge_rel, prefix in GRID4_LAG_FLIGHTS:
        role = vehicle_role(prefix)
        out = out_dir / f"grid4_{scenario}_{stamp}.png"
        plot_rpm_grid4(merge_rel, prefix, out, per_flight_rows, scenario, stamp, role)
    md = flight_summary_table_md(per_flight_rows)
    (out_dir / "flight_summary_table.md").write_text(md + "\n")
    return md


def write_extended_visualizations(out_dir: Path, per_flight_rows: list[dict]) -> None:
    """Overlay + rolling-lag plots for representative flights (additive outputs)."""

    def row_key(scenario: str, stamp: str, role: str, motor: int) -> dict | None:
        for r in per_flight_rows:
            if (
                r["scenario"] == scenario
                and r["stamp"] == stamp
                and r["vehicle_role"] == role
                and r["motor"] == motor
            ):
                return r
        return None

    def run_case(
        merge_rel: str,
        prefix: str,
        motor: int,
        overlay_name: str,
        rolling_name: str,
        title: str,
    ) -> None:
        csv_path = REPO / merge_rel
        t, arrays = load_merged_csv(csv_path)
        rd_col = f"{prefix}.rpm_m{motor}"
        rs_col = f"{prefix}.motor_m{motor}_rpm"
        rd = arrays[rd_col]
        rs = arrays[rs_col]
        motor_label = f"{prefix} motor m{motor}"
        plot_rpm_overlay(t, rd, rs, out_dir / overlay_name, title=title, motor_label=motor_label)
        meta = row_key(title.split()[0], title.split()[1], vehicle_role(prefix), motor)
        glag = meta["lag_ms"] if meta else float("nan")
        plot_rolling_lag(
            t,
            rd,
            rs,
            out_dir / rolling_name,
            title=title,
            motor_label=motor_label,
            global_lag_ms=glag,
        )

    # (a) A2 top dropout — m3 worst deck-zero % on 19-27-03
    run_case(
        "experiments/logs/c1_2026-09-23_merged/A2_2026-09-23_19-27-03/A2_2026-09-23_19-27-03_merged_usd.csv",
        "cf_second",
        3,
        "overlay_A2_19-27-03_m3.png",
        "rolling_lag_A2_19-27-03_cf_second_m3.png",
        "A2 19-27-03",
    )
    # (b) clean baseline — A3 bottom, near-zero bias on m2
    run_case(
        "experiments/logs/c1_2026-09-21_merged/A3_2026-09-21_13-00-57/A3_2026-09-21_13-00-57_merged_usd.csv",
        "cf5_A3_13-00-57",
        2,
        "overlay_A3_13-00-57_cf5_m2.png",
        "rolling_lag_A3_13-00-57_cf5_m2.png",
        "A3 13-00-57",
    )
    # Large global lag cases from existing table
    run_case(
        "experiments/logs/c1_2026-09-23_merged/A1_2026-09-23_17-17-26/A1_2026-09-23_17-17-26_merged_usd.csv",
        "cf_second",
        3,
        "overlay_A1_17-17-26_cf_second_m3.png",
        "rolling_lag_A1_17-17-26_cf_second_m3.png",
        "A1 17-17-26",
    )
    run_case(
        "experiments/logs/c1_2026-09-23_merged/A7_2026-09-23_19-11-19/A7_2026-09-23_19-11-19_merged_usd.csv",
        "cf_second",
        3,
        "overlay_A7_19-11-19_cf_second_m3.png",
        "rolling_lag_A7_19-11-19_cf_second_m3.png",
        "A7 19-11-19",
    )


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

    if not synthetic_lag_self_test() or not synthetic_lag_self_test_real_deck():
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

    write_extended_visualizations(out_dir, per_flight_rows)
    write_grid_and_flight_summary(out_dir, per_flight_rows)

    print(f"Wrote {per_path} ({len(per_flight_rows)} motor-rows)")
    print(f"Wrote {summary_path}")
    print(f"Wrote {plot_path}")
    print(f"Wrote extended overlay + rolling-lag PNGs under {out_dir}/")
    print(f"Wrote grid4_*.png and flight_summary_table.md under {out_dir}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
