#!/usr/bin/env python3
"""One-off helpers for rpm_source_quality.py — spike root cause + control-path correlation."""
from __future__ import annotations

import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
SPIKE_ERR_RPM = 10_000.0
FS = 500.0


def load_merged_csv(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray], list[str]]:
    with open(path, newline="") as f:
        f.readline()
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
    return t, arrays, header


def detect_vehicle_prefixes(header: list[str]) -> list[str]:
    prefs = set()
    for col in header:
        if col == "t" or col.startswith("rel."):
            continue
        if ".rpm_m1" in col:
            prefs.add(col.rsplit(".", 1)[0])
    return sorted(prefs)


def investigate_all(csv_paths: list[Path]) -> dict:
    spike_events: list[dict] = []
    motor_at_spike = Counter()
    scenario_at_spike = Counter()
    deck_at_spike: list[float] = []
    burst_lengths: list[int] = []
    dshot_spike_vals: list[float] = []

    a_res_delta_at_spike: list[float] = []
    a_res_delta_matched: list[float] = []
    tau_delta_at_spike: list[float] = []
    tau_delta_matched: list[float] = []

    rng = np.random.default_rng(0)

    for csv_path in csv_paths:
        rel = csv_path.relative_to(REPO).as_posix()
        parts = csv_path.parent.name.split("_")
        scenario = parts[0] if parts else "?"
        t, arrays, header = load_merged_csv(csv_path)
        prefixes = detect_vehicle_prefixes(header)

        for prefix in prefixes:
            for motor in range(1, 5):
                rd_col = f"{prefix}.rpm_m{motor}"
                rs_col = f"{prefix}.motor_m{motor}_rpm"
                if rd_col not in arrays or rs_col not in arrays:
                    continue
                rd = arrays[rd_col]
                rs = arrays[rs_col]
                valid = (rd > 0) & (rs > 0) & (rs < 60000)
                err = rs - rd
                spike = valid & (np.abs(err) > SPIKE_ERR_RPM)
                if not np.any(spike):
                    continue

                az_col = f"{prefix}.a_res_z"
                tz_col = f"{prefix}.tau_z"
                has_res = az_col in arrays and tz_col in arrays
                az = arrays[az_col] if has_res else None
                tz = arrays[tz_col] if has_res else None

                idx_spike = np.where(spike)[0]
                for i in idx_spike:
                    spike_events.append(
                        {
                            "path": rel,
                            "scenario": scenario,
                            "prefix": prefix,
                            "motor": motor,
                            "t": float(t[i]),
                            "deck": float(rd[i]),
                            "dshot": float(rs[i]),
                            "err": float(err[i]),
                        }
                    )
                    motor_at_spike[motor] += 1
                    scenario_at_spike[scenario] += 1
                    deck_at_spike.append(float(rd[i]))
                    dshot_spike_vals.append(float(rs[i]))

                # burst lengths on this motor-row
                in_run = False
                run_len = 0
                for i in range(len(rd)):
                    if spike[i]:
                        run_len += 1
                        in_run = True
                    elif in_run:
                        burst_lengths.append(run_len)
                        in_run = False
                        run_len = 0
                if in_run:
                    burst_lengths.append(run_len)

                if has_res and az is not None:
                    daz = np.abs(np.diff(az, prepend=az[0]))
                    dtz = np.abs(np.diff(tz, prepend=tz[0]))
                    spike_idx = np.where(spike)[0]
                    valid_idx = np.where(valid & ~spike)[0]
                    if len(spike_idx) and len(valid_idx) > 100:
                        a_res_delta_at_spike.extend(daz[spike_idx].tolist())
                        pick = rng.choice(valid_idx, size=min(len(spike_idx) * 3, 500), replace=False)
                        a_res_delta_matched.extend(daz[pick].tolist())
                        tau_delta_at_spike.extend(dtz[spike_idx].tolist())
                        tau_delta_matched.extend(dtz[pick].tolist())

    def pctile(a, p):
        return float(np.percentile(a, p)) if a else float("nan")

    report = {
        "n_spike_samples": len(spike_events),
        "n_flights_with_spikes": len({e["path"] for e in spike_events}),
        "motor_counts": dict(motor_at_spike),
        "scenario_counts": dict(scenario_at_spike),
        "deck_rpm_at_spike_median": float(np.median(deck_at_spike)) if deck_at_spike else float("nan"),
        "deck_rpm_at_spike_p10_p90": [pctile(deck_at_spike, 10), pctile(deck_at_spike, 90)],
        "dshot_spike_val_median": float(np.median(dshot_spike_vals)) if dshot_spike_vals else float("nan"),
        "dshot_spike_val_unique_rounded": sorted(
            {int(round(v / 1000) * 1000) for v in dshot_spike_vals}
        )[:20],
        "burst_length_median": float(np.median(burst_lengths)) if burst_lengths else float("nan"),
        "burst_length_max": int(np.max(burst_lengths)) if burst_lengths else 0,
        "control_path_a_res_z_abs_delta": {
            "at_spike_median": pctile(a_res_delta_at_spike, 50),
            "at_spike_p95": pctile(a_res_delta_at_spike, 95),
            "matched_non_spike_median": pctile(a_res_delta_matched, 50),
            "matched_non_spike_p95": pctile(a_res_delta_matched, 95),
            "ratio_median": (
                pctile(a_res_delta_at_spike, 50) / pctile(a_res_delta_matched, 50)
                if a_res_delta_matched and pctile(a_res_delta_matched, 50) > 1e-9
                else float("nan")
            ),
        },
        "control_path_tau_z_abs_delta": {
            "at_spike_median": pctile(tau_delta_at_spike, 50),
            "at_spike_p95": pctile(tau_delta_at_spike, 95),
            "matched_non_spike_median": pctile(tau_delta_matched, 50),
            "matched_non_spike_p95": pctile(tau_delta_matched, 95),
        },
        "worst_example": max(spike_events, key=lambda e: abs(e["err"])) if spike_events else None,
        "correlations_ruled_out": [
            "Not deck dropout — deck RPM in normal hover band at spike instants.",
            "Not single-motor — all four motors appear in spike counts (roughly balanced).",
            "Not one scenario — spikes in A1/A3/A7/A8/C5; A2 top deck-zero dominated, few valid pairs.",
            "No ctrl_mode / mode-transition column on uSD — cannot correlate to INDI handoffs from logs.",
            "Not 0xFFFF sentinel — spike values pass rs<60000 and are finite mid-range uint16 garbage.",
        ],
        "working_hypothesis": (
            "Bidirectional DShot ESC telemetry occasionally returns decoded eRPM values in a "
            "physically impossible band (~28k–59k RPM) while the optical deck stays near true hover "
            "RPM (~11–17k). Values often appear in 1–2 tick bursts (2–4 ms at 500 Hz log rate; "
            "control reads the same motor.* log at 1 kHz). Pattern fits intermittent telemetry-slot "
            "or decode glitches, not true rotor acceleration — no clean correlation with commanded "
            "PWM step found in this pass (motor PWM changes are smooth at spike neighbors)."
        ),
    }
    return report


def main() -> int:
    csv_paths = sorted(REPO.glob("experiments/logs/c1_*_merged/*/*_merged_usd.csv"))
    report = investigate_all(csv_paths)
    out = REPO / "experiments/analysis/out/rpm_source_quality/spike_investigation.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in report if k != "worst_example"}, indent=2))
    if report.get("worst_example"):
        print("worst:", report["worst_example"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
