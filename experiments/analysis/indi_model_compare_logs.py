#!/usr/bin/env python3
"""Oct-02 A1 log stats by controller variant (radio CSV; batch-level uSD optional)."""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
OUT = Path(__file__).resolve().parent / "out" / "indi_model_compare"
OUT.mkdir(parents=True, exist_ok=True)

STEADY_T0 = 6.0
STEADY_T1 = 13.0
LIFTOFF_Z = 0.25


def load_meta_and_data(path: Path) -> tuple[dict[str, str], dict[str, np.ndarray]]:
    meta: dict[str, str] = {}
    rows: list[list[float]] = []
    header: list[str] | None = None
    with open(path) as f:
        for line in f:
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
            elif line.startswith("time_s,"):
                header = line.strip().split(",")
            elif header and not line.startswith("#"):
                parts = line.strip().split(",")
                if len(parts) == len(header):
                    rows.append([float(x) for x in parts])
    if not header or not rows:
        raise ValueError(f"empty csv {path}")
    data = {h: np.array([r[i] for r in rows]) for i, h in enumerate(header)}
    return meta, data


def variant_label(meta: dict[str, str]) -> str:
    c = meta.get("controller", "?")
    if c == "9":
        return "Omar C"
    if c == "10":
        return "Omar Rust"
    if c == "6":
        cm = meta.get("ctrl_mode", meta.get("shared_ctrl_mode", "?"))
        return f"Ours c=6 ctrl_mode={cm}"
    return f"controller={c}"


def steady_mask(t: np.ndarray, z: np.ndarray) -> np.ndarray:
    t0 = t[0]
    rel = t - t0
    return (rel >= STEADY_T0) & (rel <= STEADY_T1) & (z > LIFTOFF_Z)


def analyze_file(path: Path) -> dict:
    meta, d = load_meta_and_data(path)
    m = steady_mask(d["time_s"], d["pos_z"])
    if np.sum(m) < 20:
        m = (d["time_s"] - d["time_s"][0] >= STEADY_T0) & (d["time_s"] - d["time_s"][0] <= STEADY_T1)
    gx, gy, gz = d["gyro_x"][m], d["gyro_y"][m], d["gyro_z"][m]
    gmag = np.sqrt(gx**2 + gy**2 + gz**2)
    tau = np.sqrt(d["tau_x"][m] ** 2 + d["tau_y"][m] ** 2 + d["tau_z"][m] ** 2)
    thrust = d["thrust"][m]
    a_res = np.sqrt(d["a_res_x"][m] ** 2 + d["a_res_y"][m] ** 2 + d["a_res_z"][m] ** 2)

    return {
        "file": path.name,
        "variant": variant_label(meta),
        "controller_meta": meta.get("controller"),
        "ctrl_mode_meta": meta.get("ctrl_mode"),
        "n_steady": int(np.sum(m)),
        "gyro_rms_deg_s": float(np.sqrt(np.mean(gmag**2))),
        "gyro_x_rms": float(np.sqrt(np.mean(gx**2))),
        "tau_abs_max": float(np.max(tau)),
        "tau_abs_mean": float(np.mean(tau)),
        "tau_near_zero_frac": float(np.mean(tau < 1e-6)),
        "thrust_mean": float(np.mean(thrust)),
        "thrust_std": float(np.std(thrust)),
        "a_res_rms": float(np.sqrt(np.mean(a_res**2))),
        "roll_std_deg": float(np.std(d["roll"][m])),
    }


def aggregate_by_variant(per_flight: list[dict]) -> list[dict]:
    from collections import defaultdict

    buckets: dict[str, list[dict]] = defaultdict(list)
    for row in per_flight:
        key = row["variant"]
        if "Omar C" in key:
            key = "Omar C"
        elif "Omar Rust" in key:
            key = "Omar Rust"
        elif "Ours" in key or row.get("controller_meta") == "6":
            key = "Ours (controller=6)"
        buckets[key].append(row)

    out = []
    for k, rows in sorted(buckets.items()):
        out.append(
            {
                "variant": k,
                "n_flights": len(rows),
                "gyro_rms_deg_s_mean": float(np.mean([r["gyro_rms_deg_s"] for r in rows])),
                "gyro_rms_deg_s_min_max": [
                    float(min(r["gyro_rms_deg_s"] for r in rows)),
                    float(max(r["gyro_rms_deg_s"] for r in rows)),
                ],
                "tau_abs_max_mean": float(np.mean([r["tau_abs_max"] for r in rows])),
                "a_res_rms_mean": float(np.mean([r["a_res_rms"] for r in rows])),
                "files": [r["file"] for r in rows],
            }
        )
    return out


def main() -> None:
    paths = sorted(LOGS.glob("A1_cf5_2026-10-02_*.csv"))
    per_flight = []
    for p in paths:
        try:
            per_flight.append(analyze_file(p))
        except ValueError:
            continue
    summary = aggregate_by_variant(per_flight)
    payload = {"per_flight": per_flight, "by_variant": summary}
    out_json = OUT / "log_stats_a1_oct02.json"
    out_json.write_text(json.dumps(payload, indent=2))
    print(f"Wrote {out_json} ({len(per_flight)} flights)")


if __name__ == "__main__":
    main()
