#!/usr/bin/env python3
"""Hardware A8 network-off reference stats (merged logs, outside crossings)."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ns2_closed_loop_sil_metrics import find_crossing_times, steady_mask

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments/logs"

HW_A8_OFF = (
    "merged_A8_rnn0_2026-10-05_17-39-27.csv",
    "merged_A8_rnn0_2026-10-05_17-41-09.csv",
)

HW_TOP_TILT_MAX_DEG = 2.7
SIL_TOP_TILT_MARGIN_DEG = 5.0


def _load_merged(path: Path) -> dict[str, np.ndarray]:
    header: list[str] | None = None
    rows: list[list[float]] = []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            if header is None:
                header = line.strip().split(",")
            else:
                rows.append([float(x) for x in line.strip().split(",")])
    cols = {n: np.array([r[i] for r in rows], float) for i, n in enumerate(header or [])}
    return cols


def flight_stats(merged_name: str, *, takeoff_s: float = 0.0) -> dict:
    path = LOGS / merged_name
    c = _load_merged(path)
    t = c["t"]
    out = {"merged": merged_name}
    for prefix, label in (("cf5", "bottom"), ("cf_second", "top")):
        x, y, z = c[f"{prefix}.x"], c[f"{prefix}.y"], c[f"{prefix}.z"]
        cx, cy, cz = c[f"{prefix}.ctrltarget_x"], c[f"{prefix}.ctrltarget_y"], c[f"{prefix}.ctrltarget_z"]
        lat = np.hypot(x - cx, y - cy)
        ez = (z - cz) * 100
        pos = np.stack([x, y, z], axis=1)
        m = steady_mask(t, pos, scenario="A8", takeoff_s=takeoff_s, n_crossings=16)
        roll = np.abs(c[f"{prefix}.roll_deg"])
        pitch = np.abs(c[f"{prefix}.pitch_deg"])
        tilt = np.maximum(roll, pitch)
        out[label] = {
            "lat_rms_cm": float(np.sqrt(np.mean(lat[m] ** 2)) * 100),
            "mean_z_err_cm": float(np.mean(ez[m])),
            "rms_z_cm": float(np.sqrt(np.mean(ez[m] ** 2))),
            "max_tilt_deg": float(np.max(tilt[m])),
        }
    return out


def hardware_summary() -> dict:
    flights = [flight_stats(n) for n in HW_A8_OFF]
    bot_lat = [f["bottom"]["lat_rms_cm"] for f in flights]
    top_lat = [f["top"]["lat_rms_cm"] for f in flights]
    top_tilt = [f["top"]["max_tilt_deg"] for f in flights]
    return {
        "flights": flights,
        "reference": {
            "bottom_lat_rms_cm": float(np.mean(bot_lat)),
            "top_lat_rms_cm": float(np.mean(top_lat)),
            "top_max_tilt_deg": float(np.max(top_tilt)),
        },
    }


if __name__ == "__main__":
    import json

    print(json.dumps(hardware_summary(), indent=2))
