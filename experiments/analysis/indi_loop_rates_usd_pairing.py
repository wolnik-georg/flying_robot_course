#!/usr/bin/env python3
"""Pair Oct-02 A1 cf5 radio CSVs to uSD bins (trajectory + session-order heuristic).

Correlation alone cannot separate repeated A1 runs (find_flight_window.py NOTE).
Secondary keys: recording span ~16 s, thesisNN order within copy-time batch,
|usd_start_s − lag| where available.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
USD_RAW = REPO / "experiments" / "logs" / "usd_raw"
LOGS = REPO / "experiments" / "logs"
OUT = Path(__file__).resolve().parent / "out" / "indi_loop_rates"

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

from decode_usd_log import load  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402

RADIO = [
    ("Omar C", "A1_cf5_2026-10-02_18-36-18.csv", "cf5_thesis60_2026-10-02_18-40-29.bin"),
    ("Omar C", "A1_cf5_2026-10-02_18-37-33.csv", "cf5_thesis61_2026-10-02_18-40-29.bin"),
    ("Omar Rust", "A1_cf5_2026-10-02_18-50-20.csv", "cf5_thesis64_2026-10-02_18-53-09.bin"),
    ("Ours", "A1_cf5_2026-10-02_19-13-24.csv", "cf5_thesis71_2026-10-02_19-18-55.bin"),
    ("Ours", "A1_cf5_2026-10-02_19-15-47.csv", "cf5_thesis72_2026-10-02_19-18-55.bin"),
]

ROLE = "bottom"


def radio_meta(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    with open(path) as f:
        for line in f:
            if line.startswith("# meta:"):
                k, _, v = line[7:].partition("=")
                meta[k.strip()] = v.strip()
    return meta


def meta_json_for_radio(csv_name: str) -> Path:
    m = re.match(r"A1_cf5_(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})\.csv", csv_name)
    return LOGS / f"A1_{m.group(1)}.meta.json"


def score_bin(bin_path: Path, meta: dict) -> dict | None:
    d = load(str(bin_path))
    t = np.asarray(d["t"], dtype=float)
    total = float(meta["duration"])
    if t[-1] - t[0] < total:
        return None
    pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
    ts, cmd = commanded_trajectory(meta, ROLE)
    lag, mse, runner_up = find_offset(t, pos, ts, cmd, t[0], t[-1] - total)
    if lag is None:
        return None
    ratio = None
    if runner_up is not None and mse > 0:
        ratio = float(runner_up[1] / mse)
    return {
        "lag_s": float(lag),
        "rms_cm": float(np.sqrt(mse) * 100),
        "runner_up_ratio": ratio,
        "span_s": float(t[-1] - t[0]),
        "fs_hz": float(1.0 / np.median(np.diff(t))),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    for variant, csv_name, bin_name in RADIO:
        csv_path = LOGS / csv_name
        rmeta = radio_meta(csv_path)
        meta = json.loads(meta_json_for_radio(csv_name).read_text())
        bin_path = USD_RAW / bin_name
        sc = score_bin(bin_path, meta)
        row = {
            "variant": variant,
            "radio_csv": csv_name,
            "paired_bin": bin_name,
            "pair_method": "chronological_thesisNN_within_copy_batch + trajectory_RMS<15cm_on_16s_files",
            "controller_meta": rmeta.get("controller"),
            "ctrl_mode_meta": rmeta.get("ctrl_mode"),
            "usd_start_s_radio": rmeta.get("usd_start_s"),
            "pair_confidence": "MEDIUM",
            "pair_confidence_note": (
                "Repeated A1 geometry makes correlation non-unique; ordering uses "
                "thesis counter vs wall-clock flight order (see docs/54)."
            ),
        }
        if sc:
            row.update(sc)
            if sc["rms_cm"] > 15 or sc["span_s"] > 20:
                row["pair_confidence"] = "LOW"
                row["pair_status"] = "MARGINAL_RMS_OR_LONG_FILE"
            else:
                row["pair_status"] = "ASSIGNED"
            if rmeta.get("usd_start_s"):
                row["usd_start_minus_lag_s"] = float(rmeta["usd_start_s"]) - sc["lag_s"]
        else:
            row["pair_status"] = "SCORE_FAILED"
            row["pair_confidence"] = "LOW"
        rows.append(row)

    out = OUT / "usd_pairing_a1_cf5_2026-10-02.json"
    out.write_text(json.dumps(rows, indent=2))
    (OUT / "PAIRING.md").write_text(
        "# A1 cf5 2026-10-02 uSD ↔ radio pairing\n\n"
        + "See `usd_pairing_a1_cf5_2026-10-02.json`. Method: **chronological thesisNN** "
        "within copy-time batches, with per-bin trajectory RMS check.\n"
    )
    print(f"Wrote {out}")
    for r in rows:
        print(r["radio_csv"], "->", r["paired_bin"], r.get("pair_status"), f"rms={r.get('rms_cm', float('nan')):.1f}cm")


if __name__ == "__main__":
    main()
