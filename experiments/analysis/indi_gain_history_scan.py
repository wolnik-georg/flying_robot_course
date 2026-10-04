#!/usr/bin/env python3
"""Scan radio CSV metas for INDI flight history (controller 6/9/10, ctrl_mode>=1)."""

from __future__ import annotations

import csv
import json
import re
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
LAB = REPO / "docs" / "lab_sessions"
OUT = Path(__file__).resolve().parent / "out" / "indi_package"
OUT.mkdir(parents=True, exist_ok=True)

META_RE = re.compile(r"^# meta:(\w+)=(.*)$")
STEADY_T0 = 6.0
STEADY_T1 = 13.0
LIFTOFF_Z = 0.2
TUMBLE_DEG = 45.0

# Desk record of retunes not always reflected in meta (yaml `all:` block logs current defaults).
LAB_KR_EVENTS = [
    ("2026-09-11", 483, "stage 2e attempt 3–4", "docs/FLIGHT_CARD_VALIDATION.md, lab_sessions/2026-09-11.md"),
    ("2026-09-11", 987, "stage 2e attempt 1–2", "docs/FLIGHT_CARD_VALIDATION.md"),
    ("2026-09-11", 632, "planned rung, not confirmed flown", "docs/lab_sessions/2026-09-16.md"),
    ("2026-07-19", 600, "ladder (pre-meta era)", "crazyflies.yaml comments ~554–563"),
]


def parse_meta(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("# meta:"):
                m = META_RE.match(line.strip())
                if m:
                    meta[m.group(1)] = m.group(2)
            elif line.startswith("time_s,"):
                break
    return meta


def load_flight_metrics(path: Path) -> dict:
    meta = parse_meta(path)
    t, roll, pitch, gx, gy, gz, z = [], [], [], [], [], [], []
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("time_s,"):
                header = line.strip().split(",")
                idx = {h: i for i, h in enumerate(header)}
                break
        else:
            return {"metrics_ok": False}
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            parts = line.strip().split(",")
            if len(parts) != len(header):
                continue
            try:
                t.append(float(parts[idx["time_s"]]))
                roll.append(float(parts[idx["roll"]]))
                pitch.append(float(parts[idx["pitch"]]))
                gx.append(float(parts[idx["gyro_x"]]))
                gy.append(float(parts[idx["gyro_y"]]))
                gz.append(float(parts[idx["gyro_z"]]))
                z.append(float(parts[idx["pos_z"]]))
            except (KeyError, ValueError):
                continue
    if len(t) < 30:
        return {"metrics_ok": False}
    t = np.array(t)
    t0 = t[0]
    rel = t - t0
    m = (rel >= STEADY_T0) & (rel <= STEADY_T1) & (np.array(z) > LIFTOFF_Z)
    if np.sum(m) < 20:
        m = (rel >= STEADY_T0) & (rel <= STEADY_T1)
    if np.sum(m) < 5:
        return {"metrics_ok": False}
    gmag = np.sqrt(np.array(gx)[m] ** 2 + np.array(gy)[m] ** 2 + np.array(gz)[m] ** 2)
    r = np.array(roll)[m]
    p = np.array(pitch)[m]
    max_tilt = float(np.max(np.sqrt(r**2 + p**2)))
    crash = max_tilt > TUMBLE_DEG or np.nanmin(np.array(z)[m]) < -0.05
    return {
        "metrics_ok": True,
        "gyro_rms_deg_s": float(np.sqrt(np.mean(gmag**2))),
        "roll_std_deg": float(np.std(r)),
        "pitch_std_deg": float(np.std(p)),
        "max_tilt_deg": max_tilt,
        "crash_or_abort": bool(crash),
        "z_mean_steady": float(np.mean(np.array(z)[m])),
    }


def meta_float(meta: dict, key: str, default=float("nan")) -> float:
    v = meta.get(key)
    if v is None or v == "":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def main() -> None:
    rows: list[dict] = []
    for path in sorted(LOGS.glob("*.csv")):
        meta = parse_meta(path)
        ctrl = meta.get("controller", "")
        try:
            cm = int(float(meta.get("ctrl_mode", "0") or 0))
        except ValueError:
            cm = 0
        if ctrl not in ("6", "9", "10"):
            continue
        if ctrl == "6" and cm < 1:
            continue
        if ctrl in ("9", "10") and cm == 0:
            # Omar flights often log ctrl_mode=0 but use ctrlOmarIndi.indi=3 — keep for reference
            pass
        m = load_flight_metrics(path)
        row = {
            "file": path.name,
            "date": path.name.split("_")[-2] if "_" in path.name else "",
            "drone": path.name.split("_")[1] if "_" in path.name else "",
            "scenario": meta.get("scenario", ""),
            "controller": ctrl,
            "ctrl_mode": cm,
            "indi_kr_meta": meta_float(meta, "indi_kr"),
            "indi_kw_meta": meta_float(meta, "indi_kw"),
            "indi_fc_bw_meta": meta_float(meta, "indi_fc_bw"),
            "pos_kp_xy": meta_float(meta, "pos_kp_xy"),
            "pos_kp_z": meta_float(meta, "pos_kp_z"),
            "pos_kv_xy": meta_float(meta, "pos_kv_xy"),
            "pos_kv_z": meta_float(meta, "pos_kv_z"),
            "pos_ki_z": meta_float(meta, "pos_ki_z"),
            "indi_j_scale": meta_float(meta, "indi_j_scale"),
            "indi_rpm_source": meta.get("indi_rpm_source", ""),
            "gains_apply": meta.get("gains_apply", ""),
            "shared_ctrl_mode": meta.get("shared_ctrl_mode", ""),
            "res_sign_meta": "NOT_LOGGED",
            "filt_dt_us_meta": "NOT_LOGGED",
            "notch_en_meta": "NOT_LOGGED",
            **{k: v for k, v in m.items() if k != "metrics_ok"},
            "metrics_ok": m.get("metrics_ok", False),
        }
        rows.append(row)

    out_csv = OUT / "gain_history_flights.csv"
    if rows:
        fields: list[str] = []
        for r in rows:
            for k in r:
                if k not in fields:
                    fields.append(k)
        with open(out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)

    c6 = [r for r in rows if r["controller"] == "6" and r["ctrl_mode"] >= 1]
    kr_meta_vals = sorted({r["indi_kr_meta"] for r in c6 if r["indi_kr_meta"] == r["indi_kr_meta"]})
    kp_xy = sorted({r["pos_kp_xy"] for r in c6 if r["pos_kp_xy"] == r["pos_kp_xy"]})

    summary = {
        "n_csv_scanned": len(list(LOGS.glob("*.csv"))),
        "n_indi_flights_in_meta": len(rows),
        "n_controller6_ctrl_mode_ge1": len(c6),
        "unique_indi_kr_in_radio_meta": kr_meta_vals,
        "unique_pos_kp_xy_in_radio_meta": kp_xy,
        "lab_documented_kr_retunes": LAB_KR_EVENTS,
        "answers": {
            "lowest_kr_in_radio_meta": min(kr_meta_vals) if kr_meta_vals else None,
            "lowest_kr_documented_anywhere": 483,
            "res_sign_minus_one_flown": False,
            "res_sign_evidence": "lib.rs:1886 NEVER FLOWN; 2026-09-09 hard-coded flip diverged (docs/lab_sessions/2026-09-09.md); default +1 traj_iface.c:441",
            "softer_kp_with_indi_in_meta": [r for r in c6 if r["pos_kp_xy"] == r["pos_kp_xy"] and r["pos_kp_xy"] < 30],
            "typical_combo": "kr/kw 2400/170 + pos 64/48/5/7 when meta populated (yaml all: block)",
        },
    }
    (OUT / "gain_history_summary.json").write_text(json.dumps(summary, indent=2))
    print(f"Wrote {out_csv} ({len(rows)} rows) and gain_history_summary.json")


if __name__ == "__main__":
    main()
