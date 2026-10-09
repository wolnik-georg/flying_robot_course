#!/usr/bin/env python3
"""A8 controller variant comparison (2026-10-09). Deterministic; see docs/71."""

from __future__ import annotations

import csv
import json
import re
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
USD_DIR = LOGS / "usd_raw"
OUT = Path(__file__).resolve().parent / "out" / "a8_compare_2026-10-09"
FIGS = OUT / "figs"

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
sys.path.insert(0, str(REPO / "experiments" / "analysis"))

from decode_usd_log import load as load_usd  # noqa: E402
import ns2_2026_10_05_crossing_dip as ns2  # noqa: E402
from a8_crossings import zero_crossings  # noqa: E402
from meeting_hw_common import load_radio_csv, apply_mpl_style  # noqa: E402
from meeting_simple_common import apply_meeting_style, Z_BAND_CM, Z_ERR_YLIM_CM  # noqa: E402

Z_AIR_M = 0.3
W_AFTER_LIFTOFF_S = 6.0
W_BEFORE_END_S = 4.0
CROSS_EXCL_S = 1.5
DIP_HALF_S = 1.0
Z_CMD_M = 0.5
PWM_CEIL = 65000

TABLE_ORDER = [
    # study variants (2026-10-09 decision: Omar INDI = Omar C + Iz 1.5)
    "Geometric baseline",
    "NS2 off (= geometric)",
    "NS2 res_sign −1",
    "NS2 res_sign +1",
    "Ours INDI (10-09 filter ON)",
    "Omar C + Iz 1.5 (10-09)",
    # reference / history
    "Omar C + Iz 1.5",
    "Ours INDI (10-09 filter OFF)",
    "Ours INDI",
    "Omar C + Iz 1.0",
    "Omar C + Iz 2.0",
    "Omar C exact (Kpos_Iz 0)",
    "Omar Rust exact (10-09 same day)",
    "Omar Rust exact (kpos_iz 0)",
    "Omar Rust + Iz 1.0",
    "Omar Rust + Iz 1.5",
    "Omar Rust + Iz 2.0",
]

# (date, stamp) -> variant label; Iz from docs/69/70 flight order (not in meta).
CANDIDATES: dict[str, list[tuple[str, str, dict[str, Any]]]] = {
    "Geometric baseline": [
        ("2026-10-03", "13-15-00", {}),
        ("2026-10-05", "17-39-27", {}),
        ("2026-10-05", "17-41-09", {}),
        ("2026-10-08", "17-37-04", {}),
        ("2026-10-08", "17-38-50", {}),
    ],
    "NS2 res_sign +1": [
        ("2026-10-05", "19-17-04", {"yaml_note": "rnn.en=1 res_sign=+1"}),
        ("2026-10-05", "19-19-27", {"yaml_note": "rnn.en=1 res_sign=+1"}),
        ("2026-10-05", "19-23-26", {"yaml_note": "rnn.en=1 res_sign=+1"}),
    ],
    "NS2 res_sign −1": [
        ("2026-10-08", "17-16-27", {"flag": "battery_sag"}),
        ("2026-10-08", "17-18-40", {}),
        ("2026-10-08", "17-22-13", {}),
        ("2026-10-08", "17-24-54", {}),
        ("2026-10-09", "17-30-16", {"yaml_note": "unified firmware, rnn.en=1 res_sign=-1"}),
        ("2026-10-09", "17-31-47", {"yaml_note": "unified firmware, rnn.en=1 res_sign=-1"}),
    ],
    "Ours INDI (10-09 filter ON)": [
        ("2026-10-09", "17-40-36", {}),
        ("2026-10-09", "17-42-12", {}),
    ],
    "Ours INDI (10-09 filter OFF)": [
        ("2026-10-09", "17-56-07", {}),
        ("2026-10-09", "17-57-38", {"flag": "battery_sag"}),
    ],
    "Ours INDI": [
        ("2026-10-02", "19-08-13", {"check_abort": True}),
        ("2026-10-02", "19-09-54", {}),
        ("2026-10-02", "19-11-30", {}),
        ("2026-10-03", "13-10-32", {"check_abort": True}),
    ],
    "Omar C exact (Kpos_Iz 0)": [
        ("2026-10-02", "18-23-23", {}),
        ("2026-10-02", "18-24-57", {}),
    ],
    "Omar Rust exact (kpos_iz 0)": [
        ("2026-10-02", "18-45-09", {}),
        ("2026-10-02", "18-46-45", {}),
    ],
    "Omar Rust exact (10-09 same day)": [
        ("2026-10-09", "18-09-45", {}),
        ("2026-10-09", "18-11-15", {}),
    ],
    "Omar Rust + Iz 1.0": [
        ("2026-10-08", "18-17-33", {"kpos_iz": 1.0}),
        ("2026-10-08", "18-19-07", {"kpos_iz": 1.0}),
    ],
    "Omar Rust + Iz 1.5": [
        ("2026-10-08", "18-23-52", {"kpos_iz": 1.5}),
        ("2026-10-08", "18-25-23", {"kpos_iz": 1.5}),
    ],
    "Omar Rust + Iz 2.0": [
        ("2026-10-08", "18-28-04", {"kpos_iz": 2.0}),
        ("2026-10-08", "18-29-35", {"kpos_iz": 2.0}),
    ],
    "Omar C + Iz 1.0": [
        ("2026-10-08", "18-40-18", {"Kpos_Iz": 1.0}),
        ("2026-10-08", "18-42-05", {"Kpos_Iz": 1.0}),
    ],
    "Omar C + Iz 1.5": [
        ("2026-10-08", "18-45-11", {"Kpos_Iz": 1.5}),
        ("2026-10-08", "18-47-48", {"Kpos_Iz": 1.5}),
    ],
    "Omar C + Iz 2.0": [
        ("2026-10-08", "18-49-37", {"Kpos_Iz": 2.0}),
        ("2026-10-08", "18-51-15", {"Kpos_Iz": 2.0}),
    ],
    "Omar C + Iz 1.5 (10-09)": [
        ("2026-10-09", "18-38-06", {"Kpos_Iz": 1.5}),
        ("2026-10-09", "18-39-35", {"Kpos_Iz": 1.5}),
    ],
}

EXPECTED_CTRL: dict[str, tuple[int, int, float | None]] = {
    "Geometric baseline": (6, 0, 16.0),
    "NS2 res_sign +1": (6, 0, 16.0),
    "NS2 res_sign −1": (6, 0, 16.0),
    "Ours INDI": (6, 3, 0.0),
    "Ours INDI (10-09 filter ON)": (6, 3, 0.0),
    "Ours INDI (10-09 filter OFF)": (6, 3, 0.0),
    "Omar Rust exact (10-09 same day)": (10, 0, None),
    "Omar C + Iz 1.5 (10-09)": (9, 0, 16.0),
    "Omar C exact (Kpos_Iz 0)": (9, 0, None),
    "Omar Rust exact (kpos_iz 0)": (10, 0, None),
    "Omar Rust + Iz 1.0": (10, 0, 16.0),
    "Omar Rust + Iz 1.5": (10, 0, 16.0),
    "Omar Rust + Iz 2.0": (10, 0, 16.0),
    "Omar C + Iz 1.0": (9, 0, 16.0),
    "Omar C + Iz 1.5": (9, 0, 16.0),
    "Omar C + Iz 2.0": (9, 0, 16.0),
}

EXPLICIT_EXCLUDE: dict[tuple[str, str], str] = {
    ("2026-10-02", "18-57-49"): "pre-fix ki_z 16 on ours INDI path",
    ("2026-10-02", "18-59-17"): "pre-fix ki_z 16 on ours INDI path",
    ("2026-10-02", "18-22-05"): "dz 0.25 in meta",
}

_USD_BY_TAG: dict[int, Path] | None = None
_USD_BY_STAMP: dict[tuple[str, str], Path] = {}


def _build_usd_index() -> None:
    global _USD_BY_TAG
    if _USD_BY_TAG is not None:
        return
    _USD_BY_TAG = {}
    for p in USD_DIR.glob("cf5_*.bin"):
        if p.stat().st_size == 0:
            continue
        m = re.search(r"(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.bin$", p.name)
        if m:
            _USD_BY_STAMP[(m.group(1), m.group(2))] = p
        try:
            d = load_usd(str(p))
            if "run_tag" in d and len(d["run_tag"]):
                rt = int(round(float(d["run_tag"][0])))
                if rt not in _USD_BY_TAG or m and m.group(2) in p.name:
                    _USD_BY_TAG[rt] = p
        except Exception:
            continue


PALETTE = [
    "#0072B2",
    "#E69F00",
    "#009E73",
    "#CC79A7",
    "#D55E00",
    "#56B4E9",
    "#F0E442",
    "#000000",
    "#882255",
    "#44AA99",
    "#999999",
    "#661100",
    "#AA4499",
]


@dataclass
class FlightMetrics:
    variant: str
    date: str
    stamp: str
    source: str  # usd | radio
    meta_ok: bool
    meta_notes: str
    flags: str
    firmware_day: str
    steady_mean_cm: float
    steady_sd_cm: float
    rms_z_cm: float
    max_abs_z_cm: float
    dip_mean_cm: float
    dip_rel_mean_cm: float
    dips_cm: list[float]
    lat_rms_m: float | None
    roll_p99: float
    pitch_p99: float
    gyro_x_sd: float
    pwm_ceiling_frac: float | None
    vbat_min: float | None
    n_crossings: int
    usd_path: str
    radio_path: str


def load_meta(date: str, stamp: str) -> dict[str, Any] | None:
    p = LOGS / f"A8_{date}_{stamp}.meta.json"
    if not p.is_file():
        return None
    return json.loads(p.read_text())


def cf5_from_meta(meta: dict) -> dict:
    return meta["per_drone"]["cf5"]


def meta_inclusion_ok(meta: dict) -> tuple[bool, str]:
    if meta.get("scenario") != "A8":
        return False, "scenario!=A8"
    p = meta.get("params", {})
    if abs(float(p.get("dz", -1)) - 0.5) > 1e-6:
        return False, f"dz={p.get('dz')}"
    if int(p.get("passes", -1)) != 4:
        return False, f"passes={p.get('passes')}"
    if abs(float(meta.get("height", -1)) - 0.5) > 1e-6:
        return False, f"height={meta.get('height')}"
    if abs(float(meta.get("duration", -1)) - 26.0) > 0.5:
        return False, f"duration={meta.get('duration')}"
    names = meta.get("names", [])
    if names[0] != "cf5" or names[1] != "cf_second":
        return False, f"names={names}"
    return True, "ok"


def validate_variant_meta(variant: str, meta: dict, extra: dict) -> tuple[bool, str]:
    ok, why = meta_inclusion_ok(meta)
    if not ok:
        return False, why
    cf5 = cf5_from_meta(meta)
    exp = EXPECTED_CTRL[variant]
    ctrl = int(cf5["controller"])
    mode = int(cf5["ctrl_mode"])
    kz = float(cf5.get("pos", {}).get("ki_z", 0) or 0)
    notes = []
    if (ctrl, mode) != (exp[0], exp[1]):
        return False, f"controller/mode {ctrl}/{mode} expected {exp[0]}/{exp[1]}"
    if exp[2] is not None and abs(kz - exp[2]) > 0.5:
        return False, f"ki_z={kz} expected {exp[2]}"
    if "kpos_iz" in extra or "Kpos_Iz" in extra:
        notes.append(f"Iz={extra.get('kpos_iz', extra.get('Kpos_Iz'))} from flight order (docs/69/70)")
    if extra.get("yaml_note"):
        notes.append(extra["yaml_note"] + " (yaml, not meta)")
    return True, "; ".join(notes) if notes else "ok"


def find_cf5_usd(meta: dict, date: str, stamp: str) -> Path | None:
    _build_usd_index()
    assert _USD_BY_TAG is not None
    p = _USD_BY_STAMP.get((date, stamp))
    if p is not None and p.stat().st_size > 0:
        return p
    tag = meta.get("usd_run_tag")
    if tag is not None:
        p = _USD_BY_TAG.get(int(tag))
        if p is not None and p.stat().st_size > 0:
            return p
    hits = sorted(USD_DIR.glob(f"cf5_*{date}_{stamp}.bin"))
    for h in hits:
        if h.stat().st_size > 0:
            return h
    return None


def load_timeseries(meta: dict, date: str, stamp: str) -> tuple[str, dict[str, np.ndarray], Path | None, Path]:
    radio = LOGS / f"A8_cf5_{date}_{stamp}.csv"
    usd_p = find_cf5_usd(meta, date, stamp)
    if usd_p is not None and usd_p.stat().st_size > 0:
        d = load_usd(str(usd_p))
        t = np.asarray(d["t"], float)
        t = t - t[0]
        out = {
            "t": t,
            "x": np.asarray(d["x"], float),
            "y": np.asarray(d["y"], float),
            "z": np.asarray(d["z"], float),
            "z_sp": np.asarray(d.get("ctrltarget_z", np.full_like(t, Z_CMD_M)), float),
            "roll": np.asarray(d["roll_deg"], float),
            "pitch": np.asarray(d["pitch_deg"], float),
            "gyro_x": np.asarray(d["gyro_x"], float),
        }
        for ck in ("ctrltarget_x", "ctrltarget_y"):
            if ck in d:
                out[ck] = np.asarray(d[ck], float)
        for i in range(1, 5):
            k = f"motor_m{i}"
            if k in d:
                out[k] = np.asarray(d[k], float)
        return "usd", out, usd_p, radio
    if not radio.is_file():
        raise FileNotFoundError(f"no usd or radio for {date} {stamp}")
    _, cols = load_radio_csv(radio)
    t = cols["time_s"] - cols["time_s"][0]
    out = {
        "t": t,
        "x": cols["pos_x"],
        "y": cols["pos_y"],
        "z": cols["pos_z"],
        "z_sp": np.full_like(t, Z_CMD_M),
        "roll": cols.get("roll", cols.get("roll_deg", np.zeros_like(t))),
        "pitch": cols.get("pitch", cols.get("pitch_deg", np.zeros_like(t))),
        "gyro_x": cols.get("gyro_x", np.zeros_like(t)),
        "vbat": cols.get("vbat", np.full_like(t, np.nan)),
    }
    return "radio", out, None, radio


def flight_is_crash(cols: dict[str, np.ndarray]) -> tuple[bool, str]:
    z, t = cols["z"], cols["t"]
    up = z > Z_AIR_M
    if not np.any(up):
        return True, "never airborne"
    t0 = float(t[up][0])
    t1 = float(t[up][-1])
    if t1 - t0 < 18:
        return True, f"short airborne {t1 - t0:.1f}s"
    w_lo, w_hi = t0 + W_AFTER_LIFTOFF_S, t1 - W_BEFORE_END_S
    W = (t >= w_lo) & (t <= w_hi) & up
    tilt = np.maximum(np.abs(cols["roll"]), np.abs(cols["pitch"]))
    max_tilt = float(np.max(tilt[W])) if np.any(W) else float(np.max(tilt[up]))
    if max_tilt > 45:
        return True, f"max_tilt={max_tilt:.0f}° in scenario window"
    return False, "ok"


def compute_metrics(
    variant: str,
    date: str,
    stamp: str,
    source: str,
    cols: dict[str, np.ndarray],
    meta_ok: bool,
    meta_notes: str,
    flags: str,
    usd_path: Path | None,
    radio_path: Path,
) -> FlightMetrics:
    t, z, y = cols["t"], cols["z"], cols["y"]
    z_sp = cols["z_sp"]
    e_z = (z - z_sp) * 100.0
    up = z > Z_AIR_M
    t0 = float(t[up][0])
    t1 = float(t[up][-1])
    w_lo, w_hi = t0 + W_AFTER_LIFTOFF_S, t1 - W_BEFORE_END_S
    W = (t >= w_lo) & (t <= w_hi) & up
    cross_t = zero_crossings(t[up], y[up])
    # map crossing times to full timeline
    cross_full = cross_t
    steady = W.copy()
    for tc in cross_full:
        steady &= ~((t >= tc - CROSS_EXCL_S) & (t <= tc + CROSS_EXCL_S))
    dips = []
    dips_rel = []
    for tc in cross_full:
        m = (t >= tc - DIP_HALF_S) & (t <= tc + DIP_HALF_S)
        if not np.any(m):
            continue
        d = float(np.min(e_z[m]))
        dips.append(d)
        dips_rel.append(d - float(np.mean(e_z[steady])) if np.any(steady) else d)
    steady_mean = float(np.mean(e_z[steady])) if np.any(steady) else float("nan")
    steady_sd = float(np.std(e_z[steady])) if np.any(steady) else float("nan")
    rms_z = float(np.sqrt(np.mean(e_z[W] ** 2))) if np.any(W) else float("nan")
    max_abs = float(np.max(np.abs(e_z[W]))) if np.any(W) else float("nan")
    lat_rms = None
    if source == "usd":
        tx, ty = cols.get("ctrltarget_x"), cols.get("ctrltarget_y")
        if tx is not None and ty is not None and np.any(W):
            lat = np.sqrt((cols["x"] - tx) ** 2 + (cols["y"] - ty) ** 2)
            lat_rms = float(np.sqrt(np.mean(lat[W] ** 2)))
    roll_p99 = float(np.percentile(np.abs(cols["roll"][W]), 99)) if np.any(W) else float("nan")
    pitch_p99 = float(np.percentile(np.abs(cols["pitch"][W]), 99)) if np.any(W) else float("nan")
    gyro_x_sd = float(np.std(cols["gyro_x"][W])) if np.any(W) else float("nan")
    pwm_frac = None
    if source == "usd":
        motors = [cols.get(f"motor_m{i}") for i in range(1, 5)]
        motors = [m for m in motors if m is not None]
        if motors:
            stack = np.vstack(motors)
            air_m = up
            pwm_frac = float(np.mean(stack[:, air_m] >= PWM_CEIL)) if np.any(air_m) else float("nan")
    vbat_min = None
    vb = cols.get("vbat")
    if vb is not None:
        loaded = vb[vb > 2.0]
        if len(loaded):
            vbat_min = float(np.min(loaded))
    if vbat_min is None and radio_path.is_file():
        try:
            _, rc = load_radio_csv(radio_path)
            vb2 = rc.get("vbat")
            if vb2 is not None:
                loaded = vb2[vb2 > 2.0]
                if len(loaded):
                    vbat_min = float(np.min(loaded))
        except Exception:
            pass
    return FlightMetrics(
        variant=variant,
        date=date,
        stamp=stamp,
        source=source,
        meta_ok=meta_ok,
        meta_notes=meta_notes,
        flags=flags,
        firmware_day=date,
        steady_mean_cm=steady_mean,
        steady_sd_cm=steady_sd,
        rms_z_cm=rms_z,
        max_abs_z_cm=max_abs,
        dip_mean_cm=float(np.mean(dips)) if dips else float("nan"),
        dip_rel_mean_cm=float(np.mean(dips_rel)) if dips_rel else float("nan"),
        dips_cm=dips,
        lat_rms_m=lat_rms,
        roll_p99=roll_p99,
        pitch_p99=pitch_p99,
        gyro_x_sd=gyro_x_sd,
        pwm_ceiling_frac=pwm_frac,
        vbat_min=vbat_min,
        n_crossings=len(cross_full),
        usd_path=str(usd_path or ""),
        radio_path=str(radio_path),
    )


def process_all() -> tuple[list[FlightMetrics], list[dict], list[dict]]:
    used: list[FlightMetrics] = []
    excluded: list[dict] = []
    mismatches: list[dict] = []
    assigned: set[tuple[str, str]] = set()
    for key, reason in EXPLICIT_EXCLUDE.items():
        excluded.append({"date": key[0], "stamp": key[1], "variant": "(explicit)", "reason": reason})

    for variant, flights in CANDIDATES.items():
        for date, stamp, extra in flights:
            assigned.add((date, stamp))
            key = (date, stamp)
            if key in EXPLICIT_EXCLUDE:
                excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": EXPLICIT_EXCLUDE[key]})
                continue
            meta = load_meta(date, stamp)
            if meta is None:
                excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": "missing meta.json"})
                continue
            vok, vnote = validate_variant_meta(variant, meta, extra)
            if not vok:
                excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": f"meta mismatch: {vnote}"})
                mismatches.append({"date": date, "stamp": stamp, "variant": variant, "issue": vnote})
                continue
            try:
                source, cols, usd_p, radio_p = load_timeseries(meta, date, stamp)
            except FileNotFoundError as e:
                excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": str(e)})
                continue
            crash, cwhy = flight_is_crash(cols)
            flags = extra.get("flag", "")
            if extra.get("check_abort"):
                tilt_max = float(np.max(np.maximum(np.abs(cols["roll"]), np.abs(cols["pitch"]))))
                if tilt_max > 35 or crash:
                    excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": f"abort/crash check: {cwhy} tilt={tilt_max:.0f}"})
                    continue
            if crash:
                excluded.append({"date": date, "stamp": stamp, "variant": variant, "reason": cwhy})
                continue
            fm = compute_metrics(
                variant, date, stamp, source, cols, vok, vnote, flags, usd_p, radio_p
            )
            used.append(fm)

    # Scan other A8 metas (meta-only; no full decode)
    for mp in sorted(LOGS.glob("A8_2026-*.meta.json")):
        m = re.search(r"A8_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.meta\.json", mp.name)
        if not m:
            continue
        date, stamp = m.group(1), m.group(2)
        if (date, stamp) in assigned or (date, stamp) in EXPLICIT_EXCLUDE:
            continue
        meta = json.loads(mp.read_text())
        ok, _why = meta_inclusion_ok(meta)
        if not ok:
            continue
        cf5 = cf5_from_meta(meta)
        label_guess = f"c{cf5['controller']}/m{cf5['ctrl_mode']}"
        has_radio = (LOGS / f"A8_cf5_{date}_{stamp}.csv").is_file()
        _build_usd_index()
        has_usd = find_cf5_usd(meta, date, stamp) is not None
        excluded.append(
            {
                "date": date,
                "stamp": stamp,
                "variant": label_guess,
                "reason": "candidate not used"
                + ("" if has_radio or has_usd else "; no cf5 log"),
            }
        )
    return used, excluded, mismatches


def steady_ez_samples(fm: FlightMetrics) -> np.ndarray:
    meta = load_meta(fm.date, fm.stamp)
    _, cols, _, _ = load_timeseries(meta, fm.date, fm.stamp)
    t, z, y = cols["t"], cols["z"], cols["y"]
    e_z = (z - cols["z_sp"]) * 100.0
    up = z > Z_AIR_M
    t0, t1 = float(t[up][0]), float(t[up][-1])
    W = (t >= t0 + W_AFTER_LIFTOFF_S) & (t <= t1 - W_BEFORE_END_S) & up
    steady = W.copy()
    for tc in zero_crossings(t[up], y[up]):
        steady &= ~((t >= tc - CROSS_EXCL_S) & (t <= tc + CROSS_EXCL_S))
    return e_z[steady]


def variant_summary(rows: list[FlightMetrics], label: str) -> dict[str, Any]:
    sub = [r for r in rows if r.variant == label]
    if not sub:
        return {"variant": label, "n": 0}
    srcs = {r.source for r in sub}
    source = srcs.pop() if len(srcs) == 1 else "mixed"
    dip_pool = [d for r in sub for d in r.dips_cm]
    pooled = np.concatenate([steady_ez_samples(r) for r in sub]) if sub else np.array([])
    pooled_sd = float(np.std(pooled)) if len(pooled) > 1 else float("nan")
    return {
        "variant": label,
        "n": len(sub),
        "source": source,
        "steady_mean_cm": float(np.mean([r.steady_mean_cm for r in sub])),
        "steady_mean_sd_cm": float(np.std([r.steady_mean_cm for r in sub], ddof=0)),
        "steady_sd_pooled_cm": pooled_sd,
        "rms_z_cm": float(np.mean([r.rms_z_cm for r in sub])),
        "max_abs_z_cm": float(np.max([r.max_abs_z_cm for r in sub])),
        "dip_mean_cm": float(np.mean(dip_pool)) if dip_pool else float("nan"),
        "dip_sd_cm": float(np.std(dip_pool, ddof=0)) if len(dip_pool) > 1 else 0.0,
        "dip_rel_mean_cm": float(np.mean([r.dip_rel_mean_cm for r in sub if not np.isnan(r.dip_rel_mean_cm)])),
        "lat_rms_m": float(np.nanmean([r.lat_rms_m for r in sub if r.lat_rms_m is not None])),
        "roll_p99": float(np.mean([r.roll_p99 for r in sub])),
        "pitch_p99": float(np.mean([r.pitch_p99 for r in sub])),
        "gyro_x_sd": float(np.mean([r.gyro_x_sd for r in sub])),
        "pwm_ceiling_frac": float(np.nanmean([r.pwm_ceiling_frac for r in sub if r.pwm_ceiling_frac is not None])),
        "vbat_min": float(np.min([r.vbat_min for r in sub if r.vbat_min is not None])) if any(r.vbat_min for r in sub) else float("nan"),
    }


def load_trace(fm: FlightMetrics) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    meta = load_meta(fm.date, fm.stamp)
    source, cols, _, _ = load_timeseries(meta, fm.date, fm.stamp)
    t = cols["t"]
    z, z_sp = cols["z"], cols["z_sp"]
    up = z > Z_AIR_M
    # align every flight on its first A8 crossing (= 5.0 s into the scenario, uSD crossings at 5.01/11.05/17.02/23.05 s):
    # radio logs start earlier (takeoff + settle), so z>0.3 m is not a common time origin
    cr = zero_crossings(t[up], cols["y"][up])
    t0 = float(cr[0]) - 5.0 if cr else float(t[up][0])
    tt = t - t0
    e_z = (z - z_sp) * 100.0
    return tt[up], z[up], e_z[up]


VARIANT_COLOR = {
    "Geometric baseline": "#4D4D4D",
    "NS2 res_sign −1": "#009E73",
    "NS2 res_sign +1": "#CC79A7",
    "Ours INDI": "#E69F00",
    "Ours INDI (10-09 filter ON)": "#D55E00",
    "Ours INDI (10-09 filter OFF)": "#F0B000",
    "Omar C + Iz 1.5 (10-09)": "#005AB5",
    "Omar Rust exact (10-09 same day)": "#E8825A",
    "Omar C exact (Kpos_Iz 0)": "#9ECAE1",
    "Omar C + Iz 1.0": "#4292C6",
    "Omar C + Iz 1.5": "#08519C",
    "Omar C + Iz 2.0": "#08306B",
    "Omar Rust exact (kpos_iz 0)": "#FCAE91",
    "Omar Rust + Iz 1.0": "#EF3B2C",
    "Omar Rust + Iz 1.5": "#A50F15",
    "Omar Rust + Iz 2.0": "#67000D",
}
T_PLOT_START_RADIO_S = 2.0  # radio logs include the takeoff ramp, uSD logs start in the scenario
T_PLOT_END_S = 26.0
T_GRID = np.arange(T_PLOT_START_RADIO_S, T_PLOT_END_S, 0.05)


def plot_trace(fm: FlightMetrics) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(t since z>0.3 m, z, e_z) trimmed for plotting: radio drops the takeoff ramp, all stop at 26 s (landing)."""
    tt, z, ez = load_trace(fm)
    m = tt <= T_PLOT_END_S
    if fm.source != "usd":
        m &= tt >= T_PLOT_START_RADIO_S
    return tt[m], z[m], ez[m]


def on_grid(fm: FlightMetrics) -> np.ndarray:
    tt, _, ez = plot_trace(fm)
    return np.interp(T_GRID, tt, ez, left=np.nan, right=np.nan)


def save_figures(rows: list[FlightMetrics]) -> None:
    FIGS.mkdir(parents=True, exist_ok=True)
    apply_meeting_style()
    by = {}
    for r in rows:
        by.setdefault(r.variant, []).append(r)

    def band(ax):
        ax.axhspan(-Z_BAND_CM, Z_BAND_CM, color="#4FB39A", alpha=0.15, zorder=0)
        ax.axhline(0, color="k", lw=0.6, alpha=0.4)

    def draw_variant(ax, v, ez_only=True, lw=0.9, alpha=0.8, label_suffix=""):
        fl = by.get(v, [])
        for i, r in enumerate(fl):
            tt, z, ez = plot_trace(r)
            ax.plot(tt, ez if ez_only else z, color=VARIANT_COLOR[v], lw=lw, alpha=alpha,
                    label=f"{v} (n={len(fl)}){label_suffix}" if i == 0 else None)

    # 1 INDI variants
    indi = ["Ours INDI (10-09 filter ON)", "Ours INDI", "Omar C + Iz 1.5 (10-09)", "Omar C + Iz 1.5", "Omar C exact (Kpos_Iz 0)"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    for v in indi:
        draw_variant(axes[0], v, ez_only=False)
        draw_variant(axes[1], v)
    axes[0].axhline(Z_CMD_M, ls="--", color="k", lw=0.8, label="commanded z")
    axes[0].set_ylabel("z (m)")
    axes[0].set_title("A8 INDI variants — cf5 bottom (one line per flight)")
    band(axes[1]); axes[1].set_ylim(-25, 32); axes[1].set_ylabel("z error (cm)"); axes[1].set_xlabel("time since scenario start (s)")
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="center right", fontsize=8, frameon=False)
    fig.tight_layout(rect=(0, 0, 0.8, 1))
    fig.savefig(FIGS / "indi_variants_z.png"); plt.close(fig)

    # 2 Omar Iz ladder
    ladder = {"Rust": ["Omar Rust exact (kpos_iz 0)", "Omar Rust + Iz 1.0", "Omar Rust + Iz 1.5", "Omar Rust + Iz 2.0"],
              "C": ["Omar C exact (Kpos_Iz 0)", "Omar C + Iz 1.0", "Omar C + Iz 1.5", "Omar C + Iz 2.0"]}
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(15, 5.2), gridspec_kw={"width_ratios": [2.2, 1]})
    for port, names in ladder.items():
        for v in names:
            fl = by.get(v, [])
            if not fl:
                continue
            mat = np.vstack([on_grid(r) for r in fl])
            ax.plot(T_GRID, np.nanmean(mat, axis=0), color=VARIANT_COLOR[v], lw=2.0, label=f"{v} (n={len(fl)})")
            for row in mat:
                ax.plot(T_GRID, row, color=VARIANT_COLOR[v], lw=0.6, alpha=0.4)
    band(ax); ax.set_ylim(-15, 28); ax.set_xlabel("time since scenario start (s)"); ax.set_ylabel("z error (cm)")
    ax.set_title("Omar Rust and C — Iz ladder (thick = mean of the flights, thin = single flights)")
    ax.legend(fontsize=7, ncol=2, loc="upper center", bbox_to_anchor=(0.5, -0.14), frameon=False)
    xs = [0.0, 1.0, 1.5, 2.0]
    for port, names, col in [("Rust", ladder["Rust"], "#A50F15"), ("C", ladder["C"], "#08519C")]:
        mean_pts = []
        for x, v in zip(xs, names):
            vals = [r.steady_mean_cm for r in by.get(v, [])]
            ax2.scatter([x + (0.03 if port == "C" else -0.03)] * len(vals), vals, color=col, s=22, alpha=0.7)
            mean_pts.append(np.mean(vals) if vals else np.nan)
        ax2.plot(xs, mean_pts, color=col, lw=1.8, marker="o", label=f"Omar {port} (mean of flights)")
    band(ax2); ax2.set_xticks(xs); ax2.set_xticklabels(["0\n(exact)", "1.0", "1.5", "2.0"])
    ax2.set_xlabel("Iz (kpos_iz / Kpos_Iz)"); ax2.set_ylabel("steady z error (cm)"); ax2.set_yscale("symlog", linthresh=3)
    ax2.set_title("Steady z error vs Iz"); ax2.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "omar_iz_ladder.png"); plt.close(fig)

    # 3 geometric
    fig, ax = plt.subplots(figsize=(10, 4.2))
    geo = by.get("Geometric baseline", [])
    for r in geo:
        tt, _, ez = plot_trace(r)
        ax.plot(tt, ez, lw=0.9, label=f"{r.date[5:]} {r.stamp}")
    band(ax); ax.set_ylim(-10, 10); ax.set_xlabel("time since scenario start (s)"); ax.set_ylabel("z error (cm)")
    ax.set_title(f"Geometric + ki_z 16, network off (n={len(geo)}, steady {np.mean([r.steady_mean_cm for r in geo]):+.2f} cm)")
    ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(FIGS / "geometric_z.png"); plt.close(fig)

    # 4 NS2 (traces + per-crossing dips)
    groups = [("Geometric baseline", "network off (= geometric)"), ("NS2 res_sign +1", "network on, res_sign +1"),
              ("NS2 res_sign −1", "network on, res_sign −1")]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 4.8), gridspec_kw={"width_ratios": [2.3, 1]})
    for k, (v, leg) in enumerate(groups):
        fl = by.get(v, [])
        sm = variant_summary(rows, v)
        for i, r in enumerate(fl):
            tt, _, ez = plot_trace(r)
            ax.plot(tt, ez, color=VARIANT_COLOR[v], lw=0.8, alpha=0.75,
                    label=f"{leg} n={len(fl)}: steady {sm['steady_mean_cm']:+.2f} cm, dip {sm['dip_mean_cm']:+.2f} cm" if i == 0 else None)
        dips = [d for r in fl for d in r.dips_cm]
        ax2.scatter(np.full(len(dips), k) + np.linspace(-0.12, 0.12, len(dips)), dips, color=VARIANT_COLOR[v], s=22, alpha=0.8)
        if dips:
            ax2.errorbar([k], [np.mean(dips)], yerr=[np.std(dips, ddof=1) if len(dips) > 1 else 0], color="k", capsize=5, marker="_", ms=18)
            ax2.text(k, np.mean(dips) + 1.0, f"{np.mean(dips):+.1f}", ha="center", fontsize=9)
    band(ax); ax.set_ylim(-15, 10); ax.set_xlabel("time since scenario start (s)"); ax.set_ylabel("z error (cm)")
    ax.set_title("NS2 network off / res_sign +1 / res_sign −1 — A8 cf5"); ax.legend(fontsize=8, loc="lower right")
    ax2.set_xticks(range(3)); ax2.set_xticklabels(["off", "+1", "−1"]); ax2.set_ylabel("crossing dip (cm)")
    ax2.set_title("per-crossing dip, mean ± sd"); ax2.axhline(0, color="k", lw=0.6)
    fig.tight_layout(); fig.savefig(FIGS / "ns2_on_off.png"); plt.close(fig)

    # 5 all variants
    order = [v for v in TABLE_ORDER if v != "NS2 off (= geometric)"]
    fig = plt.figure(figsize=(19, 7.6))
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 0.16], width_ratios=[1.5, 1.5, 1.0])
    axa, axb, axc = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[0, 2])
    handles = []
    for v in order:
        fl = by.get(v, [])
        if not fl:
            continue
        mat = np.vstack([on_grid(r) for r in fl])
        lab = f"{v}{' = NS2 off' if v == 'Geometric baseline' else ''} (n={len(fl)})"
        for ax in (axa, axb):
            if len(fl) >= 3:
                ax.fill_between(T_GRID, np.nanpercentile(mat, 10, axis=0), np.nanpercentile(mat, 90, axis=0), color=VARIANT_COLOR[v], alpha=0.15)
                ln, = ax.plot(T_GRID, np.nanmedian(mat, axis=0), color=VARIANT_COLOR[v], lw=1.6)
            else:
                ln, = ax.plot(T_GRID, np.nanmean(mat, axis=0), color=VARIANT_COLOR[v], lw=1.6)
        ln.set_label(lab); handles.append(ln)
    for ax, ttl, yl in ((axa, "all variants — z error (median; mean if n<3)", (-25, 32)), (axb, "zoom ±15 cm", (-15, 12))):
        band(ax); ax.set_ylim(*yl); ax.set_xlabel("time since scenario start (s)"); ax.set_ylabel("z error (cm)"); ax.set_title(ttl)
    summ = [variant_summary(rows, v) for v in order if by.get(v)]
    labs = [s_["variant"] for s_ in summ]
    ypos = np.arange(len(labs))
    axc.barh(ypos, [s_["steady_mean_cm"] for s_ in summ], xerr=[s_["steady_mean_sd_cm"] for s_ in summ],
             color=[VARIANT_COLOR[l] for l in labs]); axc.axvline(0, color="k", lw=0.6)
    axc.set_yticks(ypos); axc.set_yticklabels(labs, fontsize=7); axc.invert_yaxis()
    axc.set_xlabel("steady z error, mean ± sd across flights (cm)"); axc.set_title("steady height tracking")
    axl = fig.add_subplot(gs[1, :]); axl.axis("off")
    axl.legend(handles=handles, loc="center", ncol=5, fontsize=8, frameon=False)
    fig.tight_layout(); fig.savefig(FIGS / "all_variants.png"); plt.close(fig)

    # 6 study variants only (decision 2026-10-09: Omar INDI = Omar C + Iz 1.5; its flights of both days are pooled here)
    study = [("Geometric baseline", ["Geometric baseline"], "#4D4D4D"),
             ("NS2 res_sign −1", ["NS2 res_sign −1"], "#009E73"),
             ("Ours INDI (filter ON, 10-09)", ["Ours INDI (10-09 filter ON)"], "#D55E00"),
             ("Omar C + Iz 1.5 (10-08 + 10-09)", ["Omar C + Iz 1.5", "Omar C + Iz 1.5 (10-09)"], "#005AB5")]
    fig = plt.figure(figsize=(17, 7))
    gs = fig.add_gridspec(2, 4, width_ratios=[2.6, 1, 1, 1], height_ratios=[1, 1])
    axt = fig.add_subplot(gs[:, 0]); axs = [fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[0, 2]), fig.add_subplot(gs[0, 3]), fig.add_subplot(gs[1, 1])]
    stats = {}
    for lab, keys, col in study:
        fl = [r for k in keys for r in by.get(k, [])]
        mat = np.vstack([on_grid(r) for r in fl])
        axt.fill_between(T_GRID, np.nanpercentile(mat, 10, axis=0), np.nanpercentile(mat, 90, axis=0), color=col, alpha=0.15)
        axt.plot(T_GRID, np.nanmedian(mat, axis=0), color=col, lw=2.0, label=f"{lab} (n={len(fl)})")
        stats[lab] = dict(col=col, steady=[r.steady_mean_cm for r in fl], dip=[d for r in fl for d in r.dips_cm],
                          zsd=[r.steady_sd_cm for r in fl], lat=[r.lat_rms_m * 100 for r in fl if r.lat_rms_m is not None])
    band(axt); axt.set_ylim(-16, 8); axt.set_xlabel("time since scenario start (s)"); axt.set_ylabel("z error (cm)")
    axt.set_title("Study variants on A8 — median z error (band: 10–90 %)"); axt.legend(loc="lower right", fontsize=9)
    for ax, key, ttl in zip(axs, ("steady", "dip", "zsd", "lat"), ("steady z error (cm)", "crossing dip (cm)", "z sd, steady (cm)", "lateral RMS (cm)")):
        for i, (lab, st) in enumerate(stats.items()):
            v = np.asarray(st[key], float)
            ax.bar(i, v.mean(), yerr=(v.std(ddof=1) if len(v) > 1 else 0), color=st["col"], capsize=3)
            ax.text(i, v.mean() + (0.15 if v.mean() >= 0 else -0.15), f"{v.mean():+.1f}" if key in ("steady", "dip") else f"{v.mean():.1f}", ha="center", va="bottom" if v.mean() >= 0 else "top", fontsize=8)
        ax.set_xticks(range(len(stats))); ax.set_xticklabels(["geo", "NS2−1", "ours", "Omar C"], fontsize=8); ax.set_title(ttl, fontsize=10); ax.axhline(0, color="k", lw=0.5)
    fig.suptitle("Study variants, A8 bottom drone (uSD where available; mean ± sd over flights, dips over crossings)")
    fig.tight_layout(); fig.savefig(FIGS / "study_variants.png"); plt.close(fig)


def write_tables(rows: list[FlightMetrics], excluded: list[dict]) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with open(OUT / "flights_used.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(rows[0]).keys()) if rows else [])
        if rows:
            w.writeheader()
            for r in rows:
                d = asdict(r)
                d["dips_cm"] = json.dumps(d["dips_cm"])
                w.writerow(d)
    with open(OUT / "flights_excluded.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["date", "stamp", "variant", "reason"])
        w.writeheader()
        w.writerows(excluded)

    summaries = []
    for lab in TABLE_ORDER:
        if lab == "NS2 off (= geometric)":
            s = variant_summary(rows, "Geometric baseline")
            s["variant"] = lab
        else:
            s = variant_summary(rows, lab)
        summaries.append(s)

    cols = [
        "variant",
        "n",
        "source",
        "steady_mean_cm",
        "steady_mean_sd_cm",
        "steady_sd_pooled_cm",
        "rms_z_cm",
        "max_abs_z_cm",
        "dip_mean_cm",
        "dip_sd_cm",
        "dip_rel_mean_cm",
        "lat_rms_m",
        "roll_p99",
        "pitch_p99",
        "gyro_x_sd",
        "pwm_ceiling_frac",
        "vbat_min",
    ]
    with open(OUT / "table_a8_variants.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for s in summaries:
            w.writerow(s)

    lines = ["| variant | n | source | steady mean ± sd (cm) | pooled steady sd | rms_z | max abs z (worst flight) | dip abs mean ± sd | dip rel. to steady | lat_rms (m) | roll / pitch p99 (deg) | gyro_x sd | PWM-ceiling frac | vbat min |",
             "|---|---:|---|---|---:|---:|---:|---|---:|---:|---|---:|---:|---:|"]
    for s in summaries:
        if s["n"] == 0:
            lines.append(f"| {s['variant']} | 0 |" + " — |" * 12)
            continue
        lines.append(
            f"| {s['variant']} | {s['n']} | {s['source']} | "
            f"{s['steady_mean_cm']:+.2f} ± {s['steady_mean_sd_cm']:.2f} | {s['steady_sd_pooled_cm']:.2f} | "
            f"{s['rms_z_cm']:.2f} | {s['max_abs_z_cm']:.1f} | {s['dip_mean_cm']:+.2f} ± {s['dip_sd_cm']:.2f} | {s['dip_rel_mean_cm']:+.2f} | "
            f"{s['lat_rms_m']:.3f} | {s['roll_p99']:.0f} / {s['pitch_p99']:.0f} | {s['gyro_x_sd']:.0f} | {s['pwm_ceiling_frac']:.3f} | {s['vbat_min']:.2f} |"
        )
    (OUT / "table_a8_variants.md").write_text("\n".join(lines) + "\n")


def cross_check(rows: list[FlightMetrics]) -> list[str]:
    rep = []
    tol = 0.3

    def check(name: str, got: float, exp: float):
        d = abs(got - exp)
        status = "reproduced" if d <= tol else f"differs by {d:.2f} cm"
        rep.append(f"{name}: got {got:+.2f} expected {exp:+.2f} → {status}")

    for iz, exp_mean in [(1.0, 6.29), (1.5, 1.52), (2.0, 2.13)]:
        fl = [r for r in rows if r.variant == f"Omar Rust + Iz {iz:.1f}"]
        if fl:
            check(f"Omar Rust Iz {iz}", float(np.mean([r.steady_mean_cm for r in fl])), exp_mean)
    for iz, exp_mean in [(1.0, 2.21), (1.5, 2.18), (2.0, 1.46)]:
        fl = [r for r in rows if r.variant == f"Omar C + Iz {iz:.1f}"]
        if fl:
            check(f"Omar C Iz {iz}", float(np.mean([r.steady_mean_cm for r in fl])), exp_mean)
    rust_ex = [r for r in rows if r.variant == "Omar Rust exact (kpos_iz 0)"]
    if rust_ex:
        check("Omar Rust exact (older meeting-doc value, other window)", float(np.mean([r.steady_mean_cm for r in rust_ex])), 19.0)
    c_ex = [r for r in rows if r.variant == "Omar C exact (Kpos_Iz 0)"]
    if c_ex:
        check("Omar C exact (older meeting-doc value, other window)", float(np.mean([r.steady_mean_cm for r in c_ex])), 22.0)

    geo_1008 = [r for r in rows if r.variant == "Geometric baseline" and r.date == "2026-10-08"]
    if geo_1008:
        dips = [d for r in geo_1008 for d in r.dips_cm]
        check("NS2 off dips 10-08", float(np.mean(dips)), -5.92)
    m1 = [r for r in rows if r.variant == "NS2 res_sign +1"]
    if m1:
        dips = [d for r in m1 for d in r.dips_cm]
        check("NS2 +1 dips", float(np.mean(dips)), -10.97)
    mneg = [r for r in rows if r.variant == "NS2 res_sign −1"]
    if mneg:
        dips = [d for r in mneg for d in r.dips_cm]
        check("NS2 −1 dips", float(np.mean(dips)), -3.41)
    return rep


def write_doc71(rows: list[FlightMetrics], excluded: list[dict], cross: list[str]) -> None:
    md = REPO / "docs" / "71_A8_Variant_Comparison.md"
    table = (OUT / "table_a8_variants.md").read_text()
    body = f"""# 71 — A8 variant comparison (2026-10-09)

**Question:** On scenario A8 (cf5 bottom, dz=0.5 m, height 0.5 m, 4 passes, 26 s), how well does each controller track commanded height, and how do variants differ? Crossing dips are reported but are not the primary ranking criterion.

## Method
- Window `W`: from first `z>0.3 m` + 6 s to last airborne − 4 s. Steady set = `W` minus ±1.5 s around each of 4 crossings (`a8_crossings.zero_crossings`: zero crossings of `y` with hysteresis, 4 expected 6 s apart; 3 if a radio log ends early).
- `e_z = (z − z_sp)·100` cm; uSD preferred (`ctrltarget_z`), radio fallback (`z_sp=0.5 m`).
- Every flight verified against `A8_<date>_<stamp>.meta.json`; Iz values for 10-08 Omar flights assigned by flight order (docs/69/70), not stored in meta. NS2 `rnn.en` / `res_sign` from session yaml where absent in meta.

## Summary table

{table}

## Figures (traces: time since scenario start, every flight aligned on its first crossing = 5.0 s; radio flights drop the takeoff part, everything stops at 26 s)
![INDI variants](../experiments/analysis/out/a8_compare_2026-10-09/figs/indi_variants_z.png)
![Omar Iz ladder](../experiments/analysis/out/a8_compare_2026-10-09/figs/omar_iz_ladder.png)
![Geometric baseline](../experiments/analysis/out/a8_compare_2026-10-09/figs/geometric_z.png)
![NS2 network off / +1 / -1](../experiments/analysis/out/a8_compare_2026-10-09/figs/ns2_on_off.png)
![Study variants (geometric, NS2, ours INDI, Omar C + Iz 1.5)](../experiments/analysis/out/a8_compare_2026-10-09/figs/study_variants.png)
![All variants incl. history](../experiments/analysis/out/a8_compare_2026-10-09/figs/all_variants.png)

## Cross-checks (tolerance 0.3 cm; reference = independent re-runs of `omar_iz_a8_2026_10_08.py`, `omar_c_iz_a8_2026_10_08.py` and the radio NS2 analysis with the same robust detector; NS2 −1 reference is radio-only, the table mixes 3 radio + 1 uSD)
"""
    body += "\n".join(f"- {line}" for line in cross) + "\n\n"
    body += """## Factual reading (A8 cf5, this dataset; updated 2026-10-09 evening)
**Study variants** (final configurations, uSD): geometric baseline, NS2 `res_sign −1`, ours INDI (filter ON, unified firmware), **Omar C + Iz 1.5 (the only Omar variant of the study, decision 2026-10-09)**. The other rows are reference / history.
- **Steady z error:** geometric +0.8 cm, NS2 −1 +0.3, ours INDI +1.9, Omar C + Iz 1.5 +2.2 (10-08) and +2.6 (10-09; four flights over two days: +2.4). All within ≈ 2.6 cm of the command.
- **Ours INDI vs Omar C + Iz 1.5:** same level, but ours is much tighter — z sd 0.5 vs 1.6–2.0 cm, lateral RMS 0.8 vs ≈ 3 cm, roll p99 7° vs 14–22° — and its crossing dips are ≈ 1.5 cm shallower (−9.3 vs −10.7 cm; relative −11.1 vs −13.3). Omar C + Iz 1.5 reproduces across days (+2.2 → +2.6 cm). Ours is weak on A1 (saturation, oscillation) — not part of this table.
- **Geometric and NS2 are the tightest in z** (dips −5.9 cm and −3.2 cm).
- **Omar without the integral:** the offset is not fixed — +21.9 (Rust, 10-02), +23.2 (C, 10-02), +13.3 (Rust, 10-09 same-day baseline, fresh battery); with the integral +1.5…+2.6 cm. History only, no longer a study variant.
- **Ours INDI, filter OFF vs ON (same day):** same level and dips within the battery confound (OFF #2 sagged to 3.15 V); the 10-02 level (+4.2 cm) is 2.3 cm higher than both 10-09 groups (docs/lab_sessions/2026-10-09.md).
- **Dip columns:** the absolute dip of the "exact" Omar variants is positive/small because their level sits +13…+23 cm high; compare variants with "dip rel. to steady". Dips are negative numbers, shallower is better.
- **Network off = geometric:** the NS2 "network off" cohort IS the geometric baseline (same controller, `rnn.en 0`), so both rows are identical by construction.

## Excluded flights
"""
    body += "\n".join(f"- {e['date'][5:]} {e['stamp']} ({e['variant']}): {e['reason']}" for e in excluded) + """

Notes: 10-03 `13-15-00` is a genuine tumble on both logs (radio roll up to 180°, z error +43 cm in the window), not a gate artefact; 10-02 `19-08-13` ours INDI was an early abort (airborne ≈ 13 s); 10-03 `13-10-32` ours INDI ended in a crash after ≈ 12 s; the 10-05 flights listed as "candidate not used" are the crash / pose-swap attempts of that session.

## Caveats
- Different days and firmware builds (10-02/03 vs 10-05 vs 10-08); the "Omar exact" baselines are not same-session as the "+Iz" flights (a same-session baseline is planned for the next lab day).
- n is small: ours INDI n = 2, Omar variants n = 2 each, geometric n = 4, NS2 n = 3–4; "± sd across flights" with n = 2 is only a spread indicator.
- Omar Iz values (1.0/1.5/2.0) are assigned from the flight order (docs/69, docs/70); the meta files do not store them.
- NS2 `res_sign −1` mixes three radio-only flights (cf5 uSD lost on 10-08 17-16-27, 17-22-13, 17-24-54) with one uSD flight; radio samples at ≈ 20 Hz and set-point 0.5 m, so the radio dips are shallower by ≈ 0.2–0.6 cm than uSD would give (compare +1: radio −10.35 vs uSD −11.00). Lateral RMS and PWM-ceiling are uSD-only.
- 10-08 17-16-27 had a cf5 battery sag (vbat min ≈ 2.5 V) — kept, flagged.
- The 10-05 geometric/NS2 uSD files are matched by `usd_run_tag` (card stamp ≠ radio stamp).
- The PWM-ceiling fraction in the table is taken inside the window W; docs/69 and docs/70 quote it over the whole airborne time (takeoff and landing included), which is higher.
- Crossings: all 29 flights use `a8_crossings.zero_crossings` (uSD crossing times 5.01/11.05/17.02/23.05 s since liftoff, sd ≤ 0.05 s); radio logs that end before pass 4 contribute 3 crossings. The older minima-of-|y| detector mislocated a crossing in 8 of 29 flights and is no longer used.
- Observational comparison: battery, tracker and day effects are not separated; the top drone is geometric in all flights.

## Artifacts
Script: `experiments/analysis/a8_variant_comparison.py`  
Outputs: `experiments/analysis/out/a8_compare_2026-10-09/`
"""
    md.write_text(body)


def main() -> None:
    import os

    os.environ["PYTHONWARNINGS"] = "ignore"
    _build_usd_index()
    rows, excluded, _ = process_all()
    if not rows:
        print("No flights included", file=sys.stderr)
        sys.exit(1)
    write_tables(rows, excluded)
    save_figures(rows)
    cross = cross_check(rows)
    write_doc71(rows, excluded, cross)
    print(f"Included {len(rows)} flights, excluded {len(excluded)}")
    for line in cross:
        print(line)


if __name__ == "__main__":
    main()
