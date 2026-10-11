#!/usr/bin/env python3
"""Per-flight study metrics (comparative study protocol §6). One row per (flight, drone)."""
from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path
from typing import Any

import numpy as np

REPO = Path(__file__).resolve().parents[2]
LOGS = REPO / "experiments" / "logs"
USD_DIR = LOGS / "usd_raw"
OUT_DEFAULT = Path(__file__).resolve().parent / "out" / "study_metrics"

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
sys.path.insert(0, str(REPO / "experiments" / "analysis"))
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

import metrics as M  # noqa: E402
from a8_crossings import zero_crossings  # noqa: E402
from decode_usd_log import load as load_usd_raw  # noqa: E402
from study_setting import extract_setting, stamp_from_meta_path  # noqa: E402

Z_AIR_M = 0.3
W_AFTER_LIFTOFF_S = 6.0
W_BEFORE_END_S = 4.0
CROSS_EXCL_S = 1.5
DIP_HALF_S = 1.0
PWM_CEIL = 65000
G = 9.81
LOGS_DIR = REPO / "experiments" / "logs"

_USD_BY_TAG: dict[tuple[int, str], Path] | None = None
_USD_BY_STAMP: dict[tuple[str, str, str], Path] = {}


def _drone_prefix(path_name: str) -> str:
    base = Path(path_name).name
    if base.startswith("cf_second_"):
        return "cf_second"
    if base.startswith("cf5_"):
        return "cf5"
    return base.split("_")[0]


def _build_usd_index() -> None:
    global _USD_BY_TAG
    if _USD_BY_TAG is not None:
        return
    _USD_BY_TAG = {}
    for p in USD_DIR.glob("*.bin"):
        if p.stat().st_size == 0:
            continue
        drone = _drone_prefix(p.name)
        m = re.search(r"(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.bin$", p.name)
        if m:
            _USD_BY_STAMP[(m.group(1), m.group(2), drone)] = p
        try:
            d = load_usd_raw(str(p))
            if "run_tag" in d and len(d["run_tag"]):
                rt = int(round(float(d["run_tag"][0])))
                _USD_BY_TAG[(rt, drone)] = p
        except Exception:
            continue


def find_usd(meta: dict, date: str, stamp: str, drone: str) -> Path | None:
    _build_usd_index()
    assert _USD_BY_TAG is not None
    p = _USD_BY_STAMP.get((date, stamp, drone))
    if p is not None and p.stat().st_size > 0:
        return p
    tag = meta.get("usd_run_tag")
    if tag is not None:
        p = _USD_BY_TAG.get((int(tag), drone))
        if p is not None and p.stat().st_size > 0:
            return p
    hits = sorted(USD_DIR.glob(f"{drone}_*{date}_{stamp}.bin"))
    for h in hits:
        if h.stat().st_size > 0:
            return h
    return None


def quat_to_roll_pitch_deg(qw, qx, qy, qz) -> tuple[np.ndarray, np.ndarray]:
    """Scalar-first quat -> roll, pitch [deg]."""
    roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.degrees(np.arcsin(np.clip(sinp, -1, 1)))
    return roll, pitch


def nominal_ref_tilt_std_deg(d: dict, t: np.ndarray) -> tuple[np.ndarray | None, np.ndarray | None, str]:
    """Reference roll/pitch [deg] in the STANDARD right-hand Euler convention (as computed from the quaternion), yaw 0,
    from the commanded acceleration: tilting the thrust towards +y needs a NEGATIVE roll (R_x(phi) z_b = (0, -sin phi, cos phi)),
    towards +x a POSITIVE pitch. Acceleration source: logged `ctrltarget.a*` if present, else the second derivative of the logged
    `ctrltarget.x/y` (Savitzky-Golay, 0.3 s window). The logged `stabilizer.roll/pitch` follow roll_std but pitch = -pitch_std
    (verified 2026-10-11 on the geometric A2/A3/A4 C.1 flights: corr -0.5..-0.87 of both logged angles with atan(a/g))."""
    if all(k in d for k in ("ctrltarget_ax", "ctrltarget_ay")) and np.nanstd(np.asarray(d["ctrltarget_ax"], float)) + np.nanstd(np.asarray(d["ctrltarget_ay"], float)) > 0:
        ax = np.asarray(d["ctrltarget_ax"], float); ay = np.asarray(d["ctrltarget_ay"], float); src = "ctrltarget.a*"
    elif all(k in d for k in ("ctrltarget_x", "ctrltarget_y")) and len(t) > 50:
        from scipy.signal import savgol_filter
        dt = float(np.median(np.diff(t)))
        win = max(int(0.3 / dt) | 1, 7)
        ax = savgol_filter(np.asarray(d["ctrltarget_x"], float), win, 3, deriv=2, delta=dt)
        ay = savgol_filter(np.asarray(d["ctrltarget_y"], float), win, 3, deriv=2, delta=dt)
        src = "d2(ctrltarget pos)"
    else:
        return None, None, ""
    roll_ref = -np.degrees(np.arctan2(ay, G))
    pitch_ref = np.degrees(np.arctan2(ax, G))
    return roll_ref, pitch_ref, src


def measured_att_std_deg(d: dict) -> tuple[np.ndarray, np.ndarray, str]:
    """Measured roll/pitch [deg] in the standard convention: from the logged quaternion if present (new config),
    else from stabilizer.roll / -stabilizer.pitch."""
    if all(k in d for k in ("est_qw", "est_qx", "est_qy", "est_qz")) and np.nanstd(np.asarray(d["est_qw"], float)) > 0:
        r, p_ = quat_to_roll_pitch_deg(np.asarray(d["est_qw"], float), np.asarray(d["est_qx"], float),
                                       np.asarray(d["est_qy"], float), np.asarray(d["est_qz"], float))
        return r, p_, "quaternion"
    return np.asarray(d.get("roll_deg", []), float), -np.asarray(d.get("pitch_deg", []), float), "stabilizer.roll/pitch"


def _rms(x) -> float:
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")


def _p99(x) -> float:
    x = np.abs(np.asarray(x, float))
    return float(np.percentile(x, 99)) if len(x) else float("nan")


def radio_vbat_min(scen: str, drone: str, date: str, stamp: str) -> float | None:
    """Fallback for uSD logs recorded before `pm.vbat` was in the config: min loaded battery voltage from the radio CSV."""
    p = LOGS_DIR / f"{scen}_{drone}_{date}_{stamp}.csv"
    if not p.is_file():
        return None
    try:
        import pandas as pd
        df = pd.read_csv(p, comment="#")
        if "vbat" in df.columns:
            vb = df["vbat"].astype(float).values
            vb = vb[vb > 2.0]
            return float(vb.min()) if len(vb) else None
    except Exception:
        return None
    return None


def pass_dips_cm(grid, ez_cm, horiz, thresh=0.25, min_gap=1.5, half=DIP_HALF_S) -> list[float]:
    """Dip per pass for scenarios where one drone passes under the other (A3, A4, A8-like): pass = local minimum of the
    horizontal separation below `thresh` m (>= min_gap s apart); dip = lowest z error within +-half s of it."""
    h = np.asarray(horiz, float)
    idx = [i for i in range(1, len(h) - 1) if h[i] <= h[i - 1] and h[i] < h[i + 1] and h[i] < thresh]
    times, last = [], -1e9
    for i in idx:
        if grid[i] - last >= min_gap:
            times.append(grid[i]); last = grid[i]
    out = []
    for tc in times:
        m = (grid >= tc - half) & (grid <= tc + half)
        if np.any(m):
            out.append(float(np.min(ez_cm[m])))
    return out


def steady_window(t, z, scenario: str, y=None, duration: float = 26.0) -> np.ndarray:
    up = z > Z_AIR_M
    if not np.any(up):
        return np.zeros_like(t, dtype=bool)
    t0 = float(t[up][0])
    t1 = float(t[up][-1])
    w_lo, w_hi = t0 + W_AFTER_LIFTOFF_S, t1 - W_BEFORE_END_S
    steady = (t >= w_lo) & (t <= w_hi) & up
    if scenario == "A8" and y is not None:
        cross_t = zero_crossings(t[up], y[up])
        for tc in cross_t:
            steady &= ~((t >= tc - CROSS_EXCL_S) & (t <= tc + CROSS_EXCL_S))
    return steady


def a8_dip_mean_cm(t, z, z_sp, y) -> float:
    e_z = (z - z_sp) * 100.0
    up = z > Z_AIR_M
    steady = steady_window(t, z, "A8", y=y)
    cross_t = zero_crossings(t[up], y[up])
    dips = []
    for tc in cross_t:
        m = (t >= tc - DIP_HALF_S) & (t <= tc + DIP_HALF_S)
        if np.any(m):
            dips.append(float(np.min(e_z[m])))
    if not dips:
        return float("nan")
    return float(np.mean(dips))


def sync_pair(va: M.VehicleLog, vb: M.VehicleLog) -> tuple[np.ndarray, np.ndarray, str]:
    """Common time grid for relative metrics."""
    t_lo = max(va.t[0], vb.t[0])
    t_hi = min(va.t[-1], vb.t[-1])
    if t_hi <= t_lo:
        return va.t[:0], vb.t[:0], "no_overlap"
    rate_a = (len(va.t) - 1) / max(va.t[-1] - va.t[0], 1e-9)
    rate_b = (len(vb.t) - 1) / max(vb.t[-1] - vb.t[0], 1e-9)
    grid = va.t[(va.t >= t_lo) & (va.t <= t_hi)] if rate_a <= rate_b else vb.t[(vb.t >= t_lo) & (vb.t <= t_hi)]
    return grid, grid, "usd_pair_interp"


def separation_on_grid(grid, a: M.VehicleLog, b: M.VehicleLog, dz_cmd: float) -> dict:
    z_a = np.interp(grid, a.t, a.pos[:, 2])
    z_b = np.interp(grid, b.t, b.pos[:, 2])
    dz = z_a - z_b if np.nanmean(a.pos[:, 2]) >= np.nanmean(b.pos[:, 2]) else z_b - z_a
    sag = dz_cmd - dz
    horiz = np.sqrt(
        (np.interp(grid, a.t, a.pos[:, 0]) - np.interp(grid, b.t, b.pos[:, 0])) ** 2
        + (np.interp(grid, a.t, a.pos[:, 1]) - np.interp(grid, b.t, b.pos[:, 1])) ** 2
    )
    sep_3d = np.sqrt(horiz ** 2 + dz ** 2)
    return {
        "sep_err_mean_m": float(np.mean(sag)),
        "sep_err_rms_m": float(np.sqrt(np.mean(sag ** 2))),
        "min_separation_m": float(np.min(sep_3d)),
    }


def load_decisions(path: Path) -> dict[tuple[str, str, str], dict]:
    if not path.is_file():
        return {}
    out = {}
    with open(path) as fh:
        for row in csv.DictReader(fh):
            key = (row["date"], row["stamp"], row.get("scenario", ""))
            out[key] = row
    return out


def metrics_for_drone(
    meta: dict,
    drone: str,
    usd_path: Path,
    decision: dict | None,
) -> dict[str, Any]:
    d = load_usd_raw(str(usd_path))
    v = M.load_usd(usd_path, name=drone)
    t = np.asarray(d["t"], float)
    t = t - t[0]
    v.t = t
    if v.pos is not None:
        v.pos = v.pos.copy()
    z = v.pos[:, 2] if v.pos is not None else np.zeros_like(t)
    y = v.pos[:, 1] if v.pos is not None else np.zeros_like(t)
    steady = steady_window(t, z, meta.get("scenario", ""), y=y if meta.get("scenario") == "A8" else None)
    scen = meta.get("scenario", "")
    setting = extract_setting(meta)
    row = M.vehicle_metrics(v, scen, "study", 2, None)
    row.update(M.control_effort(v))
    eff = M.control_effort(v)
    row["motor_sat_frac"] = eff.get("motor_sat_frac")
    if v.gyro is not None and len(v.gyro):
        row["gyro_x_sd"] = float(np.std(v.gyro[steady, 0])) if np.any(steady) else float("nan")
    else:
        row["gyro_x_sd"] = float("nan")
    if v.pos is not None and v.pos_des is not None:
        err = v.pos - v.pos_des
        row["steady_z_err_mean_cm"] = float(np.mean(err[steady, 2]) * 100) if np.any(steady) else float("nan")
        row["z_rms_cm"] = float(np.sqrt(np.mean(err[steady, 2] ** 2)) * 100) if np.any(steady) else float("nan")
        lat = np.linalg.norm(err[steady, :2], axis=1) if np.any(steady) else np.array([])
        row["lateral_rms_m"] = float(np.sqrt(np.mean(lat ** 2))) if len(lat) else float("nan")
    roll = np.asarray(d.get("roll_deg", np.zeros_like(t)), float)
    pitch = np.asarray(d.get("pitch_deg", np.zeros_like(t)), float)
    yaw = np.asarray(d.get("yaw_deg", np.zeros_like(t)), float)
    sw = steady
    row["roll_p99_deg"] = _p99(roll[sw]); row["pitch_p99_deg"] = _p99(pitch[sw]); row["yaw_p99_deg"] = _p99(yaw[sw])
    row["roll_max_deg"] = float(np.max(np.abs(roll[sw]))) if np.any(sw) else float("nan")
    row["pitch_max_deg"] = float(np.max(np.abs(pitch[sw]))) if np.any(sw) else float("nan")
    # measured attitude, all controllers (as logged: stabilizer.roll/pitch/yaw, degrees)
    row["roll_rms_deg"] = _rms(roll[sw]); row["pitch_rms_deg"] = _rms(pitch[sw]); row["yaw_rms_deg"] = _rms(yaw[sw])

    vbat = d.get("vbat")
    row["vbat_min_v"] = float("nan")
    row["vbat_source"] = "none"
    if vbat is not None and np.nanstd(np.asarray(vbat, float)) > 0:
        vb = np.asarray(vbat, float)
        loaded = vb[vb > 2.0]
        if len(loaded):
            row["vbat_min_v"] = float(np.min(loaded))
            row["vbat_source"] = "usd_pm.vbat"
    if row["vbat_source"] == "none":
        rv = radio_vbat_min(meta.get("scenario", ""), drone, meta.get("_date", ""), meta.get("_stamp", ""))
        if rv is not None:
            row["vbat_min_v"] = rv
            row["vbat_source"] = "radio_csv"
    if decision and decision.get("vbat_min"):
        try:
            row["vbat_min_v"] = float(decision["vbat_min"])
            row["vbat_source"] = "flight_decisions.csv"
        except ValueError:
            pass

    # controller-independent attitude deviation from the nominal (flat-reference) attitude. NOT the controller's own error:
    # it includes the tilt the position feedback legitimately commands. Conventions verified on real flights (see helper).
    rr, pr, ref_src = nominal_ref_tilt_std_deg(d, t)
    for k in ("roll", "pitch"):
        row[f"att_vs_nom_ref_{k}_rms_deg"] = float("nan"); row[f"att_vs_nom_ref_{k}_p99_deg"] = float("nan")
    row["att_ref_source"] = ref_src
    row["att_meas_source"] = ""
    if rr is not None and np.any(sw):
        mr, mp, msrc = measured_att_std_deg(d)
        row["att_meas_source"] = msrc
        row["att_vs_nom_ref_roll_rms_deg"] = _rms((mr - rr)[sw]); row["att_vs_nom_ref_roll_p99_deg"] = _p99((mr - rr)[sw])
        row["att_vs_nom_ref_pitch_rms_deg"] = _rms((mp - pr)[sw]); row["att_vs_nom_ref_pitch_p99_deg"] = _p99((mp - pr)[sw])
    # controller-internal attitude errors, labelled as such, NOT comparable across controllers
    # (Omar Rust's R_des is ~level: its e_r follows the measured angle; Omar C does not log e_r at all)
    row["e_r_note"] = "controller_internal_not_comparable_across_controllers"
    for i_, ax_ in enumerate("xyz"):
        row[f"e_r_{ax_}_rms_deg"] = (float(np.sqrt(np.mean(np.degrees(v.e_r[steady, i_]) ** 2))) if (v.e_r is not None and np.any(steady)) else float("nan"))
    for ax_ in "xyz":
        k1, k2 = f"omar_rpy_{ax_}", f"omar_rpyd_{ax_}"
        if k1 in d and k2 in d and np.any(steady) and np.nanstd(np.asarray(d[k1], float)) > 0:
            row[f"omar_att_err_{ax_}_rms_raw"] = _rms((np.asarray(d[k2], float) - np.asarray(d[k1], float))[steady])  # units as logged

    row["primary_metric"] = float("nan")
    row["primary_metric_name"] = ""
    if scen in ("A2", "A5") and drone == "cf5":
        row["primary_metric_name"] = "pos_rmse_m"      # tracking RMSE against the commanded trajectory (+ sep_err_rms_m below)
        row["primary_metric"] = row.get("pos_rmse_m", float("nan"))
    elif scen == "A4" and drone == "cf5":
        row["primary_metric_name"] = "lateral_rms_m"   # lateral error; z error in steady_z_err_mean_cm / z_rms_cm
        row["primary_metric"] = row.get("lateral_rms_m", float("nan"))
    elif scen == "A1":
        row["primary_metric_name"] = "steady_z_err_mean_cm"
        row["primary_metric"] = row.get("steady_z_err_mean_cm", float("nan"))
    elif scen == "A8" and drone == "cf5":
        z_sp = v.pos_des[:, 2] if v.pos_des is not None else np.full_like(t, float(meta.get("height", 0.5)))
        row["primary_metric_name"] = "a8_dip_mean_cm"
        row["primary_metric"] = a8_dip_mean_cm(t, z, z_sp, y)
        row["n_crossings"] = len(zero_crossings(t[z > Z_AIR_M], y[z > Z_AIR_M]))

    row.update(setting)
    row["vehicle_id"] = drone
    row["usd_path"] = str(usd_path)
    return row


def process_flight(meta_path: Path, decisions_path: Path) -> list[dict]:
    meta = json.loads(meta_path.read_text())
    meta["_file"] = str(meta_path)
    m = re.match(r".*_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})\.meta\.json$", meta_path.name)
    date, stamp = (m.group(1), m.group(2)) if m else ("", "")
    meta["_date"], meta["_stamp"] = date, stamp
    scen = meta.get("scenario", "")
    decisions = load_decisions(decisions_path)
    dec = decisions.get((date, stamp, scen)) or decisions.get((date, stamp, ""))
    names = meta.get("names") or ["cf5", "cf_second"]
    rows = []
    logs = {}
    for drone in names:
        p = find_usd(meta, date, stamp, drone)
        if p is None:
            continue
        logs[drone] = M.load_usd(p, name=drone)
        r = metrics_for_drone(meta, drone, p, dec)
        r["date"] = date
        r["stamp"] = stamp
        r["scenario"] = scen
        r["strategy"] = dec.get("strategy", "") if dec else ""
        r["decision"] = dec.get("decision", "") if dec else ""
        r["meta_path"] = str(meta_path.relative_to(REPO))
        rows.append(r)
    if len(logs) >= 2:
        a, b = logs[names[0]], logs[names[1]]
        grid, _, sync_m = sync_pair(a, b)
        dz_cmd = float(meta.get("params", {}).get("dz", meta.get("height", 0.5)))
        sep = separation_on_grid(grid, a, b, dz_cmd) if len(grid) else {}
        for r in rows:
            r["sync_method"] = sync_m
            r.update(sep)
        # pass events for scenarios where one drone passes under the other: dip per pass averaged per flight (A3 primary)
        if scen in ("A3", "A4") and len(grid):
            ta = np.asarray(a.t, float); tb = np.asarray(b.t, float)  # common clock (usec.reset on the ground), do not zero per drone
            try:
                pa = np.column_stack([np.interp(grid, ta, a.pos[:, k]) for k in range(3)])
                pb = np.column_stack([np.interp(grid, tb, b.pos[:, k]) for k in range(3)])
                sp = np.interp(grid, ta, a.pos_des[:, 2]) if a.pos_des is not None else np.full_like(grid, 0.5)
                horiz = np.hypot(pa[:, 0] - pb[:, 0], pa[:, 1] - pb[:, 1])
                dips = pass_dips_cm(grid, (pa[:, 2] - sp) * 100.0, horiz)
                for r in rows:
                    if r["vehicle_id"] == names[0]:
                        r["n_passes"] = len(dips)
                        r["pass_dip_mean_cm"] = float(np.mean(dips)) if dips else float("nan")
                        if scen == "A3":
                            r["primary_metric_name"] = "a3_pass_dip_mean_cm"
                            r["primary_metric"] = r["pass_dip_mean_cm"]
            except Exception as exc:  # noqa: BLE001 - keep the row, record why the pass metric is missing
                for r in rows:
                    r["notes"] = f"pass metric failed: {exc}"
        if scen == "A7":
            for r in rows:
                if r["vehicle_id"] == names[0]:
                    r["primary_metric_name"] = "sep_err_rms_m"   # dz error vs commanded dz(t); min separation in min_separation_m
                    r["primary_metric"] = r.get("sep_err_rms_m", float("nan"))
    return rows


def write_rows(rows: list[dict], out: Path) -> None:
    if not rows:
        return
    out.parent.mkdir(parents=True, exist_ok=True)
    keys = sorted({k for r in rows for k in r.keys()})
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)


def regression_a8() -> int:
    ref = REPO / "experiments/analysis/out/a8_compare_2026-10-09/flights_used.csv"
    if not ref.is_file():
        print(f"MISSING {ref}")
        return 1
    import pandas as pd  # noqa: WPS433

    got_rows = []
    for meta_path in sorted(LOGS.glob("A8_2026-10-0*.meta.json")):
        got_rows.extend(process_flight(meta_path, LOGS / "flight_decisions.csv"))
    got = {}
    for r in got_rows:
        if r.get("vehicle_id") != "cf5":
            continue
        got[f"{r['date']}_{r['stamp']}"] = r
    ref_df = pd.read_csv(ref)
    fails = 0
    for _, row in ref_df.iterrows():
        if row.get("source") != "usd":
            continue
        key = f"{row['date']}_{row['stamp']}"
        g = got.get(key)
        if not g:
            continue
        sm = float(row["steady_mean_cm"])
        dm = float(row["dip_mean_cm"])
        gs = g.get("steady_z_err_mean_cm", float("nan"))
        gd = g.get("primary_metric", float("nan"))
        if abs(gs - sm) > 0.05 or (np.isfinite(dm) and abs(gd - dm) > 0.05):
            print(f"FAIL {key}: steady {gs:.3f} vs {sm:.3f}, dip {gd:.3f} vs {dm:.3f}")
            fails += 1
        else:
            print(f"OK {key}: steady {gs:.2f} dip {gd:.2f}")
    return fails


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--meta", type=Path, help="single flight meta.json")
    ap.add_argument("--all-a8", action="store_true", help="all A8 sidecars with uSD")
    ap.add_argument("--all", action="store_true", help="all study-scenario sidecars (A1-A5, A7, A8) that have uSD logs")
    ap.add_argument("--decisions", type=Path, default=LOGS / "flight_decisions.csv")
    ap.add_argument("--out", type=Path, default=OUT_DEFAULT / "study_rows.csv")
    ap.add_argument("--regression-a8", action="store_true")
    args = ap.parse_args()
    if args.regression_a8:
        sys.exit(regression_a8())
    rows: list[dict] = []
    if args.all_a8:
        for p in sorted(LOGS.glob("A8_*.meta.json")):
            rows.extend(process_flight(p, args.decisions))
    elif args.all:
        for p in sorted(LOGS.glob("A[1-8]_*.meta.json")):
            if not re.match(r"A[1-578]_\d{4}-", p.name):
                continue
            try:
                rows.extend(process_flight(p, args.decisions))
            except Exception as exc:  # noqa: BLE001 - one broken flight must not stop the table
                print(f"skip {p.name}: {exc}", file=sys.stderr)
    elif args.meta:
        rows.extend(process_flight(args.meta, args.decisions))
    else:
        ap.error("pass --meta, --all-a8 or --all")
    write_rows(rows, args.out)
    print(f"wrote {len(rows)} rows -> {args.out}")


if __name__ == "__main__":
    main()
