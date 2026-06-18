#!/usr/bin/env python3
"""
analyze_indi.py — INDI commissioning dashboard for Crazyflie flight logs.

Loads a CS2 log CSV and produces:
  • Terminal: health check, attitude metrics, filter quality, gain analysis + suggestions
  • PNG:       3×3 dashboard — attitude / torques / filter comparison

Gracefully handles missing columns (NaN sections show placeholder text).

Usage:
    python3 analyze_indi.py                              # auto-picks latest log
    python3 analyze_indi.py logs/hover_....csv
    python3 analyze_indi.py logs/hover_....csv --kr 100 --kw 200 --fc-bw 25
    python3 analyze_indi.py logs/hover_....csv --tau-limit 0.015
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate, welch

LOGS_DIR = Path(__file__).resolve().parent / "logs"

# CF2.1 approximate max torque per axis [N·m] — used for saturation %.
# Override with --tau-limit if your estimate differs.
TAU_LIMIT_NM = 0.010


# ── Data loading ──────────────────────────────────────────────────────────────

def load_meta(path: Path) -> dict:
    """Parse # meta:key=value lines from a CS2 log CSV header."""
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            if line.startswith("# meta:"):
                kv = line[7:].strip()
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    meta[k.strip()] = v.strip()
    return meta


def load_csv(path: Path) -> dict:
    """Load all columns from a CS2 log CSV. Returns dict of np.ndarray."""
    with open(path) as f:
        lines = [l for l in f if not l.startswith("#")]
    if not lines:
        print(f"ERROR: {path} has no data rows")
        sys.exit(1)
    header = lines[0].strip().split(",")
    cols: dict[str, list] = {k: [] for k in header}
    for line in lines[1:]:
        vals = line.strip().split(",")
        if len(vals) != len(header):
            continue
        for k, v in zip(header, vals):
            try:
                cols[k].append(float(v))
            except ValueError:
                cols[k].append(float("nan"))
    data = {k: np.array(v) for k, v in cols.items()}
    # CS2 logs two phases (takeoff + trajectory/hover) with separate resetting timers.
    # Make timestamps monotone so matplotlib draws a single forward-going line.
    ts = data.get("time_s")
    if ts is not None:
        for i in range(1, len(ts)):
            if ts[i] < ts[i - 1]:
                ts[i:] += ts[i - 1]
                break
    return data


def col(data: dict, name: str, n: int) -> np.ndarray:
    return data[name] if name in data else np.full(n, float("nan"))


def has_data(arr: np.ndarray) -> bool:
    return bool(np.any(np.isfinite(arr) & (arr != 0.0)))


def ctrl_mode_from_meta(meta: dict) -> int | None:
    """Return trajectory controller mode from CSV header meta, if present."""
    for key in ("trajectory_ctrl_mode", "yaml_ctrl_mode"):
        if key in meta:
            try:
                return int(meta[key])
            except ValueError:
                pass
    return None


def ctrl_mode_label(mode: int) -> str:
    return {0: "geometric", 1: "pos INDI", 2: "att INDI", 3: "full INDI"}.get(mode, f"mode {mode}")


# ── Signal utilities ──────────────────────────────────────────────────────────

def detect_fs(t: np.ndarray) -> float:
    diffs = np.diff(t[:min(200, len(t))])
    diffs = diffs[diffs > 0]
    return round(1.0 / float(np.median(diffs))) if len(diffs) else 20.0


def find_hover_window(t, z, roll, pitch):
    """
    Returns (i_start, i_end, crashed, i_crash).
    i_start: first sample of steady hover (after takeoff transient).
    i_end:   last usable sample (crash or landing or end of log).
    """
    Z_IN, SKIP_S, ATT_CRASH = 0.15, 1.5, 60.0

    in_flight = np.where(z > Z_IN)[0]
    if len(in_flight) == 0:
        return 0, len(t) - 1, False, None

    i_flight = int(in_flight[0])
    i_start  = min(int(np.searchsorted(t, t[i_flight] + SKIP_S)), len(t) - 1)

    i_end, crashed, i_crash = len(t) - 1, False, None
    for i in range(i_flight, len(t)):
        if abs(roll[i]) > ATT_CRASH or abs(pitch[i]) > ATT_CRASH:
            crashed, i_crash, i_end = True, i, i
            break

    if not crashed:
        for i in range(i_flight + 10, len(t)):
            if z[i] < 0.10:
                i_end = i
                break

    return i_start, i_end, crashed, i_crash


def filter_quality(alp_raw: np.ndarray, alp: np.ndarray, fs: float):
    """
    Returns (noise_reduction_db, phase_lag_ms).
    noise_reduction_db < 0 means the filter attenuated noise.
    phase_lag_ms > 0 means the filtered signal lags the raw.
    """
    mask = np.isfinite(alp_raw) & np.isfinite(alp)
    if mask.sum() < 20:
        return float("nan"), float("nan")
    r, f = alp_raw[mask], alp[mask]
    rms_r = float(np.sqrt(np.mean(r ** 2)))
    rms_f = float(np.sqrt(np.mean(f ** 2)))
    if rms_r < 1e-9:
        return float("nan"), float("nan")
    db = 20.0 * math.log10(rms_f / rms_r) if rms_f > 0 else -99.0

    # Cross-correlation — positive lag = alp lags alp_raw (expected for a filter)
    max_lag = max(1, int(fs * 0.1))   # search within ±100 ms
    xc  = correlate(f - f.mean(), r - r.mean(), mode="full")
    ctr = len(r) - 1
    sl  = xc[max(0, ctr - max_lag): ctr + max_lag + 1]
    lag_samples = int(np.argmax(sl)) - min(max_lag, ctr)
    lag_ms = lag_samples * 1000.0 / fs
    return db, lag_ms


# ── Gain analysis ─────────────────────────────────────────────────────────────

def att_poles(kr: float, kw: float):
    """
    Attitude closed-loop poles assuming perfect INDI inner loop: s²+KW·s+KR=0.
    Returns (wn_rad_s, zeta, tau_dom_ms).
    """
    if kr <= 0:
        return float("nan"), float("nan"), float("nan")
    wn   = math.sqrt(kr)
    zeta = kw / (2.0 * wn)
    tau  = (kw / kr) if zeta >= 1.0 else (1.0 / wn)   # overdamped: -KR/KW pole dominates
    return wn, zeta, tau * 1000.0


def routh_stable(kr: float, kw: float) -> bool:
    """Routh criterion for tau-integrating (3rd-order fallback) mode: s³+s²+KW·s+KR.
    Only applies when RPM deck absent (rpms_active=False). Stable iff KW > KR > 0."""
    return kw > kr > 0


# CF2.1 moments of inertia [kg·m²] — Forster 2015 / Mahony 2012 values
JXX = 16.6e-6
JYY = 16.6e-6
JZZ = 29.0e-6
J_AXES = [JXX, JYY, JZZ]


def tau_alignment(tau_cmd: np.ndarray, j_alpha: np.ndarray, fs: float):
    """
    Cross-correlate tau_cmd vs J*alpha_meas to find lag and correlation.
    Returns (lag_ms, pearson_r, rms_diff_mNm).
    Positive lag_ms means tau_cmd leads j_alpha (expected: tau causes alpha with delay).
    """
    mask = np.isfinite(tau_cmd) & np.isfinite(j_alpha)
    if mask.sum() < 20:
        return float("nan"), float("nan"), float("nan")
    a, b = tau_cmd[mask], j_alpha[mask]
    rms_diff = float(np.sqrt(np.mean((a - b) ** 2))) * 1000.0  # mN·m
    max_lag = max(1, int(fs * 0.1))
    xc  = correlate(a - a.mean(), b - b.mean(), mode="full")
    ctr = len(a) - 1
    sl  = xc[max(0, ctr - max_lag): ctr + max_lag + 1]
    lag_samples = int(np.argmax(sl)) - min(max_lag, ctr)
    lag_ms = lag_samples * 1000.0 / fs
    denom = math.sqrt(float(np.sum((a - a.mean()) ** 2)) * float(np.sum((b - b.mean()) ** 2)))
    r = float(xc[ctr + lag_samples]) / denom if denom > 0 else float("nan")
    return lag_ms, r, rms_diff


def stale_end(tau_col: np.ndarray) -> int:
    """Return index where tau first deviates from the initial stale value.
    tau_prev carries over from previous crash until RPMs kick in (~first few seconds)."""
    init = tau_col[0]
    changed = np.where(np.abs(tau_col - init) > 1e-4)[0]
    return int(changed[0]) if len(changed) else len(tau_col)


def suggest_gains(kr: float, kw: float, incremental: bool = True) -> list[dict]:
    """
    Two progressive gain step suggestions, each raising KR by ~20% while scaling KW
    with sqrt(KR_new/KR_old) to maintain the same damping ratio ζ = KW/(2·sqrt(KR)).
    In integrating mode (RPM=0 fallback, 3rd-order): KW > KR required for Routh stability.
    """
    import math

    if not incremental and not routh_stable(kr, kw):
        # Integrating mode + currently unstable: fix Routh first
        steps = [
            {"kr": round(kr),       "kw": round(kr * 2.0)},
            {"kr": round(kr * 1.5), "kw": round(kr * 2.5)},
        ]
    else:
        # Raise KR in ~20% steps; scale KW to keep ζ constant
        kr1 = round(kr * 1.20)
        kw1 = round(kw * math.sqrt(kr1 / kr))
        kr2 = round(kr * 1.45)
        kw2 = round(kw * math.sqrt(kr2 / kr))
        steps = [
            {"kr": kr1, "kw": kw1},
            {"kr": kr2, "kw": kw2},
        ]
    for s in steps:
        wn, zeta, tdms = att_poles(s["kr"], s["kw"])
        s.update(wn=wn, zeta=zeta, tau_dom_ms=tdms,
                 routh_margin=s["kw"] / s["kr"] if s["kr"] > 0 else float("inf"),
                 stable=routh_stable(s["kr"], s["kw"]))
    return steps


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="INDI commissioning dashboard")
    ap.add_argument("csv", nargs="?", help="Path to flight log CSV (default: latest in logs/)")
    ap.add_argument("--kr",        type=float, default=None, help="INDI KR gain roll/pitch (default: from CSV meta, else 1325)")
    ap.add_argument("--kw",        type=float, default=None, help="INDI KW gain roll/pitch (default: from CSV meta, else 114)")
    ap.add_argument("--kr-z",      type=float, default=None, help="INDI KR_Z yaw gain (default: same as --kr)")
    ap.add_argument("--kw-z",      type=float, default=None, help="INDI KW_Z yaw gain (default: same as --kw)")
    ap.add_argument("--fc-bw",     type=float, default=None, help="Butterworth cutoff [Hz] (default: from CSV meta, else 25)")
    ap.add_argument("--tau-limit", type=float, default=TAU_LIMIT_NM,
                    help=f"Estimated max torque per axis [N·m] for saturation %% (default {TAU_LIMIT_NM})")
    args = ap.parse_args()

    if args.csv:
        path = Path(args.csv)
    else:
        skip = {"_filter_stats", "_filter_sweep", "_indi_dashboard", "_analysis",
                "_3d_orientation", "_helix", "_yaw_spin", "_flip"}
        cands = [c for c in sorted(LOGS_DIR.glob("*.csv"))
                 if not any(s in c.name for s in skip)]
        if not cands:
            print(f"No CSV found in {LOGS_DIR}")
            sys.exit(1)
        path = cands[-1]
        print(f"[auto] {path.name}")

    # Resolve gains from CSV meta if not given on CLI
    meta = load_meta(path)
    if args.kr is None:
        args.kr = float(meta.get("indi_kr", 1325.0))
    if args.kw is None:
        args.kw = float(meta.get("indi_kw", 114.0))
    if args.fc_bw is None:
        args.fc_bw = float(meta.get("indi_fc_bw", 25.0))

    ctrl_mode = ctrl_mode_from_meta(meta)
    geometric_mode = ctrl_mode == 0

    data = load_csv(path)
    N = len(next(iter(data.values())))
    if N < 5:
        print(f"ERROR: too few rows ({N})")
        sys.exit(1)

    t       = col(data, "time_s", N)
    z       = col(data, "z", N)
    roll    = col(data, "roll_deg", N)
    pitch   = col(data, "pitch_deg", N)
    yaw     = col(data, "yaw_deg", N)
    vbat    = col(data, "vbat", N)
    rpm     = np.column_stack([col(data, f"rpm_m{i}", N) for i in range(1, 5)])
    tau     = np.column_stack([col(data, f"tau_{a}", N)     for a in "xyz"])
    alp     = np.column_stack([col(data, f"alp_{a}", N)     for a in "xyz"])
    alp_raw = np.column_stack([col(data, f"alp_raw_{a}", N) for a in "xyz"])
    acc     = np.column_stack([col(data, f"acc_{a}", N)  for a in "xyz"])   # body frame [g]
    vel     = np.column_stack([col(data, f"v{a}", N)     for a in "xyz"])   # world frame [m/s]

    fs = detect_fs(t)
    print(f"[info] N={N}  fs={fs:.0f} Hz  t=[{t[0]:.2f}, {t[-1]:.2f}]s")

    i0, i1, crashed, i_crash = find_hover_window(t, z, roll, pitch)
    sl = slice(i0, i1 + 1)
    t0, t1 = t[i0], t[i1]

    # ── Column availability ────────────────────────────────────────────────────
    rpm_ok      = has_data(rpm[sl])
    tau_present = has_data(tau[sl])
    alp_ok      = has_data(alp[sl])
    alp_r_ok    = has_data(alp_raw[sl])

    # ── Attitude metrics ───────────────────────────────────────────────────────
    def _rms(a): return float(np.sqrt(np.nanmean(a[sl] ** 2)))
    def _peak(a): return float(np.nanmax(np.abs(a[sl])))

    roll_rms,  roll_peak  = _rms(roll),  _peak(roll)
    pitch_rms, pitch_peak = _rms(pitch), _peak(pitch)
    yaw_rms               = _rms(yaw)
    vbat_min = float(np.nanmin(vbat)) if has_data(vbat) else float("nan")

    # ── RPM / INDI mode ────────────────────────────────────────────────────────
    if rpm_ok:
        rpm_mean = float(np.nanmean(rpm[sl][np.isfinite(rpm[sl])]))
        rpm_min  = float(np.nanmin( rpm[sl][np.isfinite(rpm[sl])]))
        integrating_mode = rpm_min < 100
    else:
        rpm_mean = rpm_min = float("nan")
        integrating_mode = True  # assume worst case without RPM data

    # ── Tau metrics — exclude stale tau_prev period at start ──────────────────
    # tau_prev carries over from previous crash; stays frozen until RPMs kick in.
    # Detect by finding where tau first deviates from its initial constant value.
    if tau_present:
        stale_i = max(stale_end(tau[:, 0]), stale_end(tau[:, 1]), stale_end(tau[:, 2]))
        tau_i0 = max(i0, stale_i)  # start of valid tau data within hover window
        if tau_i0 > i1:
            # Entire hover window is frozen tau_prev — no live INDI commands.
            tau_active = False
            tau_sl = sl
            tau_stale_secs = t1 - t0
        else:
            tau_active = True
            tau_sl = slice(tau_i0, i1 + 1)
            tau_stale_secs = max(0.0, t[tau_i0] - t[i0])
    else:
        tau_active = False
        tau_sl = sl
        tau_stale_secs = 0.0
    tau_rms  = [float(np.sqrt(np.nanmean(tau[tau_sl, i] ** 2))) if tau_active else float("nan") for i in range(3)]
    tau_peak = [float(np.nanmax(np.abs(tau[tau_sl, i])))        if tau_active else float("nan") for i in range(3)]
    tau_sat  = [float(np.nanmean(np.abs(tau[tau_sl, i]) > args.tau_limit * 0.9)) * 100.0
                if tau_active else float("nan") for i in range(3)]

    # ── Filter quality ─────────────────────────────────────────────────────────
    fq = [filter_quality(alp_raw[sl, i], alp[sl, i], fs) for i in range(3)]

    # ── KT sanity check — total thrust vs weight ───────────────────────────────
    # During hover: Σ kt_i * RPM_i² should equal mass * g (drone weight).
    # Also check tilt-corrected: F_total * cos(tilt) ≈ weight (valid during all flight).
    MASS_KG  = 0.0364   # AUW 36.4 g — CF2.1 + USD + RPM decks (measured 2026-06-18)
    GRAVITY  = 9.81
    WEIGHT_N = MASS_KG * GRAVITY
    KT = [1.4641e-10, 1.4872e-10, 1.4724e-10, 1.6485e-10]  # hover 2026-06-18, m=36.4g quick mode
    if rpm_ok:
        f_total = sum(KT[i] * rpm[:, i] ** 2 for i in range(4))  # N, per timestep
        tilt_rad = np.radians(np.sqrt(roll ** 2 + pitch ** 2))
        f_vert   = f_total * np.cos(tilt_rad)                     # vertical component
        f_hover_mean  = float(np.nanmean(f_total[sl]))
        f_vert_mean   = float(np.nanmean(f_vert[sl]))
        kt_weight_err = (f_hover_mean - WEIGHT_N) / WEIGHT_N * 100.0   # %
        kt_vert_err   = (f_vert_mean  - WEIGHT_N) / WEIGHT_N * 100.0
    else:
        f_total = f_vert = np.full(N, float("nan"))
        f_hover_mean = f_vert_mean = kt_weight_err = kt_vert_err = float("nan")

    # ── Option A — accelerometer-based thrust (independent of KT / RPM) ────────
    # In body frame: acc_z [g] ≈ 1.0 in hover → F_acc = mass * acc_z * g
    # Gives a 3rd independent thrust estimate alongside F_rpm and weight.
    acc_ok = has_data(acc[sl])
    if acc_ok:
        f_acc         = MASS_KG * acc[:, 2] * GRAVITY          # N, body z-axis thrust
        f_acc_mean    = float(np.nanmean(f_acc[sl]))
        acc_weight_err = (f_acc_mean - WEIGHT_N) / WEIGHT_N * 100.0
    else:
        f_acc = np.full(N, float("nan"))
        f_acc_mean = acc_weight_err = float("nan")

    # ── Option B — world-frame linear acceleration: IMU vs EKF velocity ────────
    # a_imu: rotate body acc to world frame, remove gravity → actual linear accel
    # a_ekf: numerical derivative of EKF velocity → what the state estimator says
    # Agreement validates that IMU and EKF are consistent.
    vel_ok = has_data(vel[sl])
    if acc_ok:
        r_r = np.radians(roll);  cr, sr = np.cos(r_r), np.sin(r_r)
        r_p = np.radians(pitch); cp, sp = np.cos(r_p), np.sin(r_p)
        r_y = np.radians(yaw);   cy, sy = np.cos(r_y), np.sin(r_y)
        # ZYX rotation: world = R * body
        ax_b, ay_b, az_b = acc[:, 0], acc[:, 1], acc[:, 2]
        a_imu_x = (cy*cp)*ax_b + (cy*sp*sr - sy*cr)*ay_b + (cy*sp*cr + sy*sr)*az_b
        a_imu_y = (sy*cp)*ax_b + (sy*sp*sr + cy*cr)*ay_b + (sy*sp*cr - cy*sr)*az_b
        a_imu_z = (  -sp)*ax_b + (      cp*sr)*ay_b       + (      cp*cr)*az_b
        # Convert g → m/s² and remove gravity from z
        a_imu = np.column_stack([a_imu_x * GRAVITY,
                                  a_imu_y * GRAVITY,
                                  (a_imu_z - 1.0) * GRAVITY])
    else:
        a_imu = np.full((N, 3), float("nan"))
    if vel_ok:
        dt_arr = np.gradient(t)
        a_ekf  = np.gradient(vel, axis=0) / dt_arr[:, None]   # m/s²
    else:
        a_ekf = np.full((N, 3), float("nan"))

    # ── Gain analysis ──────────────────────────────────────────────────────────
    kr_z = args.kr_z if args.kr_z is not None else args.kr
    kw_z = args.kw_z if args.kw_z is not None else args.kw

    wn, zeta, tau_dom_ms = att_poles(args.kr, args.kw)
    routh_ok     = routh_stable(args.kr, args.kw)
    routh_ok_z   = routh_stable(kr_z, kw_z)
    routh_margin = args.kw / args.kr if args.kr > 0 else float("inf")
    suggestions  = suggest_gains(args.kr, args.kw, incremental=not integrating_mode)

    # ── Terminal output ────────────────────────────────────────────────────────
    AX = ["X (roll)", "Y (pitch)", "Z (yaw)"]

    def fmt(v, fmt_str, fallback="N/A"):
        return format(v, fmt_str) if math.isfinite(v) else fallback

    print()
    print("=" * 64)
    print("  HEALTH CHECK")
    print("=" * 64)
    print(f"  File         : {path.name}")
    if ctrl_mode is not None:
        print(f"  Ctrl mode    : {ctrl_mode} ({ctrl_mode_label(ctrl_mode)})")
    print(f"  Hover window : {t0:.2f}s – {t1:.2f}s  ({t1 - t0:.1f}s)")
    print(f"  Crashed      : {'YES at t=' + fmt(t[i_crash], '.2f') + 's' if crashed else 'NO'}")
    print(f"  RPM deck     : {f'OK  (mean={rpm_mean:.0f} RPM, min={rpm_min:.0f})' if rpm_ok else 'NOT IN LOG (NaN) — add rpm topic'}")
    if tau_active:
        tau_status = "ACTIVE — non-zero torque commands seen"
    elif geometric_mode:
        tau_status = (f"N/A — geometric controller (ctrl_mode=0); "
                      f"tau_* not logged — use mode 2/3 for INDI commissioning")
    elif tau_present:
        tau_status = "STALE — tau_prev frozen (no INDI commands in hover window)"
    else:
        tau_status = "NOT IN LOG — add indi_state topic"
    print(f"  INDI tau     : {tau_status}")
    print(f"  alp (filt)   : {'present' if alp_ok else 'NOT IN LOG'}")
    print(f"  alp_raw      : {'present' if alp_r_ok else 'NOT IN LOG — add indi_alp_raw topic'}")
    print(f"  Vbat min     : {fmt(vbat_min, '.3f')} V")
    print(f"  INDI mode    : {'INCREMENTAL (RPM healthy → 2nd-order att loop)' if not integrating_mode else 'TAU-INTEGRATING (RPM=0 or not read → 3rd-order, Routh applies)'}")

    print()
    print("  ATTITUDE (hover window)")
    print(f"    Roll  : RMS={roll_rms:.2f}°   peak={roll_peak:.1f}°")
    print(f"    Pitch : RMS={pitch_rms:.2f}°   peak={pitch_peak:.1f}°")
    print(f"    Yaw   : RMS={yaw_rms:.2f}°")

    print()
    if tau_active:
        stale_note = f"  (stale tau_prev excluded: {tau_stale_secs:.1f}s at start)" if tau_stale_secs > 0.1 else ""
        print(f"  INDI TORQUE (hover window{stale_note})")
        for i, ax in enumerate(AX):
            print(f"    {ax}: RMS={tau_rms[i]*1000:.3f} mN·m   "
                  f"peak={tau_peak[i]*1000:.3f} mN·m   sat={tau_sat[i]:.1f}%")
    elif geometric_mode:
        print("  INDI TORQUE : not available in geometric mode (ctrl_mode=0)")
        print("                alp/RPM still logged; re-fly with ctrl_mode 2 or 3 for tau metrics")
    elif tau_present:
        print(f"  INDI TORQUE : stale tau_prev only ({tau_stale_secs:.1f}s frozen in hover window)")
    else:
        print("  INDI TORQUE : not available — add indi_state topic and reflash if needed")

    print()
    if rpm_ok:
        kt_flag  = "OK" if abs(kt_weight_err) < 15 else ("HIGH" if kt_weight_err > 0 else "LOW")
        ktv_flag = "OK" if abs(kt_vert_err)   < 15 else ("HIGH" if kt_vert_err  > 0 else "LOW")
        print(f"  KT SANITY CHECK  (expected weight = {WEIGHT_N*1000:.1f} mN, mass={MASS_KG*1000:.0f}g)")
        print(f"    Σkt·RPM²        : {f_hover_mean*1000:.1f} mN  →  {kt_weight_err:+.1f}%  [{kt_flag}]")
        print(f"    Σkt·RPM²·cos(θ) : {f_vert_mean*1000:.1f} mN  →  {kt_vert_err:+.1f}%  [{ktv_flag}]")
        if acc_ok:
            acc_flag = "OK" if abs(acc_weight_err) < 15 else "CHECK"
            print(f"    m·acc_z·g (IMU) : {f_acc_mean*1000:.1f} mN  →  {acc_weight_err:+.1f}%  [{acc_flag}]  (independent check)")
        if abs(kt_weight_err) > 30:
            print("    WARNING: >30% error — KT values may need re-identification")
        elif abs(kt_weight_err) > 15:
            print("    CAUTION: 15-30% error — consider re-running KT hover identification")
    else:
        print("  KT SANITY CHECK  : RPM not in log — add rpm topic")

    print()
    if alp_r_ok and alp_ok:
        print(f"  FILTER QUALITY  (fc_bw={args.fc_bw:.0f} Hz, log fs={fs:.0f} Hz)")
        if fs < 50:
            print(f"  NOTE: fs={fs:.0f} Hz limits phase-lag resolution to {1000/fs:.0f} ms steps — "
                  f"raise state topic to 100 Hz for better accuracy")
        for i, ax in enumerate(AX):
            db, lag = fq[i]
            print(f"    {ax}: noise reduction={fmt(db,'.1f')} dB   phase lag≈{fmt(lag,'.0f')} ms")
    elif alp_ok:
        print("  FILTER QUALITY : alp_raw not in log — add indi_alp_raw topic")
    else:
        print("  FILTER QUALITY : alp columns not in log")

    print()
    def routh_str(ok, margin, in_integrating):
        if not in_integrating:
            return "2nd-order ✓ (RPM active, KW>KR not needed)"
        return f"Routh={margin:.2f}  {'STABLE ✓' if ok else '✗ UNSTABLE (fallback mode) — raise KW above KR'}"

    print("  GAIN ANALYSIS")
    print(f"    Roll/Pitch : KR={args.kr:.0f}   KW={args.kw:.0f}   {routh_str(routh_ok, routh_margin, integrating_mode)}")
    wn_z, zeta_z, td_z = att_poles(kr_z, kw_z)
    rm_z = kw_z / kr_z if kr_z > 0 else float("inf")
    print(f"    Yaw        : KR={kr_z:.0f}   KW={kw_z:.0f}   {routh_str(routh_ok_z, rm_z, integrating_mode)}")
    print(f"    Att poles  : ωn={fmt(wn,'.1f')} rad/s   ζ={fmt(zeta,'.1f')}   "
          f"τ_dom≈{fmt(tau_dom_ms,'.0f')} ms  (roll/pitch, assumes perfect INDI inner loop)")
    print(f"    fc_bw={args.fc_bw:.0f} Hz")
    if tau_active:
        max_sat = max((v for v in tau_sat if math.isfinite(v)), default=float("nan"))
        if math.isfinite(max_sat):
            if max_sat < 5:
                verdict = "tau_sat<5% → HEADROOM AVAILABLE — can raise gains"
            elif max_sat < 20:
                verdict = f"tau_sat={max_sat:.0f}% → MODERATE — small gain increase ok"
            else:
                verdict = f"tau_sat={max_sat:.0f}% → SATURATING — do NOT raise gains"
            print(f"    Saturation   : {verdict}")
    print()
    KR_PRACTICAL_LIMIT = 1400.0
    print("    SUGGESTED NEXT STEPS:")
    if args.kr >= KR_PRACTICAL_LIMIT:
        print(f"      KR={args.kr:.0f} is at/above practical limit (KR>{KR_PRACTICAL_LIMIT:.0f} oscillates on this drone).")
        print(f"      → Do NOT raise KR further. Next priorities:")
        print(f"      → 1. KT_MOTOR bench calibration (fixes INDI feedforward accuracy)")
        print(f"      → 2. Raise KV (position damping, ζ_pos=KV/(2·sqrt(KP)))")
        print(f"      → 3. Try fc_bw=60 Hz (cleaner alp, slight lag tradeoff)")
    else:
        for i, s in enumerate(suggestions):
            if integrating_mode:
                ok = "✓ Routh stable" if s["stable"] else "✗ Routh UNSTABLE (KW<KR, fallback unsafe)"
            else:
                ok = "✓ 2nd-order stable (RPM active)"
            print(f"      Step {i+1}: KR={s['kr']}  KW={s['kw']}  → "
                  f"ωn={s['wn']:.1f}  ζ={s['zeta']:.1f}  τ_dom={s['tau_dom_ms']:.0f}ms  {ok}")
    print()

    # ── Dashboard plot ─────────────────────────────────────────────────────────
    fig, axs = plt.subplots(3, 3, figsize=(15, 11))
    fig.suptitle(f"INDI Dashboard — {path.name}", fontsize=11, fontweight="bold")

    t_w = t[sl]

    def shade(ax):
        ax.axvspan(t0, t1, alpha=0.08, color="limegreen", label="hover window")
        if crashed and i_crash is not None:
            ax.axvline(t[i_crash], color="red", lw=1.5, ls="--",
                       label=f"crash t={t[i_crash]:.1f}s")

    # Row 0 — Attitude time series
    att_data  = [roll, pitch, yaw]
    att_names = ["Roll [°]", "Pitch [°]", "Yaw [°]"]
    for i, (a, name) in enumerate(zip(att_data, att_names)):
        ax = axs[0, i]
        ax.axhline(0, color="k", lw=0.4)
        ax.axhspan(-5,   5,  alpha=0.08, color="limegreen")
        ax.axhspan(-10, 10,  alpha=0.05, color="orange")
        ax.plot(t, a, lw=0.7, color="C0")
        shade(ax)
        ax.set_ylabel(name, fontsize=9)
        ax.set_xlabel("t [s]", fontsize=8)
        rms_w  = float(np.sqrt(np.nanmean(a[sl] ** 2)))
        peak_w = float(np.nanmax(np.abs(a[sl])))
        ax.set_title(f"RMS={rms_w:.2f}°  peak={peak_w:.1f}°", fontsize=9)
        if i == 0:
            ax.legend(fontsize=7, loc="upper right")

    # Row 1 — INDI torque time series (full flight, but stats from valid window only)
    tau_names = ["tau_x [N·m]", "tau_y [N·m]", "tau_z [N·m]"]
    t_tau_start = t[tau_sl.start] if tau_active else t[i0]
    for i in range(3):
        ax = axs[1, i]
        if tau_present:
            ax.plot(t, tau[:, i], lw=0.7, color="C1")
            ax.axhline( args.tau_limit, color="red", lw=0.8, ls="--",
                        label=f"±{args.tau_limit*1000:.0f} mN·m est. limit")
            ax.axhline(-args.tau_limit, color="red", lw=0.8, ls="--")
            if tau_stale_secs > 0.1:
                stale_end_t = t1 if not tau_active else t_tau_start
                ax.axvspan(t[i0], stale_end_t, alpha=0.15, color="orange",
                           label=f"stale τ_prev ({tau_stale_secs:.1f}s)" if i == 0 else None)
            shade(ax)
            if tau_active:
                ax.set_title(f"RMS={tau_rms[i]*1000:.3f} mN·m  sat={tau_sat[i]:.1f}%", fontsize=9)
            else:
                ax.set_title("stale τ_prev (frozen)", fontsize=9)
            ax.legend(fontsize=7)
        else:
            ax.text(0.5, 0.5, "tau not in log\n(add indi_state topic to yaml)",
                    ha="center", va="center", transform=ax.transAxes,
                    fontsize=9, color="gray", style="italic")
        ax.set_ylabel(tau_names[i], fontsize=9)
        ax.set_xlabel("t [s]", fontsize=8)

    # Row 2 — Filter: alp_raw vs alp (hover window only)
    alp_names = ["alp_x [rad/s²]", "alp_y [rad/s²]", "alp_z [rad/s²]"]
    for i in range(3):
        ax = axs[2, i]
        if alp_r_ok and alp_ok:
            ax.plot(t_w, alp_raw[sl, i], lw=0.5, alpha=0.55, color="gray",  label="alp_raw (pre-filter)")
            ax.plot(t_w, alp[sl, i],     lw=0.9, color="C2",                label="alp (filtered)")
            db, lag = fq[i]
            db_s  = f"{db:.1f} dB"  if math.isfinite(db)  else "N/A"
            lag_s = f"{lag:.0f} ms" if math.isfinite(lag) else "N/A"
            ax.set_title(f"noise red.={db_s}   lag≈{lag_s}", fontsize=9)
            ax.legend(fontsize=7, loc="upper right")
        elif alp_ok:
            ax.plot(t_w, alp[sl, i], lw=0.9, color="C2", label="alp (filtered)")
            ax.set_title("alp_raw not available", fontsize=9)
            ax.legend(fontsize=7)
        else:
            ax.text(0.5, 0.5, "alp not in log\n(add indi_state topic to yaml)",
                    ha="center", va="center", transform=ax.transAxes,
                    fontsize=9, color="gray", style="italic")
        ax.set_ylabel(alp_names[i], fontsize=9)
        ax.set_xlabel("t [s]", fontsize=8)

    # Bottom annotation strip
    mode_str  = "incremental" if not integrating_mode else "tau-integrating (3rd-order!)"
    routh_str = f"Routh={routh_margin:.2f} {'✓' if routh_ok else '✗ UNSTABLE'}"
    poles_str = f"ωn={fmt(wn,'.1f')} rad/s  ζ={fmt(zeta,'.1f')}  τ_dom≈{fmt(tau_dom_ms,'.0f')}ms"
    info = (f"KR={args.kr:.0f}  KW={args.kw:.0f}  fc_bw={args.fc_bw:.0f} Hz  │  "
            f"{routh_str}  │  {poles_str}  │  "
            f"INDI mode: {mode_str}  │  RPM: {'ok' if rpm_ok else 'not in log'}")
    fig.text(0.01, 0.005, info, fontsize=7.5, color="dimgray",
             bbox=dict(boxstyle="round", facecolor="lightyellow", alpha=0.85))

    plt.tight_layout(rect=[0, 0.04, 1, 1])
    out = path.with_name(path.stem + "_indi_dashboard.png")
    plt.savefig(out, dpi=150)
    print(f"Saved: {out}")
    plt.show()

    # ── tau vs J·alpha alignment panel ────────────────────────────────────────
    # Overlays tau_cmd (what INDI commanded) with J*alpha_meas (what the drone felt).
    # INDI assumption: tau_current ≈ J*alpha_meas → delta_tau should be small.
    # Lag > 0: tau leads alpha (motor spin-up delay). DC offset: KT calibration error.
    if tau_present and alp_ok:
        j_alpha = alp * np.array([JXX, JYY, JZZ])  # [N, N, N] → shape (N, 3) [N·m]

        n_rows = 4 if rpm_ok else 3
        fig2, axs2 = plt.subplots(n_rows, 1, figsize=(12, 4 * n_rows), sharex=False)
        fig2.suptitle(f"INDI Alignment — tau_cmd vs J·alpha_meas — {path.name}",
                      fontsize=11, fontweight="bold")

        ax_names = ["X (roll)", "Y (pitch)", "Z (yaw)"]
        for i in range(3):
            ax = axs2[i]
            t_w2 = t[tau_sl]
            tc   = tau[tau_sl, i]
            ja   = j_alpha[tau_sl, i]

            lag_ms, r_corr, rms_diff = tau_alignment(tc, ja, fs)

            ax.plot(t_w2, tc   * 1000, lw=0.9, color="C1", label="tau_cmd [mN·m]")
            ax.plot(t_w2, ja   * 1000, lw=0.9, color="C2", alpha=0.85,
                    label="J·alpha_meas [mN·m]")
            ax.fill_between(t_w2, tc * 1000, ja * 1000, alpha=0.18, color="C3",
                            label="delta (tau_cmd − J·alp)")
            ax.axhline(0, color="k", lw=0.4)

            lag_s = f"{lag_ms:.1f} ms" if math.isfinite(lag_ms) else "N/A"
            r_s   = f"{r_corr:.3f}"   if math.isfinite(r_corr) else "N/A"
            rms_s = f"{rms_diff:.3f} mN·m" if math.isfinite(rms_diff) else "N/A"
            ax.set_title(
                f"{ax_names[i]}  │  lag≈{lag_s}  │  r={r_s}  │  RMS(delta)={rms_s}",
                fontsize=9)
            ax.set_ylabel("Torque [mN·m]", fontsize=9)
            ax.set_xlabel("t [s]", fontsize=8)
            ax.legend(fontsize=8, loc="upper right")

            # Print to terminal too
            print(f"  tau-vs-J·alp [{ax_names[i]}]: lag={lag_s}  r={r_s}  "
                  f"RMS(delta)={rms_s}")

        # Row 3 — KT sanity: Σkt·RPM² vs weight
        if rpm_ok:
            ax = axs2[3]
            ax.plot(t, f_total * 1000, lw=0.9, color="C0", label="Σkt·RPM²  (total thrust)")
            ax.plot(t, f_vert  * 1000, lw=0.9, color="C4", alpha=0.8,
                    label="Σkt·RPM²·cos(θ)  (vertical)")
            if acc_ok:
                ax.plot(t, f_acc * 1000, lw=0.9, color="C2", alpha=0.85, ls="--",
                        label=f"m·acc_z·g  (IMU, independent)  {acc_weight_err:+.1f}%")
            ax.axhline(WEIGHT_N * 1000, color="red", lw=1.2, ls="--",
                       label=f"weight = {WEIGHT_N*1000:.1f} mN  ({MASS_KG*1000:.0f}g)")
            ax.axvspan(t0, t1, alpha=0.08, color="limegreen", label="hover window")
            ax.set_ylabel("Force [mN]", fontsize=9)
            ax.set_xlabel("t [s]", fontsize=8)
            flag = "OK" if abs(kt_weight_err) < 15 else "CHECK KT"
            ax.set_title(
                f"KT sanity: Σkt·RPM²={f_hover_mean*1000:.1f} mN ({kt_weight_err:+.1f}%)  "
                f"│  IMU={f_acc_mean*1000:.1f} mN ({acc_weight_err:+.1f}%)  "
                f"│  weight={WEIGHT_N*1000:.1f} mN  [{flag}]",
                fontsize=9)
            ax.legend(fontsize=8, loc="upper right")

        plt.tight_layout()
        out2 = path.with_name(path.stem + "_indi_panel.png")
        plt.savefig(out2, dpi=150)
        print(f"Saved: {out2}")
        plt.show()
    else:
        print("[info] tau or alp not in log — skipping alignment panel")

    # ── Option B — world-frame linear acceleration figure ─────────────────────
    # a_imu (rotated accelerometer) vs a_ekf (EKF velocity derivative).
    # Agreement = IMU and state estimator are consistent.
    # Gap = either IMU noise/bias or EKF smoothing artefact.
    if acc_ok or vel_ok:
        fig3, axs3 = plt.subplots(3, 1, figsize=(12, 9), sharex=False)
        fig3.suptitle(f"Linear Acceleration — IMU vs EKF — {path.name}",
                      fontsize=11, fontweight="bold")
        ax_names_w = ["X (forward) [m/s²]", "Y (left) [m/s²]", "Z (up) [m/s²]"]
        for i in range(3):
            ax = axs3[i]
            t_w3 = t[sl]
            if acc_ok:
                ax.plot(t_w3, a_imu[sl, i], lw=0.8, color="C1", alpha=0.9,
                        label="a_imu  (acc rotated to world, gravity removed)")
            if vel_ok:
                ax.plot(t_w3, a_ekf[sl, i], lw=0.9, color="C0", alpha=0.85,
                        label="a_ekf  (d/dt EKF velocity)")
            ax.axhline(0, color="k", lw=0.4)
            ax.set_ylabel(ax_names_w[i], fontsize=9)
            ax.set_xlabel("t [s]", fontsize=8)
            if acc_ok and vel_ok:
                rms_gap = float(np.sqrt(np.nanmean((a_imu[sl, i] - a_ekf[sl, i]) ** 2)))
                ax.set_title(f"RMS gap = {rms_gap:.2f} m/s²  "
                             f"(small = IMU and EKF agree)", fontsize=9)
                # Print summary to terminal
                print(f"  a_imu vs a_ekf [{ax_names_w[i]}]: RMS gap = {rms_gap:.2f} m/s²")
            ax.legend(fontsize=8, loc="upper right")
        plt.tight_layout()
        out3 = path.with_name(path.stem + "_indi_accel.png")
        plt.savefig(out3, dpi=150)
        print(f"Saved: {out3}")
        plt.show()


if __name__ == "__main__":
    main()
