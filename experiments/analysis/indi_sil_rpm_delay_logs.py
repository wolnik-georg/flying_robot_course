#!/usr/bin/env python3
"""Step 1: RPM-based torque vs gyro-derived alpha from 500 Hz uSD (Oct-02 A1 batch)."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy import signal

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "out" / "indi_sil_rpm_delay"
PAIRING = REPO / "experiments/analysis/out/indi_loop_rates/usd_pairing_a1_cf5_2026-10-02.json"
USD_RAW = REPO / "experiments/logs/usd_raw"
LOGS = REPO / "experiments/logs"

sys.path.insert(0, str(REPO / "flying_drone_stack/tools"))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/tools/usdlog")
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

import cfusdlog  # noqa: E402
from decode_usd_log import load  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402

STEADY_T0 = 6.0
STEADY_T1 = 13.0


def butter_alpha(gyro_rad_s: np.ndarray, fs: float, fc: float) -> np.ndarray:
    """Differentiate gyro then low-pass (matches INDI alpha chain at logged rate, approximate)."""
    if len(gyro_rad_s) < 8:
        return np.zeros_like(gyro_rad_s)
    b, a = signal.butter(2, fc / (0.5 * fs), btype="low")
    d = np.gradient(gyro_rad_s, 1.0 / fs)
    return signal.filtfilt(b, a, d)


def torque_from_rpm_ours(rpm: np.ndarray, kt: np.ndarray, arm: float, t2t: float) -> np.ndarray:
    """Body torques [tx, ty, tz] from per-motor RPM^2 thrust model."""
    f = kt * np.square(rpm)
    a = 0.707106781 * arm
    tx = a * (-f[:, 0] - f[:, 1] + f[:, 2] + f[:, 3])
    ty = a * (-f[:, 0] + f[:, 1] + f[:, 2] - f[:, 3])
    tz = t2t * (-f[:, 0] + f[:, 1] - f[:, 2] + f[:, 3])
    return np.stack([tx, ty, tz], axis=1)


def torque_from_rpm_omar(rpm: np.ndarray, kt: float, arm: float, t2t: float) -> np.ndarray:
    kt4 = np.full(4, kt)
    return torque_from_rpm_ours(rpm, kt4, arm, t2t)


def cross_corr_lag_ms(a: np.ndarray, b: np.ndarray, fs: float, max_lag_ms: float = 80.0) -> float:
    a = a - np.mean(a)
    b = b - np.mean(b)
    max_lag = int(max_lag_ms * 1e-3 * fs)
    if len(a) < 8 * max_lag or max_lag < 1:
        return float("nan")
    corr = np.correlate(a, b, mode="full")
    lags = np.arange(-len(b) + 1, len(a))
    win = (lags >= -max_lag) & (lags <= max_lag)
    best = lags[win][np.argmax(corr[win])]
    return float(-best / fs * 1000.0)


def coherence_peak(a: np.ndarray, b: np.ndarray, fs: float) -> tuple[float, float]:
    nperseg = min(1024, len(a) // 3)
    if nperseg < 64:
        return float("nan"), float("nan")
    f, cxy = signal.coherence(a, b, fs=fs, nperseg=nperseg)
    band = (f >= 2) & (f <= 15)
    if not np.any(band):
        return float("nan"), float("nan")
    i = int(np.argmax(cxy[band]))
    return float(f[band][i]), float(cxy[band][i])


def rpm_hold_stats(rpm: np.ndarray) -> dict:
    """Repeated identical samples ⇒ effective update / hold length at log rate."""
    if len(rpm) < 2:
        return {"median_hold_samples": float("nan"), "unique_rate_hz": float("nan")}
    runs = []
    run = 1
    for i in range(1, len(rpm)):
        if rpm[i] == rpm[i - 1]:
            run += 1
        else:
            runs.append(run)
            run = 1
    runs.append(run)
    med = float(np.median(runs))
    return {"median_hold_samples": med, "implied_update_hz_at_500hz_log": 500.0 / med if med > 0 else float("nan")}


def pick_rpm_columns(d: dict, rpm_source: int) -> tuple[np.ndarray, str]:
    """Return (N,4) RPM and label. Prefer source from meta."""
    if rpm_source != 0:
        cols = ["motor_m1_rpm", "motor_m2_rpm", "motor_m3_rpm", "motor_m4_rpm"]
        name = "dshot"
    else:
        cols = ["rpm_m1", "rpm_m2", "rpm_m3", "rpm_m4"]
        name = "deck"
    if all(c in d for c in cols):
        return np.stack([d[c] for c in cols], axis=1), name
    # fallback
    if all(f"motor_m{i}_rpm" in d for i in range(1, 5)):
        return np.stack([d[f"motor_m{i}_rpm"] for i in range(1, 5)], axis=1), "dshot_fallback"
    return np.stack([d[f"rpm_m{i}"] for i in range(1, 5)], axis=1), "deck_fallback"


def analyze_pair(pair: dict) -> dict | None:
    if pair.get("pair_status") not in ("ASSIGNED", "MARGINAL_RMS_OR_LONG_FILE"):
        return None
    bin_path = USD_RAW / pair["paired_bin"]
    if not bin_path.is_file():
        return None
    m = re.search(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})", pair["radio_csv"])
    meta_path = LOGS / f"A1_{m.group(1)}.meta.json"
    meta = json.loads(meta_path.read_text())
    drone = meta["names"][0]
    indi = meta["per_drone"][drone].get("indi", {})
    fc_bw = float(indi.get("fc_bw", 206.0))
    rpm_source = int(indi.get("rpm_source", 1))
    ctrl = int(meta["per_drone"][drone].get("controller", 6))

    d = load(str(bin_path))
    t = np.asarray(d["t"])
    pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
    ts, cmd = commanded_trajectory(meta, "bottom")
    lag, _, _ = find_offset(t, pos, ts, cmd, t[0], t[-1] - meta["duration"])
    tsce = t - lag
    sm = (tsce >= STEADY_T0) & (tsce <= STEADY_T1)

    rpm, rpm_label = pick_rpm_columns(d, rpm_source if ctrl == 6 else 1)
    rpm = rpm[sm]
    t_s = tsce[sm]
    fs = 1.0 / np.median(np.diff(t_s))

    gyro_x = np.deg2rad(np.asarray(d["gyro_x"][sm]))
    alpha_x = butter_alpha(gyro_x, fs, fc_bw)

    arm = 0.05
    t2t = 0.0056927884
    if ctrl == 6:
        kt = np.array([indi["kt1"], indi["kt2"], indi["kt3"], indi["kt4"]])
        tau = torque_from_rpm_ours(rpm, kt, arm, t2t)
    else:
        kt = 4.2899225838333166e-10
        tau = torque_from_rpm_omar(rpm, kt, arm, t2t)

    # Use logged Omar torque if present (validation of reconstruction)
    if "tau_x" in d:
        tau_x_log = np.asarray(d["tau_x"][sm])
        recon_err = float(np.std(tau[:, 0] - tau_x_log))
    else:
        recon_err = float("nan")

    gyro_cols = ["gyro_x", "gyro_y", "gyro_z"]
    axes = ["roll", "pitch", "yaw"]
    per_axis = {}
    for i, ax in enumerate(axes):
        gcol = np.deg2rad(np.asarray(d[gyro_cols[i]][sm]))
        alp = butter_alpha(gcol, fs, fc_bw)
        lag_ms = cross_corr_lag_ms(alp, tau[:, i], fs)
        num = float(np.std(tau[:, i]))
        den = float(np.std(alp)) + 1e-12
        gain_ratio = num / den
        f_coh, c_coh = coherence_peak(alp, tau[:, i], fs)
        per_axis[ax] = {
            "lag_tau_leads_alpha_ms": lag_ms,
            "gain_ratio_std_tau_over_std_alpha": gain_ratio,
            "coh_peak_hz": f_coh,
            "coh_peak": c_coh,
        }

    hold_d = hold_s = None
    if all(f"rpm_m{i}" in d for i in range(1, 5)):
        hold_d = {f"m{i}": rpm_hold_stats(np.asarray(d[f"rpm_m{i}"][sm])) for i in range(1, 5)}
    if all(f"motor_m{i}_rpm" in d for i in range(1, 5)):
        hold_s = {f"m{i}": rpm_hold_stats(np.asarray(d[f"motor_m{i}_rpm"][sm])) for i in range(1, 5)}

    return {
        "variant": pair["variant"],
        "radio_csv": pair["radio_csv"],
        "paired_bin": pair["paired_bin"],
        "controller": ctrl,
        "rpm_source_meta": rpm_source,
        "rpm_column_used": rpm_label,
        "fc_bw_hz": fc_bw,
        "steady_window_scenario_s": [STEADY_T0, STEADY_T1],
        "fs_hz": fs,
        "tau_recon_std_err_vs_log_tau_x": recon_err,
        "per_axis": per_axis,
        "rpm_hold_deck": hold_d,
        "rpm_hold_dshot": hold_s,
        "pair_confidence": pair.get("pair_confidence"),
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    pairs = json.loads(PAIRING.read_text())
    rows = [analyze_pair(p) for p in pairs]
    rows = [r for r in rows if r]
    def _san(o):
        if isinstance(o, dict):
            return {k: _san(v) for k, v in o.items()}
        if isinstance(o, float) and not np.isfinite(o):
            return None
        return o

    (OUT / "rpm_gyro_timing.json").write_text(json.dumps(_san(rows), indent=2))

    # Aggregate by variant (median lag roll axis)
    by_var: dict[str, list] = {}
    for r in rows:
        by_var.setdefault(r["variant"], []).append(r["per_axis"]["roll"]["lag_tau_leads_alpha_ms"])
    summary = {
        v: {
            "n": len(lags),
            "lag_tau_leads_alpha_ms_median": float(np.median(lags)),
            "lag_spread_ms": [float(x) for x in lags],
        }
        for v, lags in by_var.items()
    }
    (OUT / "rpm_gyro_timing_summary.json").write_text(json.dumps(summary, indent=2))
    print(f"Wrote {len(rows)} flight analyses to {OUT}")


if __name__ == "__main__":
    main()
