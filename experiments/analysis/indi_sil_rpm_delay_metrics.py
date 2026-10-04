#!/usr/bin/env python3
"""Metrics for RPM-delay SIL (fixed regime labelling vs doc 58 envelope bug)."""

from __future__ import annotations

import numpy as np
from scipy import signal

STEADY_T0_S = 6.0
STEADY_T1_S = 13.0
WELCH_NPERSEG = 2048
WELCH_NOVERLAP = 1024
BAND_LO, BAND_HI = 2.0, 20.0
LC_FREQ_LO, LC_FREQ_HI = 4.0, 6.5


def steady_slice(t: np.ndarray, t0: float = STEADY_T0_S, t1: float = STEADY_T1_S) -> np.ndarray:
    return (t >= t0) & (t <= t1)


def analyze_series(t: np.ndarray, y: np.ndarray) -> dict:
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    if len(y) < 64:
        return {"peak_hz": float("nan"), "fs_hz": float("nan"), "band_energy": float("nan")}
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    nperseg = min(WELCH_NPERSEG, max(256, len(y) // 4))
    f, pxx = signal.welch(y - np.mean(y), fs=fs, nperseg=nperseg, noverlap=min(WELCH_NOVERLAP, nperseg // 2))
    band = (f >= BAND_LO) & (f <= BAND_HI)
    if not np.any(band):
        return {"peak_hz": float("nan"), "fs_hz": float(fs), "band_energy": float("nan")}
    i = int(np.argmax(pxx[band]))
    e_band = float(np.trapezoid(pxx[band], f[band]))
    return {"peak_hz": float(f[band][i]), "peak_psd": float(pxx[band][i]), "fs_hz": float(fs), "band_energy": e_band}


def classify_run(
    gyro_rms: float,
    roll_rms: float,
    pitch_rms: float,
    z: np.ndarray,
    z_sp: float,
    peak_hz: float,
    gyro_steady: np.ndarray,
) -> str:
    if not np.all(np.isfinite(z)) or np.min(z) < 0.08:
        return "divergence"
    if np.max(np.abs(z - z_sp)) > 2.5:
        return "divergence"
    if gyro_rms > 400 or roll_rms > 45 or pitch_rms > 45:
        return "divergence"
    if len(gyro_steady) >= 32 and gyro_rms >= 50.0:
        env = np.abs(signal.hilbert(gyro_steady - np.mean(gyro_steady)))
        if np.mean(env[-len(env) // 4 :]) > 2.5 * np.mean(env[: len(env) // 4]):
            return "divergence"

    calm = gyro_rms < 35.0 and roll_rms < 8.0 and pitch_rms < 8.0
    if calm:
        return "stable"

    lc_freq = np.isfinite(peak_hz) and LC_FREQ_LO <= peak_hz <= LC_FREQ_HI
    if gyro_rms >= 40.0 and lc_freq:
        return "limit_cycle"
    if gyro_rms >= 80.0:
        return "limit_cycle"
    if gyro_rms >= 25.0 and np.isfinite(peak_hz) and peak_hz >= 3.0:
        return "limit_cycle"
    return "stable"


def summarize_trace(
    t: np.ndarray,
    gyro_deg: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    pos: np.ndarray,
    z_sp: float,
) -> dict:
    sm = steady_slice(t)
    t_s = t[sm]
    g = gyro_deg[sm]
    pos_s = pos[sm]
    gx = g[:, 0] if g.ndim > 1 else g
    gyro_rms = float(np.sqrt(np.mean(np.sum(g**2, axis=-1)))) if g.ndim > 1 else float(np.sqrt(np.mean(gx**2)))
    gyro_x_rms = float(np.sqrt(np.mean(gx**2)))
    roll_rms = float(np.sqrt(np.mean(roll_deg[sm] ** 2)))
    pitch_rms = float(np.sqrt(np.mean(pitch_deg[sm] ** 2)))
    mean_z_err_cm = float(np.mean((pos_s[:, 2] - z_sp) * 100.0))
    lat_err_cm = float(np.sqrt(np.mean(np.sum((pos_s[:, :2]) ** 2, axis=1))) * 100.0)
    welch_gx = analyze_series(t_s, gx)
    regime = classify_run(
        gyro_rms, roll_rms, pitch_rms, pos[:, 2], z_sp, welch_gx["peak_hz"], gx
    )
    return {
        "gyro_rms_deg_s": gyro_rms,
        "gyro_x_rms_deg_s": gyro_x_rms,
        "roll_rms_deg": roll_rms,
        "pitch_rms_deg": pitch_rms,
        "mean_z_err_cm": mean_z_err_cm,
        "lateral_rms_cm": lat_err_cm,
        "dominant_hz_gyro_x": welch_gx["peak_hz"],
        "gyro_band_energy": welch_gx.get("band_energy"),
        "regime": regime,
        "steady_window_s": [STEADY_T0_S, STEADY_T1_S],
    }
