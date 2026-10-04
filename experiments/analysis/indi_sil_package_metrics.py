#!/usr/bin/env python3
"""Metrics shared by CS2 SIL package runs (Welch / steady window aligned with uSD analysis)."""

from __future__ import annotations

import numpy as np
from scipy import signal

STEADY_T0_S = 6.0
STEADY_T1_S = 13.0
WELCH_NPERSEG = 2048
WELCH_NOVERLAP = 1024
BAND_LO, BAND_HI = 2.0, 20.0


def steady_slice(t: np.ndarray, t0: float = STEADY_T0_S, t1: float = STEADY_T1_S) -> np.ndarray:
    return (t >= t0) & (t <= t1)


def analyze_series(t: np.ndarray, y: np.ndarray) -> dict:
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    if len(y) < 64:
        return {"peak_hz": float("nan"), "fs_hz": float("nan")}
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    nperseg = min(WELCH_NPERSEG, max(256, len(y) // 4))
    f, pxx = signal.welch(y - np.mean(y), fs=fs, nperseg=nperseg, noverlap=min(WELCH_NOVERLAP, nperseg // 2))
    band = (f >= BAND_LO) & (f <= BAND_HI)
    if not np.any(band):
        return {"peak_hz": float("nan"), "fs_hz": float(fs)}
    i = int(np.argmax(pxx[band]))
    return {"peak_hz": float(f[band][i]), "peak_psd": float(pxx[band][i]), "fs_hz": float(fs)}


def envelope_ratio(t: np.ndarray, y: np.ndarray) -> float:
    y = np.asarray(y, dtype=float)
    if len(y) < 32:
        return float("nan")
    env = np.abs(signal.hilbert(y - np.mean(y)))
    half = len(env) // 2
    e1, e2 = float(np.mean(env[:half])), float(np.mean(env[half:]))
    return e2 / e1 if e1 > 1e-9 else float("nan")


def classify_run(
    t: np.ndarray,
    z: np.ndarray,
    z_sp: float,
    gyro_x: np.ndarray,
    max_z: float = 3.0,
) -> str:
    if not np.all(np.isfinite(z)) or np.min(z) < 0.05:
        return "divergence"
    if np.max(np.abs(z - z_sp)) > max_z:
        return "divergence"
    sm = steady_slice(t)
    if not np.any(sm):
        return "unknown"
    er = envelope_ratio(t[sm], gyro_x[sm])
    if not np.isfinite(er):
        return "unknown"
    if er > 1.35:
        return "divergence"
    if 0.75 <= er <= 1.35:
        pk = analyze_series(t[sm], gyro_x[sm])["peak_hz"]
        if np.isfinite(pk) and BAND_LO <= pk <= BAND_HI:
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
    return {
        "gyro_rms_deg_s": gyro_rms,
        "gyro_x_rms_deg_s": gyro_x_rms,
        "roll_rms_deg": roll_rms,
        "pitch_rms_deg": pitch_rms,
        "mean_z_err_cm": mean_z_err_cm,
        "lateral_rms_cm": lat_err_cm,
        "dominant_hz_gyro_x": welch_gx["peak_hz"],
        "envelope_ratio_gyro_x": envelope_ratio(t_s, gx),
        "regime": classify_run(t, pos[:, 2], z_sp, gx),
        "steady_window_s": [STEADY_T0_S, STEADY_T1_S],
    }
