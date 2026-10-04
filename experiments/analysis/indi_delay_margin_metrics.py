#!/usr/bin/env python3
"""Metrics for delay-margin SIL (fixed regime labelling + self-check)."""

from __future__ import annotations

import numpy as np
from scipy import signal

STEADY_T0_S = 6.0
STEADY_T1_S = 13.0
LC_FREQ_LO, LC_FREQ_HI = 4.0, 6.5
CALM_GYRO_DEG_S = 5.0


def steady_slice(t: np.ndarray, t0: float = STEADY_T0_S, t1: float = STEADY_T1_S) -> np.ndarray:
    return (t >= t0) & (t <= t1)


def analyze_series(t: np.ndarray, y: np.ndarray) -> dict:
    y = np.asarray(y, dtype=float)
    t = np.asarray(t, dtype=float)
    if len(y) < 64:
        return {"peak_hz": float("nan"), "fs_hz": float("nan")}
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    nperseg = min(2048, max(256, len(y) // 4))
    f, pxx = signal.welch(y - np.mean(y), fs=fs, nperseg=nperseg, noverlap=nperseg // 2)
    band = (f >= 2.0) & (f <= 20.0)
    if not np.any(band):
        return {"peak_hz": float("nan"), "fs_hz": float(fs)}
    i = int(np.argmax(pxx[band]))
    return {"peak_hz": float(f[band][i]), "fs_hz": float(fs)}


def classify_run(
    gyro_rms: float,
    roll_rms: float,
    pitch_rms: float,
    z: np.ndarray,
    z_sp: float,
    peak_hz: float,
) -> str:
    if not np.all(np.isfinite(z)) or np.min(z) < 0.08:
        return "diverging"
    if np.max(np.abs(z - z_sp)) > 2.5:
        return "diverging"
    if gyro_rms > 400 or roll_rms > 45 or pitch_rms > 45:
        return "diverging"
    if gyro_rms < CALM_GYRO_DEG_S and roll_rms < 8 and pitch_rms < 8:
        return "stable"
    lc_f = np.isfinite(peak_hz) and LC_FREQ_LO <= peak_hz <= LC_FREQ_HI
    if gyro_rms >= 80 or (gyro_rms >= 40 and lc_f):
        return "limit_cycle"
    if gyro_rms >= 25:
        return "limit_cycle"
    return "stable"


def self_check_regime(metrics: dict) -> dict:
    g = metrics.get("gyro_rms_deg_s", 0.0)
    if g < CALM_GYRO_DEG_S and metrics.get("regime") in ("divergence", "diverging"):
        metrics = {**metrics, "regime": "stable", "regime_self_corrected": True}
    if not np.isfinite(g):
        metrics = {**metrics, "regime": "invalid", "regime_self_corrected": True}
    return metrics


def summarize_trace(
    t: np.ndarray,
    gyro_deg: np.ndarray,
    roll_deg: np.ndarray,
    pitch_deg: np.ndarray,
    pos: np.ndarray,
    z_sp: float,
) -> dict:
    sm = steady_slice(t)
    g = gyro_deg[sm]
    gx = g[:, 0] if g.ndim > 1 else g
    gyro_rms = float(np.sqrt(np.mean(np.sum(g**2, axis=-1)))) if g.ndim > 1 else float(np.sqrt(np.mean(gx**2)))
    roll_rms = float(np.sqrt(np.mean(roll_deg[sm] ** 2)))
    pitch_rms = float(np.sqrt(np.mean(pitch_deg[sm] ** 2)))
    pos_s = pos[sm]
    welch = analyze_series(t[sm], gx)
    regime = classify_run(gyro_rms, roll_rms, pitch_rms, pos[:, 2], z_sp, welch["peak_hz"])
    out = {
        "gyro_rms_deg_s": gyro_rms,
        "gyro_x_rms_deg_s": float(np.sqrt(np.mean(gx**2))),
        "roll_rms_deg": roll_rms,
        "pitch_rms_deg": pitch_rms,
        "mean_z_err_cm": float(np.mean((pos_s[:, 2] - z_sp) * 100.0)),
        "lateral_rms_cm": float(np.sqrt(np.mean(np.sum(pos_s[:, :2] ** 2, axis=1))) * 100.0),
        "dominant_hz_gyro_x": welch["peak_hz"],
        "regime": regime,
        "steady_window_s": [STEADY_T0_S, STEADY_T1_S],
    }
    return self_check_regime(out)
