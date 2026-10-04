#!/usr/bin/env python3
"""500 Hz uSD Welch PSD + envelope for paired A1 cf5 Oct-02 flights."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

REPO = Path(__file__).resolve().parents[2]
USD_RAW = REPO / "experiments" / "logs" / "usd_raw"
OUT = Path(__file__).resolve().parent / "out" / "indi_loop_rates"
PAIRING = OUT / "usd_pairing_a1_cf5_2026-10-02.json"

sys.path.insert(0, str(REPO / "flying_drone_stack" / "tools"))
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/tools/usdlog")
sys.path.insert(0, str(Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")))

import cfusdlog  # noqa: E402
from decode_usd_log import load  # noqa: E402
from find_flight_window import commanded_trajectory, find_offset  # noqa: E402

STEADY_T0_S = 6.0
STEADY_T1_S = 13.0
WELCH_NPERSEG = 2048
WELCH_NOVERLAP = 1024
BAND_LO, BAND_HI = 2.0, 20.0
TARGET_F = 6.3


def load_extended(bin_path: Path) -> dict[str, np.ndarray]:
    d = load(str(bin_path))
    raw = cfusdlog.decode(str(bin_path))["fixedFrequency"]
    for src, dst in [
        ("ctrlOmarIndi.torquex", "tau_x"),
        ("ctrlOmarIndi.torquey", "tau_y"),
        ("ctrlOmarIndi.torquez", "tau_z"),
    ]:
        if src in raw:
            d[dst] = np.asarray(raw[src], dtype=float)
    return d


def steady_mask(t: np.ndarray, lag: float, duration: float) -> np.ndarray:
    ts = t - lag
    return (ts >= STEADY_T0_S) & (ts <= min(STEADY_T1_S, duration - 1.0))


def analyze_series(t: np.ndarray, y: np.ndarray) -> dict:
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    f, pxx = signal.welch(y - np.mean(y), fs=fs, nperseg=min(WELCH_NPERSEG, len(y) // 2), noverlap=WELCH_NOVERLAP)
    band = (f >= BAND_LO) & (f <= BAND_HI)
    i = int(np.argmax(pxx[band]))
    f_peak = float(f[band][i])
    p_peak = float(pxx[band][i])
    target_band = (f >= 5.5) & (f <= 7.5)
    e_target = float(np.trapezoid(pxx[target_band], f[target_band])) if np.any(target_band) else 0.0
    e_total = float(np.trapezoid(pxx[band], f[band]))
    return {
        "fs_hz": float(fs),
        "peak_hz": f_peak,
        "peak_psd": p_peak,
        "energy_5p5_7p5": e_target,
        "energy_2_20": e_total,
        "frac_energy_near_6p3": e_target / e_total if e_total > 0 else float("nan"),
    }


def jitter_stats(t: np.ndarray) -> dict:
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 0.01)]
    return {
        "dt_mean_ms": float(np.mean(dt) * 1000),
        "dt_std_ms": float(np.std(dt) * 1000),
        "dt_p99_ms": float(np.percentile(dt, 99) * 1000),
        "dropouts_gt_3ms": int(np.sum(dt > 0.003)),
    }


def envelope_drift(t: np.ndarray, y: np.ndarray) -> dict:
    env = np.abs(signal.hilbert(y - np.mean(y)))
    half = len(env) // 2
    e1, e2 = float(np.mean(env[:half])), float(np.mean(env[half:]))
    ratio = e2 / e1 if e1 > 1e-9 else float("nan")
    return {"env_first_half_mean": e1, "env_second_half_mean": e2, "env_ratio_2nd_1st": ratio}


def tau_hold_metric(tau: np.ndarray) -> dict:
    d = np.diff(tau)
    frac_flat = float(np.mean(np.abs(d) < 1e-6))
    return {"tau_diff_frac_near_zero": frac_flat}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    pairs = json.loads(PAIRING.read_text())
    results: list[dict] = []

    fig_psd, axes = plt.subplots(len(pairs), 1, figsize=(9, 2.2 * len(pairs)), sharex=True)
    if len(pairs) == 1:
        axes = [axes]

    for ax, pair in zip(axes, pairs):
        if pair.get("pair_status") not in ("ASSIGNED", "MARGINAL_RMS_OR_LONG_FILE"):
            continue
        bin_path = USD_RAW / pair["paired_bin"]
        m = re.search(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})", pair["radio_csv"])
        meta_path = REPO / "experiments/logs" / f"A1_{m.group(1)}.meta.json"
        meta = json.loads(meta_path.read_text())

        d = load_extended(bin_path)
        t = np.asarray(d["t"])
        pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
        ts, cmd = commanded_trajectory(meta, "bottom")
        lag, _, _ = find_offset(t, pos, ts, cmd, t[0], t[-1] - meta["duration"])
        sm = steady_mask(t, lag, meta["duration"])
        t_s = t[sm]

        row = {
            "variant": pair["variant"],
            "radio_csv": pair["radio_csv"],
            "paired_bin": pair["paired_bin"],
            "lag_s": lag,
            "steady_window_s": [STEADY_T0_S, STEADY_T1_S],
            "jitter": jitter_stats(t_s),
        }
        gy = d["gyro_x"][sm]
        row["gyro_x"] = analyze_series(t_s, gy)
        row["envelope_gyro_x"] = envelope_drift(t_s, gy)
        if "tau_x" in d:
            row["tau_x_hold"] = tau_hold_metric(d["tau_x"][sm])
            row["tau_x"] = analyze_series(t_s, d["tau_x"][sm])
        row["limit_cycle_guess"] = (
            "stationary"
            if 0.7 < row["envelope_gyro_x"]["env_ratio_2nd_1st"] < 1.3
            else "growing"
            if row["envelope_gyro_x"]["env_ratio_2nd_1st"] > 1.3
            else "decaying"
        )
        results.append(row)

        fs = row["gyro_x"]["fs_hz"]
        f, pxx = signal.welch(gy - np.mean(gy), fs=fs, nperseg=WELCH_NPERSEG, noverlap=WELCH_NOVERLAP)
        ax.semilogy(f, pxx, label=f"{pair['variant']} peak={row['gyro_x']['peak_hz']:.2f} Hz")
        ax.axvline(TARGET_F, color="gray", ls="--", lw=0.8)
        ax.set_ylabel("PSD")
        ax.set_title(f"{pair['radio_csv']} ↔ {pair['paired_bin']}")
        ax.legend(fontsize=7)
        ax.set_xlim(0, 20)

    axes[-1].set_xlabel("Frequency [Hz]")
    fig_psd.tight_layout()
    fig_psd.savefig(OUT / "fig_usd_gyro_x_psd_paired.png", dpi=150)
    plt.close(fig_psd)

    # Envelope vs scenario time
    fig_env, axes2 = plt.subplots(len(results), 1, figsize=(9, 2 * len(results)), sharex=True)
    if len(results) == 1:
        axes2 = [axes2]
    for ax, row in zip(axes2, results):
        pair = next(p for p in pairs if p["radio_csv"] == row["radio_csv"])
        d = load_extended(USD_RAW / pair["paired_bin"])
        meta_m = re.search(r"(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})", pair["radio_csv"])
        meta = json.loads((REPO / "experiments/logs" / f"A1_{meta_m.group(1)}.meta.json").read_text())
        t = d["t"]
        pos = np.stack([d["x"], d["y"], d["z"]], axis=1)
        ts, cmd = commanded_trajectory(meta, "bottom")
        lag, _, _ = find_offset(t, pos, ts, cmd, t[0], t[-1] - meta["duration"])
        sm = steady_mask(t, lag, meta["duration"])
        tsce = t[sm] - lag
        gy = d["gyro_x"][sm]
        env = np.abs(signal.hilbert(gy - np.mean(gy)))
        ax.plot(tsce, env, lw=0.8)
        ax.set_ylabel("|env(gyro_x)|")
        ax.set_title(row["variant"])
    axes2[-1].set_xlabel("Scenario time [s]")
    fig_env.tight_layout()
    fig_env.savefig(OUT / "fig_usd_gyro_x_envelope.png", dpi=150)
    plt.close(fig_env)

    (OUT / "usd_spectrum_paired.json").write_text(json.dumps(results, indent=2))
    print(f"Wrote {len(results)} analyses to {OUT / 'usd_spectrum_paired.json'}")


if __name__ == "__main__":
    main()
