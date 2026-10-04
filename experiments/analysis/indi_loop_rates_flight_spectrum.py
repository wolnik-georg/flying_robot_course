#!/usr/bin/env python3
"""A1 cf5 2026-10-02 radio-log jitter + attitude oscillation spectrum (read-only)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "experiments" / "analysis"))
from meeting_hw_common import load_radio_csv  # noqa: E402

LOGS = REPO / "experiments" / "logs"
OUT = Path(__file__).resolve().parent / "out" / "indi_loop_rates"

FLIGHTS = {
    "Ours": [
        "A1_cf5_2026-10-02_19-13-24.csv",
        "A1_cf5_2026-10-02_19-15-47.csv",
    ],
    "Omar C": [
        "A1_cf5_2026-10-02_18-36-18.csv",
        "A1_cf5_2026-10-02_18-37-33.csv",
    ],
    "Omar Rust": [
        "A1_cf5_2026-10-02_18-50-20.csv",
    ],
}

STEADY_AFTER_S = 6.0
LIFTOFF_Z = 0.05


def jitter_stats(t: np.ndarray) -> dict[str, float]:
    dt = np.diff(t)
    dt = dt[(dt > 0) & (dt < 0.5)]
    return {
        "n": int(len(dt)),
        "mean_hz": float(1.0 / np.mean(dt)) if len(dt) else float("nan"),
        "dt_mean_ms": float(np.mean(dt) * 1000) if len(dt) else float("nan"),
        "dt_std_ms": float(np.std(dt) * 1000) if len(dt) else float("nan"),
        "dt_p99_ms": float(np.percentile(dt, 99) * 1000) if len(dt) else float("nan"),
    }


def peak_spectrum_hz(t: np.ndarray, y: np.ndarray, f_lo: float = 2.0, f_hi: float = 15.0) -> dict[str, float]:
    dt = np.median(np.diff(t))
    if dt <= 0:
        return {"peak_hz": float("nan"), "peak_power": float("nan")}
    fs = 1.0 / dt
    y = y - np.mean(y)
    n = len(y)
    win = np.hanning(n)
    spec = np.fft.rfft(y * win)
    freqs = np.fft.rfftfreq(n, d=dt)
    pxx = (np.abs(spec) ** 2) / n
    mask = (freqs >= f_lo) & (freqs <= f_hi)
    if not np.any(mask):
        return {"peak_hz": float("nan"), "peak_power": float("nan")}
    i = int(np.argmax(pxx[mask]))
    f_peak = float(freqs[mask][i])
    return {"peak_hz": f_peak, "peak_power": float(pxx[mask][i]), "fs_hz": float(fs)}


def steady_mask(t: np.ndarray, z: np.ndarray) -> np.ndarray:
    lift = z > LIFTOFF_Z
    if not np.any(lift):
        return np.zeros_like(t, dtype=bool)
    t0 = float(t[np.argmax(lift)])
    return (t >= t0 + STEADY_AFTER_S) & lift


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    results: list[dict] = []

    fig, axes = plt.subplots(len(FLIGHTS), 1, figsize=(8, 2.2 * len(FLIGHTS)), sharex=True)
    if len(FLIGHTS) == 1:
        axes = [axes]

    for ax, (variant, files) in zip(axes, FLIGHTS.items()):
        for fn in files:
            path = LOGS / fn
            meta, cols = load_radio_csv(path)
            t = cols["time_s"]
            sm = steady_mask(t, cols["pos_z"])
            gy = cols["gyro_x"]
            if np.sum(sm) < 64:
                continue
            ts = t[sm]
            ys = gy[sm]
            jit = jitter_stats(ts)
            spec = peak_spectrum_hz(ts, ys)
            row = {
                "file": fn,
                "variant": variant,
                "controller_meta": meta.get("controller"),
                "ctrl_mode_meta": meta.get("ctrl_mode"),
                "jitter": jit,
                "gyro_x_spectrum": spec,
            }
            results.append(row)
            ax.psd(
                ys,
                Fs=spec.get("fs_hz", jit["mean_hz"]),
                NFFT=1024,
                noverlap=512,
                label=f"{fn[-12:-4]} peak={spec['peak_hz']:.2f} Hz",
            )
        ax.set_ylabel("PSD")
        ax.set_title(variant)
        ax.legend(fontsize=7)
        ax.set_xlim(0, 20)

    axes[-1].set_xlabel("Frequency [Hz]")
    fig.tight_layout()
    fig.savefig(OUT / "fig_a1_cf5_gyro_x_psd.png", dpi=150)
    plt.close(fig)

    (OUT / "flight_spectrum_a1_cf5_2026-10-02.json").write_text(json.dumps(results, indent=2))

    md = ["# A1 cf5 2026-10-02 — radio log timing + gyro_x spectrum", ""]
    md.append("| Variant | File | Log rate (Hz) | dt std (ms) | Peak gyro_x (Hz) | n flights in group |")
    md.append("|---|---|---:|---:|---:|---|")
    counts = {v: len(f) for v, f in FLIGHTS.items()}
    for r in results:
        j = r["jitter"]
        s = r["gyro_x_spectrum"]
        md.append(
            f"| {r['variant']} | `{r['file']}` | {j['mean_hz']:.1f} | {j['dt_std_ms']:.2f} | {s['peak_hz']:.2f} | {counts[r['variant']]} |"
        )
    (OUT / "flight_spectrum_summary.md").write_text("\n".join(md) + "\n")
    print(f"Wrote {len(results)} flight rows to {OUT}")


if __name__ == "__main__":
    main()
