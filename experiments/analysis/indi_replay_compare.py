#!/usr/bin/env python3
"""Metrics for INDI replay CSVs (ours vs Omar, Omar C vs Rust)."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "indi_replay"


def load_csv(path: Path) -> dict[str, np.ndarray]:
    cols: dict[str, list] = {}
    with path.open() as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                cols.setdefault(k, []).append(float(v))
    return {k: np.asarray(v) for k, v in cols.items()}


def align_pair(a: dict, b: dict) -> tuple[np.ndarray, np.ndarray]:
    n = min(len(a["thrust_si"]), len(b["thrust_si"]))
    return a["thrust_si"][:n], b["thrust_si"][:n]


def metrics_diff(a: np.ndarray, b: np.ndarray) -> dict:
    d = a - b
    if len(d) == 0:
        return {"n": 0}
    c = np.corrcoef(a, b)[0, 1] if np.std(a) > 0 and np.std(b) > 0 else float("nan")
    return {
        "n": int(len(d)),
        "rms": float(np.sqrt(np.mean(d * d))),
        "max_abs": float(np.max(np.abs(d))),
        "mean": float(np.mean(d)),
        "corr": float(c),
    }


def freq_gain_phase(x: np.ndarray, y: np.ndarray, fs: float = 1000.0) -> dict:
    n = min(len(x), len(y))
    if n < 256:
        return {"note": "too few samples for FFT"}
    x, y = x[:n] - np.mean(x[:n]), y[:n] - np.mean(y[:n])
    win = np.hanning(n)
    X = np.fft.rfft(x * win)
    Y = np.fft.rfft(y * win)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    mask = (freqs >= 1.0) & (freqs <= 80.0)
    if not mask.any():
        return {"note": "no band"}
    H = Y[mask] / (X[mask] + 1e-30)
    mag = np.abs(H)
    return {
        "f_peak_hz": float(freqs[mask][np.argmax(mag)]),
        "gain_median_db": float(20 * np.log10(np.median(mag) + 1e-30)),
        "phase_median_deg": float(np.degrees(np.angle(np.median(H)))),
    }


def compare_run(flight_dir: Path) -> dict:
    out = {"flight": flight_dir.name, "levels": {}}
    for level in ("L0", "L1", "L2", "L3"):
        paths = {s: flight_dir / f"replay_{s}_{level}.csv" for s in ("ours", "omar_c", "omar_rust")}
        if not all(p.is_file() for p in paths.values()):
            continue
        data = {k: load_csv(p) for k, p in paths.items()}
        lv = {}
        # sanity Omar C vs Rust
        for ch in ("thrust_si", "tau_x", "tau_y", "tau_z"):
            oc = data["omar_c"][ch]
            orust = data["omar_rust"][ch]
            n = min(len(oc), len(orust))
            lv[f"omar_c_vs_rust_{ch}"] = metrics_diff(oc[:n], orust[:n])
        for ch in ("thrust_si", "tau_x", "tau_y", "tau_z"):
            ours = data["ours"][ch]
            oc = data["omar_c"][ch]
            n = min(len(ours), len(oc))
            lv[f"ours_vs_omar_c_{ch}"] = metrics_diff(ours[:n], oc[:n])
            lv[f"ours_vs_omar_c_{ch}_freq"] = freq_gain_phase(oc[:n], ours[:n])
        out["levels"][level] = lv
    return out


def main() -> None:
    summary = {}
    for flight_dir in sorted(OUT.iterdir()) if OUT.is_dir() else []:
        if flight_dir.is_dir() and (flight_dir / "inputs.npz").is_file():
            summary[flight_dir.name] = compare_run(flight_dir)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "compare_summary.json").write_text(json.dumps(summary, indent=2))
    print("wrote", OUT / "compare_summary.json")


if __name__ == "__main__":
    main()
