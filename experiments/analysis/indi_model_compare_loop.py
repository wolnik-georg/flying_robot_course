#!/usr/bin/env python3
"""
Linear small-signal attitude loop comparison (roll axis, illustrative).

Assumptions documented in output JSON — NOT a flight predictor.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

OUT = Path(__file__).resolve().parent / "out" / "indi_model_compare"
OUT.mkdir(parents=True, exist_ok=True)

TAU_ACT = 0.044  # s, bench first-order motor lag
TAU_DELAY = 0.002  # s, 2×1 ms (investigation assumption)
FS = 1000.0


def butter_fc(fc_hz: float, fs: float) -> tuple[np.ndarray, np.ndarray]:
    wc = fc_hz / (fs / 2.0)
    wc = min(wc, 0.99)
    b, a = signal.butter(2, wc, btype="low")
    return b, a


def margins_from_tf(num: np.ndarray, den: np.ndarray) -> dict:
    sys = signal.TransferFunction(num, den)
    w = np.logspace(-1, 3, 4000)
    w, mag, phase = signal.bode(sys, w)
    mag_db = 20 * np.log10(np.maximum(mag, 1e-12))
    # Gain crossover
    gc_idx = None
    for i in range(1, len(mag_db)):
        if mag_db[i - 1] >= 0 and mag_db[i] < 0:
            gc_idx = i
            break
    if gc_idx is None:
        return {"omega_c_rad_s": float("nan"), "phase_margin_deg": float("nan"), "gain_margin_db": float("nan")}
    # linear interp
    f = (0 - mag_db[gc_idx - 1]) / (mag_db[gc_idx] - mag_db[gc_idx - 1])
    wc = w[gc_idx - 1] * (w[gc_idx] / w[gc_idx - 1]) ** f
    ph = phase[gc_idx - 1] + f * (phase[gc_idx] - phase[gc_idx - 1])
    pm = 180 + ph
    # Phase crossover for GM
    pc_idx = None
    for i in range(1, len(phase)):
        if phase[i - 1] > -180 and phase[i] <= -180:
            pc_idx = i
            break
    gm_db = float("nan")
    if pc_idx is not None:
        f2 = (-180 - phase[pc_idx - 1]) / (phase[pc_idx] - phase[pc_idx - 1])
        wp = w[pc_idx - 1] * (w[pc_idx] / w[pc_idx - 1]) ** f2
        _, mag_p, _ = signal.bode(sys, [wp])
        gm_db = float(-20 * np.log10(max(mag_p[0], 1e-12)))
    return {
        "omega_c_rad_s": float(wc),
        "f_c_hz": float(wc / (2 * math.pi)),
        "phase_margin_deg": float(pm),
        "gain_margin_db": gm_db,
    }


def nominal_undamped_hz(kr_eff: float) -> float:
    """For ours-style alpha loop, undamped mode sqrt(kr)/(2π) [Hz]."""
    return math.sqrt(max(kr_eff, 0.0)) / (2 * math.pi)


def build_plant(J: float) -> tuple[np.ndarray, np.ndarray]:
    """Torque command (after actuator) to angle: 1/(J s^2 * (tau_act s + 1))."""
    num = [1.0]
    den = [J * TAU_ACT, J, 0.0]
    return np.array(num, dtype=float), np.array(den, dtype=float)


def pade_delay(n: int, d: int, delay: float) -> tuple[np.ndarray, np.ndarray]:
    # scipy has no built-in pade in older versions — first-order Pade
    return np.array([-delay / 2, 1.0]), np.array([delay / 2, 1.0])


def series_tf(n1, d1, n2, d2):
    return np.polymul(n1, n2), np.polymul(d1, d2)


def variant_ours_indi(J: float, kr: float, kw: float, fc_bw: float) -> dict:
    """Simplified Tal-style inner loop: tau ≈ J*( -kr*θ - kw*s*θ ) with BW on alpha path."""
    b, a = butter_fc(fc_bw, FS)
    # Controller: -J*(kr + kw*s) with filter on measurement path ~ same on ref+meas → net on error
    num_c = -J * np.array([kw, kr])
    den_c = np.array([1.0])
    num_p, den_p = build_plant(J)
    num_d, den_d = pade_delay(1, 1, TAU_DELAY)
    num_ol, den_ol = series_tf(num_c, den_c, b, a)
    num_ol, den_ol = series_tf(num_ol, den_ol, num_p, den_p)
    num_ol, den_ol = series_tf(num_ol, den_ol, num_d, den_d)
    wn = math.sqrt(kr)
    zeta = kw / (2 * wn) if wn > 0 else float("nan")
    m = margins_from_tf(num_ol, den_ol)
    m["omega_n_nominal_rad_s"] = wn
    m["zeta_nominal"] = zeta
    m["predicted_osc_hz_nominal_sqrt_kr"] = nominal_undamped_hz(kr)
    return m


def variant_omar_geo(J: float, KR: float, KW: float, fc_hz: float, fs: float) -> dict:
    """Geometric torque law only (INDI increment neglected for linear core)."""
    b, a = butter_fc(fc_hz, fs)
    num_c = -np.array([KW, KR])
    den_c = np.array([1.0])
    num_p, den_p = build_plant(J)
    num_d, den_d = pade_delay(1, 1, TAU_DELAY)
    num_ol, den_ol = series_tf(num_c, den_c, b, a)
    num_ol, den_ol = series_tf(num_ol, den_ol, num_p, den_p)
    num_ol, den_ol = series_tf(num_ol, den_ol, num_d, den_d)
    wn = math.sqrt(KR / J)
    zeta = KW / (2 * math.sqrt(KR * J))
    m = margins_from_tf(num_ol, den_ol)
    m["omega_n_nominal_rad_s"] = wn
    m["zeta_nominal"] = zeta
    m["predicted_osc_hz_nominal_sqrt_KR_over_J"] = math.sqrt(KR / J) / (2 * math.pi)
    return m


def kr_stable_ceiling_investigation(J: float, kw: float, zeta_target: float = 1.74) -> float:
    return (kw / (2 * zeta_target)) ** 2


def main() -> None:
    rows = []

    ours = variant_ours_indi(23.951e-6, kr=2400.0, kw=170.0, fc_bw=206.0)
    ours["variant"] = "Ours full INDI (flown kr/kw)"
    ours["kr_kw_source"] = "crazyflies.yaml + Oct-02 meta"
    rows.append(ours)

    ours_800 = variant_ours_indi(23.951e-6, kr=800.0, kw=170.0 * math.sqrt(800 / 2400), fc_bw=206.0)
    ours_800["variant"] = "Ours @ investigation kr≈800 ceiling"
    rows.append(ours_800)

    omar = variant_omar_geo(16.571710e-6, KR=0.007, KW=0.00115, fc_hz=30.0, fs=500.0)
    omar["variant"] = "Omar C/Rust geometric core (500 Hz, 30 Hz filt)"
    rows.append(omar)

    ceiling = kr_stable_ceiling_investigation(23.951e-6, 170.0)
    meta = {
        "assumptions": [
            "Single-axis roll, small-angle θ",
            "Plant: torque→angle via J s² and first-order actuator τ=44 ms",
            "Pade delay 2 ms on actuator path",
            "Ours: inner INDI approximated as PD on θ through alpha loop (−J(kr θ + kw θ̇))",
            "Omar: geometric −KR θ − KW θ̇ only; additive INDI path omitted in this linear model",
            "Filters: 2nd-order Butterworth at flown cutoffs",
        ],
        "investigation_kr_ceiling_calc": ceiling,
        "flown_kr_over_ceiling": 2400.0 / ceiling,
    }

    out_json = OUT / "loop_margins.json"
    out_json.write_text(json.dumps({"meta": meta, "variants": rows}, indent=2))

    # Bode overlay (open-loop magnitude)
    fig, ax = plt.subplots(figsize=(7, 4))
    w = np.logspace(-1, 2.5, 500)
    for label, J, kr, kw, fc, fs, style in [
        ("Ours kr=2400", 23.951e-6, 2400, 170, 206, 1000, "-"),
        ("Ours kr=800", 23.951e-6, 800, 170 * math.sqrt(800 / 2400), 206, 1000, "--"),
        ("Omar geo", 16.571710e-6, 0.007, 0.00115, 30, 500, "-."),
    ]:
        b, a = butter_fc(fc, fs)
        num_c = -J * np.array([kw, kr]) if kr > 1 else -np.array([kw, kr])
        if kr > 1:
            num_c = -J * np.array([kw, kr])
        else:
            num_c = -np.array([kw, kr])
        num_p, den_p = build_plant(J)
        num_d, den_d = pade_delay(1, 1, TAU_DELAY)
        num_ol, den_ol = series_tf(num_c, np.array([1.0]), b, a)
        num_ol, den_ol = series_tf(num_ol, den_ol, num_p, den_p)
        num_ol, den_ol = series_tf(num_ol, den_ol, num_d, den_d)
        _, mag, _ = signal.bode(signal.TransferFunction(num_ol, den_ol), w)
        ax.semilogx(w / (2 * math.pi), 20 * np.log10(np.maximum(mag, 1e-12)), style, label=label)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Open-loop magnitude [dB]")
    ax.set_title("Illustrative attitude loop (linear model)")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig_path = OUT / "fig_loop_ol_bode.png"
    fig.savefig(fig_path)
    plt.close(fig)
    print(f"Wrote {out_json} and {fig_path}")


if __name__ == "__main__":
    main()
