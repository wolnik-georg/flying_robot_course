#!/usr/bin/env python3
"""Small-signal residual path + attitude loop margins (illustrative linear models)."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from scipy import signal

OUT = Path(__file__).resolve().parent / "out" / "indi_package"
OUT.mkdir(parents=True, exist_ok=True)

J_OURS = 23.951e-6
J_OMAR = 16.571710e-6
TAU_ACT = 0.044
TAU_RES = 0.055  # assumption: RPM/accel residual measurement lag
MASS = 0.041
FS_ATT = 1000.0
FS_OMAR = 500.0
FC_BW = 206.0
FC_OMAR = 30.0


def butter2(fc: float, fs: float) -> tuple[np.ndarray, np.ndarray]:
    w = min(fc / (fs / 2.0), 0.99)
    return signal.butter(2, w, btype="low")


def series(n1, d1, n2, d2):
    return np.polymul(n1, n2), np.polymul(d1, d2)


def margins(num, den):
    sys = signal.TransferFunction(num, den)
    w = np.logspace(-2, 3, 3000)
    w, mag, phase = signal.bode(sys, w)
    mag_db = 20 * np.log10(np.maximum(mag, 1e-12))
    gc = np.where((mag_db[:-1] >= 0) & (mag_db[1:] < 0))[0]
    if len(gc) == 0:
        return {"omega_c": float("nan"), "pm_deg": float("nan"), "stable_guess": False}
    i = gc[0]
    f = (0 - mag_db[i]) / (mag_db[i + 1] - mag_db[i])
    wc = w[i] * (w[i + 1] / w[i]) ** f
    ph = phase[i] + f * (phase[i + 1] - phase[i])
    pm = 180 + ph
    return {
        "omega_c_rad_s": float(wc),
        "f_c_hz": float(wc / (2 * math.pi)),
        "pm_deg": float(pm),
        "stable_guess": bool(pm > 0),
    }


def residual_path(kp_z: float, kv_z: float, res_sign: float) -> dict:
    """a_res -> delta_a_cmd -> tilt coupling -> attitude error (scalar)."""
    # Plant from disturbance force to vertical accel: 1/(m s) after lag
    g_act = [1.0], [TAU_ACT, 1.0]
    g_res = [1.0], [TAU_RES, 1.0]
    # Position loop on z: kp + kv s on accel error; simplified unity feed to a_cmd
    g_pos = [kv_z, kp_z], [1.0, 0.0]
    sign = res_sign
    # Coupling: delta theta_ref ~ (1/g) * horizontal accel command ~ k_tilt * sign * a_res
    k_tilt = 0.15 / 9.81  # rad per (m/s^2) — ASSUMPTION
    g_couple = [sign * k_tilt], [1.0]
    num, den = series(g_res[0], g_res[1], g_pos[0], g_pos[1])
    num, den = series(num, den, g_couple[0], g_couple[1])
    num, den = series(num, den, g_act[0], g_act[1])
    m = margins(num, den)
    m.update({"kp_z": kp_z, "kv_z": kv_z, "res_sign": res_sign, "model": "residual_to_tilt"})
    return m


def attitude_ours(kr: float, kw: float, J: float, fs: float, fc: float) -> dict:
    b, a = butter2(fc, fs)
    num_c = -J * np.array([kw, kr])
    den_c = [1.0]
    num_p, den_p = [1.0], [J * TAU_ACT, J, 0.0]
    num, den = series(num_c, den_c, b, a)
    num, den = series(num, den, num_p, den_p)
    m = margins(num, den)
    m.update(
        {
            "variant": "ours_indi",
            "kr": kr,
            "kw": kw,
            "J": J,
            "Jkr_Nm_per_rad": J * kr,
            "omega_n_hz": math.sqrt(kr) / (2 * math.pi),
            "zeta": kw / (2 * math.sqrt(kr)),
        }
    )
    return m


def attitude_omar(KR: float, KW: float, J: float, fs: float, fc: float) -> dict:
    b, a = butter2(fc, fs)
    num_c = -np.array([KW, KR])
    den_c = [1.0]
    num_p, den_p = [1.0], [J * TAU_ACT, J, 0.0]
    num, den = series(num_c, den_c, b, a)
    num, den = series(num, den, num_p, den_p)
    m = margins(num, den)
    m.update(
        {
            "variant": "omar_geo_core",
            "KR": KR,
            "KW": KW,
            "J": J,
            "KR_Nm_per_rad": KR,
            "omega_n_hz": math.sqrt(KR / J) / (2 * math.pi),
            "zeta": KW / (2 * math.sqrt(KR * J)),
            "kr_equiv_ours_units": KR / J,
        }
    )
    return m


def main() -> None:
    pos_sets = [
        ("flown_stiff", 64, 5),
        ("soft_28", 28, 5),
        ("omar_like", 7, 4),
    ]
    res_rows = [residual_path(kpz, kvz, s) for name, kpz, kvz in [(a, b, c) for a, b, c in pos_sets] for s in (+1, -1)]

    att_rows = [
        attitude_ours(2400, 170, J_OURS, FS_ATT, FC_BW),
        attitude_ours(1200, 120, J_OURS, FS_ATT, FC_BW),
        attitude_ours(420, 69, J_OURS, FS_ATT, FC_BW),
        attitude_ours(290, 48, J_OURS, FS_ATT, FC_BW),
        attitude_omar(0.007, 0.00115, J_OMAR, FS_OMAR, FC_OMAR),
    ]

    payload = {
        "assumptions": [
            "Linear SISO models; Omar additive INDI path omitted in attitude row",
            "Residual path uses scalar lag tau_res=55ms and k_tilt coupling (ASSUMPTION)",
            "Butterworth on measurement path only",
        ],
        "residual_path": res_rows,
        "attitude_loop": att_rows,
        "unit_conversion": {
            "omar_KR_to_kr_ours": "kr_equiv = KR / J_ours",
            "KR_0.007_over_J_ours": 0.007 / J_OURS,
            "KR_0.007_over_J_omar": 0.007 / J_OMAR,
        },
    }
    out = OUT / "small_signal_margins.json"
    out.write_text(json.dumps(payload, indent=2))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
