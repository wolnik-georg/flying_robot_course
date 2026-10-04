#!/usr/bin/env python3
"""
Closed-loop package simulation (1-D roll + lagged residual + downwash).

NOT the compiled CS2 SIL — simplified host model for factor screening only.
Env INDI_PACKAGE_SWEEP=1 runs the full grid.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from scipy import signal

OUT = Path(__file__).resolve().parent / "out" / "indi_package"
OUT.mkdir(parents=True, exist_ok=True)

J = 23.951e-6
J_OMAR = 16.571710e-6
TAU_ACT = 0.044
TAU_RES = 0.055
TAU_CLAMP = 0.045
FC_BW = 206.0
FC_OMAR = 30.0
DT = 0.001
T_END = 14.0
DOWNWASH = -1.5  # m/s^2 target a_res (ASSUMPTION: sustained downwash offset)
K_TILT = 0.12 / 9.81  # rad per (m/s^2) residual coupling into effective eR (ASSUMPTION)


class Bw2:
    def __init__(self, fc: float, dt: float):
        b, a = signal.butter(2, min(fc / (0.5 / dt), 0.99), btype="low")
        self.b0, self.b1, self.b2 = b[0], b[1], b[2]
        self.a1, self.a2 = a[1], a[2]
        self.x1 = self.x2 = self.y1 = self.y2 = 0.0

    def update(self, x: float) -> float:
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y


@dataclass
class Case:
    name: str
    law: str  # ours | omar
    kr: float
    kw: float
    kp: float
    kv: float
    res_sign: float
    decimate_500: bool = False
    pos_coupling: bool = True


@dataclass
class Result:
    omega_sigma: float
    freq_hz: float
    gyro_rms_deg_s: float
    z_err_mean: float
    diverged: bool
    stable: bool


def simulate(c: Case) -> Result:
    n = int(T_END / DT)
    theta, omega = 0.05, 0.0
    z_err = 0.0
    a_res = 0.0
    tau_applied = 0.0
    rpm_delay = 0.0
    dead = [0.0, 0.0]
    k_act = DT / (DT + TAU_ACT)
    k_res = DT / (DT + TAU_RES)

    fc = FC_BW if c.law == "ours" else FC_OMAR
    fs_filt = DT if not c.decimate_500 else 0.002
    bw_w = Bw2(fc, fs_filt)
    bw_a = Bw2(fc, fs_filt)
    bw_t = Bw2(fc, fs_filt)
    omega_filt_prev = 0.0
    tau_hold = 0.0
    trace = []

    KR = 0.007
    KW = 0.00115

    for tick in range(n):
        dt_step = DT
        update = (tick % 2 == 0) if c.decimate_500 else True
        if c.decimate_500 and update:
            dt_step = 0.002  # corrected derivative interval (vs old harness bug)

        # Lagged downwash residual
        a_res += (DOWNWASH - a_res) * k_res

        # Position loop (scalar z)
        z_err += (0.0 - z_err) * 0.001  # hold target
        a_cmd = c.kp * z_err + c.kv * 0.0 + c.res_sign * a_res
        theta_des = (c.res_sign * K_TILT * a_cmd) if c.pos_coupling else 0.0

        if update:
            er = math.sin(theta) - theta_des
            if c.law == "ours":
                omega_f = bw_w.update(omega)
                alpha_meas = (omega_f - omega_filt_prev) / dt_step
                omega_filt_prev = omega_f
                alpha_meas = bw_a.update(alpha_meas)
                alpha_ref = -c.kr * er - c.kw * omega
                alpha_ref = bw_a.update(alpha_ref)
                base = rpm_delay
                delta = J * (alpha_ref - alpha_meas)
                tau_cmd = base + delta
            else:
                omega_f = bw_w.update(omega)
                alpha = (omega_f - omega_filt_prev) / dt_step
                omega_filt_prev = omega_f
                alpha_f = bw_a.update(alpha)
                tau_rpm_f = bw_t.update(rpm_delay)
                u_geo = -KR * er - KW * omega
                tau_cmd = u_geo + (tau_rpm_f - J_OMAR * alpha_f)
            tau_cmd = float(np.clip(tau_cmd, -TAU_CLAMP, TAU_CLAMP))
            tau_hold = tau_cmd

        dead[1] = dead[0]
        dead[0] = tau_hold
        tau_delayed = dead[1]
        tau_applied += (tau_delayed - tau_applied) * k_act
        rpm_delay = tau_applied
        alpha = tau_applied / J
        omega += alpha * DT
        theta += omega * DT
        trace.append(omega)

    tail = np.array(trace[int(8.0 / DT) :])
    if len(tail) < 100:
        tail = np.array(trace[-1000:])
    mean = float(np.mean(tail))
    sigma = float(np.std(tail))
    gyro_rms = sigma * 180.0 / math.pi
    crossings = np.sum((tail[:-1] - mean) * (tail[1:] - mean) < 0)
    freq = crossings / 2.0 / (len(tail) * DT)
    diverged = np.max(np.abs(tail)) > 80.0 or np.isnan(sigma)
    return Result(
        omega_sigma=sigma,
        freq_hz=float(freq),
        gyro_rms_deg_s=gyro_rms,
        z_err_mean=float(z_err),
        diverged=bool(diverged),
        stable=sigma < 0.08 and not diverged,
    )


def baselines() -> list[tuple[Case, dict]]:
    return [
        (
            Case("ours_flown", "ours", 2400, 170, 64, 5, +1, pos_coupling=False),
            {"gyro_rms_target": "260-290", "freq_target": "4.7-5.7"},
        ),
        (
            Case("omar_c", "omar", 0, 0, 7, 4, +1, pos_coupling=False),
            {"gyro_rms_target": "55-97", "freq_target": "3.3-3.9"},
        ),
    ]


def main() -> None:
    validation = []
    for case, tgt in baselines():
        r = simulate(case)
        validation.append({"case": asdict(case), "result": asdict(r), "flight_target": tgt})

    ours = validation[0]["result"]
    omar = validation[1]["result"]
    validated = (
        ours["freq_hz"] >= 4.0
        and ours["gyro_rms_deg_s"] > omar["gyro_rms_deg_s"] * 1.5
        and omar["gyro_rms_deg_s"] < ours["gyro_rms_deg_s"]
    )

    grid: list[dict] = []
    if os.environ.get("INDI_PACKAGE_SWEEP") == "1":
        pos_sets = [(64, 5, "stiff"), (28, 5, "soft28"), (7, 4, "omar_pos")]
        gain_sets = [
            (2400, 170, "flown"),
            (1200, 120, "mid"),
            (420, 69, "omar_J_ours"),
            (290, 48, "omar_equiv"),
        ]
        for kp, kv, plab in pos_sets:
            for kr, kw, glab in gain_sets:
                for rs in (+1, -1):
                    for dec in (False, True):
                        if dec and kr != 2400:
                            continue
                        c = Case(f"{glab}_{plab}_rs{int(rs)}", "ours", kr, kw, kp, kv, rs, dec)
                        r = simulate(c)
                        grid.append({**asdict(c), **asdict(r), "validated_harness": validated})

    payload = {
        "sim_type": "python_lumped_1D",
        "limits": [
            "Not CS2 SIL / not lib.rs",
            "No mocap/EKF; downwash and Rd coupling are scalar ASSUMPTIONS",
        ],
        "baseline_validation": validation,
        "harness_validated_for_baseline_separation": validated,
        "grid": grid,
    }
    path = OUT / "package_sim_results.json"
    path.write_text(json.dumps(payload, indent=2))
    print(f"Wrote {path} validated={validated} grid_n={len(grid)}")


if __name__ == "__main__":
    main()
