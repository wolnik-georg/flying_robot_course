#!/usr/bin/env python3
"""Step 1: desk delay budget from firmware constants (no flight)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy import signal

OUT = Path(__file__).resolve().parent / "out" / "indi_delay_margin"

# FACT: crazyflie-firmware/src/hal/src/sensors_bmi088_bmp3xx.c L140-141, L555-556
GYRO_LPF_HZ = 80.0
ACCEL_LPF_HZ = 30.0
SAMPLE_HZ = 1000.0

# stabilizer.c L315-357: sensors -> estimator -> controller -> controlMotors -> DShot burst same tick


def biquad_group_delay_ms(fc_hz: float, f_hz: float, fs_hz: float = 1000.0) -> float:
    """2nd-order Butterworth group delay estimate at f_hz (SIMULATION/desk)."""
    b, a = signal.butter(2, fc_hz / (0.5 * fs_hz), btype="low")
    w, gd = signal.group_delay((b, a), fs=fs_hz)
    i = int(np.argmin(np.abs(w - 2 * np.pi * f_hz)))
    return float(gd[i] / fs_hz * 1000.0)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    f_op = 6.0  # Hz, mid band of flight LC
    gd_gyro = biquad_group_delay_ms(GYRO_LPF_HZ, f_op)
    gd_acc = biquad_group_delay_ms(ACCEL_LPF_HZ, f_op)

    items = [
        {
            "path": "gyro → controller",
            "element": "BMI088 software LPF 80 Hz (2nd order lpf2p)",
            "delay_ms_at_6hz": gd_gyro,
            "source": "sensors_bmi088_bmp3xx.c:140,555; scipy biquad group delay",
            "confidence": "MEDIUM",
            "evidence": "FACT constants; SIMULATION group delay",
        },
        {
            "path": "gyro → controller",
            "element": "Stabilizer tick order (sample then controller same 1 kHz tick)",
            "delay_ms_at_6hz": 0.0,
            "source": "stabilizer.c:319-350",
            "confidence": "HIGH",
            "evidence": "FACT",
            "note": "No extra full-tick delay within loop; sensor task queue not modelled here.",
        },
        {
            "path": "accel → INDI residual",
            "element": "Accel LPF 30 Hz",
            "delay_ms_at_6hz": gd_acc,
            "source": "sensors_bmi088_bmp3xx.c:141,556",
            "confidence": "MEDIUM",
            "evidence": "FACT + SIMULATION",
        },
        {
            "path": "controller → thrust",
            "element": "DShot burst after controller (same tick)",
            "delay_ms_at_6hz": 0.5,
            "source": "stabilizer.c:350-357; JUDGEMENT",
            "confidence": "LOW",
            "evidence": "JUDGEMENT",
            "note": "Sub-ms compute + DMA; not separately measured.",
        },
        {
            "path": "controller → thrust",
            "element": "ESC/DShot frame + commutation (investigation §16)",
            "delay_ms_at_6hz": 2.0,
            "source": "investigation_indi_oscillation_2026-07-21.md §16 (~4 ms dead cited for 500 Hz model)",
            "confidence": "MEDIUM",
            "evidence": "JUDGEMENT from prior analysis",
            "note": "Distinct from motor_tau=44 ms pole in SIL plant.",
        },
        {
            "path": "controller → thrust",
            "element": "Motor first-order lag τ=44 ms",
            "delay_ms_at_6hz": None,
            "source": "SIL plant motor_tau; bench ID",
            "confidence": "HIGH",
            "evidence": "MEASUREMENT bench",
            "note": "Already in plant; not added again as dead time.",
        },
    ]

    cmd_path_ms = 0.5 + 2.0
    sens_path_ms = gd_gyro
    total_extra_ms = cmd_path_ms + sens_path_ms
    budget = {
        "operating_freq_hz_for_gd": f_op,
        "gyro_lpf_group_delay_ms": gd_gyro,
        "accel_lpf_group_delay_ms": gd_acc,
        "command_path_extra_ms_nominal": cmd_path_ms,
        "command_path_range_ms": [1.0, 4.0],
        "sensor_path_extra_ms_nominal": sens_path_ms,
        "total_extra_dead_equiv_ms_nominal": total_extra_ms,
        "total_uncertainty_ms": [2.0, 8.0],
        "bench_tau_44ms_contains": "Rotor speed dynamics; investigation states separate ~4 ms dead beyond pole fit.",
        "items": items,
    }
    (OUT / "hardware_delay_budget.json").write_text(json.dumps(budget, indent=2))
    print(json.dumps(budget, indent=2))


if __name__ == "__main__":
    main()
