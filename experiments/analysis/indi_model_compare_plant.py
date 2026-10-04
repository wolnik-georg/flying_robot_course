#!/usr/bin/env python3
"""Control-effectiveness constants and mixer consistency (ours vs Omar)."""

from __future__ import annotations

import json
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent / "out" / "indi_model_compare"
OUT.mkdir(parents=True, exist_ok=True)

RAD_PER_RPM = 2.0 * math.pi / 60.0
MOTORRPM2FORCE = 3.911_940_273_307_74e-8
KT_MEAN = 4.109_05e-10  # mean of flown kt1..kt4 (yaml / Oct-02 meta)
ARM = 0.707_106_781 * 0.050
T2T = 0.005_692_788_4
THRUST_MAX_MOTOR = 0.2  # CF21BL per motor, platform_defaults_cf21bl.h

# Roll moment from unit per-motor force imbalance: tau_x = ARM * (F3+F4-F1-F2) with ±1 N on pair
tau_per_unit_roll_force = ARM
yaw_torque_per_unit_force_spread = T2T


def kt_to_motorrpm2force_equiv(kt: float) -> float:
    return kt / (RAD_PER_RPM**2)


def main() -> None:
    kt_equiv = kt_to_motorrpm2force_equiv(KT_MEAN)
    scalar_ratio = KT_MEAN / (MOTORRPM2FORCE * RAD_PER_RPM**2)

    # Mixer: d(tau_x)/d(F_roll) where F_roll is differential force [N]
    mixer = {
        "arm_m": ARM,
        "thrust2torque_m": T2T,
        "thrust_max_per_motor_N": THRUST_MAX_MOTOR,
        "roll_torque_per_N_differential": tau_per_unit_roll_force,
        "yaw_torque_per_N_differential": yaw_torque_per_unit_force_spread,
        "source_mixer": "crazyflie-firmware power_distribution_quadrotor.c:95-107",
    }

    thrust_model = {
        "omar_scalar_MOTORRPM2FORCE": MOTORRPM2FORCE,
        "ours_kt_mean_N_per_RPM2": KT_MEAN,
        "kt_mean_over_scalar_equiv": scalar_ratio,
        "pct_diff_kt_vs_scalar": (scalar_ratio - 1.0) * 100.0,
        "note": "Same RPM→force at mean kt if scalar matches; per-motor spread ~±1.3% on flown kts.",
    }

    inertia = {
        "ours_Jxx_kgm2": 23.951e-6,
        "omar_Jxx_kgm2": 16.571710e-6,
        "ratio_ours_over_omar": 23.951e-6 / 16.571710e-6,
        "ours_j_scale_flown": 1.0,
        "effective_indi_J_scale_vs_omar": (23.951e-6 * 1.0) / 16.571710e-6,
    }

    mass = {
        "ours_flown_kg": 0.041,
        "omar_rust_const_kg": 0.0427,
        "omar_c_CF_MASS_build_kg": 0.0427,
        "pct_ours_vs_omar_mass": (0.041 / 0.0427 - 1.0) * 100.0,
    }

    # Commanded torque at mixer output: identical map for all using controlModeForceTorque
    effectiveness = {
        "mixer_path_identical": True,
        "torque_cmd_to_motor_force_gain_roll": tau_per_unit_roll_force,
        "assumption": "If ESC/mixer is linear below saturation, g_torque≈1 for all variants at same SI torque cmd.",
        "indi_internal_torque_estimate_ratio_ours_over_omar_J": inertia["effective_indi_J_scale_vs_omar"],
    }

    payload = {
        "mixer": mixer,
        "thrust_model": thrust_model,
        "inertia": inertia,
        "mass": mass,
        "effectiveness": effectiveness,
    }
    out_json = OUT / "plant_constants.json"
    out_json.write_text(json.dumps(payload, indent=2))
    print(f"Wrote {out_json}")


if __name__ == "__main__":
    main()
