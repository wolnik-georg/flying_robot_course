"""Constants of the simulated vehicle and the calibrated plant, and the location of the firmware bindings.

Vehicle: Crazyflie 2.1 Brushless with the thesis mass; inertia = the exact values of the Omar C controller tree (all
controllers use the
same inertia and mass). Plant values (motor lag, noise, downwash scale, plume) are the frozen calibration of 2026-10-30
(frsim/data/calibration.json). Nothing here depends on the working directory or on absolute paths.
"""

import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PACKAGE_DIR = Path(__file__).resolve().parent
SIMULATOR_DIR = PACKAGE_DIR.parent
DATA_DIR = PACKAGE_DIR / 'data'
RESULTS_DIR = SIMULATOR_DIR / 'results'
PATCH_DIR = SIMULATOR_DIR / 'firmware' / 'patches'

# ---- vehicle
# ---------------------------------------------------------------------------------------------------------------------
MASS_KG = 0.0427
GRAVITY = 9.81  # m/s^2
INERTIA_KG_M2 = np.diag([16.571710e-6, 16.655602e-6, 29.261652e-6])  # exact Omar C tree values
INERTIA_DIAG = np.diag(INERTIA_KG_M2).copy()
THRUST_MAX_N = 0.2  # per motor, battery compensation assumed ideal (platform CF21BL)
ARM_TORQUE_M = 0.707106781 * 0.050  # lever arm of the roll / pitch torque
THRUST_TO_TORQUE_M = 0.00569278844371417  # yaw torque per newton
ROTOR_POSITIONS_M = [
    (0.0354, -0.0354),
    (-0.0354, -0.0354),
    (-0.0354, 0.0354),
    (0.0354, 0.0354),
]  # x, y of the four rotors
MOTOR_RPM_TO_FORCE = 3.91194027330774e-08  # force = k * (rpm in rad/s)^2, used for the simulated rpm telemetry
THRUST_LAW_GAIN_N = 0.1098  # real thrust = 0.1098 N * (command / (0.2 N * 0.618))^2, calibrated at hover
THRUST_LAW_HOVER_COMMAND = 0.2 * 0.618

# ---- timing and flight plan
# --------------------------------------------------------------------------------------------------------
DT_S = 1e-3  # integration step
COMMAND_DELAY_S = 0.004  # motor command latency (commands are sent every 2 ms)
PEER_EVERY_TICKS = 10  # top-vehicle position is fed to the firmware every 10 ms
TAKEOFF_S, HOLD_S, LANDING_S, GROUND_S = 3.0, 1.5, 3.0, 1.0
SCORE_AFTER_S = 2.0  # errors are scored from 2 s after the scenario start
SCENARIO_ANCHOR_M = (0.0, 0.0, 0.5)
STATE_NOISE_POSITION_M = 0.0005
STATE_NOISE_VELOCITY_M_S = 0.005

# ---- crash rule: an episode is crashed (error 100 cm) when the tilt reaches the limit inside the scoring window
# --------------------------
CRASH_TILT_DEG = {'tree': 70.0, 'fbl': 45.0}

# ---- firmware bindings
# -----------------------------------------------------------------------------------------------------------
FIRMWARE_ENV = 'FRSIM_FIRMWARE_BUILD'
DEFAULT_FIRMWARE_BUILD = (
    SIMULATOR_DIR / '_build' / 'crazyflie-firmware' / 'build'
)  # where firmware/build_bindings.sh puts it


def firmware_build_dir():
    """Directory with `_cffirmware*.so` and `cffirmware.py`: env FRSIM_FIRMWARE_BUILD, else the default build
    location."""
    return Path(os.environ.get(FIRMWARE_ENV, str(DEFAULT_FIRMWARE_BUILD)))


# ---- calibrated plant
# ------------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Plant:
    """Plant parameters of one episode (before the robustness variant is applied)."""

    motor_lag_ms: float
    gyro_noise_deg_s: float
    accelerometer_noise_g: float
    rpm_delay_ms: float
    downwash_scale: float
    max_motor_force_n: float
    plume_amplitude_n: float
    plume_width_m: float
    plume_exponent: float


def load_calibration():
    """Frozen calibration (data/calibration.json) as a Plant."""
    c = json.load(open(DATA_DIR / 'calibration.json'))
    p, f = c['plant'], c['plume_fit']
    return Plant(
        motor_lag_ms=p['motor_lag_ms'],
        gyro_noise_deg_s=p['gyro_noise_deg_s'],
        accelerometer_noise_g=p['accelerometer_noise_g'],
        rpm_delay_ms=p['rpm_delay_ms'],
        downwash_scale=p['downwash_scale'],
        max_motor_force_n=p['max_motor_force_N'],
        plume_amplitude_n=f['amplitude_N'] * p['plume_amplitude_scale'],
        plume_width_m=f['width_m'] * p['plume_width_scale'],
        plume_exponent=f['exponent'],
    )


CALIBRATED_PLANT = load_calibration()
