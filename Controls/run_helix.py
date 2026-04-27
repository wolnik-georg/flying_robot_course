#!/usr/bin/env python3
"""
Helix Flight — ascending + descending spiral via full-state setpoints at 25 Hz
===============================================================================
Analytic circle in XY + linear Z ramp → differential flatness → full-state setpoints.

Two laps:
  Lap 1 (t=0..LAP_TIME):    ascending   HOVER_HEIGHT → HOVER_HEIGHT + HELIX_DZ
  Lap 2 (t=LAP_TIME..2×):   descending  HOVER_HEIGHT + HELIX_DZ → HOVER_HEIGHT

The circle is centred one radius ahead of the hover position so the drone starts
exactly on the circle with no position jump.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python run_helix.py

Requires OOT firmware: cd firmware_app && make cload
"""

import math
import time
import argparse
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from flight_common import (
    DEFAULT_URI,
    HOVER_HEIGHT,
    KALMAN_THRESH,
    FlightLogger,
    wait_for_kalman,
    start_live_log,
    reset_kalman,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
HELIX_RADIUS = 0.30  # m — circle radius
HELIX_DZ = 0.40  # m — height gained over one ascending lap
LAP_TIME = 10.5  # s — one full circle
HELIX_DT = 0.04  # s — 25 Hz loop period
MASS = 0.031  # kg — CF 2.1 + decks
GRAVITY = 9.81  # m/s²

OMEGA_C = 2.0 * math.pi / LAP_TIME  # circle angular velocity [rad/s]
VZ_UP = HELIX_DZ / LAP_TIME  # vertical speed [m/s]


# ---------------------------------------------------------------------------
# Differential flatness helper
# ---------------------------------------------------------------------------


def _rot_to_quat_xyzw(R):
    """Convert 3×3 rotation matrix to (qx, qy, qz, qw) — cflib scalar-last."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def compute_flatness(ax, ay, az, jx, jy, yaw):
    """Differential flatness: acceleration + jerk → (qx,qy,qz,qw), (wx,wy,wz).

    Implements Faessler et al. 2018 / Lee 2010.
    az is the trajectory z-acceleration (gravity added internally).
    Returns quaternion in cflib order (scalar last) and angular rates in rad/s.
    """
    # Thrust direction (body z-axis in world)
    f = np.array([ax, ay, az + GRAVITY])
    fn = float(np.linalg.norm(f))
    z_B = f / fn

    # Body x/y from yaw
    yc = np.array([-math.sin(yaw), math.cos(yaw), 0.0])  # yaw-aligned perpendicular
    xc = np.array([math.cos(yaw), math.sin(yaw), 0.0])

    # xb = normalise(yc × (acc+g·ez))  — Faessler slide 8
    xb_raw = np.cross(yc, f)
    xb_n = float(np.linalg.norm(xb_raw))
    if xb_n < 1e-9:
        xb_raw = np.cross(np.array([0.0, 0.0, 1.0]), f)
        xb_n = float(np.linalg.norm(xb_raw))
    x_B = xb_raw / xb_n
    y_B = np.cross(z_B, x_B)

    R = np.column_stack([x_B, y_B, z_B])
    qx, qy, qz, qw = _rot_to_quat_xyzw(R)

    # Angular rates from jerk (Faessler eq. ω_x = d2/c, ω_y = d1/c)
    j = np.array([jx, jy, 0.0])
    wx = float(np.dot(y_B, j)) / fn  # ωx (roll rate)
    wy = -float(np.dot(x_B, j)) / fn  # ωy (pitch rate)
    wz = 0.0  # constant yaw → yaw rate ≈ 0

    return (qx, qy, qz, qw), (wx, wy, wz)


# ---------------------------------------------------------------------------
# Flight function
# ---------------------------------------------------------------------------


def fly_helix(scf):
    cf = scf.cf
    hl = cf.high_level_commander

    print("Resetting Kalman estimator...")
    reset_kalman(cf)
    wait_for_kalman(scf)

    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    cf.param.set_value("stabilizer.controller", "1")
    time.sleep(0.1)

    log_cfg = start_live_log(cf)
    logger = FlightLogger(cf, name="helix")
    logger.start()

    print(f"\nTakeoff to {HOVER_HEIGHT:.2f} m over 2 s...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    print("  Waiting 3 s for climb + estimator stabilization...")
    time.sleep(3.0)
    cf.param.set_value("stabilizer.controller", "6")

    # Latch XY origin from EKF
    pos_log = LogConfig(name="PosLatch", period_in_ms=100)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
    pos_log.add_variable("stateEstimate.yaw", "float") if False else None
    pos_log.add_variable("stabilizer.yaw", "float")
    xs, ys, yaws = [], [], []
    with SyncLogger(scf, pos_log) as sync_log:
        for entry in sync_log:
            d = entry[1]
            xs.append(d["stateEstimate.x"])
            ys.append(d["stateEstimate.y"])
            yaws.append(d["stabilizer.yaw"])
            if len(xs) >= 10:
                break
    x0 = sum(xs) / len(xs)
    y0 = sum(ys) / len(ys)
    yaw0 = math.radians(sum(yaws) / len(yaws))
    print(
        f"  Latched origin: x0={x0:+.3f}  y0={y0:+.3f}  yaw={math.degrees(yaw0):+.1f}°"
    )

    # Circle centre one radius ahead so drone starts on circle at (x0, y0)
    cx = x0 + HELIX_RADIUS
    cy = y0
    phase0 = math.pi  # at phase=π: cos(π)=-1 → px = cx - r = x0 ✓

    print(f"\nExecuting helix: 2 × {LAP_TIME:.1f} s laps at 25 Hz")
    print(
        f"  z: {HOVER_HEIGHT:.2f} m → {HOVER_HEIGHT + HELIX_DZ:.2f} m → {HOVER_HEIGHT:.2f} m"
    )
    max_tilt = math.degrees(math.atan(HELIX_RADIUS * OMEGA_C**2 / GRAVITY))
    print(f"  Max tilt ≈ {max_tilt:.1f}°  (centripetal acceleration)")
    logger.mark_traj_start()

    t_start = time.time()
    T_TOTAL = 2.0 * LAP_TIME

    while True:
        t = time.time() - t_start
        if t >= T_TOTAL:
            break

        phase = phase0 + OMEGA_C * t

        # Position
        px = cx + HELIX_RADIUS * math.cos(phase)
        py = cy + HELIX_RADIUS * math.sin(phase)
        vz = VZ_UP if t < LAP_TIME else -VZ_UP
        pz = (
            (HOVER_HEIGHT + VZ_UP * t)
            if t <= LAP_TIME
            else (HOVER_HEIGHT + HELIX_DZ - VZ_UP * (t - LAP_TIME))
        )

        # Velocity (exact derivatives of analytic circle)
        vx = -HELIX_RADIUS * OMEGA_C * math.sin(phase)
        vy = HELIX_RADIUS * OMEGA_C * math.cos(phase)

        # Acceleration (centripetal only)
        ax = -HELIX_RADIUS * OMEGA_C**2 * math.cos(phase)
        ay = -HELIX_RADIUS * OMEGA_C**2 * math.sin(phase)

        # Jerk
        jx = HELIX_RADIUS * OMEGA_C**3 * math.sin(phase)
        jy = -HELIX_RADIUS * OMEGA_C**3 * math.cos(phase)

        (qx, qy, qz, qw), (wx, wy, wz) = compute_flatness(ax, ay, 0.0, jx, jy, yaw0)

        cf.commander.send_full_state_setpoint(
            [px, py, pz],
            [vx, vy, vz],
            [ax, ay, 0.0],
            [qx, qy, qz, qw],
            wx, wy, wz,
        )

        time.sleep(HELIX_DT)

    # Recovery hover
    print("Helix complete — holding 3 s...")
    t_hold = time.time()
    while time.time() - t_hold < 3.0:
        cf.commander.send_position_setpoint(x0, y0, HOVER_HEIGHT, 0.0)
        time.sleep(HELIX_DT)

    print("\nLanding...")
    time.sleep(1.5)
    cf.param.set_value("stabilizer.controller", "1")
    logger.stop()
    log_cfg.stop()
    hl.land(0.0, 2.0)
    time.sleep(2.0)
    hl.stop()
    print("Done.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Helix flight via full-state setpoints"
    )
    parser.add_argument("--uri", default=None)
    args = parser.parse_args()

    uri = args.uri or uri_helper.uri_from_env(default=DEFAULT_URI)
    print(f"Connecting to {uri}...")
    print(f"HELIX: r={HELIX_RADIUS}m  dz={HELIX_DZ}m  lap={LAP_TIME}s  2 laps")
    print(f"Requires OOT firmware: cd firmware_app && make cload")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        fly_helix(scf)
