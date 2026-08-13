#!/usr/bin/env python3
"""
Roll Flight — smooth 360° right roll via full-state setpoints
==============================================================
Same QP angle trajectory as the flip (0 → 2π), but rotation is around the
body x-axis instead of y-axis → right side drops first (right roll).

Usage:
    ~/.pyenv/versions/flying_robots/bin/python run_roll.py

Requires OOT firmware: cd firmware_app && make cload
"""

import math
import time
import argparse

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
FLIP_HEIGHT = HOVER_HEIGHT + 0.2  # 1.2 m — extra clearance during inversion
FLIP_DT = 0.04  # 25 Hz loop period [s]

# ---------------------------------------------------------------------------
# Roll angle trajectory — identical to flip (same QP coefficients).
# Only the rotation axis changes: qy → qx, omega_y → omega_x.
# Two segments, each row: [T_seg, c0, c1, ..., c7]
# theta(t_local) = sum(c_k * t_local^k),  t_local in [0, T_seg]  [radians]
# ---------------------------------------------------------------------------
roll_angle_segs = [
    [
        0.350000,
        -0.000000247,
        +0.000003508,
        +0.000009778,
        +0.000002773,
        +470.858642578,
        +113.301620483,
        -4221.361328125,
        +5036.835937500,
    ],
    [
        0.350000,
        +3.141595602,
        +21.038593292,
        +0.000606730,
        -176.377578735,
        -470.869232178,
        +4206.153320312,
        -8119.797363281,
        +5037.533691406,
    ],
]

T_ROLL = sum(row[0] for row in roll_angle_segs)


# ---------------------------------------------------------------------------
# Trajectory evaluation
# ---------------------------------------------------------------------------


def roll_eval(segs, t_global, deriv=0):
    """Evaluate roll angle trajectory at global time t_global."""
    t = t_global
    for seg in segs:
        dur, coeffs = seg[0], seg[1:]
        if t <= dur + 1e-9:
            t_local = min(t, dur)
            if deriv == 0:
                return sum(c * t_local**i for i, c in enumerate(coeffs))
            else:
                return sum(
                    i * c * t_local ** (i - 1) for i, c in enumerate(coeffs) if i > 0
                )
        t -= dur
    dur, coeffs = segs[-1][0], segs[-1][1:]
    if deriv == 0:
        return sum(c * dur**i for i, c in enumerate(coeffs))
    else:
        return sum(i * c * dur ** (i - 1) for i, c in enumerate(coeffs) if i > 0)


# ---------------------------------------------------------------------------
# Flight function
# ---------------------------------------------------------------------------


def fly_roll(scf):
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
    logger = FlightLogger(cf, name="roll")
    logger.start()

    print(f"\nTakeoff to {HOVER_HEIGHT:.2f} m over 2 s...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    print("  Waiting 3 s for climb + estimator stabilization...")
    time.sleep(3.0)
    cf.param.set_value("stabilizer.controller", "6")

    print(f"\nClimbing to roll height {FLIP_HEIGHT:.2f} m...")
    cf.commander.send_position_setpoint(0.0, 0.0, FLIP_HEIGHT, 0.0)
    time.sleep(2.0)

    # Latch XY origin from EKF
    pos_log = LogConfig(name="PosLatch", period_in_ms=100)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
    xs, ys = [], []
    with SyncLogger(scf, pos_log) as sync_log:
        for entry in sync_log:
            xs.append(entry[1]["stateEstimate.x"])
            ys.append(entry[1]["stateEstimate.y"])
            if len(xs) >= 10:
                break
    x0 = sum(xs) / len(xs)
    y0 = sum(ys) / len(ys)
    print(f"  Latched XY origin: x0={x0:+.3f}  y0={y0:+.3f}")

    print(f"\nExecuting roll: T_ROLL={T_ROLL:.3f} s at 25 Hz...")
    print(f"  Peak roll rate ≈ 1205 °/s  |  right side drops first")
    logger.mark_traj_start()

    t_start = time.time()
    while True:
        t = time.time() - t_start
        if t >= T_ROLL:
            break

        theta = roll_eval(roll_angle_segs, t)
        omega = roll_eval(roll_angle_segs, t, deriv=1)  # rad/s

        # Pure roll: rotate around body x-axis by theta.
        # [w, x, y, z] = [cos(θ/2), sin(θ/2), 0, 0]
        # cflib send_full_state_setpoint arg order: qx, qy, qz, qw (scalar last).
        qw = math.cos(theta / 2.0)
        qx = math.sin(theta / 2.0)

        cf.commander.send_full_state_setpoint(
            [x0, y0, FLIP_HEIGHT],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [qx, 0.0, 0.0, qw],  # qx, qy, qz, qw
            omega, 0.0, 0.0,  # wx, wy, wz [rad/s]
        )

        time.sleep(FLIP_DT)

    # Recovery: hold upright for 1 s
    print("Roll complete — holding upright for 1 s...")
    t_rec = time.time()
    while time.time() - t_rec < 1.0:
        cf.commander.send_full_state_setpoint(
            [x0, y0, FLIP_HEIGHT],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],  # identity quaternion (upright)
            0.0, 0.0, 0.0,
        )
        time.sleep(FLIP_DT)

    time.sleep(1.5)
    cf.param.set_value("stabilizer.controller", "1")
    print("\nLanding...")
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
    parser = argparse.ArgumentParser(description="Roll flight via full-state setpoints")
    parser.add_argument("--uri", default=None)
    args = parser.parse_args()

    uri = args.uri or uri_helper.uri_from_env(default=DEFAULT_URI)
    print(f"Connecting to {uri}...")
    print(f"T_ROLL={T_ROLL:.3f}s  |  FLIP_HEIGHT={FLIP_HEIGHT:.2f}m")
    print(f"Requires OOT firmware: cd firmware_app && make cload")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        fly_roll(scf)
