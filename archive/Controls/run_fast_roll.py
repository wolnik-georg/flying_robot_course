#!/usr/bin/env python3
"""
Fast Roll Flight — smooth 360° right roll in T=0.45 s via full-state setpoints
================================================================================
Same as run_roll.py but T_ROLL=0.45 s instead of 0.7 s → peak ω ≈ 1876°/s.
Rotation is around the body x-axis (right side drops first).

Usage:
    ~/.pyenv/versions/flying_robots/bin/python run_fast_roll.py

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
    DEFAULT_URI, HOVER_HEIGHT, KALMAN_THRESH,
    FlightLogger, wait_for_kalman, start_live_log, reset_kalman,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
FLIP_HEIGHT = HOVER_HEIGHT + 0.3   # 1.3 m — extra buffer for faster motion
FLIP_DT     = 0.04                 # 25 Hz loop period [s]

# ---------------------------------------------------------------------------
# Fast roll angle trajectory — same QP coefficients as fast_flip.
# Only the rotation axis changes: qy → qx, omega_y → omega_x.
# Two segments, each row: [T_seg, c0, c1, ..., c7]
# ---------------------------------------------------------------------------
fast_roll_angle_segs = [
    [0.225000, +0.000516237, +0.002698161, +0.004064898, +0.003826565,
     +2753.500488281, +1058.499389648, -59866.273437500, +111039.554687500],
    [0.225000, +3.140736580, +32.718929291, +0.148125976, -663.908203125,
     -2762.185791016, +38330.464843750, -115059.164062500, +111003.046875000],
]

T_ROLL = sum(row[0] for row in fast_roll_angle_segs)


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
                return sum(c * t_local ** i for i, c in enumerate(coeffs))
            else:
                return sum(i * c * t_local ** (i - 1)
                           for i, c in enumerate(coeffs) if i > 0)
        t -= dur
    dur, coeffs = segs[-1][0], segs[-1][1:]
    if deriv == 0:
        return sum(c * dur ** i for i, c in enumerate(coeffs))
    else:
        return sum(i * c * dur ** (i - 1)
                   for i, c in enumerate(coeffs) if i > 0)


# ---------------------------------------------------------------------------
# Flight function
# ---------------------------------------------------------------------------

def fly_fast_roll(scf):
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
    logger = FlightLogger(cf, name="fast_roll")
    logger.start()

    print(f"\nTakeoff to {HOVER_HEIGHT:.2f} m over 2 s...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    print("  Waiting 3 s for climb + estimator stabilization...")
    time.sleep(3.0)

    print("\nOOT Rust geometric controller active.")
    cf.param.set_value("stabilizer.controller", "6")
    time.sleep(0.1)

    print(f"\nClimbing to fast roll height {FLIP_HEIGHT:.2f} m (+0.3 m clearance)...")
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

    print(f"\nExecuting fast roll: T_ROLL={T_ROLL:.3f} s at 25 Hz...")
    print(f"  Peak roll rate ≈ 1876 °/s  |  right side drops first  |  2× faster")
    logger.mark_traj_start()

    t_start = time.time()
    while True:
        t = time.time() - t_start
        if t >= T_ROLL:
            break

        theta = roll_eval(fast_roll_angle_segs, t)
        omega = roll_eval(fast_roll_angle_segs, t, deriv=1)

        # Pure roll: body x-axis.  [w,x,y,z] = [cos(θ/2), sin(θ/2), 0, 0]
        # cflib send_full_state_setpoint: qx, qy, qz, qw (scalar last)
        qw = math.cos(theta / 2.0)
        qx = math.sin(theta / 2.0)

        cf.commander.send_full_state_setpoint(
            [x0, y0, FLIP_HEIGHT],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [qx, 0.0, 0.0, qw],
            omega, 0.0, 0.0,
        )

        time.sleep(FLIP_DT)

    print("Fast roll complete — holding upright for 1 s...")
    t_rec = time.time()
    while time.time() - t_rec < 1.0:
        cf.commander.send_full_state_setpoint(
            [x0, y0, FLIP_HEIGHT],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            0.0, 0.0, 0.0,
        )
        time.sleep(FLIP_DT)

    print("\nLanding...")
    cf.param.set_value("stabilizer.controller", "1")
    time.sleep(0.2)
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
    parser = argparse.ArgumentParser(description="Fast roll flight via full-state setpoints")
    parser.add_argument("--uri", default=None)
    args = parser.parse_args()

    uri = args.uri or uri_helper.uri_from_env(default=DEFAULT_URI)
    print(f"Connecting to {uri}...")
    print(f"T_ROLL={T_ROLL:.3f}s  |  FLIP_HEIGHT={FLIP_HEIGHT:.2f}m  |  peak ω≈1876°/s")
    print(f"Requires OOT firmware: cd firmware_app && make cload")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        fly_fast_roll(scf)
