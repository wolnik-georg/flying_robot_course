#!/usr/bin/env python3
"""
Yaw Spin + Altitude Oscillation
================================
Drone holds XY position, spins continuously in yaw, and breathes up/down in Z.

  yaw(t)  = yaw0 + YAW_RATE_DEG_S * t          (unwrapped, firmware normalises)
  z(t)    = Z_BASE + Z_AMP * sin(2π * t / Z_PERIOD)

Uses OOT geometric controller (stabilizer.controller=6) with position setpoints —
no full-state or flatness needed since XY is stationary.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python run_yaw_spin.py

Requires OOT firmware: cd firmware_app && make cload
"""

import math
import time
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from flight_common import (
    DEFAULT_URI,
    HOVER_HEIGHT,
    FlightLogger,
    wait_for_kalman,
    start_live_log,
    reset_kalman,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
YAW_RATE_DEG_S = 90.0   # deg/s — full rotation every 4 s
Z_BASE         = HOVER_HEIGHT
Z_AMP          = 0.15   # m — altitude oscillation half-amplitude
Z_PERIOD       = 5.0    # s — one up-down cycle
DURATION       = 20.0   # s — total maneuver time (~5 yaw rotations, 4 z-cycles)
DT             = 0.05   # s — 20 Hz setpoint loop


# ---------------------------------------------------------------------------
# Flight function
# ---------------------------------------------------------------------------
def fly_yaw_spin(scf):
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
    logger = FlightLogger(cf, name="yaw_spin")
    logger.start()

    print(f"\nTakeoff to {HOVER_HEIGHT:.2f} m over 2 s...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    time.sleep(3.0)
    cf.param.set_value("stabilizer.controller", "6")
    time.sleep(0.5)

    # Latch current XY and yaw
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncLogger import SyncLogger
    pos_log = LogConfig(name="PosLatch", period_in_ms=100)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
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
    yaw0_deg = sum(yaws) / len(yaws)
    print(f"  Latched: x0={x0:+.3f}  y0={y0:+.3f}  yaw0={yaw0_deg:+.1f}°")

    print(f"\nExecuting: {DURATION:.0f} s  |  yaw {YAW_RATE_DEG_S:.0f} °/s  |"
          f"  z ±{Z_AMP:.2f} m @ {Z_PERIOD:.1f} s")
    logger.mark_traj_start()

    t_start = time.time()
    while True:
        t = time.time() - t_start
        if t >= DURATION:
            break

        yaw_deg = yaw0_deg + YAW_RATE_DEG_S * t
        z = Z_BASE + Z_AMP * math.sin(2.0 * math.pi * t / Z_PERIOD)

        cf.commander.send_position_setpoint(x0, y0, z, yaw_deg)
        time.sleep(DT)

    # Hold upright at hover height before landing
    print("Spin complete — settling 2 s...")
    t_hold = time.time()
    while time.time() - t_hold < 2.0:
        cf.commander.send_position_setpoint(x0, y0, Z_BASE, yaw0_deg)
        time.sleep(DT)

    print("\nLanding...")
    time.sleep(1.0)
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
    parser = argparse.ArgumentParser(description="Yaw spin + altitude oscillation")
    parser.add_argument("--uri", default=None)
    parser.add_argument("--yaw-rate", type=float, default=YAW_RATE_DEG_S,
                        help="Yaw rate in deg/s (default 90)")
    parser.add_argument("--z-amp", type=float, default=Z_AMP,
                        help="Altitude oscillation amplitude in m (default 0.15)")
    parser.add_argument("--duration", type=float, default=DURATION,
                        help="Maneuver duration in s (default 20)")
    args = parser.parse_args()

    YAW_RATE_DEG_S = args.yaw_rate
    Z_AMP = args.z_amp
    DURATION = args.duration

    uri = args.uri or uri_helper.uri_from_env(default=DEFAULT_URI)
    rot_period = 360.0 / YAW_RATE_DEG_S
    n_rot = DURATION / rot_period
    print(f"Connecting to {uri}...")
    print(f"Yaw: {YAW_RATE_DEG_S:.0f} °/s  ({rot_period:.1f} s/rev, {n_rot:.1f} revs)")
    print(f"Z:   ±{Z_AMP:.2f} m  period={Z_PERIOD:.1f} s")
    print(f"Duration: {DURATION:.0f} s")
    print(f"Requires OOT firmware: cd firmware_app && make cload")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        fly_yaw_spin(scf)
