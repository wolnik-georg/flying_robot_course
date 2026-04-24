#!/usr/bin/env python3
"""
Hover Flight — takeoff, hold position, land
============================================
Simple hover: take off to HOVER_HEIGHT, hold for --duration seconds, land.
No trajectory upload. Uses OOT Rust geometric controller throughout.

Usage:
    ~/.pyenv/versions/flying_robots/bin/python run_hover.py
    ~/.pyenv/versions/flying_robots/bin/python run_hover.py --duration 20

Requires OOT firmware: cd firmware_app && make cload
"""

import time
import argparse

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflief
from cflib.utils import uri_helper

from flight_common import (
    DEFAULT_URI, HOVER_HEIGHT, KALMAN_THRESH,
    FlightLogger, wait_for_kalman, start_live_log, reset_kalman,
)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_DURATION = 10.0  # seconds to hover after reaching height

# ---------------------------------------------------------------------------
# Flight function
# ---------------------------------------------------------------------------


def fly_hover(scf, duration):
    cf = scf.cf
    hl = cf.high_level_commander

    print("Resetting Kalman estimator...")
    reset_kalman(cf)
    wait_for_kalman(scf)

    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    cf.param.set_value("stabilizer.controller", "6")
    time.sleep(0.1)

    log_cfg = start_live_log(cf)
    logger = FlightLogger(cf, name="hover")
    logger.start()

    print(f"\nTakeoff to {HOVER_HEIGHT:.2f} m over 2 s...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    print("  Waiting 3 s for climb + estimator stabilization...")
    time.sleep(3.0)

    print(f"\nHovering for {duration:.1f} s...")
    logger.mark_traj_start()
    time.sleep(duration)

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
    parser = argparse.ArgumentParser(description="Hover flight — takeoff, hold, land")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help=f"Hover duration in seconds (default: {DEFAULT_DURATION})")
    parser.add_argument("--uri", default=None,
                        help="Crazyflie URI (default: radio://0/80/2M/E7E7E7E7E7)")
    args = parser.parse_args()

    uri = args.uri or uri_helper.uri_from_env(default=DEFAULT_URI)
    print(f"Connecting to {uri}...")
    print(f"Hover height: {HOVER_HEIGHT:.2f} m  |  Duration: {args.duration:.1f} s")

    cflib.crtp.init_drivers()
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        fly_hover(scf, duration=args.duration)
