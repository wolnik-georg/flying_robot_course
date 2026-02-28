#!/usr/bin/env python3
"""
Low-level commander hover demo - Crazyflie with Flow deck
=========================================================

Fixed: split logging into two configs to avoid "log configuration too large" error.
"""

import sys
import time
import math
import threading
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# ────────────────────────────────────────────────
# CONFIGURATION
# ────────────────────────────────────────────────

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

HOVER_HEIGHT = 0.25  # target z (m)
HOVER_TOLERANCE = 0.04  # ok if inside this band
MAX_HOVER_TIME = 12.0  # seconds — how long to hold

# Controller gains (start conservative!)
KP_XY = 1.4  # proportional gain x/y (m/s per meter error)
KP_Z = 2.1  # proportional gain z   (m/s per meter error)

MAX_VEL_XY = 0.6  # m/s  — saturation
MAX_VEL_Z_UP = 0.5
MAX_VEL_Z_DOWN = 0.4

TILT_LIMIT_DEG = 25.0  # watchdog emergency if exceeded

# Thrust ramp parameters
THRUST_START = 8000
THRUST_STEP = 600
THRUST_MAX = 42000  # adjust depending on battery / props / weight
RAMP_STEP_TIME = 0.025  # ~40 Hz ramp

# Safety ceiling
CEILING_SOFT = 0.40  # start descending if above
CEILING_HARD = 0.50  # emergency stop

# Logging rate
LOG_PERIOD_MS = 20  # 50 Hz

# ────────────────────────────────────────────────
# Global state (protected by lock)
# ────────────────────────────────────────────────

state_lock = threading.Lock()
state = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "zrange_mm": 0,
    "battery_v": 0.0,
    "emergency": False,
}


def update_state(ts, data, logconf):
    with state_lock:
        state.update(
            {
                "x": data.get("stateEstimate.x", state["x"]),
                "y": data.get("stateEstimate.y", state["y"]),
                "z": data.get("stateEstimate.z", state["z"]),
                "vx": data.get("stateEstimate.vx", state["vx"]),
                "vy": data.get("stateEstimate.vy", state["vy"]),
                "vz": data.get("stateEstimate.vz", state["vz"]),
                "roll": data.get("stabilizer.roll", state["roll"]),
                "pitch": data.get("stabilizer.pitch", state["pitch"]),
                "yaw": data.get("stabilizer.yaw", state["yaw"]),
                "zrange_mm": data.get("range.zrange", state["zrange_mm"]),
                "battery_v": data.get("pm.vbat", state["battery_v"]),
            }
        )

        # Emergency checks
        if abs(state["roll"]) > TILT_LIMIT_DEG or abs(state["pitch"]) > TILT_LIMIT_DEG:
            state["emergency"] = True
            print(
                f"[EMERGENCY] Tilt exceeded: roll={state['roll']:.1f}° pitch={state['pitch']:.1f}°"
            )

        if state["z"] > CEILING_HARD:
            state["emergency"] = True
            print(f"[EMERGENCY] Hard ceiling hit: z = {state['z']:.3f} m")

        if state["battery_v"] < 3.3:
            state["emergency"] = True
            print(f"[EMERGENCY] Low battery: {state['battery_v']:.2f} V")


# ────────────────────────────────────────────────
# Simple P controller
# ────────────────────────────────────────────────


def compute_velocity_commands():
    with state_lock:
        ex = -state["x"]
        ey = -state["y"]
        ez = HOVER_HEIGHT - state["z"]

        vx = KP_XY * ex
        vy = KP_XY * ey
        vz = KP_Z * ez

        vx = max(min(vx, MAX_VEL_XY), -MAX_VEL_XY)
        vy = max(min(vy, MAX_VEL_XY), -MAX_VEL_XY)
        vz = max(min(vz, MAX_VEL_Z_UP), -MAX_VEL_Z_DOWN)

        if abs(ez) < HOVER_TOLERANCE * 0.5:
            vz *= 0.4

        return vx, vy, vz, 0.0


# ────────────────────────────────────────────────
# Main hover sequence
# ────────────────────────────────────────────────


def run_hover(scf):
    cf = scf.cf

    # ── Log config 1: position + velocity (fits) ──
    lg1 = LogConfig(name="PosVel", period_in_ms=LOG_PERIOD_MS)
    for v in [
        "stateEstimate.x",
        "stateEstimate.y",
        "stateEstimate.z",
        "stateEstimate.vx",
        "stateEstimate.vy",
        "stateEstimate.vz",
    ]:
        lg1.add_variable(v)

    lg1.data_received_cb.add_callback(update_state)
    cf.log.add_config(lg1)
    lg1.start()

    # ── Log config 2: attitude + sensors (fits) ──
    lg2 = LogConfig(name="AttitudeSensors", period_in_ms=LOG_PERIOD_MS)
    for v in [
        "stabilizer.roll",
        "stabilizer.pitch",
        "stabilizer.yaw",
        "range.zrange",
        "pm.vbat",
    ]:
        lg2.add_variable(v)

    lg2.data_received_cb.add_callback(update_state)
    cf.log.add_config(lg2)
    lg2.start()

    # Arm
    print("Arming ...")
    cf.platform.send_arming_request(True)
    time.sleep(0.6)

    # Thrust ramp
    print("Thrust ramp starting ...")
    thrust = THRUST_START
    while thrust < THRUST_MAX and not state["emergency"]:
        cf.commander.send_setpoint(0, 0, 0, int(thrust))
        thrust += THRUST_STEP
        time.sleep(RAMP_STEP_TIME)

        with state_lock:
            if state["z"] > 0.08 or state["zrange_mm"] > 80:
                print(
                    f"  Airborne detected (z={state['z']:.3f} m, zrange={state['zrange_mm']} mm)"
                )
                break

    if state["emergency"]:
        print("Emergency during ramp → aborting")
        cf.commander.send_setpoint(0, 0, 0, 0)
        lg1.stop()
        lg2.stop()
        return

    # Velocity control phase
    print("Switching to velocity control — starting hover controller")
    t0 = time.time()
    cycle_count = 0

    try:
        while not state["emergency"]:
            t = time.time() - t0

            if t > MAX_HOVER_TIME + 5:
                print("Max hover time reached → descending")
                break

            vx, vy, vz, yawrate = compute_velocity_commands()

            with state_lock:
                if state["z"] > CEILING_SOFT:
                    vz = max(vz, -0.6)

            cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)

            cycle_count += 1
            if cycle_count % 25 == 0:
                with state_lock:
                    print(
                        f"t={t:5.1f}s | x={state['x']:+5.3f} y={state['y']:+5.3f} z={state['z']:+5.3f} "
                        f"| vz_cmd={vz:+5.3f} | batt={state['battery_v']:.2f}V",
                        end="\r",
                    )

            time.sleep(0.010)

    except KeyboardInterrupt:
        print("\nCtrl+C caught → emergency stop")

    finally:
        print("\nStopping motors ...")
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.3)
        cf.platform.send_arming_request(False)
        lg1.stop()
        lg2.stop()
        print("Hover sequence finished.")


# ────────────────────────────────────────────────
# Main entry point
# ────────────────────────────────────────────────

if __name__ == "__main__":
    cflib.crtp.init_drivers(enable_debug_driver=False)

    print("Low-level hover demo")
    print(f"Target: {HOVER_HEIGHT:.2f} ± {HOVER_TOLERANCE:.2f} m")
    print(f"URI:    {URI}")
    print("-" * 60)
    input("Place Crazyflie on flat textured surface. Press ENTER when ready ...\n")

    cf = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(URI, cf=cf) as scf:
        try:
            run_hover(scf)
        except Exception as e:
            print(f"\nError during flight: {e}")
            try:
                cf.commander.send_setpoint(0, 0, 0, 0)
                cf.platform.send_arming_request(False)
            except:
                pass
