#!/usr/bin/env python3
"""
Assignment 5 — onboard baseline flight: hover → circle → figure-8
==================================================================
Updated 2025-02 version with improved takeoff stability for Flow deck.

Changes:
- Increased post-takeoff settle time to 6 seconds
- Extended position lock wait to 6 seconds
- _read_ekf_xy now samples over ~2 seconds (40 samples)
- Second Kalman reset after takeoff settle
- More lenient initial Kalman convergence threshold (0.0025)
- Watchdog attitude limits relaxed to 30°
- More verbose console output during critical phases
"""

import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

HOVER_HEIGHT = 0.25  # m
CIRCLE_RADIUS = 0.30  # m — fits inside ±0.50 m safety box
SPEED_SCALE = 0.5  # lower = slower & safer for testing

KALMAN_THRESHOLD = 0.0025  # more lenient than original 0.001

# ---------------------------------------------------------------------------
# Figure-8 polynomial trajectory (copy your original list here!)
# ---------------------------------------------------------------------------
_figure8_base = [
    [
        1.050000,
        0.000000,
        -0.000000,
        0.000000,
        -0.000000,
        0.830443,
        -0.276140,
        -0.384219,
        0.180493,
        -0.000000,
        0.000000,
        -0.000000,
        0.000000,
        -1.356107,
        0.688430,
        0.587426,
        -0.329106,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.710000,
        0.396058,
        0.918033,
        0.128965,
        -0.773546,
        0.339704,
        0.034310,
        -0.026417,
        -0.030049,
        -0.445604,
        -0.684403,
        0.888433,
        1.493630,
        -1.361618,
        -0.139316,
        0.158875,
        0.095799,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.620000,
        0.922409,
        0.405715,
        -0.582968,
        -0.092188,
        -0.114670,
        0.101046,
        0.075834,
        -0.037926,
        -0.291165,
        0.967514,
        0.421451,
        -1.086348,
        0.545211,
        0.030109,
        -0.050046,
        -0.068177,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.700000,
        0.923174,
        -0.431533,
        -0.682975,
        0.177173,
        0.319468,
        -0.043852,
        -0.111269,
        0.023166,
        0.289869,
        0.724722,
        -0.512011,
        -0.209623,
        -0.218710,
        0.108797,
        0.128756,
        -0.055461,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.560000,
        0.405364,
        -0.834716,
        0.158939,
        0.288175,
        -0.373738,
        -0.054995,
        0.036090,
        0.078627,
        0.450742,
        -0.385534,
        -0.954089,
        0.128288,
        0.442620,
        0.055630,
        -0.060142,
        -0.076163,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.560000,
        0.001062,
        -0.646270,
        -0.012560,
        -0.324065,
        0.125327,
        0.119738,
        0.034567,
        -0.063130,
        0.001593,
        -1.031457,
        0.015159,
        0.820816,
        -0.152665,
        -0.130729,
        -0.045679,
        0.080444,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.700000,
        -0.402804,
        -0.820508,
        -0.132914,
        0.236278,
        0.235164,
        -0.053551,
        -0.088687,
        0.031253,
        -0.449354,
        -0.411507,
        0.902946,
        0.185335,
        -0.239125,
        -0.041696,
        0.016857,
        0.016709,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.620000,
        -0.921641,
        -0.464596,
        0.661875,
        0.286582,
        -0.228921,
        -0.051987,
        0.004669,
        0.038463,
        -0.292459,
        0.777682,
        0.565788,
        -0.432472,
        -0.060568,
        -0.082048,
        -0.009439,
        0.041158,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        0.710000,
        -0.923935,
        0.447832,
        0.627381,
        -0.259808,
        -0.042325,
        -0.032258,
        0.001420,
        0.005294,
        0.288570,
        0.873350,
        -0.515586,
        -0.730207,
        -0.026023,
        0.288755,
        0.215678,
        -0.148061,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
    [
        1.053185,
        -0.398611,
        0.850510,
        -0.144007,
        -0.485368,
        -0.079781,
        0.176330,
        0.234482,
        -0.153567,
        0.447039,
        -0.532729,
        -0.855023,
        0.878509,
        0.775168,
        -0.391051,
        -0.713519,
        0.391628,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ],  # noqa
]


def _scale_xy(raw, scale):
    """Scale only x and y polynomial coefficients"""
    out = []
    for row in raw:
        dur = row[0]
        x_c = [c * scale for c in row[1:9]]
        y_c = [c * scale for c in row[9:17]]
        z_c = list(row[17:25])
        yaw_c = list(row[25:33])
        out.append([dur] + x_c + y_c + z_c + yaw_c)
    return out


figure8 = _scale_xy(_figure8_base, CIRCLE_RADIUS)

# ---------------------------------------------------------------------------
# Estimator helpers
# ---------------------------------------------------------------------------


def wait_for_position_estimator(scf, threshold=KALMAN_THRESHOLD):
    print("  Waiting for position estimator to converge ...")
    log_cfg = LogConfig(name="KalmanVar", period_in_ms=200)
    log_cfg.add_variable("kalman.varPX", "float")
    log_cfg.add_variable("kalman.varPY", "float")
    log_cfg.add_variable("kalman.varPZ", "float")

    hist_x = [1e3] * 10
    hist_y = [1e3] * 10
    hist_z = [1e3] * 10

    with SyncLogger(scf, log_cfg) as logger:
        for entry in logger:
            d = entry[1]
            hist_x.append(d["kalman.varPX"])
            hist_x.pop(0)
            hist_y.append(d["kalman.varPY"])
            hist_y.pop(0)
            hist_z.append(d["kalman.varPZ"])
            hist_z.pop(0)
            sx = max(hist_x) - min(hist_x)
            sy = max(hist_y) - min(hist_y)
            sz = max(hist_z) - min(hist_z)
            print(f"    Kalman spread: x={sx:.5f}  y={sy:.5f}  z={sz:.5f}", end="\r")
            if sx < threshold and sy < threshold and sz < threshold:
                print(f"\n  Estimator converged (spread < {threshold}).")
                break


def reset_estimator(scf):
    cf = scf.cf
    print("  Checking physical tilt from raw accelerometer ...")
    acc_log = LogConfig(name="AccCheck", period_in_ms=50)
    acc_log.add_variable("acc.x", "float")
    acc_log.add_variable("acc.y", "float")
    acc_log.add_variable("acc.z", "float")
    acc_samples = []

    def _acc_cb(ts, data, cfg):
        acc_samples.append((data["acc.x"], data["acc.y"], data["acc.z"]))

    acc_log.data_received_cb.add_callback(_acc_cb)
    cf.log.add_config(acc_log)
    acc_log.start()
    time.sleep(1.2)
    acc_log.stop()

    if acc_samples:
        ax = sum(s[0] for s in acc_samples) / len(acc_samples)
        ay = sum(s[1] for s in acc_samples) / len(acc_samples)
        az = sum(s[2] for s in acc_samples) / len(acc_samples)
        mag = math.sqrt(ax**2 + ay**2 + az**2)
        tilt_deg = math.degrees(math.acos(min(1.0, abs(az) / mag))) if mag > 0 else 0.0
        print(f"  Raw accel → tilt = {tilt_deg:.1f}°")
        if tilt_deg > 10.0:
            raise RuntimeError(f"Drone physically tilted {tilt_deg:.1f}° — place flat!")

    print("  Resetting Kalman filter ...")
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.15)
    cf.param.set_value("kalman.resetEstimation", "0")
    wait_for_position_estimator(scf)


def _read_ekf_xy(cf, n_samples=40):  # ~2 seconds @ 50 ms
    log = LogConfig(name="_TmpXY", period_in_ms=50)
    log.add_variable("stateEstimate.x", "float")
    log.add_variable("stateEstimate.y", "float")
    samples = []

    def _cb(ts, data, cfg):
        samples.append((data["stateEstimate.x"], data["stateEstimate.y"]))

    log.data_received_cb.add_callback(_cb)
    cf.log.add_config(log)
    log.start()
    deadline = time.time() + n_samples * 0.05 + 0.6
    while time.time() < deadline and len(samples) < n_samples:
        time.sleep(0.05)
    log.stop()

    if samples:
        ox = sum(s[0] for s in samples) / len(samples)
        oy = sum(s[1] for s in samples) / len(samples)
        print(f"  Sampled EKF xy over {len(samples)} points: ({ox:+.3f}, {oy:+.3f})")
        return ox, oy
    print("  Warning: no EKF xy samples collected")
    return 0.0, 0.0


# ---------------------------------------------------------------------------
# Trajectory upload
# ---------------------------------------------------------------------------


def upload_trajectory(cf, traj_id, trajectory):
    traj_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    traj_mem.trajectory = []
    total = 0.0
    for row in trajectory:
        dur = row[0]
        traj_mem.trajectory.append(
            Poly4D(
                dur,
                Poly4D.Poly(row[1:9]),
                Poly4D.Poly(row[9:17]),
                Poly4D.Poly(row[17:25]),
                Poly4D.Poly(row[25:33]),
            )
        )
        total += dur
    ok = traj_mem.write_data_sync()
    if not ok:
        raise RuntimeError("Trajectory upload failed")
    cf.high_level_commander.define_trajectory(traj_id, 0, len(traj_mem.trajectory))
    return total


# ---------------------------------------------------------------------------
# Safety & emergency
# ---------------------------------------------------------------------------


def emergency_land(scf):
    try:
        hl = scf.cf.high_level_commander
        print("\n[SAFETY] Emergency land triggered")
        hl.land(0.0, 2.0)
        time.sleep(3.0)
        hl.stop()
        scf.cf.platform.send_arming_request(False)
    except Exception as ex:
        print(f"[SAFETY] Emergency land failed: {ex}")


def start_watchdog(scf, roll_limit_deg=30.0, pitch_limit_deg=30.0):
    cf = scf.cf
    log_cfg = LogConfig(name="watchdog", period_in_ms=100)
    log_cfg.add_variable("stabilizer.roll", "float")
    log_cfg.add_variable("stabilizer.pitch", "float")
    log_cfg.add_variable("stabilizer.thrust", "float")

    def _cb(timestamp, data, logconf):
        roll = abs(data.get("stabilizer.roll", 0.0))
        pitch = abs(data.get("stabilizer.pitch", 0.0))
        if roll > roll_limit_deg or pitch > pitch_limit_deg:
            print(f"[WATCHDOG] Large attitude: roll={roll:.1f}° pitch={pitch:.1f}°")
            emergency_land(scf)

    log_cfg.data_received_cb.add_callback(_cb)
    cf.log.add_config(log_cfg)
    log_cfg.start()
    print(
        f"[WATCHDOG] Started (roll/pitch limits: {roll_limit_deg}° / {pitch_limit_deg}°)"
    )
    return log_cfg


def stop_watchdog(scf, log_cfg):
    if log_cfg:
        try:
            log_cfg.stop()
        except:
            pass


# ---------------------------------------------------------------------------
# Flight logger (your original implementation — insert here)
# ---------------------------------------------------------------------------
# Paste your complete FlightLogger class here
# For completeness I include a minimal skeleton — replace with your full version


class FlightLogger:
    COLS = [
        "t",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "roll",
        "pitch",
        "yaw",
        "thrust",
        "zrange_mm",
    ]

    def __init__(self, cf, mode):
        self._cf = cf
        self._mode = mode
        self._t0 = None
        self._rows = []
        self._log1 = self._log2 = None

    def start(self):
        self._t0 = time.time()
        # Implement your full logging setup here (two LogConfigs at 50 Hz)
        print("[LOG] Flight logger started (placeholder)")

    def stop(self):
        # Stop logs and save CSV
        print("[LOG] Flight logger stopped (placeholder)")


# ---------------------------------------------------------------------------
# Flight sequences — with improved timing
# ---------------------------------------------------------------------------


def fly_hover(scf):
    cf = scf.cf
    hl = cf.high_level_commander
    print(f"\n[HOVER] Takeoff to {HOVER_HEIGHT:.2f} m")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    print("  Waiting 6 s for z-ranger dead zone + initial settling ...")
    time.sleep(6.0)

    print("  Second Kalman reset after takeoff ...")
    reset_estimator(scf)

    ox, oy = _read_ekf_xy(cf)
    print(f"  Locking position hold at ({ox:+.3f}, {oy:+.3f})")
    hl.go_to(ox, oy, HOVER_HEIGHT, 0.0, 2.0, relative=False)
    print("  Waiting 6 s for position controller to stabilize ...")
    time.sleep(6.0)

    # Monitor during hover
    pos_log = LogConfig(name="HoverPos", period_in_ms=200)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
    pos_log.add_variable("stateEstimate.z", "float")
    with SyncLogger(scf, pos_log) as logger:
        t_end = time.time() + 8.0
        for entry in logger:
            x = entry[1]["stateEstimate.x"]
            y = entry[1]["stateEstimate.y"]
            z = entry[1]["stateEstimate.z"]
            print(f"  pos  x={x:+.3f} y={y:+.3f} z={z:.3f}          ", end="\r")
            if time.time() >= t_end:
                break
    print()

    print("[HOVER] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(3.0)
    hl.stop()


def fly_circle(scf):
    cf = scf.cf
    hl = cf.high_level_commander
    R = CIRCLE_RADIUS
    z = HOVER_HEIGHT
    n = 16
    seg = 1.0 / SPEED_SCALE

    print(f"\n[CIRCLE] Takeoff to {z:.2f} m")
    hl.takeoff(z, 2.0)
    time.sleep(6.0)

    reset_estimator(scf)

    ox, oy = _read_ekf_xy(cf)
    hl.go_to(ox, oy, z, 0.0, 2.0, relative=False)
    time.sleep(6.0)

    ox, oy = _read_ekf_xy(cf)  # re-sample after stabilization
    print(f"  Circle centre: ({ox:+.3f}, {oy:+.3f})  radius={R:.2f} m")

    # Move to start point
    x0, y0 = ox + R, oy
    hl.go_to(x0, y0, z, 0.0, seg * 2, relative=False)
    time.sleep(seg * 2 + 0.5)

    print("  Flying circle waypoints ...")
    for i in range(1, n + 1):
        angle = 2.0 * math.pi * i / n
        x = ox + R * math.cos(angle)
        y = oy + R * math.sin(angle)
        hl.go_to(x, y, z, 0.0, seg, relative=False)
        time.sleep(seg)

    hl.go_to(ox, oy, z, 0.0, 2.0, relative=False)
    time.sleep(3.0)

    print("[CIRCLE] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(3.0)
    hl.stop()


def fly_figure8(scf):
    cf = scf.cf
    hl = cf.high_level_commander
    traj_id = 1

    print("[FIGURE-8] Uploading scaled trajectory ...")
    duration = upload_trajectory(cf, traj_id, figure8)
    print(f"  Total duration at {SPEED_SCALE:.1f}x speed: {duration/SPEED_SCALE:.1f} s")

    print(f"Takeoff to {HOVER_HEIGHT:.2f} m")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    time.sleep(6.0)

    reset_estimator(scf)

    ox, oy = _read_ekf_xy(cf)
    hl.go_to(ox, oy, HOVER_HEIGHT, 0.0, 2.0, relative=False)
    time.sleep(6.0)

    try:
        cf.param.set_value("stabilizer.controller", "2")  # Mellinger
        print("  Using Mellinger controller for trajectory")
    except:
        print("  Could not set Mellinger — staying with PID")

    print("  Starting figure-8 trajectory ...")
    hl.start_trajectory(traj_id, SPEED_SCALE, relative_position=True)

    pos_log = LogConfig(name="Fig8Pos", period_in_ms=200)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
    pos_log.add_variable("stateEstimate.z", "float")
    with SyncLogger(scf, pos_log) as logger:
        t_end = time.time() + duration / SPEED_SCALE + 1.5
        for entry in logger:
            x = entry[1]["stateEstimate.x"]
            y = entry[1]["stateEstimate.y"]
            z = entry[1]["stateEstimate.z"]
            print(f"  pos  x={x:+.3f} y={y:+.3f} z={z:.3f}          ", end="\r")
            if time.time() >= t_end:
                break
    print()

    try:
        cf.param.set_value("stabilizer.controller", "1")
    except:
        pass

    print("[FIGURE-8] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(3.0)
    hl.stop()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "hover"
    uri = uri_helper.uri_from_env(default=DEFAULT_URI)
    if len(sys.argv) > 2:
        uri = sys.argv[2]

    if mode not in ("hover", "circle", "figure8"):
        print("Choose mode: hover | circle | figure8")
        sys.exit(1)

    print("=" * 70)
    print(f"  Mode          : {mode}")
    print(f"  URI           : {uri}")
    print(f"  Hover height  : {HOVER_HEIGHT:.2f} m")
    print(f"  Circle radius : {CIRCLE_RADIUS:.2f} m")
    print(f"  Speed scale   : {SPEED_SCALE:.1f}x")
    print("=" * 70)
    input("\nPlace Crazyflie flat on textured surface. Press ENTER to begin ...\n")

    cflib.crtp.init_drivers()

    cf_obj = Crazyflie(rw_cache="./cache")
    cf_obj.console.receivedChar.add_callback(lambda t: print(t, end=""))

    with SyncCrazyflie(uri, cf=cf_obj) as scf:
        print("\nInitial Kalman reset ...")
        reset_estimator(scf)

        print("Arming drone ...")
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.6)

        flight_log = FlightLogger(scf.cf, mode)
        watchdog_cfg = None

        try:
            flight_log.start()
            watchdog_cfg = start_watchdog(
                scf, roll_limit_deg=30.0, pitch_limit_deg=30.0
            )

            if mode == "hover":
                fly_hover(scf)
            elif mode == "circle":
                fly_circle(scf)
            elif mode == "figure8":
                fly_figure8(scf)

        except KeyboardInterrupt:
            print("\nCtrl+C → emergency landing")
            emergency_land(scf)
        except Exception as ex:
            print(f"\nFlight error: {ex}")
            emergency_land(scf)
            raise
        finally:
            flight_log.stop()
            if watchdog_cfg:
                stop_watchdog(scf, watchdog_cfg)
            try:
                scf.cf.platform.send_arming_request(False)
            except:
                pass

        print("\nFlight sequence complete.")
