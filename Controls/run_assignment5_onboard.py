#!/usr/bin/env python3
"""
Assignment 5 — onboard baseline flight: hover → circle → figure-8
==================================================================
Uses the Crazyflie high-level commander + onboard position controller
(Kalman filter + flow deck + range sensor) to fly the same three
manoeuvres defined in assignment5.rs, but with the onboard closed-loop
controller instead of an open-loop CSV replay.

Safety box matches assignment5.rs:
  x/y: ±0.50 m from take-off position  (1.0 × 1.0 m footprint)
  z:   0.00 – 0.30 m

The circle uses go_to waypoints (robust, no polynomial needed).
The figure-8 uses the Bitcraze polynomial scaled to fit the box.

Usage
-----
  # hover only (default)
  python3 Controls/run_assignment5_onboard.py

  # circle
  python3 Controls/run_assignment5_onboard.py circle

  # figure-8
  python3 Controls/run_assignment5_onboard.py figure8

  # override URI (default: radio://0/80/2M/E7E7E7E7E7)
  python3 Controls/run_assignment5_onboard.py figure8 radio://0/80/2M/E7E7E7E7E7

Requirements
------------
  cflib installed, Crazyflie with Flow Deck v2 + Z-ranger (or Multi-ranger).
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
# Configuration — tweak these to match your space
# ---------------------------------------------------------------------------
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"

HOVER_HEIGHT = 0.25  # m  — cruise altitude (assignment5.rs: start_z=0.25)
CIRCLE_RADIUS = 0.30  # m  — radius for circle and figure-8 (fits in ±0.50 m)
SPEED_SCALE = 0.5  # dimensionless — set < 1.0 for slower first flights

# Kalman variance convergence threshold
KALMAN_THRESHOLD = 0.001

# ---------------------------------------------------------------------------
# Figure-8 polynomial trajectory
# ---------------------------------------------------------------------------
# Original from Bitcraze autonomous_sequence_high_level.py — this shape has
# x/y amplitude ≈ ±1.0 m.  We scale x/y coefficients by CIRCLE_RADIUS so the
# trajectory fits inside the ±0.50 m safety box.
#
# z-channel is set to 0.0 everywhere; we use relative=True when calling
# start_trajectory(), so the drone's current height (HOVER_HEIGHT) is added
# automatically by the firmware.
#
# Row format: [duration, x0..x7, y0..y7, z0..z7, yaw0..yaw7]

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
    """Return a copy of raw with x and y polynomial coefficients multiplied by scale."""
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
# Helper: wait for Kalman estimator to converge
# ---------------------------------------------------------------------------


def wait_for_position_estimator(scf, threshold=KALMAN_THRESHOLD):
    """Block until Kalman variance spread has been < threshold for 10 samples."""
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
                print("\n  Estimator converged.")
                break


def reset_estimator(scf):
    """Reset Kalman filter and wait for convergence.

    Checks physical tilt from the raw accelerometer (reliable even when the
    EKF attitude is wrong after a crash), then issues the standard
    kalman.resetEstimation and waits for position variance to converge.

    The estimator 1→2 switch that was here previously triggered
    'ESTKALMAN: WARNING: Kalman prediction rate off' and is not needed when
    the EKF is already healthy (which the accel check confirms).
    """
    cf = scf.cf

    # --- Verify physical tilt from raw accelerometer ---
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
    time.sleep(1.0)
    acc_log.stop()

    if acc_samples:
        ax = sum(s[0] for s in acc_samples) / len(acc_samples)
        ay = sum(s[1] for s in acc_samples) / len(acc_samples)
        az = sum(s[2] for s in acc_samples) / len(acc_samples)
        mag = math.sqrt(ax**2 + ay**2 + az**2)
        tilt_deg = math.degrees(math.acos(min(1.0, abs(az) / mag))) if mag > 0 else 0.0
        print(
            f"  Raw accel: ax={ax:+.3f}g  ay={ay:+.3f}g  az={az:+.3f}g"
            f"  → tilt={tilt_deg:.1f}°"
        )
        if tilt_deg > 10.0:
            raise RuntimeError(
                f"Drone is physically tilted {tilt_deg:.1f}° — place flat on the floor and retry."
            )
    else:
        print("  Warning: no accel data received — proceeding anyway.")

    # --- Standard Kalman reset ---
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")
    wait_for_position_estimator(scf)


# ---------------------------------------------------------------------------
# Helper: upload polynomial trajectory to CF memory
# ---------------------------------------------------------------------------


def upload_trajectory(cf, traj_id, trajectory):
    """Write Poly4D segments to trajectory memory, return total duration (s)."""
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
# Safety landing helper
# ---------------------------------------------------------------------------


def emergency_land(scf):
    """
    Best-effort emergency landing — called from finally blocks.
    Sends land + stop regardless of any prior error state.
    """
    try:
        hl = scf.cf.high_level_commander
        print("\n[SAFETY] Emergency land triggered — sending land command ...")
        hl.land(0.0, 2.0)
        time.sleep(3.0)
        hl.stop()
        scf.cf.platform.send_arming_request(False)
        print("[SAFETY] Drone disarmed.")
    except Exception as ex:
        print(f"[SAFETY] Emergency land failed: {ex}")


def start_watchdog(scf, roll_limit_deg=30.0, pitch_limit_deg=30.0):
    """
    Start a lightweight logger watchdog that monitors stabilizer.roll and
    stabilizer.pitch and triggers `emergency_land` if either exceeds a limit.

    Returns the LogConfig object which should be passed to `stop_watchdog`.
    """
    cf = scf.cf
    log_cfg = LogConfig(name="watchdog", period_in_ms=100)
    try:
        log_cfg.add_variable("stabilizer.roll", "float")
        log_cfg.add_variable("stabilizer.pitch", "float")
        log_cfg.add_variable("stabilizer.thrust", "float")
    except Exception as e:
        print(f"[WATCHDOG] Cannot add variables to LogConfig: {e}")
        return None

    def _cb(timestamp, data, logconf):
        try:
            # stabilizer.roll/pitch are logged in DEGREES by the firmware
            roll = abs(data.get("stabilizer.roll", 0.0))  # degrees
            pitch = abs(data.get("stabilizer.pitch", 0.0))  # degrees
            thrust = data.get("stabilizer.thrust", 0.0)
            if roll > roll_limit_deg or pitch > pitch_limit_deg:
                print(
                    f"[WATCHDOG] Large attitude: roll={roll:.1f}°  pitch={pitch:.1f}°  thrust={thrust:.3f}"
                )
                emergency_land(scf)
        except Exception:
            pass

    log_cfg.data_received_cb.add_callback(_cb)
    try:
        cf.log.add_config(log_cfg)
        log_cfg.start()
        print(
            "[WATCHDOG] Started (roll/pitch limits: %.1f°/%.1f°)"
            % (roll_limit_deg, pitch_limit_deg)
        )
        return log_cfg
    except Exception as e:
        print(f"[WATCHDOG] Failed to start: {e}")
        return None


def stop_watchdog(scf, log_cfg):
    """Stop the watchdog LogConfig."""
    if not log_cfg:
        return
    try:
        log_cfg.stop()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Flight sequences
# ---------------------------------------------------------------------------


def _read_ekf_xy(cf, n_samples=10):
    """Return the mean EKF (x, y) position over n_samples at 50 ms each."""
    log = LogConfig(name="_TmpXY", period_in_ms=50)
    log.add_variable("stateEstimate.x", "float")
    log.add_variable("stateEstimate.y", "float")
    samples = []

    def _cb(ts, data, cfg):
        samples.append((data["stateEstimate.x"], data["stateEstimate.y"]))

    log.data_received_cb.add_callback(_cb)
    cf.log.add_config(log)
    log.start()
    # wait for at least n_samples
    deadline = time.time() + n_samples * 0.05 + 0.3
    while time.time() < deadline:
        time.sleep(0.05)
    log.stop()
    if samples:
        ox = sum(s[0] for s in samples) / len(samples)
        oy = sum(s[1] for s in samples) / len(samples)
    else:
        ox, oy = 0.0, 0.0
    return ox, oy


def fly_hover(scf):
    """Takeoff → position-lock hover 6 s at HOVER_HEIGHT → land."""
    cf = scf.cf
    hl = cf.high_level_commander
    print(f"\n[HOVER] Takeoff to {HOVER_HEIGHT:.2f} m ...")
    hl.takeoff(HOVER_HEIGHT, 2.0)

    # Wait for the 2 s takeoff ramp PLUS z-ranger dead-zone oscillation to damp.
    #
    # Root cause of previous crashes: the flow-deck z-ranger reads 0 mm below
    # ~50 mm.  As the drone climbs through that zone the EKF integrates a bad
    # velocity measurement and shifts the x/y estimate by 10–20 cm.  The HLC
    # takeoff locked x/y=0 at t=0, so the controller keeps fighting to push the
    # drone back to (0,0) against a phantom offset — oscillation grows until crash.
    #
    # Fix: wait 3 s (2 s ramp + 1 s damping), then read the actual settled EKF
    # position and issue go_to there.  The controller target now matches the EKF
    # state, the oscillation stops, and the drone holds steady wherever it is.
    print("[HOVER] Waiting 3 s for z-ranger dead zone to clear ...")
    time.sleep(3.0)

    ox, oy = _read_ekf_xy(cf)
    print(f"[HOVER] Locking position at EKF ({ox:+.3f}, {oy:+.3f}) ...")
    hl.go_to(ox, oy, HOVER_HEIGHT, 0.0, 1.5, relative=False)
    time.sleep(2.0)  # let the position hold converge

    # Stream live x/y/z for the remaining hover time
    pos_log = LogConfig(name="HoverPos", period_in_ms=200)
    pos_log.add_variable("stateEstimate.x", "float")
    pos_log.add_variable("stateEstimate.y", "float")
    pos_log.add_variable("stateEstimate.z", "float")
    with SyncLogger(scf, pos_log) as logger:
        t_end = time.time() + 6.0
        for entry in logger:
            x = entry[1]["stateEstimate.x"]
            y = entry[1]["stateEstimate.y"]
            z = entry[1]["stateEstimate.z"]
            print(f"  pos  x={x:+.3f}  y={y:+.3f}  z={z:.3f}  [lock {ox:+.3f},{oy:+.3f}]", end="\r")
            if time.time() >= t_end:
                break
    print()

    print("[HOVER] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(2.5)
    hl.stop()


def fly_circle(scf):
    """
    Takeoff → hover → circle using go_to waypoints → land.

    The circle origin is read from the EKF *after* the drone has settled at
    hover height.  Using the live EKF position rather than the raw frame
    origin (0, 0) avoids the "flies into the wall" failure mode: if the flow
    deck drifts during motor spin-up the EKF origin and the physical takeoff
    point diverge, so hardcoding (0, 0) as the circle centre sends the drone
    to the wrong physical location.
    """
    cf = scf.cf
    hl = cf.high_level_commander
    R = CIRCLE_RADIUS
    z = HOVER_HEIGHT
    n = 16  # waypoints per revolution (22.5° steps)
    seg = 1.0 / SPEED_SCALE  # seconds per segment

    print(f"\n[CIRCLE] Takeoff to {z:.2f} m ...")
    hl.takeoff(z, 2.0)  # 2 s ramp — fast through low-alt noisy zone
    time.sleep(2.5)

    # Lock position: read settled EKF coordinates and issue an explicit go_to.
    # This activates full 3-axis hold and lets any post-takeoff drift converge
    # before we take a snapshot for the circle origin.
    ox, oy = _read_ekf_xy(cf)
    print(f"[CIRCLE] Locking hover at EKF ({ox:+.3f}, {oy:+.3f}) ...")
    hl.go_to(ox, oy, z, 0.0, 2.0, relative=False)
    time.sleep(2.5)  # wait for position hold to converge

    # Re-read after the hold to get the best settled position as circle origin
    ox, oy = _read_ekf_xy(cf)
    print(
        f"[CIRCLE] Centre: EKF ({ox:+.3f}, {oy:+.3f}) m  radius={R:.2f} m  seg={seg:.1f} s"
    )

    # ── Move to circle start (angle = 0°) before the loop ─────────────────
    # Give this first move twice the normal segment time so the drone arrives
    # smoothly rather than sprinting to the first waypoint.
    x0, y0 = ox + R, oy
    print(f"[CIRCLE] Moving to start ({x0:+.3f}, {y0:+.3f}) ...")
    hl.go_to(x0, y0, z, 0.0, seg * 2, relative=False)
    time.sleep(seg * 2 + 0.3)

    # ── Circle: waypoints 1 … n (angle=0° is already reached above) ───────
    print(f"[CIRCLE] Flying {n} waypoints ...")
    for i in range(1, n + 1):
        angle = 2.0 * math.pi * i / n
        x = ox + R * math.cos(angle)
        y = oy + R * math.sin(angle)
        hl.go_to(x, y, z, 0.0, seg, relative=False)
        time.sleep(seg)

    # ── Return to hover origin ─────────────────────────────────────────────
    print("[CIRCLE] Returning to origin ...")
    hl.go_to(ox, oy, z, 0.0, 2.0, relative=False)
    time.sleep(2.5)

    print("[CIRCLE] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(2.5)
    hl.stop()


def fly_figure8(scf):
    """Takeoff → hover → figure-8 polynomial trajectory → land."""
    cf = scf.cf
    hl = cf.high_level_commander
    traj_id = 1

    print(f"\n[FIGURE-8] Uploading trajectory (x/y scale={CIRCLE_RADIUS:.2f} m) ...")
    duration = upload_trajectory(cf, traj_id, figure8)
    print(f"  Trajectory duration: {duration:.1f} s")

    print(f"[FIGURE-8] Takeoff to {HOVER_HEIGHT:.2f} m ...")
    hl.takeoff(HOVER_HEIGHT, 2.0)
    time.sleep(2.5)

    # Lock position before starting trajectory
    ox, oy = _read_ekf_xy(cf)
    print(f"[FIGURE-8] Locking hover at EKF ({ox:+.3f}, {oy:+.3f}) ...")
    cf.high_level_commander.go_to(ox, oy, HOVER_HEIGHT, 0.0, 2.0, relative=False)
    time.sleep(3.0)  # let it settle

    # Mellinger controller tracks polynomial trajectories more accurately
    # than the default PID.
    print("[FIGURE-8] Switching to Mellinger controller ...")
    try:
        cf.param.set_value("stabilizer.controller", "2")
    except Exception as e:
        print(
            f"[FIGURE-8] Warning: could not switch to Mellinger: {e} — continuing with PID"
        )

    try:
        print(
            f"[FIGURE-8] Starting trajectory at {SPEED_SCALE}x speed ({duration/SPEED_SCALE:.1f} s) ..."
        )
        hl.start_trajectory(traj_id, SPEED_SCALE, relative_position=True)
        # Print live position while trajectory runs so we can confirm motion
        pos_log = LogConfig(name="Fig8Pos", period_in_ms=200)
        pos_log.add_variable("stateEstimate.x", "float")
        pos_log.add_variable("stateEstimate.y", "float")
        pos_log.add_variable("stateEstimate.z", "float")
        with SyncLogger(scf, pos_log) as logger:
            t_end = time.time() + duration / SPEED_SCALE + 1.0
            for entry in logger:
                x = entry[1]["stateEstimate.x"]
                y = entry[1]["stateEstimate.y"]
                z = entry[1]["stateEstimate.z"]
                print(f"  pos  x={x:+.3f}  y={y:+.3f}  z={z:.3f}", end="\r")
                if time.time() >= t_end:
                    break
        print()  # newline after the \r position line
    finally:
        print("[FIGURE-8] Switching back to PID controller ...")
        try:
            cf.param.set_value("stabilizer.controller", "1")
        except Exception:
            pass  # link may be gone — emergency_land will handle it

    print("[FIGURE-8] Landing ...")
    hl.land(0.0, 2.0)
    time.sleep(2.5)
    hl.stop()


# ---------------------------------------------------------------------------
# Flight data logger
# ---------------------------------------------------------------------------


class FlightLogger:
    """Non-blocking flight data recorder.

    Starts two background log blocks at 50 Hz as soon as start() is called,
    accumulates every sample in RAM, and writes a timestamped CSV to logs/
    when stop() is called — including on crashes, so you always get the data.

    Two log blocks (both at 20 ms / 50 Hz):
      FLog1 — attitude + height : roll, pitch, yaw, thrust, zrange
      FLog2 — position+velocity : x, y, z, vx, vy, vz

    The blocks deliberately share no variables with the watchdog so the
    firmware can serve all of them independently at their own rates.
    """

    COLS = [
        "t",
        "x", "y", "z",
        "vx", "vy", "vz",
        "roll", "pitch", "yaw",
        "thrust", "zrange_mm",
    ]

    _VAR_MAP = {
        "stateEstimate.x":   "x",
        "stateEstimate.y":   "y",
        "stateEstimate.z":   "z",
        "stateEstimate.vx":  "vx",
        "stateEstimate.vy":  "vy",
        "stateEstimate.vz":  "vz",
        "stabilizer.roll":   "roll",
        "stabilizer.pitch":  "pitch",
        "stabilizer.yaw":    "yaw",
        "stabilizer.thrust": "thrust",
        "range.zrange":      "zrange_mm",
    }

    def __init__(self, cf, mode):
        self._cf = cf
        self._mode = mode
        self._t0 = None
        self._latest = {c: float("nan") for c in self.COLS if c != "t"}
        self._rows = []
        self._lock = threading.Lock()
        self._log1 = self._log2 = None

    def start(self):
        self._t0 = time.time()
        try:
            # Block 1: attitude + height sensor  (18 bytes/packet — fits in 26 B limit)
            self._log1 = LogConfig(name="FLog1", period_in_ms=20)
            self._log1.add_variable("stabilizer.roll",   "float")
            self._log1.add_variable("stabilizer.pitch",  "float")
            self._log1.add_variable("stabilizer.yaw",    "float")
            self._log1.add_variable("stabilizer.thrust", "float")
            self._log1.add_variable("range.zrange",      "uint16_t")

            # Block 2: EKF position + velocity  (24 bytes/packet — fits in 26 B limit)
            self._log2 = LogConfig(name="FLog2", period_in_ms=20)
            self._log2.add_variable("stateEstimate.x",  "float")
            self._log2.add_variable("stateEstimate.y",  "float")
            self._log2.add_variable("stateEstimate.z",  "float")
            self._log2.add_variable("stateEstimate.vx", "float")
            self._log2.add_variable("stateEstimate.vy", "float")
            self._log2.add_variable("stateEstimate.vz", "float")

            self._log1.data_received_cb.add_callback(self._cb)
            self._log2.data_received_cb.add_callback(self._cb)

            self._cf.log.add_config(self._log1)
            self._cf.log.add_config(self._log2)
            self._log1.start()
            self._log2.start()
            print("[LOG] Flight logger started (50 Hz) — will save to logs/")
        except Exception as exc:
            print(f"[LOG] Warning: could not start flight logger: {exc}")
            self._log1 = self._log2 = None

    def _cb(self, ts, data, cfg):
        renamed = {self._VAR_MAP[k]: v for k, v in data.items() if k in self._VAR_MAP}
        with self._lock:
            self._latest.update(renamed)
            row = {"t": round(time.time() - self._t0, 4)}
            row.update(self._latest)
            self._rows.append(row)

    def stop(self):
        for log in (self._log1, self._log2):
            if log:
                try:
                    log.stop()
                except Exception:
                    pass
        time.sleep(0.05)  # let any in-flight callbacks drain
        self._save()

    def _save(self):
        with self._lock:
            rows = list(self._rows)
        if not rows:
            print("[LOG] No flight data collected.")
            return
        try:
            os.makedirs("logs", exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join("logs", f"flight_{self._mode}_{ts}.csv")
            with open(fname, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=self.COLS, extrasaction="ignore")
                w.writeheader()
                w.writerows(rows)
            duration = rows[-1]["t"] if rows else 0.0
            print(f"[LOG] {len(rows)} rows  {duration:.1f} s  → {fname}")
        except Exception as exc:
            print(f"[LOG] Warning: could not save flight log: {exc}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "hover"
    uri = uri_helper.uri_from_env(default=DEFAULT_URI)
    if len(sys.argv) > 2:
        uri = sys.argv[2]

    if mode not in ("hover", "circle", "figure8"):
        print(f'Unknown mode "{mode}". Choose: hover | circle | figure8')
        sys.exit(1)

    print("=" * 60)
    print(f"Assignment 5 — onboard baseline")
    print(f"  mode        : {mode}")
    print(f"  uri         : {uri}")
    print(f"  hover height: {HOVER_HEIGHT:.2f} m  (max {0.30:.2f} m)")
    print(f"  circle r    : {CIRCLE_RADIUS:.2f} m  (safety box ±0.50 m)")
    print(f"  speed scale : {SPEED_SCALE:.1f}")
    print("=" * 60)
    print()
    input("Place the Crazyflie flat on the floor, then press ENTER to connect ...")

    cflib.crtp.init_drivers()

    cf_obj = Crazyflie(rw_cache="./cache")
    cf_obj.console.receivedChar.add_callback(lambda t: print(t, end=""))

    with SyncCrazyflie(uri, cf=cf_obj) as scf:
        # 1. Reset Kalman filter and wait for estimator to converge
        print("\nResetting Kalman estimator ...")
        reset_estimator(scf)

        # 2. Arm (must happen before the first high-level commander command)
        print("Sending arm request ...")
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.5)

        # 3. Start flight logger, safety watchdog, then fly
        flight_log = FlightLogger(scf.cf, mode)
        watchdog_cfg = None
        try:
            flight_log.start()
            watchdog_cfg = start_watchdog(
                scf, roll_limit_deg=20.0, pitch_limit_deg=20.0
            )
            if mode == "hover":
                fly_hover(scf)
            elif mode == "circle":
                fly_circle(scf)
            elif mode == "figure8":
                fly_figure8(scf)
        except KeyboardInterrupt:
            print("\n[SAFETY] Ctrl-C received during flight!")
            emergency_land(scf)
        except Exception as ex:
            print(f"\n[SAFETY] Exception during flight: {ex}")
            emergency_land(scf)
            raise
        finally:
            # Always save the flight log first (captures crash data too)
            flight_log.stop()
            try:
                stop_watchdog(scf, watchdog_cfg)
            except Exception:
                pass
            try:
                scf.cf.platform.send_arming_request(False)
            except Exception:
                pass

        print("\nFlight complete.")
