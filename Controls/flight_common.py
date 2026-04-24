#!/usr/bin/env python3
"""
flight_common.py — shared boilerplate for all HLC flight scripts
=================================================================
Provides: FlightLogger, upload_trajectory, wait_for_kalman,
          start_live_log, reset_kalman

Usage in a flight script:
    from flight_common import (
        DEFAULT_URI, HOVER_HEIGHT, KALMAN_THRESH,
        FlightLogger, upload_trajectory,
        wait_for_kalman, start_live_log, reset_kalman,
    )
"""

import sys
import time
import csv
import datetime
import os
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.mem import MemoryElement, Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

# ---------------------------------------------------------------------------
# Shared defaults — individual scripts may override locally
# ---------------------------------------------------------------------------
DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"
HOVER_HEIGHT = 1.0   # m
KALMAN_THRESH = 0.001


# ---------------------------------------------------------------------------
# Flight logger — saves CSV for post-flight trajectory analysis
# ---------------------------------------------------------------------------

class FlightLogger:
    """Logs flight state to CSV at 20 Hz for trajectory analysis.

    Usage:
        logger = FlightLogger(cf, name='circle')
        logger.start()
        logger.mark_traj_start()   # call just before first start_trajectory()
        ...
        logger.stop()              # before landing

    Output: logs/<name>_YYYYMMDD_HHMMSS.csv
    Columns: time_s, x, y, z, vx, vy, vz, roll_deg, pitch_deg, yaw_deg,
             thrust, gyro_x, gyro_y, gyro_z
    """

    _PERIOD_MS = 50  # 20 Hz
    COLUMNS = [
        "time_s",
        "x", "y", "z",
        "vx", "vy", "vz",
        "roll_deg", "pitch_deg", "yaw_deg",
        "thrust",
        "vbat",                          # battery voltage [V]
        "gyro_x", "gyro_y", "gyro_z",   # body angular velocity [deg/s]
        "acc_x", "acc_y", "acc_z",       # body accelerometer [g]
    ]

    def __init__(self, cf, name="flight"):
        os.makedirs("logs", exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join("logs", f"{name}_{ts}.csv")
        self._t0 = None
        self._lock = threading.Lock()
        self._buf = {}
        self._file = open(self.path, "w", newline="")
        self._csv = csv.writer(self._file)
        self._csv.writerow(self.COLUMNS)

        # Config 1: position + attitude (6 floats = 24 bytes)
        self._lc1 = LogConfig("FLpos", period_in_ms=self._PERIOD_MS)
        for v in (
            "stateEstimate.x",
            "stateEstimate.y",
            "stateEstimate.z",
            "stabilizer.roll",
            "stabilizer.pitch",
            "stabilizer.yaw",
        ):
            self._lc1.add_variable(v, "float")
        self._lc1.data_received_cb.add_callback(self._cb1)
        cf.log.add_config(self._lc1)

        # Config 2: velocity + thrust + vbat (5 floats = 20 bytes)
        self._lc2 = LogConfig("FLvel", period_in_ms=self._PERIOD_MS)
        for v in (
            "stateEstimate.vx",
            "stateEstimate.vy",
            "stateEstimate.vz",
            "stabilizer.thrust",
            "pm.vbat",
        ):
            self._lc2.add_variable(v, "float")
        self._lc2.data_received_cb.add_callback(self._cb2)
        cf.log.add_config(self._lc2)

        # Config 3: gyro angular velocity (3 floats = 12 bytes, deg/s)
        self._lc3 = LogConfig("FLgyro", period_in_ms=self._PERIOD_MS)
        for v in ("gyro.x", "gyro.y", "gyro.z"):
            self._lc3.add_variable(v, "float")
        self._lc3.data_received_cb.add_callback(self._cb2)
        cf.log.add_config(self._lc3)

        # Config 4: accelerometer (3 floats = 12 bytes, body frame [g])
        self._lc4 = LogConfig("FLacc", period_in_ms=self._PERIOD_MS)
        for v in ("acc.x", "acc.y", "acc.z"):
            self._lc4.add_variable(v, "float")
        self._lc4.data_received_cb.add_callback(self._cb2)
        cf.log.add_config(self._lc4)

    def start(self):
        self._lc1.start()
        self._lc2.start()
        self._lc3.start()
        self._lc4.start()

    def mark_traj_start(self):
        """Set t=0. Call just before the first start_trajectory()."""
        with self._lock:
            self._t0 = time.time()

    def _cb1(self, timestamp, data, logconf):
        with self._lock:
            self._buf.update(data)
            if self._t0 is None:
                return
            t = time.time() - self._t0
            s = self._buf
            self._csv.writerow([
                f"{t:.3f}",
                s.get("stateEstimate.x", ""),
                s.get("stateEstimate.y", ""),
                s.get("stateEstimate.z", ""),
                s.get("stateEstimate.vx", ""),
                s.get("stateEstimate.vy", ""),
                s.get("stateEstimate.vz", ""),
                s.get("stabilizer.roll", ""),
                s.get("stabilizer.pitch", ""),
                s.get("stabilizer.yaw", ""),
                s.get("stabilizer.thrust", ""),
                s.get("pm.vbat", ""),
                s.get("gyro.x", ""),
                s.get("gyro.y", ""),
                s.get("gyro.z", ""),
                s.get("acc.x", ""),
                s.get("acc.y", ""),
                s.get("acc.z", ""),
            ])
            self._file.flush()

    def _cb2(self, timestamp, data, logconf):
        with self._lock:
            self._buf.update(data)

    def stop(self):
        self._lc1.stop()
        self._lc2.stop()
        self._lc3.stop()
        self._lc4.stop()
        self._file.close()
        print(f"  [log] Saved: {self.path}")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def reset_kalman(cf):
    """Pulse kalman.resetEstimation 1→0."""
    cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    cf.param.set_value("kalman.resetEstimation", "0")


def wait_for_kalman(scf, threshold=KALMAN_THRESH):
    """Block until position variance spread drops below threshold."""
    print("  Waiting for Kalman estimator to converge...")
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
            hist_x.append(d["kalman.varPX"]); hist_x.pop(0)
            hist_y.append(d["kalman.varPY"]); hist_y.pop(0)
            hist_z.append(d["kalman.varPZ"]); hist_z.pop(0)
            sx = max(hist_x) - min(hist_x)
            sy = max(hist_y) - min(hist_y)
            sz = max(hist_z) - min(hist_z)
            print(f"    spread: x={sx:.5f}  y={sy:.5f}  z={sz:.5f}", end="\r")
            if sx < threshold and sy < threshold and sz < threshold:
                print(f"\n  Converged (spread < {threshold}).")
                return
    print("\n  Warning: Kalman did not converge within log window.")


def upload_trajectory(cf, traj_id, trajectory, scale=1.0):
    """Upload Poly4D trajectory to drone memory. Returns total planned duration (s)."""
    traj_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    traj_mem.trajectory = []

    total = 0.0
    for row in trajectory:
        dur = row[0]
        x   = Poly4D.Poly([c * scale for c in row[1:9]])
        y   = Poly4D.Poly([c * scale for c in row[9:17]])
        z   = Poly4D.Poly(row[17:25])   # z unscaled — stays flat
        yaw = Poly4D.Poly(row[25:33])   # yaw unscaled
        traj_mem.trajectory.append(Poly4D(dur, x, y, z, yaw))
        total += dur

    ok = traj_mem.write_data_sync()
    if not ok:
        print("Upload failed, aborting!")
        sys.exit(1)
    cf.high_level_commander.define_trajectory(traj_id, 0, len(traj_mem.trajectory))
    return total


def start_live_log(cf):
    """Persistent async position log printed to terminal at 2 Hz."""
    log_cfg = LogConfig(name="FlightLog", period_in_ms=500)
    log_cfg.add_variable("stateEstimate.x", "float")
    log_cfg.add_variable("stateEstimate.y", "float")
    log_cfg.add_variable("stateEstimate.z", "float")
    log_cfg.add_variable("stabilizer.roll", "float")
    log_cfg.add_variable("stabilizer.pitch", "float")
    log_cfg.add_variable("stabilizer.thrust", "float")

    def _cb(timestamp, data, _logconf):
        x   = data["stateEstimate.x"]
        y   = data["stateEstimate.y"]
        z   = data["stateEstimate.z"]
        r   = data["stabilizer.roll"]
        p   = data["stabilizer.pitch"]
        thr = data["stabilizer.thrust"]
        print(
            f"  [t={timestamp/1000:.1f}s] pos=({x:+.2f},{y:+.2f},{z:+.2f}) "
            f"roll={r:+.1f} pitch={p:+.1f} thrust={thr:.0f}"
        )

    log_cfg.data_received_cb.add_callback(_cb)
    cf.log.add_config(log_cfg)
    log_cfg.start()
    return log_cfg
