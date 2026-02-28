#!/usr/bin/env python3
"""
Sensor diagnostic — connect, read, print, disconnect.  NO arming or flying.

Checks every sensor needed for stable hover:
  acc.x/y/z       raw accelerometer (g)      physical tilt — should be < 1°
  stabilizer.roll/pitch  EKF attitude (deg)  should be near 0° on flat ground
  range.zrange    z-ranger (mm)              should be 0–30 mm on the floor
  pm.vbat         battery voltage (V)        must be > 3.6 V to fly
  kalman.varPX/Y/Z  position variance        should converge < 0.001

Usage:
  python3 Controls/diagnose_sensors.py [uri]
"""
import math
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

DEFAULT_URI = "radio://0/80/2M/E7E7E7E7E7"
DURATION = 8  # seconds of readings


def main():
    uri = uri_helper.uri_from_env(default=DEFAULT_URI)
    if len(sys.argv) > 1:
        uri = sys.argv[1]

    print(f"Connecting to {uri} ...")
    cflib.crtp.init_drivers()
    cf_obj = Crazyflie(rw_cache="./cache")

    with SyncCrazyflie(uri, cf=cf_obj) as scf:
        cf = scf.cf
        print("Connected.\n")

        # ── Params ───────────────────────────────────────────────────────────
        for name in ("stabilizer.estimator", "stabilizer.controller"):
            try:
                print(f"  {name} = {cf.param.get_value(name)}")
            except Exception as e:
                print(f"  {name}: read failed ({e})")

        # ── Log block 1: IMU + attitude ───────────────────────────────────────
        log_imu = LogConfig(name="DiagIMU", period_in_ms=200)
        log_imu.add_variable("acc.x", "float")
        log_imu.add_variable("acc.y", "float")
        log_imu.add_variable("acc.z", "float")
        log_imu.add_variable("stabilizer.roll", "float")
        log_imu.add_variable("stabilizer.pitch", "float")

        # ── Log block 2: position sensors ────────────────────────────────────
        log_pos = LogConfig(name="DiagPos", period_in_ms=200)
        log_pos.add_variable("range.zrange", "uint16_t")
        log_pos.add_variable("pm.vbat", "float")
        log_pos.add_variable("kalman.varPX", "float")
        log_pos.add_variable("kalman.varPY", "float")
        log_pos.add_variable("kalman.varPZ", "float")

        latest_imu = {}
        latest_pos = {}

        def _imu_cb(ts, data, cfg):
            latest_imu.update(data)

        def _pos_cb(ts, data, cfg):
            latest_pos.update(data)

        log_imu.data_received_cb.add_callback(_imu_cb)
        log_pos.data_received_cb.add_callback(_pos_cb)

        cf.log.add_config(log_imu)
        cf.log.add_config(log_pos)
        log_imu.start()
        log_pos.start()

        print(
            f"\n{'t':>4}  "
            f"{'tilt':>5}  {'roll':>6} {'pitch':>6}  "
            f"{'zrange':>7}  {'vbat':>5}  "
            f"{'varPX':>8} {'varPY':>8} {'varPZ':>8}"
        )
        print("-" * 80)

        t0 = time.time()
        while time.time() - t0 < DURATION:
            time.sleep(0.5)
            elapsed = time.time() - t0

            ax = latest_imu.get("acc.x", float("nan"))
            ay = latest_imu.get("acc.y", float("nan"))
            az = latest_imu.get("acc.z", float("nan"))
            roll  = latest_imu.get("stabilizer.roll",  float("nan"))
            pitch = latest_imu.get("stabilizer.pitch", float("nan"))
            zrange = latest_pos.get("range.zrange", float("nan"))
            vbat   = latest_pos.get("pm.vbat",      float("nan"))
            vpx = latest_pos.get("kalman.varPX", float("nan"))
            vpy = latest_pos.get("kalman.varPY", float("nan"))
            vpz = latest_pos.get("kalman.varPZ", float("nan"))

            mag = math.sqrt(ax**2 + ay**2 + az**2) if not math.isnan(ax) else 0
            tilt = (
                math.degrees(math.acos(min(1.0, abs(az) / mag)))
                if mag > 0 else float("nan")
            )

            print(
                f"{elapsed:4.1f}  "
                f"{tilt:5.1f}°  {roll:+6.1f}° {pitch:+6.1f}°  "
                f"{zrange:7.0f}mm  {vbat:5.3f}V  "
                f"{vpx:8.5f} {vpy:8.5f} {vpz:8.5f}"
            )

        log_imu.stop()
        log_pos.stop()

        # ── Summary ──────────────────────────────────────────────────────────
        ax = latest_imu.get("acc.x", float("nan"))
        ay = latest_imu.get("acc.y", float("nan"))
        az = latest_imu.get("acc.z", float("nan"))
        roll  = latest_imu.get("stabilizer.roll",  float("nan"))
        pitch = latest_imu.get("stabilizer.pitch", float("nan"))
        zrange = latest_pos.get("range.zrange", float("nan"))
        vbat   = latest_pos.get("pm.vbat",      float("nan"))
        vpx = latest_pos.get("kalman.varPX", float("nan"))
        vpy = latest_pos.get("kalman.varPY", float("nan"))
        vpz = latest_pos.get("kalman.varPZ", float("nan"))

        mag = math.sqrt(ax**2 + ay**2 + az**2) if not math.isnan(ax) else 0
        tilt = (
            math.degrees(math.acos(min(1.0, abs(az) / mag))) if mag > 0 else float("nan")
        )

        print("\n── Verdict ────────────────────────────────────────────────────────────")

        def check(label, ok, value_str, advice):
            mark = "[OK]" if ok else "[!!]"
            print(f"  {mark}  {label:22s} {value_str}")
            if not ok:
                print(f"         → {advice}")

        check("Physical tilt (accel)",
              not math.isnan(tilt) and tilt < 3.0,
              f"{tilt:.1f}°",
              "Drone is physically tilted — place flat on floor")

        check("EKF attitude",
              not math.isnan(roll) and abs(roll) < 5.0 and abs(pitch) < 5.0,
              f"roll={roll:+.1f}°  pitch={pitch:+.1f}°",
              "EKF stuck — run reset_estimator 1→2 flush or power-cycle")

        check("Z-ranger",
              not math.isnan(zrange) and zrange < 200,
              f"{zrange:.0f} mm",
              "Z-ranger reads high on floor — sensor may be damaged or deck unseated")

        check("Battery",
              not math.isnan(vbat) and vbat > 3.6,
              f"{vbat:.3f} V",
              "Battery too low — charge before flying (need > 3.6 V, prefer > 3.9 V)")

        kalman_ok = (
            not math.isnan(vpx)
            and vpx < 0.001 and vpy < 0.001 and vpz < 0.001
        )
        check("Kalman variance",
              kalman_ok,
              f"varPX={vpx:.5f}  varPY={vpy:.5f}  varPZ={vpz:.5f}",
              "Estimator not converged — wait longer after reset or check flow deck")

        print("\nDone.  Drone was NOT armed or flown.")


if __name__ == "__main__":
    main()
