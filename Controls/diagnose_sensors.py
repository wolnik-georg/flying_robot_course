#!/usr/bin/env python3
"""
Sensor diagnostic — connect, read, print, disconnect.  NO arming or flying.

Reads and prints every 0.5 s for DURATION seconds:
  acc.x/y/z          raw accelerometer (g)  → tells us true tilt
  stabilizer.roll     EKF attitude (deg)     → should be ~0 when flat
  stabilizer.pitch    EKF attitude (deg)     → should be ~0 when flat
  kalman.varPX/Y/Z    position variance      → should converge < 0.001

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
DURATION = 12  # seconds of readings


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

        # Read current estimator type
        try:
            est = cf.param.get_value("stabilizer.estimator")
            print(f"  stabilizer.estimator = {est}  (1=complementary, 2=Kalman)")
        except Exception as e:
            print(f"  Could not read stabilizer.estimator: {e}")

        # ── Log config ──────────────────────────────────────────────────────
        log_cfg = LogConfig(name="Diag", period_in_ms=200)
        log_cfg.add_variable("acc.x", "float")
        log_cfg.add_variable("acc.y", "float")
        log_cfg.add_variable("acc.z", "float")
        log_cfg.add_variable("stabilizer.roll", "float")
        log_cfg.add_variable("stabilizer.pitch", "float")

        log_var = LogConfig(name="DiagVar", period_in_ms=500)
        log_var.add_variable("kalman.varPX", "float")
        log_var.add_variable("kalman.varPY", "float")
        log_var.add_variable("kalman.varPZ", "float")

        latest = {}
        latest_var = {}

        def _cb(ts, data, cfg):
            latest.update(data)

        def _var_cb(ts, data, cfg):
            latest_var.update(data)

        log_cfg.data_received_cb.add_callback(_cb)
        log_var.data_received_cb.add_callback(_var_cb)

        cf.log.add_config(log_cfg)
        cf.log.add_config(log_var)
        log_cfg.start()
        log_var.start()

        print(
            f"\n{'t':>5}  {'ax':>7} {'ay':>7} {'az':>7}  {'tilt':>5}  "
            f"{'roll':>7} {'pitch':>7}  {'varPX':>9} {'varPY':>9} {'varPZ':>9}"
        )
        print("-" * 90)

        t0 = time.time()
        while time.time() - t0 < DURATION:
            time.sleep(0.5)
            elapsed = time.time() - t0

            ax = latest.get("acc.x", float("nan"))
            ay = latest.get("acc.y", float("nan"))
            az = latest.get("acc.z", float("nan"))
            roll = latest.get("stabilizer.roll", float("nan"))
            pitch = latest.get("stabilizer.pitch", float("nan"))
            vpx = latest_var.get("kalman.varPX", float("nan"))
            vpy = latest_var.get("kalman.varPY", float("nan"))
            vpz = latest_var.get("kalman.varPZ", float("nan"))

            # Physical tilt from accel
            mag = math.sqrt(ax**2 + ay**2 + az**2) if not math.isnan(ax) else 0
            tilt = (
                math.degrees(math.acos(min(1.0, abs(az) / mag)))
                if mag > 0 else float("nan")
            )

            print(
                f"{elapsed:5.1f}  "
                f"{ax:+7.3f} {ay:+7.3f} {az:+7.3f}  "
                f"{tilt:5.1f}°  "
                f"{roll:+7.1f}° {pitch:+7.1f}°  "
                f"{vpx:9.5f} {vpy:9.5f} {vpz:9.5f}"
            )

        log_cfg.stop()
        log_var.stop()

        # ── Summary ─────────────────────────────────────────────────────────
        ax = latest.get("acc.x", float("nan"))
        ay = latest.get("acc.y", float("nan"))
        az = latest.get("acc.z", float("nan"))
        roll = latest.get("stabilizer.roll", float("nan"))
        pitch = latest.get("stabilizer.pitch", float("nan"))

        mag = math.sqrt(ax**2 + ay**2 + az**2) if not math.isnan(ax) else 0
        tilt = (
            math.degrees(math.acos(min(1.0, abs(az) / mag))) if mag > 0 else float("nan")
        )

        print("\n── Final snapshot ─────────────────────────────────────────────────────")
        print(f"  Raw accel : ax={ax:+.4f}g  ay={ay:+.4f}g  az={az:+.4f}g  → tilt={tilt:.2f}°")
        print(f"  EKF att.  : roll={roll:+.2f}°   pitch={pitch:+.2f}°")
        print()

        if not math.isnan(tilt) and tilt < 3.0:
            print("  [OK] Drone is physically flat (tilt < 3°) — accel is good.")
        else:
            print(f"  [!!] Drone appears tilted {tilt:.1f}° from accel — place it flat!")

        if not math.isnan(roll) and abs(roll) < 5.0 and abs(pitch) < 5.0:
            print("  [OK] EKF attitude is near zero — no stuck-attitude problem.")
        else:
            print(
                f"  [!!] EKF attitude stuck: roll={roll:+.1f}°  pitch={pitch:+.1f}° "
                "— reset_estimator() 1→2 flush needed."
            )

        print("\nDone.  Drone was NOT armed or flown.")


if __name__ == "__main__":
    main()
