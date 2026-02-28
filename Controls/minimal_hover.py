#!/usr/bin/env python3
"""
Enhanced MotionCommander demo — official Bitcraze tutorial style + rich logging
- All original functions preserved
- Rich logging (pose/vel, attitude/power, Kalman/flow + raw acc)
- CSV + plots saved into dedicated run folder (runs/YYYYMMDD_HHMMSS_motion/)
- Post-flight performance summary + verdict
"""

import logging
import sys
import time
from threading import Event
import csv
import os
from datetime import datetime

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

try:
    import numpy as np
    import matplotlib.pyplot as plt

    PLOTTING = True
except ImportError:
    PLOTTING = False
    print("Matplotlib/numpy not found — plots disabled")

URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.3

deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)

position_estimate = [0.0, 0.0]  # live x,y for box demo

# Global: motion name for folder & filename
current_motion = "unknown"

# ────────────────────────────────────────────────
# Global flight log storage
# ────────────────────────────────────────────────

flight_data = []


def log_callback(ts, data, logconf):
    row = {
        "t_ms": ts,
        "x": data.get("stateEstimate.x", 0.0),
        "y": data.get("stateEstimate.y", 0.0),
        "z": data.get("stateEstimate.z", 0.0),
        "vx": data.get("stateEstimate.vx", 0.0),
        "vy": data.get("stateEstimate.vy", 0.0),
        "vz": data.get("stateEstimate.vz", 0.0),
        "roll": data.get("stabilizer.roll", 0.0),
        "pitch": data.get("stabilizer.pitch", 0.0),
        "yaw": data.get("stabilizer.yaw", 0.0),
        "thrust": data.get("stabilizer.thrust", 0.0),
        "zrange_mm": data.get("range.zrange", 0),
        "vbat": data.get("pm.vbat", 0.0),
        "varPX": data.get("kalman.varPX", 0.0),
        "varPY": data.get("kalman.varPY", 0.0),
        "varPZ": data.get("kalman.varPZ", 0.0),
        "flow_deltaX": data.get("flow.deltaX", 0),
        "flow_deltaY": data.get("flow.deltaY", 0),
        "flow_squal": data.get("flow.squal", 0),
        "flow_rawPixelSum": data.get("flow.rawPixelSum", 0),
        "acc_x": data.get("acc.x", 0.0),
        "acc_y": data.get("acc.y", 0.0),
        "acc_z": data.get("acc.z", 0.0),
    }
    flight_data.append(row)

    # Live position for box demo
    global position_estimate
    position_estimate[0] = row["x"]
    position_estimate[1] = row["y"]


# ────────────────────────────────────────────────
# Original callbacks & functions (unchanged except minor timing)
# ────────────────────────────────────────────────


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print("Deck is attached!")
    else:
        print("Deck is NOT attached!")


def take_off_simple(scf):
    global current_motion
    current_motion = "hover"
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print(f"Takeoff → hovering at {DEFAULT_HEIGHT:.2f} m for 10 s")
        time.sleep(5)
        mc.stop()


def move_linear_simple(scf):
    global current_motion
    current_motion = "linear"
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.3)
        time.sleep(0.5)
        mc.turn_left(90)
        time.sleep(0.5)
        mc.forward(0.3)
        time.sleep(0.5)
        mc.turn_left(90)
        time.sleep(0.5)
        mc.forward(0.3)
        time.sleep(0.5)
        mc.turn_left(90)


def move_box_limit(scf):
    global current_motion
    current_motion = "box"
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2
        print("Bouncing inside box ... (Ctrl+C to stop)")
        while True:
            if position_estimate[0] > BOX_LIMIT:
                body_x_cmd = -max_vel
            elif position_estimate[0] < -BOX_LIMIT:
                body_x_cmd = max_vel
            if position_estimate[1] > BOX_LIMIT:
                body_y_cmd = -max_vel
            elif position_estimate[1] < -BOX_LIMIT:
                body_y_cmd = max_vel
            mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)
            time.sleep(0.1)


def move_circle(scf):
    global current_motion
    current_motion = "circle"
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Starting approximate circle ...")
        num_steps = 48
        segment_length = 0.04
        turn_angle = 360.0 / num_steps
        segment_time = 0.25
        time.sleep(1.0)
        for i in range(num_steps):
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_left(turn_angle)
        print("Circle complete.")


def move_figure8(scf):
    global current_motion
    current_motion = "figure8"
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Starting approximate figure-8 ...")
        num_steps = 48
        segment_length = 0.035
        turn_angle = 360.0 / num_steps
        segment_time = 0.22
        time.sleep(1.0)
        print("  First loop")
        for i in range(num_steps):
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_left(turn_angle)
        print("  Transition")
        mc.forward(0.12)
        time.sleep(0.7)
        print("  Second loop")
        for i in range(num_steps):
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_right(turn_angle)
        print("Figure-8 complete.")


# ────────────────────────────────────────────────
# Main — choose motion + rich logging
# ────────────────────────────────────────────────

if __name__ == "__main__":
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        scf.cf.param.add_update_callback(
            group="deck", name="bcFlow2", cb=param_deck_flow
        )
        time.sleep(1)

        # Logging configs (3 to avoid packet size limit)
        logconf1 = LogConfig(name="PoseVel", period_in_ms=20)
        for v in [
            "stateEstimate.x",
            "stateEstimate.y",
            "stateEstimate.z",
            "stateEstimate.vx",
            "stateEstimate.vy",
            "stateEstimate.vz",
        ]:
            logconf1.add_variable(v)

        logconf2 = LogConfig(name="AttitudeThrustBat", period_in_ms=20)
        for v in [
            "stabilizer.roll",
            "stabilizer.pitch",
            "stabilizer.yaw",
            "stabilizer.thrust",
            "pm.vbat",
        ]:
            logconf2.add_variable(v)

        logconf3 = LogConfig(name="Kalman", period_in_ms=40)
        for v in ["kalman.varPX", "kalman.varPY", "kalman.varPZ"]:
            logconf3.add_variable(v)

        logconf4 = LogConfig(name="RangeAcc", period_in_ms=40)  # new small config
        for v in ["range.zrange", "acc.x", "acc.y", "acc.z"]:
            logconf4.add_variable(v)

        scf.cf.log.add_config(logconf1)
        scf.cf.log.add_config(logconf2)
        scf.cf.log.add_config(logconf3)
        scf.cf.log.add_config(logconf4)

        logconf1.data_received_cb.add_callback(log_callback)
        logconf2.data_received_cb.add_callback(log_callback)
        logconf3.data_received_cb.add_callback(log_callback)
        logconf4.data_received_cb.add_callback(log_callback)

        if not deck_attached_event.wait(timeout=5):
            print("No flow deck detected!")
            sys.exit(1)

        scf.cf.platform.send_arming_request(True)
        time.sleep(1.2)

        logconf1.start()
        logconf2.start()
        logconf3.start()
        logconf4.start()

        try:
            # ── Choose ONE motion ──
            # take_off_simple(scf)  # pure hover (recommended first)
            # move_linear_simple(scf)
            # move_box_limit(scf)
            # move_circle(scf)
            move_figure8(scf)

        except KeyboardInterrupt:
            print("\nCtrl+C → stopping")

        except Exception as e:
            print(f"\nError during motion: {e}")

        finally:
            logconf1.stop()
            logconf2.stop()
            logconf3.stop()
            logconf4.stop()
            scf.cf.platform.send_arming_request(False)

            # ── Save to dedicated run folder ──
            if flight_data:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                run_folder = f"runs/{ts}_{current_motion}"
                os.makedirs(run_folder, exist_ok=True)
                csv_path = os.path.join(run_folder, "log.csv")

                with open(csv_path, "w", newline="") as f:
                    f.write(f"# Motion type:     {current_motion}\n")
                    f.write(f"# Target height:   {DEFAULT_HEIGHT:.2f} m\n")
                    f.write(
                        f"# Date & time:     {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n"
                    )
                    f.write(f"# URI:             {URI}\n\n")
                    writer = csv.DictWriter(f, fieldnames=flight_data[0].keys())
                    writer.writeheader()
                    writer.writerows(flight_data)
                print(f"\nSaved CSV → {csv_path}")

                # ── Performance analysis ──
                xs = np.array([r["x"] for r in flight_data])
                ys = np.array([r["y"] for r in flight_data])
                zs = np.array([r["z"] for r in flight_data])
                vxs = np.array([r["vx"] for r in flight_data])
                vys = np.array([r["vy"] for r in flight_data])
                vzs = np.array([r["vz"] for r in flight_data])
                thrusts = np.array([r["thrust"] for r in flight_data])
                vbats = np.array([r["vbat"] for r in flight_data])

                max_drift_x = np.max(np.abs(xs))
                max_drift_y = np.max(np.abs(ys))
                max_z_dev = np.max(np.abs(zs - DEFAULT_HEIGHT))
                rms_xy = np.sqrt(np.mean(xs**2 + ys**2))
                std_z = np.std(zs)
                max_speed = np.max(np.sqrt(vxs**2 + vys**2 + vzs**2))
                avg_thrust = np.mean(thrusts)
                thrust_var = np.std(thrusts)

                print("\n" + "=" * 70)
                print(f"Performance Analysis — Motion: {current_motion}")
                print("-" * 70)
                print(
                    f"  Max horizontal drift:      x = {max_drift_x:.3f} m    y = {max_drift_y:.3f} m"
                )
                print(f"  RMS horizontal position error: {rms_xy:.3f} m")
                print(f"  Max z deviation from target:   {max_z_dev:.3f} m")
                print(f"  Z height stability (std dev):  {std_z:.3f} m")
                print(f"  Max horizontal speed:          {max_speed:.3f} m/s")
                print(f"  Average thrust:                {avg_thrust:.0f}")
                print(f"  Thrust variance:               {thrust_var:.0f}")
                print("-" * 70)

                verdict = "EXCELLENT"
                if max_drift_x > 0.12 or max_drift_y > 0.12 or max_z_dev > 0.05:
                    verdict = "GOOD"
                if max_drift_x > 0.20 or max_drift_y > 0.20 or max_z_dev > 0.08:
                    verdict = "OK"
                if max_drift_x > 0.30 or max_drift_y > 0.30 or max_z_dev > 0.12:
                    verdict = "POOR – significant drift / instability"
                print(f"  Verdict: {verdict}")
                print("=" * 70)

                # After flight analysis
                if "flow_squal" in flight_data[0]:
                    avg_squal = np.mean(
                        [r["flow_squal"] for r in flight_data if r["flow_squal"] > 0]
                    )
                    print(
                        f"  Average flow quality (squal): {avg_squal:.1f}  (good >60–80)"
                    )

                # ── Plots saved into run folder ──
                if PLOTTING:
                    fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
                    ts_arr = np.array([r["t_ms"] / 1000 for r in flight_data])
                    axs[0].plot(ts_arr, xs, label="x")
                    axs[0].plot(ts_arr, ys, label="y")
                    axs[0].axhline(0, color="gray", ls="--", alpha=0.5)
                    axs[0].legend()
                    axs[0].set_ylabel("Position XY (m)")
                    axs[0].grid(True, alpha=0.3)
                    axs[1].plot(ts_arr, zs, label="z")
                    axs[1].axhline(DEFAULT_HEIGHT, color="gray", ls="--")
                    axs[1].legend()
                    axs[1].set_ylabel("Height (m)")
                    axs[1].grid(True, alpha=0.3)
                    axs[2].plot(ts_arr, thrusts, label="thrust")
                    axs[2].legend()
                    axs[2].set_ylabel("Thrust")
                    axs[2].set_xlabel("Time (s)")
                    axs[2].grid(True, alpha=0.3)
                    plt.suptitle(
                        f"{current_motion} – Verdict: {verdict}\nMax drift xy: {max_drift_x:.3f}/{max_drift_y:.3f} m"
                    )
                    plt.tight_layout()
                    plot_path = os.path.join(run_folder, "plot_position_thrust.png")
                    plt.savefig(plot_path)
                    plt.close()
                    print(f"Plot saved: {plot_path}")

            print("Flight finished.")
