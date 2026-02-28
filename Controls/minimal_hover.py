import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper


URI = uri_helper.uri_from_env(default="radio://0/80/2M/E7E7E7E7E7")

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.3

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print("Deck is attached!")
    else:
        print("Deck is NOT attached!")


def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data["stateEstimate.x"]
    position_estimate[1] = data["stateEstimate.y"]


# ────────────────────────────────────────────────
# New function: approximate circle using small steps
# ────────────────────────────────────────────────


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()


def move_circle(scf):
    """
    Fly an approximate circle using many small forward + turn steps.
    Radius ≈ 0.3 m, full circle in ~12 seconds.
    """
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Starting approximate circle ...")

        # Parameters for the circle approximation
        num_steps = 36  # 36 steps = 10° per turn → smooth enough
        segment_length = 0.05  # small forward distance per step
        turn_angle = 360.0 / num_steps  # ≈10°
        segment_time = 0.0  # time per segment (adjust for speed)

        time.sleep(3.0)  # small settle before moving

        for i in range(num_steps):
            print(f"  Circle step {i+1}/{num_steps}")
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_left(turn_angle)

        print("Circle complete → returning to center")
        time.sleep(1.0)


# ────────────────────────────────────────────────
# New function: approximate figure-8 (two connected circles)
# ────────────────────────────────────────────────
def move_linear_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(1)
        mc.forward(0.3)
        mc.turn_left(90)
        mc.forward(0.3)
        mc.turn_left(90)
        mc.forward(0.3)
        mc.turn_left(90)


def move_box_limit(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        body_x_cmd = 0.2
        body_y_cmd = 0.1
        max_vel = 0.2

        while 1:
            # if position_estimate[0] > BOX_LIMIT:
            #    mc.start_back()
            # elif position_estimate[0] < -BOX_LIMIT:
            #    mc.start_forward()

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


def move_figure8(scf):
    """
    Approximate figure-8: left circle → right circle.
    Uses the same small-step technique.
    """
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        print("Starting approximate figure-8 ...")

        num_steps = 36
        segment_length = 0.04  # smaller segments for tighter loops
        turn_angle = 360.0 / num_steps
        segment_time = 0.30

        time.sleep(3.0)

        # First loop (counter-clockwise)
        print("  First loop (left)")
        for i in range(num_steps):
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_left(turn_angle)
            time.sleep(0.08)

        # Transition + second loop (clockwise)
        print("  Transition + second loop (right)")
        mc.forward(0.15)  # move to center of second circle
        time.sleep(0.8)

        for i in range(num_steps):
            mc.forward(segment_length)
            time.sleep(segment_time)
            mc.turn_right(turn_angle)  # opposite direction
            time.sleep(0.08)

        print("Figure-8 complete → returning")
        time.sleep(1.0)


# ────────────────────────────────────────────────
# Main — choose which motion to run
# ────────────────────────────────────────────────

if __name__ == "__main__":
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:

        scf.cf.param.add_update_callback(
            group="deck", name="bcFlow2", cb=param_deck_flow
        )
        time.sleep(1)

        logconf = LogConfig(name="Position", period_in_ms=10)
        logconf.add_variable("stateEstimate.x", "float")
        logconf.add_variable("stateEstimate.y", "float")
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        if not deck_attached_event.wait(timeout=5):
            print("No flow deck detected!")
            sys.exit(1)

        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        logconf.start()

        # ── Choose ONE of these ──
        # take_off_simple(scf)  # simple takeoff + hover
        # move_linear_simple(scf)  # your current linear + turns
        # move_box_limit(scf)  # bouncing box
        # move_circle(scf)  # new: circle
        move_figure8(scf)  # new: figure-8

        logconf.stop()
