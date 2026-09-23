#!/usr/bin/env python3
"""Runs one side (reference or ours) of test_omar_indi_reference.py's comparison in an
isolated subprocess, so the two same-named `cffirmware` extension modules never collide in
one Python process. Prints a JSON line with thrust/torque to stdout. Invoked by
test_omar_indi_reference.py, not meant to be run directly.

'reference' loads the .so compiled straight from crazyflie-firmware-omar's own, completely
unmodified controller_lee.c (symbols controllerLee_t/controllerLeeInit/controllerLee).
'ours' loads this project's own crazyflie-firmware build, calling the literal C copy
controller_omar_indi.c (symbols controllerOmarIndi_t/controllerOmarIndiInit/controllerOmarIndi)
-- renamed identifiers only, see docs/41_Pure_INDI_Implementation_Comparison.md."""
import json
import math
import sys


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


def build_sp_sensors_state(fw, case):
    sp = fw.setpoint_t()
    sp.position.x, sp.position.y, sp.position.z = case["sp_pos"]
    sp.velocity.x, sp.velocity.y, sp.velocity.z = case["sp_vel"]
    sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = case["sp_acc"]
    sp.attitude.yaw = math.degrees(case["yaw_d"])
    sp.mode.x = fw.modeAbs; sp.mode.y = fw.modeAbs; sp.mode.z = fw.modeAbs
    sp.mode.yaw = fw.modeAbs

    sensors = fw.sensorData_t()
    sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = case["gyro"]

    st = fw.state_t()
    st.position.x, st.position.y, st.position.z = case["pos"]
    st.velocity.x, st.velocity.y, st.velocity.z = case["vel"]
    qw, qx, qy, qz = quat_from_rpy(*case["rpy"])
    st.attitudeQuaternion.x = qx
    st.attitudeQuaternion.y = qy
    st.attitudeQuaternion.z = qz
    st.attitudeQuaternion.w = qw
    return sp, sensors, st


def run_reference(fw, case):
    ctrl = fw.controllerLee_t()
    fw.controllerLeeInit(ctrl)
    ctrl.indi = 3  # both position (bit0) and attitude (bit1) INDI active
    fw.oot_test_set_rpm(*case["rpm"])

    sp, sensors, st = build_sp_sensors_state(fw, case)
    control = fw.control_t()
    for tick in range(0, 40):
        fw.controllerLee(ctrl, control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def run_ours(fw, case):
    ctrl = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(ctrl)
    ctrl.indi = 3
    fw.oot_set_rpm(*case["rpm"])

    sp, sensors, st = build_sp_sensors_state(fw, case)
    control = fw.control_t()
    for tick in range(0, 40):
        fw.controllerOmarIndi(ctrl, control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def main():
    side, so_dir, case_json = sys.argv[1], sys.argv[2], sys.argv[3]
    case = json.loads(case_json)

    sys.path.insert(0, so_dir)
    import cffirmware as fw

    if side == "reference":
        thrust, tau = run_reference(fw, case)
    else:
        thrust, tau = run_ours(fw, case)

    print(json.dumps({"thrust": thrust, "tau": list(tau)}))


if __name__ == "__main__":
    main()
