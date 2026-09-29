#!/usr/bin/env python3
"""One side of test_omar_indi_rust_vs_c.py — isolated subprocess."""
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
    sp.mode.x = fw.modeAbs
    sp.mode.y = fw.modeAbs
    sp.mode.z = fw.modeAbs
    sp.mode.yaw = fw.modeAbs
    if case.get("mode_disable_z"):
        sp.mode.x = fw.modeDisable
        sp.mode.y = fw.modeDisable
        sp.mode.z = fw.modeDisable
        sp.thrust = case.get("thrust_cmd", 0)

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


def run_c(fw, case):
    ctrl = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(ctrl)
    ctrl.indi = 3
    fw.oot_set_rpm(*case["rpm"])
    sp, sensors, st = build_sp_sensors_state(fw, case)
    control = fw.control_t()
    for tick in range(0, 40):
        fw.controllerOmarIndi(ctrl, control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def run_rust(fw, case):
    fw.controllerOutOfTree5Init()
    fw.omar_indi_rust_set_indi(3)
    fw.oot_set_rpm(*case["rpm"])
    sp, sensors, st = build_sp_sensors_state(fw, case)
    control = fw.control_t()
    for tick in range(0, 40):
        fw.controllerOutOfTree5(control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def main():
    side, so_dir, case_json = sys.argv[1], sys.argv[2], sys.argv[3]
    case = json.loads(case_json)
    sys.path.insert(0, so_dir)
    import cffirmware as fw

    if side == "c":
        thrust, tau = run_c(fw, case)
    else:
        thrust, tau = run_rust(fw, case)
    print(json.dumps({"thrust": thrust, "tau": list(tau)}))


if __name__ == "__main__":
    main()
