#!/usr/bin/env python3
"""Runs one side (reference or ours) of test_naindi_hybrid_reference.py's comparison in an
isolated subprocess, so the two same-named `cffirmware` extension modules never collide in one
Python process -- same reason _naindi_case_runner.py (controller=7) does this. Prints
"thrust,tx,ty,tz" to stdout. Invoked by test_naindi_hybrid_reference.py, not meant to be run
directly."""
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


def run_reference(fw, case, gains):
    ctrl = fw.controllerLee_t()
    fw.controllerLeeInit(ctrl)
    ctrl.mass = gains["mass"]
    ctrl.J = fw.mkvec(*gains["J"])
    ctrl.Kpos_P = fw.mkvec(*gains["Kpos_P"]); ctrl.Kpos_P_limit = gains["Kpos_P_limit"]
    ctrl.Kpos_D = fw.mkvec(*gains["Kpos_D"]); ctrl.Kpos_D_limit = gains["Kpos_D_limit"]
    ctrl.Kpos_I = fw.mkvec(*gains["Kpos_I"]); ctrl.Kpos_I_limit = gains["Kpos_I_limit"]
    ctrl.KR = fw.mkvec(*gains["KR"])
    ctrl.Komega = fw.mkvec(*gains["Komega"])
    ctrl.KI = fw.mkvec(*gains["KI"])
    ctrl.indi = 3
    ctrl.use_nn = 7  # the ONE difference from _naindi_case_runner.py's run_reference: NA-INDI, not plain INDI

    fw.oot_test_set_rpm(*case["rpm"])
    fw.oot_test_set_kappa_f(*gains["kt"])
    fw.oot_test_set_pwm_ratio(*case["pwm"])

    sp = fw.setpoint_t()
    sp.position.x, sp.position.y, sp.position.z = case["sp_pos"]
    sp.velocity.x, sp.velocity.y, sp.velocity.z = case["sp_vel"]
    sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = case["sp_acc"]
    sp.attitude.yaw = math.degrees(case["yaw_d"])
    sp.mode.x = fw.modeAbs; sp.mode.y = fw.modeAbs; sp.mode.z = fw.modeAbs
    sp.mode.yaw = fw.modeAbs

    sensors = fw.sensorData_t()
    # gyroNoLpf drives omega/attitude-INDI; the NN's own input vector reads the separate,
    # regular (LPF'd) sensors.gyro -- see naindi_hybrid.rs's module doc. Case "gyro_reg" lets a
    # test vector set these to DIFFERENT values, so a wiring swap between the two would fail.
    sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = case["gyro_reg"]
    sensors.gyroNoLpf.x, sensors.gyroNoLpf.y, sensors.gyroNoLpf.z = case["gyro"]

    st = fw.state_t()
    st.position.x, st.position.y, st.position.z = case["pos"]
    st.velocity.x, st.velocity.y, st.velocity.z = case["vel"]
    qw, qx, qy, qz = quat_from_rpy(*case["rpy"])
    st.attitudeQuaternion.x = qx
    st.attitudeQuaternion.y = qy
    st.attitudeQuaternion.z = qz
    st.attitudeQuaternion.w = qw
    st.acc.x, st.acc.y, st.acc.z = case["acc"]

    control = fw.control_t()
    for tick in range(0, 40):
        if hasattr(fw, 'oot_test_set_tick'):
            fw.oot_test_set_tick(tick)
        fw.controllerLee(ctrl, control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def run_ours(fw_ours, case, gains):
    fw_ours.controllerOutOfTree3Init()
    fw_ours.oot_set_rpm(*case["rpm"])
    fw_ours.oot_set_pwm_ratio(*case["pwm"])
    fw_ours.cvar.g_indi_mass = gains["mass"]
    fw_ours.cvar.g_indi_kt1, fw_ours.cvar.g_indi_kt2, \
        fw_ours.cvar.g_indi_kt3, fw_ours.cvar.g_indi_kt4 = gains["kt"]
    fw_ours.naindi_hybrid_test_set_j(*gains["J"])
    fw_ours.naindi_hybrid_test_set_arm(gains["arm"], gains["t2t"])

    sp = fw_ours.setpoint_t()
    sp.position.x, sp.position.y, sp.position.z = case["sp_pos"]
    sp.velocity.x, sp.velocity.y, sp.velocity.z = case["sp_vel"]
    sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = case["sp_acc"]
    sp.attitude.yaw = math.degrees(case["yaw_d"])

    sensors = fw_ours.sensorData_t()
    sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = case["gyro_reg"]
    sensors.gyroNoLpf.x, sensors.gyroNoLpf.y, sensors.gyroNoLpf.z = case["gyro"]

    st = fw_ours.state_t()
    st.position.x, st.position.y, st.position.z = case["pos"]
    st.velocity.x, st.velocity.y, st.velocity.z = case["vel"]
    qw, qx, qy, qz = quat_from_rpy(*case["rpy"])
    st.attitudeQuaternion.x = qx
    st.attitudeQuaternion.y = qy
    st.attitudeQuaternion.z = qz
    st.attitudeQuaternion.w = qw
    st.acc.x, st.acc.y, st.acc.z = case["acc"]

    control = fw_ours.control_t()
    for tick in range(0, 40):
        fw_ours.controllerOutOfTree3(control, sp, sensors, st, tick)
    return control.thrustSi, (control.torqueX, control.torqueY, control.torqueZ)


def main():
    side, so_dir, case_json, gains_json = sys.argv[1:5]
    sys.path.insert(0, so_dir)
    import cffirmware as fw

    case = json.loads(case_json)
    gains = json.loads(gains_json)
    if side == "reference":
        thrust, tau = run_reference(fw, case, gains)
    else:
        thrust, tau = run_ours(fw, case, gains)
    print(json.dumps({"thrust": thrust, "tau": list(tau)}))


if __name__ == "__main__":
    main()
