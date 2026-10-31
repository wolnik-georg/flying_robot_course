#!/usr/bin/env python3
"""Host check of a firmware build for Step A: FBL (inner 1) at hover and after a 5 cm step, mixer output per motor.
  CFFIRMWARE_DIR=<build dir with cffirmware.py> python3 step_a_platform_check.py <mass_kg> [preset] [cd]"""
import os, sys
import numpy as np
mass = float(sys.argv[1]); preset = int(sys.argv[2]) if len(sys.argv) > 2 else 0; cd = float(sys.argv[3]) if len(sys.argv) > 3 else 0.236
sys.path.insert(0, os.environ["CFFIRMWARE_DIR"]); sys.path.insert(0, os.path.expanduser("~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host"))
import cffirmware as fw
print("THRUST_MAX per motor %.3f N, thrust2torque %.5f" % (fw.oot_thrust_max(), fw.oot_thrust2torque()) if hasattr(fw, "oot_thrust_max") else "no oot_thrust_max")
omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = mass; omar.indi = 0
omar.fbl.en = 1; omar.fbl.inner = 1; omar.fbl.preset = preset; omar.fbl.c_d = cd; omar.fbl.nn_en = 0
omar.fbl.tilt_max_deg = 180.0; omar.fbl.z_on = -10.0; omar.fbl.z_off = -20.0
fw.oot_set_peer_count(1)
sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
st.attitudeQuaternion.w = 1.0
def step(i, sx):
    sp.position.x, sp.position.y, sp.position.z = sx, 0.0, 0.5; st.position.x, st.position.y, st.position.z = 0.0, 0.0, 0.5
    st.attitude.roll = st.attitude.pitch = st.attitude.yaw = 0.0
    fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
    u = fw.motors_thrust_uncapped_t(); fw.powerDistribution(ctl, u); pwm = fw.motors_thrust_pwm_t(); fw.powerDistributionCap(u, pwm)
    return np.array([pwm.motors.m1, pwm.motors.m2, pwm.motors.m3, pwm.motors.m4], float) / 65535.0, ctl
for i in range(300): m, c = step(i, 0.0)
thr_max = fw.oot_thrust_max() if hasattr(fw, "oot_thrust_max") else float("nan")
F = m * thr_max
print("hover: motors (fraction of THRUST_MAX) %s -> force per motor %s N, total %.4f N, weight %.4f N" % (np.round(m, 4), np.round(F, 4), F.sum(), mass * 9.81))
for i in range(300, 305): m2, c = step(i, 0.05)
d = (m2 - m) * thr_max
print("after a +5 cm x setpoint step: force change per motor [N] %s (differential means a roll/pitch command)" % np.round(d, 5))
