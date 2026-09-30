#!/usr/bin/env python3
"""Compute Task 4/5 SIL time-series data for docs/51's plots (needs cffirmware -> system
python3.10, not the pyenv flying_robots env matplotlib needs -- see render_z_only_integral_plots.py
for stage 2). Writes z_integral_plot_data.json for the render script to consume.

⚠️ The g_ki_z/g_ki_z_limit SWIG cvar binding was found broken in the live cffirmware.i
(unbalanced %{ %} blocks -- 2 opens vs 4 closes) during this pass on 2026-09-30 and worked
around here by writing the C symbol directly via ctypes in earlier debugging; this script uses
the normal cvar path and was re-verified working after a from-scratch bindings rebuild. If
`cvar has g_ki_z: True` stops printing True, the binding has regressed again -- see
LOCAL_MODIFICATIONS.md.

Run: /usr/bin/python3.10 z_only_integral_plot_data.py
Then: ~/.pyenv/versions/flying_robots/bin/python render_z_only_integral_plots.py
"""
import sys, json
from unittest.mock import MagicMock
for name in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(name, MagicMock())
sys.path.insert(0, "/home/georg/Desktop/crazyflie-firmware/build")
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")
import numpy as np
import cffirmware as firm
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim import sim_data_types
from crazyflie_sim.backend.np import Quadrotor

def run_hover(ki_z, ki_z_limit, f_ext_z, duration, height=1.0, bias_from_t0=False):
    c = firm.cvar
    for k, v in dict(kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=60.0, mass=0.041,
                      kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10, ff_free=0,
                      filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11, tau_xy_max=0.045,
                      tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8, notch_en=0,
                      notch_f0=6.9, notch_bw=3.0).items():
        setattr(c, "g_indi_" + k, v)
    c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 8.0, 7.0
    c.g_ki_z, c.g_ki_z_limit = ki_z, ki_z_limit
    c.g_controller_mode = 0
    CrazyflieSIL._oot_count = 0
    J = [firm.oot_inertia(i) for i in range(3)]
    PH = dict(mass=0.041, kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
              arm_length=firm.oot_arm_length(), t2t=firm.oot_thrust2torque(),
              inertia=J, motor_tau=0.044)
    p0 = np.array([0., 0., 0.])
    t = [0.0]
    cf = CrazyflieSIL("cf", p0, "oot", lambda: t[0])
    q = Quadrotor(sim_data_types.State(pos=p0.copy()), PH)
    cf.takeoff(height, 3.0)
    dt = 1e-3
    ts, errs = [], []
    for k in range(1, int(duration / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        if bias_from_t0:
            fa = np.array([0.0, 0.0, f_ext_z]) if f_ext_z != 0.0 else np.zeros(3)
        else:
            fa = np.array([0.0, 0.0, f_ext_z]) if (f_ext_z != 0.0 and t[0] > 5.0) else np.zeros(3)
        q.step(act, dt, fa)
        if k % 20 == 0:
            ts.append(t[0]); errs.append((q.state.pos[2] - height) * 1000)
    return ts, errs

c = firm.cvar
print("cvar has g_ki_z:", hasattr(c, "g_ki_z"))

out = {"task4": {}, "task5": {}}
for kz in (8, 16, 32):
    ts, errs = run_hover(kz, 1.5, -0.200, duration=14.0, bias_from_t0=False)
    out["task4"][str(kz)] = {"t": ts, "err_mm": errs}
    print("task4 ki_z=", kz, "steady err at t=14:", errs[-1])

for kz in (0, 8, 16, 32):
    ts, errs = run_hover(float(kz), 1.5, -0.19, duration=8.0, bias_from_t0=True)
    out["task5"][str(kz)] = {"t": ts, "err_mm": errs}
    print("task5 ki_z=", kz, "err at t=8:", errs[-1])

with open("out/z_only_integral/z_integral_plot_data.json", "w") as f:
    json.dump(out, f)
print("wrote data json")
