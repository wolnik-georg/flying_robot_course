#!/usr/bin/env python3
"""Corrected scenario: constant bias present from t=0 (models a persistent thrust/mass
model error, matching real flight data -- sag present in EVERY scenario/vehicle, not just
downwash-exposed ones), not a mid-flight-injected disturbance. Measures convergence over a
REALISTIC hold duration starting from takeoff, not recovery-after-onset."""
import sys
from unittest.mock import MagicMock
for name in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(name, MagicMock())
sys.path.insert(0, "/tmp/cffirmware_zint_on/on")
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")
import numpy as np
import cffirmware as firm
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim import sim_data_types
from crazyflie_sim.backend.np import Quadrotor


def run_hover(ki_z, ki_z_limit, f_ext_z, duration, height=1.0):
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
    zs_settled, zs_full, ts_full = [], [], []
    for k in range(1, int(duration / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        # bias present from the moment takeoff begins -- t=0, not mid-flight
        fa = np.array([0.0, 0.0, f_ext_z]) if f_ext_z != 0.0 else np.zeros(3)
        q.step(act, dt, fa)
        if k % 100 == 0:
            zs_full.append(q.state.pos[2]); ts_full.append(t[0])
        if t[0] > 3.5:  # after the 3s takeoff ramp settles, within a realistic 5-10s hold
            zs_settled.append(q.state.pos[2])
    z = np.array(zs_settled)
    return dict(mean=float(np.mean(z - height)), rmse=float(np.sqrt(np.mean((z - height) ** 2))),
                trace=list(zip(ts_full, [zz - height for zz in zs_full])))


# Real-data-grounded bias magnitudes: A3/A8-scale (~3-5% sag) and A1-scale (~15-20% sag),
# found via -f_ext sweep on the OFF arm to match the observed real percentages at height=1.0m.
print("--- calibrating f_ext to match real-data sag magnitudes (OFF, ki_z=0) ---")
for f in (-0.02, -0.04, -0.06, -0.08, -0.10, -0.15, -0.20):
    m = run_hover(0.0, 1.5, f, duration=8.0)
    print(f"f_ext={f:6.2f}N  OFF steady z_err={m['mean']*100:6.2f}cm ({m['mean']*100:+.1f}%)")

print()
print("--- realistic 8s hold, bias from t=0, OFF vs default ki_z=8 vs stronger gains ---")
for label, f in (("A3/A8-scale (~4% sag)", -0.045), ("A1-scale (~17% sag)", -0.19)):
    print(f"-- {label}, f_ext={f}N --")
    for kz in (0.0, 8.0, 16.0, 32.0):
        m = run_hover(kz, 1.5, f, duration=8.0)
        print(f"   ki_z={kz:5.1f}  mean_err_over_8s_hold={m['mean']*100:7.2f}cm  rmse={m['rmse']*100:6.2f}cm")
    print()
