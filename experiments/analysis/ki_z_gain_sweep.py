#!/usr/bin/env python3
"""Fast ki_z/ki_z_limit sweep against the already-verified ON binary. No rebuild needed --
these are runtime params (cvar), same mechanism the SIL harness itself uses.

⚠️ The "mean error over t>8s" table this script prints is MISLEADING on its own -- see
ki_z_gain_sweep_transient_trace.py and docs/51 Task 4. Higher ki_z looks like it reduces
error in this table, but the transient trace shows the PEAK disturbance dip is unchanged
by gain (~127mm at -200mN regardless of ki_z); only RECOVERY SPEED changes, and even the
fastest tested gain (ki_z=32) takes ~9s to recover -- longer than a real A1 hold. Read
both scripts' output together, not this one alone.

Requires the pre-built ON binary at /tmp/cffirmware_zint_on/on/ (built by
position_integral_z_only_sil_compare.py --full-suite; SHA256 60d0a571... as of
2026-09-29). Rebuild via that script first if this path is stale/missing."""
import sys
from unittest.mock import MagicMock
for name in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(name, MagicMock())

SO_DIR = "/tmp/cffirmware_zint_on/on"
CS2_SIM = "/home/georg/Desktop/crazyswarm2/crazyflie_sim"
sys.path.insert(0, SO_DIR)
sys.path.insert(0, CS2_SIM)

import numpy as np
import rowan
import cffirmware as firm
from crazyflie_sim.crazyflie_sil import CrazyflieSIL
from crazyflie_sim import sim_data_types
from crazyflie_sim.backend.np import Quadrotor


def run_hover(ki_z, ki_z_limit, f_ext_z, duration=14.0, height=1.0):
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
    zs, roll_all, pitch_all = [], [], []
    for k in range(1, int(duration / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        fa = np.array([0.0, 0.0, f_ext_z]) if (f_ext_z != 0.0 and t[0] > 5.0) else np.zeros(3)
        q.step(act, dt, fa)
        r, p, _ = rowan.to_euler(q.state.quat, convention="xyz")
        roll_all.append(abs(np.degrees(r)))
        pitch_all.append(abs(np.degrees(p)))
        if t[0] > 8.0:
            zs.append(q.state.pos[2])
    z = np.array(zs)
    return dict(
        z_err_mean=float(np.mean(z - height)),
        z_err_rmse=float(np.sqrt(np.mean((z - height) ** 2))),
        roll_max=float(np.max(roll_all)),
        pitch_max=float(np.max(pitch_all)),
    )


if __name__ == "__main__":
    FORCES = [0.0, -0.008, -0.040, -0.120, -0.200]
    CONFIGS = [
        ("default (ki_z=8, limit=1.5)", 8.0, 1.5),
        ("2x gain (ki_z=16, limit=1.5)", 16.0, 1.5),
        ("4x gain (ki_z=32, limit=1.5)", 32.0, 1.5),
        ("2x limit (ki_z=8, limit=3.0)", 8.0, 3.0),
        ("2x both (ki_z=16, limit=3.0)", 16.0, 3.0),
        ("4x both (ki_z=32, limit=6.0)", 32.0, 6.0),
        ("8x gain only (ki_z=64, limit=1.5)", 64.0, 1.5),
    ]
    print(f"{'config':<32} {'f_ext(N)':>9} {'z_err(mm)':>10} {'roll_max':>9} {'pitch_max':>10}")
    for name, kz, kzl in CONFIGS:
        for f in FORCES:
            m = run_hover(kz, kzl, f)
            print(f"{name:<32} {f:>9.3f} {m['z_err_mean']*1000:>10.2f} {m['roll_max']:>9.3f} {m['pitch_max']:>10.3f}")
        print()
