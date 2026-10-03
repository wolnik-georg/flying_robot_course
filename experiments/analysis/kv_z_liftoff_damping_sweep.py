#!/usr/bin/env python3
"""SIL liftoff transient: verify Z-integral reset at thrust onset, sweep g_kv_z / g_ki_z.

Uses cffirmware build with ENABLE_Z_INTEGRAL (e.g. /tmp/cffirmware_zint_on/on).
Baseline pos gains match cf5 in crazyswarm2/crazyflie/config/crazyflies.yaml:
  kp_xy=64, kp_z=48, kv_xy=5, kv_z=7, ki_z=16, ki_z_limit=1.5

Run: /usr/bin/python3.10 experiments/analysis/kv_z_liftoff_damping_sweep.py
"""
from __future__ import annotations

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

HW_GAINS = dict(kp_xy=64.0, kp_z=48.0, kv_xy=5.0, kv_z=7.0, ki_z=16.0, ki_z_limit=1.5)


def _setup_indi(c: firm.cvar) -> None:
    for k, v in dict(
        kr=2400.0,
        kw=170.0,
        kr_z=2400.0,
        kw_z=170.0,
        fc_bw=60.0,
        mass=0.041,
        kt1=4.1623e-10,
        kt2=4.0592e-10,
        kt3=4.1116e-10,
        kt4=4.0631e-10,
        ff_free=0,
        filt_order=1,
        filt_tau=1,
        j_scale=1.0,
        clamp_en=11,
        tau_xy_max=0.045,
        tau_z_max=0.0025,
        tilt_max_deg=30.0,
        thrust_max=0.8,
        notch_en=0,
        notch_f0=6.9,
        notch_bw=3.0,
    ).items():
        setattr(c, "g_indi_" + k, v)
    c.g_controller_mode = 0


def run_liftoff(
    *,
    kv_z: float,
    ki_z: float = HW_GAINS["ki_z"],
    height: float = 1.0,
    duration: float = 8.0,
) -> dict:
    c = firm.cvar
    _setup_indi(c)
    c.g_kp_xy = HW_GAINS["kp_xy"]
    c.g_kp_z = HW_GAINS["kp_z"]
    c.g_kv_xy = HW_GAINS["kv_xy"]
    c.g_kv_z = float(kv_z)
    c.g_ki_z = float(ki_z)
    c.g_ki_z_limit = HW_GAINS["ki_z_limit"]

    CrazyflieSIL._oot_count = 0
    J = [firm.oot_inertia(i) for i in range(3)]
    PH = dict(
        mass=0.041,
        kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
        arm_length=firm.oot_arm_length(),
        t2t=firm.oot_thrust2torque(),
        inertia=J,
        motor_tau=0.044,
    )
    p0 = np.array([0.0, 0.0, 0.0])
    t = [0.0]
    cf = CrazyflieSIL("cf", p0, "oot", lambda: t[0])
    q = Quadrotor(sim_data_types.State(pos=p0.copy()), PH)
    cf.takeoff(height, 3.0)
    dt = 1e-3

    ts, zs, thrusts = [], [], []
    first_cross = None
    prev_th = 0.0
    for k in range(1, int(duration / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        th = cf.control.thrustSi
        if prev_th <= 0.05 and th > 0.05 and first_cross is None:
            first_cross = (t[0], th, q.state.pos[2])
        prev_th = th
        q.step(act, dt, np.zeros(3))
        if k % 20 == 0:
            ts.append(t[0])
            zs.append(q.state.pos[2])
            thrusts.append(th)

    t_arr = np.array(ts)
    err = np.array(zs) - height
    pk_i = int(np.argmax(err))
    pk_mm = float(err[pk_i] * 1000)
    pk_t = float(t_arr[pk_i])
    post = err[pk_i:]
    zc = int(np.sum(post[:-1] * post[1:] < 0))
    ss = err[t_arr >= duration - 2.0]
    return dict(
        peak_mm=pk_mm,
        peak_t=pk_t,
        zero_cross_after_peak=zc,
        oscillatory=zc >= 1,
        steady_mm=float(np.mean(ss) * 1000),
        first_thrust_cross=first_cross,
    )


def verify_i_ez_reset_behavioral() -> None:
    """ki_z=0 vs 16 should match for ~100 ms after thrust crosses 0.05 N if i_ez is zero."""
    def trace(ki_z: float):
        c = firm.cvar
        _setup_indi(c)
        c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = (
            HW_GAINS["kp_xy"],
            HW_GAINS["kp_z"],
            HW_GAINS["kv_xy"],
            HW_GAINS["kv_z"],
        )
        c.g_ki_z, c.g_ki_z_limit = ki_z, HW_GAINS["ki_z_limit"]
        CrazyflieSIL._oot_count = 0
        J = [firm.oot_inertia(i) for i in range(3)]
        PH = dict(
            mass=0.041,
            kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
            arm_length=firm.oot_arm_length(),
            t2t=firm.oot_thrust2torque(),
            inertia=J,
            motor_tau=0.044,
        )
        p0 = np.array([0.0, 0.0, 0.0])
        t = [0.0]
        cf = CrazyflieSIL("cf", p0, "oot", lambda: t[0])
        q = Quadrotor(sim_data_types.State(pos=p0.copy()), PH)
        cf.takeoff(1.0, 3.0)
        dt = 1e-3
        rows = []
        for k in range(1, 4001):
            t[0] = k * dt
            cf.setState(q.state)
            cf.getSetpoint()
            act = cf.executeController()
            th = cf.control.thrustSi
            q.step(act, dt, np.zeros(3))
            rows.append((t[0], th, q.state.pos[2]))
        return rows

    r0, r1 = trace(0.0), trace(16.0)
    i0 = next(i for i, r in enumerate(r0) if r[1] > 0.05)
    i1 = next(i for i, r in enumerate(r1) if r[1] > 0.05)
    diffs = [abs(r0[i0 + j][2] - r1[i1 + j][2]) for j in range(100)]
    print("=== i_ez reset (behavioral) ===")
    print(f"  first thrust>0.05 N: t={r0[i0][0]:.3f}s (ki_z=0 and ki_z=16 align)")
    print(f"  max |Δz| over next 100 ms (ki_z=0 vs 16): {max(diffs)*1000:.3f} mm")
    print("  (≈0 mm ⇒ no integral force yet at liftoff — consistent with ground reset path)")


def main() -> None:
    verify_i_ez_reset_behavioral()
    print("\n=== kv_z sweep (ki_z=16, hardware yaml gains) ===")
    print(f"{'kv_z':>6} {'peak_mm':>8} {'t_peak':>6} {'zc':>4} {'osc':>5} {'ss_mm':>8}")
    for kv in (4, 5, 6, 7, 8, 9, 10, 12, 14, 18):
        m = run_liftoff(kv_z=kv)
        print(
            f"{kv:6.1f} {m['peak_mm']:8.2f} {m['peak_t']:6.2f} "
            f"{m['zero_cross_after_peak']:4d} {str(m['oscillatory']):>5} {m['steady_mm']:+8.2f}"
        )

    print("\n=== ki_z sweep (kv_z=7) ===")
    print(f"{'ki_z':>6} {'peak_mm':>8} {'t_peak':>6} {'zc':>4} {'ss_mm':>8}")
    for kz in (0, 4, 8, 12, 16, 20, 24, 32):
        m = run_liftoff(kv_z=7.0, ki_z=kz)
        print(f"{kz:6.1f} {m['peak_mm']:8.2f} {m['peak_t']:6.2f} {m['zero_cross_after_peak']:4d} {m['steady_mm']:+8.2f}")


if __name__ == "__main__":
    main()
