#!/usr/bin/env python3
"""ki_z trade-off: liftoff overshoot (cost) vs hover under model-mismatch force (benefit).

Fixed hardware gains (cf5 crazyflies.yaml), kv_z=7 / kp_z=48 not swept.
Builds on kv_z_liftoff_damping_sweep.py (cffirmware z-integral ON).

Run: /usr/bin/python3.10 experiments/analysis/ki_z_tradeoff_sweep.py
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
KI_GRID = (0, 2, 4, 6, 8, 10, 12, 14, 16, 20)
# ~4% hover-thrust mismatch (A3/A8-scale), calibrated in ki_z_constant_bias_sil.py @ 1.0 m
F_EXT_Z = -0.045
DIST_ONSET_S = 5.0
HEIGHT = 1.0


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


def _apply_gains(ki_z: float) -> None:
    c = firm.cvar
    _setup_indi(c)
    c.g_kp_xy = HW_GAINS["kp_xy"]
    c.g_kp_z = HW_GAINS["kp_z"]
    c.g_kv_xy = HW_GAINS["kv_xy"]
    c.g_kv_z = HW_GAINS["kv_z"]
    c.g_ki_z = float(ki_z)
    c.g_ki_z_limit = HW_GAINS["ki_z_limit"]


def _sim(duration: float, f_ext_z: float, dist_onset: float | None) -> tuple[np.ndarray, np.ndarray]:
    c = firm.cvar
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
    cf.takeoff(HEIGHT, 3.0)
    dt = 1e-3
    ts, zs = [], []
    for k in range(1, int(duration / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        if f_ext_z != 0.0:
            if dist_onset is None:
                fa = np.array([0.0, 0.0, f_ext_z])
            elif t[0] >= dist_onset:
                fa = np.array([0.0, 0.0, f_ext_z])
            else:
                fa = np.zeros(3)
        else:
            fa = np.zeros(3)
        q.step(act, dt, fa)
        if k % 20 == 0:
            ts.append(t[0])
            zs.append(q.state.pos[2])
    return np.array(ts), np.array(zs)


def liftoff_metrics(ki_z: float, duration: float = 8.0) -> dict:
    _apply_gains(ki_z)
    t, z = _sim(duration, 0.0, None)
    e = z - HEIGHT
    pk_i = int(np.argmax(e))
    pk_mm = float(e[pk_i] * 1000)
    pk_t = float(t[pk_i])
    post = e[pk_i:]
    zc = int(np.sum(post[:-1] * post[1:] < 0))
    settle_t = None
    for i in range(pk_i, len(e)):
        if t[i] < pk_t:
            continue
        if abs(e[i]) <= 0.01:
            if np.all(np.abs(e[i : min(i + 15, len(e))]) <= 0.01):
                settle_t = float(t[i])
                break
    return dict(peak_mm=pk_mm, peak_t=pk_t, settle_s=settle_t, zc=zc, oscillatory=zc >= 1)


def disturbance_metrics(ki_z: float, duration: float = 14.0) -> dict:
    _apply_gains(ki_z)
    t, z = _sim(duration, F_EXT_Z, DIST_ONSET_S)
    e = z - HEIGHT
    mask_ss = t >= duration - 2.0
    e_ss = float(np.mean(e[mask_ss]) * 1000)
    # post-step segment (first sample at or after onset)
    i0 = int(np.searchsorted(t, DIST_ONSET_S))
    e0 = float(e[i0])
    band = 0.1 * max(abs(e0 - np.mean(e[mask_ss])), 1e-6)
    target = np.mean(e[mask_ss])
    t_conv = None
    for i in range(i0, len(t)):
        if abs(e[i] - target) <= band:
            t_conv = float(t[i] - DIST_ONSET_S)
            break
    if t_conv is None:
        t_conv = float("nan")
    return dict(steady_mm=e_ss, t_conv_s=t_conv, e0_mm=e0 * 1000)


def main() -> None:
    hover_n = 0.041 * 9.81
    print(f"Disturbance: F_ext_z={F_EXT_Z} N step at t={DIST_ONSET_S}s "
          f"(~{100*abs(F_EXT_Z)/hover_n:.0f}% of hover thrust, A3/A8-scale mismatch)")
    print(f"ki_z grid: {KI_GRID}\n")
    print(
        f"{'ki_z':>5} | {'lo peak mm':>10} | {'lo settle s':>11} | {'lo osc':>6} | "
        f"{'dist ss mm':>10} | {'t_conv s':>8}"
    )
    rows = []
    for kz in KI_GRID:
        lo = liftoff_metrics(kz)
        dist = disturbance_metrics(kz)
        rows.append((kz, lo, dist))
        st = lo["settle_s"]
        st_s = f"{st:.2f}" if st is not None else "  —"
        tc = dist["t_conv_s"]
        tc_s = f"{tc:.2f}" if tc == tc else "  —"
        print(
            f"{kz:5.0f} | {lo['peak_mm']:10.2f} | {st_s:>11} | {str(lo['oscillatory']):>6} | "
            f"{dist['steady_mm']:+10.2f} | {tc_s:>8}"
        )


if __name__ == "__main__":
    main()
