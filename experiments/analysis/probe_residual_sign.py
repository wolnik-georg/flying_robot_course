#!/usr/bin/env python3
"""Does position INDI cancel a known external force, or reinforce it?

A controlled substitute for downwash: hover one vehicle, inject a constant downward
force, and measure the steady-state height error under each control law.

    geometric  -- no residual feedback at all, so its sag is the uncompensated baseline
    pos INDI   -- exists to cancel exactly this, so its sag should be SMALLER
    full INDI  -- same outer loop, so the same expectation

If the sag instead comes out at roughly twice the geometric value, the residual is being
applied with the wrong sign: added to the disturbance rather than subtracted from it.

`indi.a_res_*` = a_meas - a_model = f_res/m is the thesis's core measurement, so its sign
is printed alongside. With a downward force the vehicle's accelerometer reads less upward
specific force than the rotors alone would produce, so a_res_z must come out NEGATIVE.
"""
import sys

import numpy as np
import cffirmware as firm

sys.path.insert(0, '/home/georg/Desktop/crazyswarm2')
from crazyflie_sim.crazyflie_sil import CrazyflieSIL          # noqa: E402
from crazyflie_sim import sim_data_types                       # noqa: E402
from crazyflie_sim.backend.np import Quadrotor                 # noqa: E402

c = firm.cvar
GAINS = dict(kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=60.0, mass=0.041,
             kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10, ff_free=0,
             filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11, tau_xy_max=0.045,
             tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8, notch_en=0,
             notch_f0=6.9, notch_bw=3.0)


def run(mode, f_ext_z, dt=1e-3, T=14.0, settle=7.0):
    for k, v in GAINS.items():
        setattr(c, 'g_indi_' + k, v)
    c.g_kp_xy, c.g_kp_z, c.g_kv_xy, c.g_kv_z = 64.0, 48.0, 8.0, 7.0
    c.g_controller_mode = mode
    c.g_indi_omega_src = 0
    CrazyflieSIL._oot_count = 0
    J = [firm.oot_inertia(i) for i in range(3)]
    PH = dict(mass=0.041, kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
              arm_length=firm.oot_arm_length(), t2t=firm.oot_thrust2torque(),
              inertia=J, motor_tau=0.044)
    p0 = np.array([0., 0., 0.])
    t = [0.0]
    cf = CrazyflieSIL('cf', p0, 'oot', lambda: t[0])
    q = Quadrotor(sim_data_types.State(pos=p0.copy()), PH)
    cf.takeoff(1.0, 3.0)
    zs, ares = [], []
    for k in range(1, int(T / dt) + 1):
        t[0] = k * dt
        cf.setState(q.state)
        cf.getSetpoint()
        act = cf.executeController()
        # Inject only after the vehicle has settled, so we measure the response to the
        # force and not the takeoff transient.
        fa = np.array([0.0, 0.0, f_ext_z]) if t[0] > settle else np.zeros(3)
        q.step(act, dt, fa)
        if t[0] > T - 3.0:                      # last 3 s = steady state
            zs.append(q.state.pos[2])
            ares.append([firm.oot_get_a_res(i) for i in range(3)])
    return float(np.mean(zs)), np.mean(np.array(ares), axis=0)


if __name__ == '__main__':
    F = -0.020                                  # N, downward -- about 5% of weight
    print(f'  hover at 1.000 m, constant external force {F * 1000:.0f} mN downward '
          f'({abs(F) / (0.041 * 9.81) * 100:.0f}% of weight)\n')
    print(f'    {"controller":<12}{"z [m]":>9}{"sag [mm]":>11}{"a_res_z [m/s^2]":>18}   verdict')
    base = None
    for mode, lbl in ((0, 'geometric'), (1, 'pos INDI'), (3, 'full INDI')):
        z, ar = run(mode, F)
        sag = (1.0 - z) * 1000
        if mode == 0:
            base = sag
            verdict = 'baseline (no residual feedback)'
        elif abs(sag) < 0.4 * abs(base):
            verdict = 'CANCELS the force -- correct'
        elif abs(sag) > 1.6 * abs(base):
            verdict = f'REINFORCES it ({sag / base:.2f}x baseline) -- SIGN ERROR'
        else:
            verdict = f'{sag / base:.2f}x baseline -- no clear compensation'
        print(f'    {lbl:<12}{z:9.4f}{sag:11.1f}{ar[2]:18.4f}   {verdict}')
    print('\n    a_res_z should be NEGATIVE: a downward force makes the accelerometer read'
          '\n    less upward specific force than the rotors alone would explain.')
