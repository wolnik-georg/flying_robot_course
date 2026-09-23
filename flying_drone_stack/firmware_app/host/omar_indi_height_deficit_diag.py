#!/usr/bin/env python3
"""Diagnostic for the ~15-20% steady-state height deficit found in the CS2 SIL runs
(omar_indi_reference_build_notes.md, 2026-09-22): controller_omar_indi.c (stabilizer.controller=9)
plateaus around z=0.82m on a z=1.0m hover command. Standalone closed-loop harness (mirrors
naindi_reference_closed_loop.py's pattern exactly) so each hypothesis can be tested in ~1s
instead of an ~8s ROS2 launch cycle.

RESULT (2026-09-22): every config below reaches z=1.0000 exactly -- the Kpos_I=0 hypothesis
this script was written to test is WRONG, disproven by this exact tool. That "disagrees with
the real SIL" result was the actual signal: this harness correctly passes `kt`/`arm`/`t2t` to
the plant (see the params={...} call below), while `crazyflie_sil.py`'s `'oot4'` branch at the
time did not (`self.kt`/`self.thrust_max` were never set for it -- an omission, fixed same day).
The real root cause and fix are in `omar_indi_reference_build_notes.md`'s "Update 2026-09-23"
section. Kept as a general-purpose gain/indi-mode diagnostic tool, not because the height
deficit is still open -- it isn't.

Airframe pinned to exactly what the real SIL run used (docs/41 §8):
  mass=CF_MASS=0.0393, kt_equiv=MOTORRPM2FORCE-converted=4.2899e-10 (x4, one scalar for
  all motors -- his own convention), arm=0.050, t2t=0.005692788399755955.

Usage:
    python3 omar_indi_height_deficit_diag.py --indi 3 --kpos-i 0.0    # baseline, reproduce
    python3 omar_indi_height_deficit_diag.py --indi 3 --kpos-i 2.0    # does integral close it?
    python3 omar_indi_height_deficit_diag.py --indi 0 --kpos-i 0.0    # pure geometric, no INDI
    python3 omar_indi_height_deficit_diag.py --indi 1 --kpos-i 0.0    # position INDI only
    python3 omar_indi_height_deficit_diag.py --indi 2 --kpos-i 0.0    # attitude INDI only
"""
import argparse
import sys

import numpy as np
import rowan

MASS = 0.03929999843239784
KT_EQUIV = 4.2899225838333166e-10
KT = (KT_EQUIV, KT_EQUIV, KT_EQUIV, KT_EQUIV)
ARM_LENGTH = 0.05000000074505806
T2T = 0.005692788399755955
J = (16.571710e-6, 16.655602e-6, 29.261652e-6)  # his own struct default, unchanged

TARGET_Z = 1.0
CLIMB_T = 2.0
HOVER_T = 15.0  # longer than the SIL's 8s so a slow integral-driven convergence would show up
DT = 0.002  # 500 Hz


def quintic(tau):
    s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3
    ds = 30 * tau**4 - 60 * tau**3 + 30 * tau**2
    return s, ds


def setpoint_z(t):
    if t < CLIMB_T:
        tau = t / CLIMB_T
        s, ds = quintic(tau)
        return TARGET_Z * s, TARGET_Z * ds / CLIMB_T
    else:
        return TARGET_Z, 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--indi", type=int, default=3, choices=[0, 1, 2, 3])
    ap.add_argument("--kpos-i", type=float, default=0.0)
    ap.add_argument("--kpos-p", type=float, default=7.0)
    ap.add_argument("--kpos-d", type=float, default=4.0)
    ap.add_argument("--so-dir", default="/home/georg/Desktop/crazyflie-firmware/build")
    ap.add_argument("--duration", type=float, default=CLIMB_T + HOVER_T)
    ap.add_argument("--real-substeps", action="store_true",
                    help="replicate crazyflie_sil.py's actual discretization: physics at "
                         "2 kHz, controller recomputes every 4th substep (500 Hz), holding "
                         "the same Action across the other 3 -- instead of this script's "
                         "default direct 1:1 500 Hz stepping.")
    args = ap.parse_args()

    sys.path.insert(0, args.so_dir)
    import cffirmware as fw  # noqa: E402
    sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")
    from crazyflie_sim.backend.np import Quadrotor  # noqa: E402
    from crazyflie_sim.sim_data_types import Action, State  # noqa: E402

    ctrl = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(ctrl)
    ctrl.indi = args.indi
    ctrl.Kpos_I = fw.mkvec(args.kpos_i, args.kpos_i, args.kpos_i)
    ctrl.Kpos_P = fw.mkvec(args.kpos_p, args.kpos_p, args.kpos_p)
    ctrl.Kpos_D = fw.mkvec(args.kpos_d, args.kpos_d, args.kpos_d)

    plant = Quadrotor(State(pos=np.array([0.0, 0.0, 0.0])), params={
        'mass': MASS, 'inertia': list(J), 'kt': list(KT),
        'arm_length': ARM_LENGTH, 't2t': T2T, 'motor_tau': 0.044,
    })
    B0_inv = np.linalg.inv(plant.B0)

    n_ticks = int(args.duration / DT)
    rpm_meas = np.zeros(4)
    zs = []
    for loop_i in range(n_ticks):
        tick = 2 * loop_i
        t = loop_i * DT
        z_sp, vz_sp = setpoint_z(t)

        sp = fw.setpoint_t()
        sp.position.x, sp.position.y, sp.position.z = 0.0, 0.0, z_sp
        sp.velocity.x, sp.velocity.y, sp.velocity.z = 0.0, 0.0, vz_sp
        sp.attitude.yaw = 0.0
        sp.mode.x = fw.modeAbs; sp.mode.y = fw.modeAbs; sp.mode.z = fw.modeAbs
        sp.mode.yaw = fw.modeAbs

        gyro_deg = np.degrees(plant.state.omega)
        sensors = fw.sensorData_t()
        sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = gyro_deg

        st = fw.state_t()
        st.position.x, st.position.y, st.position.z = plant.state.pos
        st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
        qw, qx, qy, qz = plant.state.quat
        st.attitudeQuaternion.w, st.attitudeQuaternion.x = qw, qx
        st.attitudeQuaternion.y, st.attitudeQuaternion.z = qy, qz
        acc_world_g = rowan.rotate(plant.state.quat, plant.state.acc)
        st.acc.x, st.acc.y, st.acc.z = acc_world_g[0], acc_world_g[1], acc_world_g[2] - 1.0

        fw.oot_set_rpm(int(rpm_meas[0]), int(rpm_meas[1]), int(rpm_meas[2]), int(rpm_meas[3]))
        control = fw.control_t()
        fw.controllerOmarIndi(ctrl, control, sp, sensors, st, tick)

        rhs = np.array([control.thrustSi, control.torqueX, control.torqueY, control.torqueZ])
        force = np.maximum(B0_inv @ rhs, 0.0)
        with np.errstate(invalid="ignore"):
            rpm_cmd = np.sqrt(force / np.array(KT))
        if not np.all(np.isfinite(rpm_cmd)):
            print(f"DIVERGED at t={t:.3f}s"); break

        if args.real_substeps:
            for _ in range(4):
                plant.step(Action(rpm_cmd), DT / 4)
        else:
            plant.step(Action(rpm_cmd), DT)
        rpm_meas = plant.state.rpm
        zs.append(plant.state.pos[2])

        if not np.isfinite(plant.state.pos[2]) or plant.state.pos[2] < -0.1:
            print(f"CRASHED at t={t:.3f}s z={plant.state.pos[2]:.3f}"); break

    zs = np.array(zs)
    tail = zs[-int(2.0 / DT):]  # last 2s -- steady-state value
    print(f"indi={args.indi} Kpos_I={args.kpos_i} Kpos_P={args.kpos_p} Kpos_D={args.kpos_d}: "
          f"final z={zs[-1]:.4f}  steady-state mean(last 2s)={tail.mean():.4f}  "
          f"max z reached={zs.max():.4f}  (target {TARGET_Z:.2f})")


if __name__ == "__main__":
    main()
