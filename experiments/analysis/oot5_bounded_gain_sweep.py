#!/usr/bin/env python3
"""Bounded ±~30% one-knob gain sweeps for controller=10 desk prep (§20).

Height bias: **indi=3** (flight) with **IMU z specific-force bias** calibrated to +162 mm —
plant kt/mass mismatch alone is **nulled by INDI** in this loop (see §20). This bias splits
RPM-model vs IMU (thrust/estimation mismatch class), not constant world force.

Also reports **indi=0** + calibrated f_ext (+162 mm) for **Kpos_I** (position-branch proxy).

Figure8: **`figure8_mode1_kt0.05.csv`** — matches **2026-09-30 hardware** (controller=10
figure8 that produced §18 **7.5° / 24.5°** roll, and same-night c=6 figure8s). **Not**
`kt0.008` (slower profile; was wrongly used in the first §20 pass).

Run: /usr/bin/python3.10 oot5_bounded_gain_sweep.py
Run figure8/attitude only: /usr/bin/python3.10 oot5_bounded_gain_sweep.py --figure8-only
"""
from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np
import rowan

SO = "/home/georg/Desktop/crazyflie-firmware/build"
CS2_SIM = "/home/georg/Desktop/crazyswarm2/crazyflie_sim"
TRAJ_CSV = (
    "/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/data/"
    "figure8_mode1_kt0.05.csv"
)
sys.path.insert(0, SO)
sys.path.insert(0, CS2_SIM)

import cffirmware as fw  # noqa: E402
from crazyflie_sim.backend.np import Quadrotor  # noqa: E402
from crazyflie_sim.sim_data_types import Action, State  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "uav_traj",
    "/home/georg/Desktop/crazyswarm2/crazyflie_py/crazyflie_py/uav_trajectory.py",
)
_uav = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_uav)

MASS = float(fw.oot_omar_mass())
KT = float(fw.oot_omar_kt_equiv())
ARM = float(fw.oot_arm_length())
T2T = float(fw.oot_thrust2torque())
J = [fw.oot_inertia(i) for i in range(3)]
DT = 0.002
TARGET = 1.0
CLIMB = 2.0
HW_BIAS_MM = 162.0
HW_ROLL_STD = 7.5
HW_ROLL_PEAK = 24.5
MIN_B1 = 0.5
MIN_C3 = 0.05


def _suppress():
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_out, old_err = os.dup(1), os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    return devnull, old_out, old_err


def _restore(devnull, old_out, old_err):
    os.dup2(old_out, 1)
    os.dup2(old_err, 2)
    os.close(devnull)


def _init_ctrl(
    kpos_p_z=7.0,
    kpos_d_z=4.0,
    kpos_i_z=0.0,
    kr_scale=1.0,
    kom_scale=1.0,
    ki_scale=1.0,
    indi=3,
):
    ctrl = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(ctrl)
    ctrl.indi = indi
    ctrl.Kpos_P = fw.mkvec(7.0, 7.0, kpos_p_z)
    ctrl.Kpos_D = fw.mkvec(4.0, 4.0, kpos_d_z)
    ctrl.Kpos_I = fw.mkvec(0.0, 0.0, kpos_i_z)
    ctrl.KR = fw.mkvec(0.007 * kr_scale, 0.007 * kr_scale, 0.008 * kr_scale)
    ctrl.Komega = fw.mkvec(0.00115 * kom_scale, 0.00115 * kom_scale, 0.002 * kom_scale)
    ctrl.KI = fw.mkvec(0.03 * ki_scale, 0.03 * ki_scale, 0.03 * ki_scale)
    return ctrl


def _plant():
    return Quadrotor(
        State(pos=np.zeros(3)),
        dict(mass=MASS, inertia=J, kt=[KT] * 4, arm_length=ARM, t2t=T2T, motor_tau=0.044),
    )


def calibrate_acc_bias_g() -> float:
    """More negative st.acc.z [g] -> flies higher (+z error)."""
    lo, hi = -0.20, -0.005
    for _ in range(32):
        mid = (lo + hi) / 2.0
        err = run_hover(indi=3, acc_z_bias_g=mid)["err_mm"]
        if err > HW_BIAS_MM:
            lo = mid  # less negative bias
        else:
            hi = mid  # more negative bias
    return (lo + hi) / 2.0


def calibrate_f_ext_indi0() -> float:
    lo, hi = 0.0, 0.12
    for _ in range(32):
        mid = (lo + hi) / 2.0
        err = run_hover(indi=0, f_ext_z=mid)["err_mm"]
        if err < HW_BIAS_MM:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def run_hover(
    indi=3,
    acc_z_bias_g=0.0,
    f_ext_z=0.0,
    kpos_p_z=7.0,
    kpos_d_z=4.0,
    kpos_i_z=0.0,
    kr_scale=1.0,
    kom_scale=1.0,
    ki_scale=1.0,
    duration=18.0,
):
    devnull, o1, o2 = _suppress()
    ctrl = _init_ctrl(kpos_p_z, kpos_d_z, kpos_i_z, kr_scale, kom_scale, ki_scale, indi)
    plant = _plant()
    B0_inv = np.linalg.inv(plant.B0)
    rpm = np.zeros(4)
    zs = []
    try:
        for i in range(int(duration / DT)):
            t = i * DT
            tau = min(t / CLIMB, 1.0)
            s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3 if t < CLIMB else 1.0
            sp = fw.setpoint_t()
            sp.position.z = TARGET * s
            sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
            sp.mode.yaw = fw.modeAbs

            st = fw.state_t()
            st.position.x, st.position.y, st.position.z = plant.state.pos
            st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
            qw, qx, qy, qz = plant.state.quat
            st.attitudeQuaternion.w, st.attitudeQuaternion.x = qw, qx
            st.attitudeQuaternion.y, st.attitudeQuaternion.z = qy, qz
            acc_w = rowan.rotate(plant.state.quat, plant.state.acc)
            st.acc.x, st.acc.y, st.acc.z = acc_w[0], acc_w[1], acc_w[2] - 1.0 + acc_z_bias_g

            gyr = np.degrees(plant.state.omega)
            sensors = fw.sensorData_t()
            sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = gyr[0], gyr[1], gyr[2]

            fw.oot_set_rpm(int(rpm[0]), int(rpm[1]), int(rpm[2]), int(rpm[3]))
            c = fw.control_t()
            fw.controllerOmarIndi(ctrl, c, sp, sensors, st, 2 * i)

            rhs = np.array([c.thrustSi, c.torqueX, c.torqueY, c.torqueZ])
            rpm_cmd = np.sqrt(np.maximum(np.maximum(B0_inv @ rhs, 0.0) / KT, 0.0))
            fa = np.array([0.0, 0.0, f_ext_z]) if (f_ext_z != 0.0 and t > 5.0) else np.zeros(3)
            plant.step(Action(rpm_cmd), DT, fa)
            rpm = plant.state.rpm
            if t >= 12.0:
                zs.append(plant.state.pos[2])
    finally:
        _restore(devnull, o1, o2)

    z = np.array(zs)
    err_mm = float((np.mean(z) - TARGET) * 1000) if len(z) else float("nan")
    return dict(err_mm=err_mm, std_mm=float(np.std(z) * 1000) if len(z) else float("nan"))


def _poly_jerk_snap(piece, t_loc):
    d1 = piece.derivative()
    d2 = d1.derivative()
    d3 = d2.derivative()
    d4 = d3.derivative()
    jerk = np.array([d3.px.eval(t_loc), d3.py.eval(t_loc), d3.pz.eval(t_loc)])
    snap = np.array([d4.px.eval(t_loc), d4.py.eval(t_loc), d4.pz.eval(t_loc)])
    yaw_dot = d1.pyaw.eval(t_loc)
    yaw_ddot = d2.pyaw.eval(t_loc)
    return jerk, snap, yaw_dot, yaw_ddot


def run_figure8(
    kr_scale=1.0,
    kom_scale=1.0,
    ki_scale=1.0,
    acc_z_bias_g=0.0,
):
    traj = _uav.Trajectory()
    traj.loadcsv(TRAJ_CSV)
    devnull, o1, o2 = _suppress()
    ctrl = _init_ctrl(kr_scale=kr_scale, kom_scale=kom_scale, ki_scale=ki_scale, indi=3)
    plant = _plant()
    B0_inv = np.linalg.inv(plant.B0)
    rpm = np.zeros(4)
    rolls, pitches = [], []
    guard_hits = 0
    ticks = 0
    t_end = CLIMB + traj.duration
    try:
        for i in range(int(t_end / DT) + 1):
            t = i * DT
            if t < CLIMB:
                tau = t / CLIMB
                s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3
                pos = np.array([0.0, 0.0, TARGET * s])
                vel = np.zeros(3)
                acc = np.zeros(3)
                jerk = np.zeros(3)
                snap = np.zeros(3)
                yaw_dot = yaw_ddot = 0.0
            else:
                tt = t - CLIMB
                out = traj.eval(min(tt, traj.duration - 1e-9))
                pos = out.pos.copy()
                if pos[2] < 0.1:
                    pos[2] = TARGET
                vel = out.vel
                acc = out.acc
                current_t = 0.0
                t_loc = tt
                for p in traj.polynomials:
                    if tt <= current_t + p.duration:
                        t_loc = tt - current_t
                        jerk, snap, yaw_dot, yaw_ddot = _poly_jerk_snap(p, t_loc)
                        break
                    current_t += p.duration

            sp = fw.setpoint_t()
            sp.position.x, sp.position.y, sp.position.z = pos
            sp.velocity.x, sp.velocity.y, sp.velocity.z = vel
            sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = acc
            sp.jerk.x, sp.jerk.y, sp.jerk.z = jerk
            sp.snap.x, sp.snap.y, sp.snap.z = snap
            sp.attitudeRate.yaw = np.degrees(yaw_dot)
            sp.attitudeAcc.yaw = np.degrees(yaw_ddot)
            sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
            sp.mode.yaw = fw.modeAbs

            st = fw.state_t()
            st.position.x, st.position.y, st.position.z = plant.state.pos
            st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
            qw, qx, qy, qz = plant.state.quat
            st.attitudeQuaternion.w, st.attitudeQuaternion.x = qw, qx
            st.attitudeQuaternion.y, st.attitudeQuaternion.z = qy, qz
            acc_w = rowan.rotate(plant.state.quat, plant.state.acc)
            st.acc.x, st.acc.y, st.acc.z = acc_w[0], acc_w[1], acc_w[2] - 1.0 + acc_z_bias_g

            gyr = np.degrees(plant.state.omega)
            sensors = fw.sensorData_t()
            sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = gyr[0], gyr[1], gyr[2]

            fw.oot_set_rpm(int(rpm[0]), int(rpm[1]), int(rpm[2]), int(rpm[3]))
            c = fw.control_t()
            fw.controllerOmarIndi(ctrl, c, sp, sensors, st, 2 * i)

            thrust_si = c.thrustSi
            b1 = thrust_si / MASS if MASS > 0 else 0.0
            rpy = rowan.to_euler(plant.state.quat, convention="xyz")
            yaw = rpy[2]
            yc = np.array([-np.sin(yaw), np.cos(yaw), 0.0])
            R = rowan.to_matrix(plant.state.quat)
            zb = R[:, 2]
            c3 = np.linalg.norm(np.cross(yc, zb))
            if thrust_si != 0.0 and (abs(b1) <= MIN_B1 or abs(c3) <= MIN_C3):
                guard_hits += 1
            ticks += 1

            rhs = np.array([c.thrustSi, c.torqueX, c.torqueY, c.torqueZ])
            rpm_cmd = np.sqrt(np.maximum(np.maximum(B0_inv @ rhs, 0.0) / KT, 0.0))
            plant.step(Action(rpm_cmd), DT, np.zeros(3))
            rpm = plant.state.rpm

            if t >= CLIMB + 0.5:
                r, p, _ = rowan.to_euler(plant.state.quat, convention="xyz")
                rolls.append(abs(np.degrees(r)))
                pitches.append(abs(np.degrees(p)))
    finally:
        _restore(devnull, o1, o2)

    rolls = np.array(rolls)
    pitches = np.array(pitches)
    return dict(
        roll_std=float(np.std(rolls)),
        roll_peak=float(np.max(rolls)) if len(rolls) else float("nan"),
        pitch_std=float(np.std(pitches)),
        pitch_peak=float(np.max(pitches)) if len(pitches) else float("nan"),
        guard_frac=guard_hits / max(ticks, 1),
    )


def _print_sweep(title, values, results, unit="mm", baseline=None):
    print(f"\n{title}")
    print(f"{'value':>10} {'metric':>12}")
    for v, m in zip(values, results):
        tag = ""
        if baseline is not None and v == baseline:
            tag = " (Omar)"
        print(f"{v:10.3g} {m:12.2f}{tag}")


def run_figure8_attitude_sweeps():
    print(f"\n=== Figure8 kt=0.05 | indi=3 | traj={TRAJ_CSV.split('/')[-1]} ===")
    fig_base = run_figure8()
    print(
        f"Omar: roll std {fig_base['roll_std']:.2f}° peak {fig_base['roll_peak']:.1f}° "
        f"| pitch std {fig_base['pitch_std']:.2f}° peak {fig_base['pitch_peak']:.1f}° "
        f"(hardware ref roll {HW_ROLL_STD}° / {HW_ROLL_PEAK}°)"
    )
    print(f"MIN_B1/MIN_C3 guard would trigger {fig_base['guard_frac']*100:.2f}% of ticks (C has no guard)")

    for label, scales in [
        ("KR scale", [0.7, 0.85, 1.0, 1.15, 1.3]),
        ("KOMEGA scale", [0.7, 0.85, 1.0, 1.15, 1.3]),
        ("KI_ATT scale", [0.7, 0.85, 1.0, 1.15, 1.3]),
    ]:
        res = []
        for s in scales:
            if label.startswith("KR"):
                r = run_figure8(kr_scale=s)
            elif label.startswith("KOMEGA"):
                r = run_figure8(kom_scale=s)
            else:
                r = run_figure8(ki_scale=s)
            res.append(r["roll_std"])
        _print_sweep(label + " -> roll std [deg]", scales, res, baseline=1.0)
    return fig_base


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--figure8-only",
        action="store_true",
        help="Skip height-bias sweeps; run figure8 attitude block only",
    )
    args = parser.parse_args()

    if not args.figure8_only:
        acc_bias = calibrate_acc_bias_g()
        f_ext0 = calibrate_f_ext_indi0()
        print("=== Calibration ===")
        print(f"indi=3 IMU acc_z bias [g]: {acc_bias:+.5f} -> {run_hover(indi=3, acc_z_bias_g=acc_bias)['err_mm']:+.1f} mm")
        print(f"indi=0 f_ext_z [N]:       {f_ext0:+.5f} -> {run_hover(indi=0, f_ext_z=f_ext0)['err_mm']:+.1f} mm")

        print("\n=== Height bias | indi=3 | IMU-bias proxy (flight INDI) ===")
        for name, vals, runner in [
            ("Kpos_P.z", [4.9, 5.6, 7.0, 8.4, 9.1], lambda v: run_hover(3, acc_bias, 0, v, 4, 0)["err_mm"]),
            ("Kpos_D.z", [2.8, 3.2, 4.0, 4.8, 5.2], lambda v: run_hover(3, acc_bias, 0, 7, v, 0)["err_mm"]),
            ("Kpos_I.z", [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], lambda v: run_hover(3, acc_bias, 0, 7, 4, v)["err_mm"]),
        ]:
            res = [runner(v) for v in vals]
            _print_sweep(name, vals, res, baseline=7.0 if "P" in name else (4.0 if "D" in name else 0.0))

        print("\n=== Height bias | indi=0 | f_ext proxy (position branch) ===")
        for name, vals, runner in [
            ("Kpos_P.z", [4.9, 5.6, 7.0, 8.4, 9.1], lambda v: run_hover(0, 0, f_ext0, v, 4, 0)["err_mm"]),
            ("Kpos_D.z", [2.8, 3.2, 4.0, 4.8, 5.2], lambda v: run_hover(0, 0, f_ext0, 7, v, 0)["err_mm"]),
            ("Kpos_I.z", [0.0, 0.2, 0.4, 0.6, 0.8, 1.0], lambda v: run_hover(0, 0, f_ext0, 7, 4, v)["err_mm"]),
        ]:
            res = [runner(v) for v in vals]
            _print_sweep(name, vals, res, baseline=7.0 if "P" in name else (4.0 if "D" in name else 0.0))

    run_figure8_attitude_sweeps()


if __name__ == "__main__":
    main()
