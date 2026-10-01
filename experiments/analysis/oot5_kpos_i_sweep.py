#!/usr/bin/env python3
"""SIL sweep: Omar position-loop Kpos_I vs steady hover height under a calibrated bias.

Controller=10 (`omar_indi_rust.rs`) uses compile-time Kpos_P/D/I; this script sweeps Kpos_I via
the **C reference** in cffirmware (same law as Rust, 7/7 in `test_omar_indi_rust_vs_c.py`).

Plant / mass: **`fw.oot_omar_mass()`** (42.7 g) and **`oot_omar_*`** — same source as
`run_omar_indi_rust_sil_smoke.sh` / docs/41 §8, not stock CF2.1 39.3 g.

Disturbance: world-frame **`f_a`** on `Quadrotor.step(..., f_a)` in **Newtons** (see
`crazyflie_sim/backend/np.py`: `vel += (g + (R f_u + f_a)/m) dt`). This is **not** ignored.

Important — **`ctrl.indi = 3` (flight default) rejects constant `f_a`** via the INDI/RPM path,
so a docs/51-style force step **cannot** test Kpos_I with indi=3 (baseline error stays ~0 mm).
This sweep uses **`indi = 0`** to isolate the **position P+D+I branch**, and calibrates
**`f_ext_z`** so **`Kpos_I.z = 0`** gives **+162 mm** mean error — matching §18 hardware hover
bias (thesis30, steady **1.162 m** vs **1.000 m**). Sign: **+f_ext_z** = upward world force →
flies **high**, same direction as tonight's bias.

Sanity: with **`indi=0`** and **`f_ext_z = -0.19 N`**, baseline error is **~600 mm low**
(physically plausible P+D-only sag); the old script's **~0.19 mm** at KI=0 was INDI masking,
not broken `f_a`.

Run:  /usr/bin/python3.10 oot5_kpos_i_sweep.py
Needs: cffirmware built at ~/Desktop/crazyflie-firmware/build (controller Omar linked).

Does NOT rebuild firmware or change `omar_indi_rust.rs` — desk recommendation only.
"""
import os
import sys

import numpy as np
import rowan

SO = "/home/georg/Desktop/crazyflie-firmware/build"
sys.path.insert(0, SO)
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_sim")

import cffirmware as fw  # noqa: E402
from crazyflie_sim.backend.np import Quadrotor  # noqa: E402
from crazyflie_sim.sim_data_types import Action, State  # noqa: E402

MASS = float(fw.oot_omar_mass())
KT = float(fw.oot_omar_kt_equiv())
ARM = float(fw.oot_arm_length())
T2T = float(fw.oot_thrust2torque())
J = tuple(fw.oot_inertia(i) for i in range(3))

TARGET = 1.0
CLIMB = 2.0
DT = 0.002
DUR = 17.0
DIST_START_S = 5.0
METRIC_FROM_S = 10.0
HW_BIAS_MM = 162.0  # §18 controller=10 hover steady-state high bias [mm]
INDI_MODE = 0  # position-loop isolation; see module docstring
F_EXT_SANITY = -0.19  # N — validation only (large sag at Kpos_I.z=0, indi=0)


def setpoint_z(t):
    if t < CLIMB:
        tau = t / CLIMB
        s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3
        return TARGET * s
    return TARGET


def _calibrate_f_ext_upward(target_bias_mm=HW_BIAS_MM):
    """Binary search world +Z force [N] for mean z error ≈ target_bias_mm at Kpos_I.z=0."""
    lo, hi = 0.0, 0.5
    for _ in range(28):
        mid = (lo + hi) / 2.0
        err = run(kpos_i_z=0.0, f_ext=mid, indi=INDI_MODE)["err_mm"]
        if err < target_bias_mm:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def run(kpos_i_z, f_ext, indi=INDI_MODE):
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_out, old_err = os.dup(1), os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)

    ctrl = fw.controllerOmarIndi_t()
    fw.controllerOmarIndiInit(ctrl)
    ctrl.indi = indi
    ctrl.Kpos_I = fw.mkvec(0.0, 0.0, kpos_i_z)
    ctrl.Kpos_P = fw.mkvec(7.0, 7.0, 7.0)
    ctrl.Kpos_D = fw.mkvec(4.0, 4.0, 4.0)

    plant = Quadrotor(
        State(pos=np.zeros(3)),
        dict(mass=MASS, inertia=list(J), kt=[KT] * 4, arm_length=ARM, t2t=T2T, motor_tau=0.044),
    )
    B0_inv = np.linalg.inv(plant.B0)
    rpm = np.zeros(4)
    zs = []

    try:
        for i in range(int(DUR / DT)):
            t = i * DT
            z_sp = setpoint_z(t)
            sp = fw.setpoint_t()
            sp.position.z = z_sp
            sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs
            sp.mode.yaw = fw.modeAbs

            st = fw.state_t()
            st.position.x, st.position.y, st.position.z = plant.state.pos
            st.velocity.x, st.velocity.y, st.velocity.z = plant.state.vel
            qw, qx, qy, qz = plant.state.quat
            st.attitudeQuaternion.w, st.attitudeQuaternion.x = qw, qx
            st.attitudeQuaternion.y, st.attitudeQuaternion.z = qy, qz
            acc_w = rowan.rotate(plant.state.quat, plant.state.acc)
            st.acc.x, st.acc.y, st.acc.z = acc_w[0], acc_w[1], acc_w[2] - 1.0

            sensors = fw.sensorData_t()
            sensors.gyro.x = sensors.gyro.y = sensors.gyro.z = 0.0

            fw.oot_set_rpm(int(rpm[0]), int(rpm[1]), int(rpm[2]), int(rpm[3]))
            c = fw.control_t()
            fw.controllerOmarIndi(ctrl, c, sp, sensors, st, 2 * i)

            rhs = np.array([c.thrustSi, c.torqueX, c.torqueY, c.torqueZ])
            force = np.maximum(B0_inv @ rhs, 0.0)
            rpm_cmd = np.sqrt(np.maximum(force / KT, 0.0))
            fa = (
                np.array([0.0, 0.0, f_ext])
                if (f_ext != 0.0 and t > DIST_START_S)
                else np.zeros(3)
            )
            plant.step(Action(rpm_cmd), DT, fa)
            rpm = plant.state.rpm
            if t >= METRIC_FROM_S:
                zs.append(plant.state.pos[2])
    finally:
        os.dup2(old_out, 1)
        os.dup2(old_err, 2)
        os.close(devnull)

    z = np.array(zs)
    if len(z) < 100:
        return dict(err_mm=float("nan"), rmse_mm=float("nan"), std_mm=float("nan"))
    err = z - TARGET
    return dict(
        err_mm=float(np.mean(err) * 1000),
        rmse_mm=float(np.sqrt(np.mean(err**2)) * 1000),
        std_mm=float(np.std(z) * 1000),
    )


def main():
    f_cal = _calibrate_f_ext_upward()
    sanity = run(0.0, F_EXT_SANITY, indi=0)
    indi3_check = run(0.0, f_cal, indi=3)

    print(
        f"Omar C Kpos_I.z sweep — mass={MASS:.4f} kg, indi={INDI_MODE} (position branch), "
        f"f_ext_z={f_cal:+.5f} N after t>{DIST_START_S}s (calibrated to +{HW_BIAS_MM:.0f} mm at Kpos_I.z=0)"
    )
    print(
        f"Sanity (indi=0, f_ext={F_EXT_SANITY} N): mean err {sanity['err_mm']:+.1f} mm "
        f"(expect large negative — P+D cannot reject DC force)"
    )
    print(
        f"Flight-realistic check (indi=3, same f_ext): mean err {indi3_check['err_mm']:+.2f} mm "
        f"(INDI rejects constant f_a — not a valid Kpos_I testbed)"
    )
    print(f"{'Kpos_I.z':>10} {'mean err (mm)':>14} {'RMSE (mm)':>10} {'z std (mm)':>10}")
    rows = []
    for kz in (0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0):
        m = run(kz, f_cal)
        rows.append((kz, m))
        print(f"{kz:10.1f} {m['err_mm']:14.2f} {m['rmse_mm']:10.2f} {m['std_mm']:10.2f}")
    best = min(rows, key=lambda r: abs(r[1]["err_mm"]))
    print(
        f"\nClosest to zero mean error in this grid: Kpos_I.z={best[0]} "
        f"(mean err {best[1]['err_mm']:+.1f} mm)"
    )


if __name__ == "__main__":
    main()
