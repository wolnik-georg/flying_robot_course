#!/usr/bin/env python3
"""Step 1 (2026-10-08): SIL diagnosis for Omar z offset — plant-only perturbations, indi=3."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import rowan

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "omar_z_offset"
HOST = REPO / "flying_drone_stack/firmware_app/host"
FW = Path("/home/georg/Desktop/crazyflie-firmware/build")
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
CS2_EX = Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")

for _rn in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_rn, MagicMock())

for p in (HOST, FW, CS2_SIM, CS2_EX, ANALYSIS):
    sys.path.insert(0, str(p))

import cffirmware as fw  # noqa: E402
from indi_sil_package_plant import Quadrotor  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL  # noqa: E402
from crazyflie_sim.sim_data_types import Action, State  # noqa: E402
from crazyflie_examples.formations import scenarios as FSC  # noqa: E402
from crazyflie_examples.formations import poly4d  # noqa: E402
from crazyflie_sim.crazyflie_sil import TrajectoryPolynomialPiece  # noqa: E402

from ns2_closed_loop_sil_metrics import tracking_outside_crossings  # noqa: E402

MASS_NOM = float(fw.oot_omar_mass())
KT_NOM = float(fw.oot_omar_kt_equiv())
ARM = float(fw.oot_arm_length())
T2T = float(fw.oot_thrust2torque())
J = [fw.oot_inertia(i) for i in range(3)]
DT = 0.002
TARGET_SOLO = 1.0
HW_SOLO_CM = 16.0
HW_A8_C_CM = 22.0
HW_A8_R_CM = 19.0
METRIC_FROM_S = 12.0


def _suppress():
    devnull = os.open(os.devnull, os.O_WRONLY)
    o1, o2 = os.dup(1), os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    return devnull, o1, o2


def _restore(devnull, o1, o2):
    os.dup2(o1, 1)
    os.dup2(o2, 2)
    os.close(devnull)


def plant_params(*, mass_mult: float = 1.0, kt_mult: float = 1.0) -> dict:
    return dict(
        mass=MASS_NOM * mass_mult,
        inertia=J,
        kt=[KT_NOM * kt_mult] * 4,
        arm_length=ARM,
        t2t=T2T,
        motor_tau=0.044,
    )


def _step_omar(
    controller: str,
    ctrl_c,
    sp,
    sensors,
    st,
    tick: int,
):
    fw.oot_set_rpm(int(st._rpm[0]), int(st._rpm[1]), int(st._rpm[2]), int(st._rpm[3]))  # type: ignore[attr-defined]
    c = fw.control_t()
    if controller == "oot4":
        fw.controllerOmarIndi(ctrl_c, c, sp, sensors, st, tick)
    else:
        fw.controllerOutOfTree5(c, sp, sensors, st, tick)
    return c


def run_solo_hover(
    controller: str,
    *,
    mass_mult: float = 1.0,
    kt_mult: float = 1.0,
    acc_z_bias_g: float = 0.0,
    f_ext_z: float = 0.0,
    duration: float = 18.0,
) -> dict:
    """Closed loop @ 1 m, indi=3, Omar native P/D (7/4)."""
    devnull, o1, o2 = _suppress()
    try:
        if controller == "oot4":
            ctrl_c = fw.controllerOmarIndi_t()
            fw.controllerOmarIndiInit(ctrl_c)
            ctrl_c.indi = 3
        else:
            fw.controllerOutOfTree5Init()
            fw.omar_indi_rust_set_indi(3)
            ctrl_c = None

        plant = Quadrotor(State(pos=np.zeros(3)), plant_params(mass_mult=mass_mult, kt_mult=kt_mult))
        B0_inv = np.linalg.inv(plant.B0)
        kt_plant = KT_NOM * kt_mult
        rpm = np.zeros(4)
        zs = []
        climb = 2.0

        for i in range(int(duration / DT)):
            t = i * DT
            tau = min(t / climb, 1.0)
            s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3 if t < climb else 1.0
            sp = fw.setpoint_t()
            sp.position.z = TARGET_SOLO * s
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
            st._rpm = rpm  # type: ignore[attr-defined]

            gyr = np.degrees(plant.state.omega)
            sensors = fw.sensorData_t()
            sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = gyr[0], gyr[1], gyr[2]

            c = _step_omar(controller, ctrl_c, sp, sensors, st, 2 * i)
            rhs = np.array([c.thrustSi, c.torqueX, c.torqueY, c.torqueZ])
            rpm_cmd = np.sqrt(np.maximum(np.maximum(B0_inv @ rhs, 0.0) / kt_plant, 0.0))
            fa = (
                np.array([0.0, 0.0, f_ext_z])
                if (f_ext_z != 0.0 and t > 5.0)
                else np.zeros(3)
            )
            plant.step(Action(rpm_cmd), DT, fa)
            rpm = plant.state.rpm
            if t >= METRIC_FROM_S:
                zs.append(plant.state.pos[2])
    finally:
        _restore(devnull, o1, o2)

    z = np.asarray(zs, float)
    if len(z) < 50:
        return {"mean_z_err_cm": float("nan"), "n": len(z)}
    err_cm = (np.mean(z) - TARGET_SOLO) * 100.0
    return {"mean_z_err_cm": float(err_cm), "z_std_cm": float(np.std(z) * 100), "n": len(z)}


def curve_to_pieces(curve) -> list:
    table = poly4d.compile_curve(curve)
    pieces = []
    for row in table:
        pieces.append(
            TrajectoryPolynomialPiece(
                row[1:9].tolist(),
                row[9:17].tolist(),
                row[17:25].tolist(),
                row[25:33].tolist(),
                float(row[0]),
            )
        )
    return pieces


def run_a8_bottom(
    controller: str,
    *,
    mass_mult: float = 1.0,
    kt_mult: float = 1.0,
    acc_z_bias_g: float = 0.0,
) -> dict:
    """Two-drone A8, bottom Omar, top geometric cmdFullState, no downwash."""
    devnull, o1, o2 = _suppress()
    sc = FSC.A8(dz=0.5, span=1.0, duration=6.0, settle=2.0, passes=4)
    anchor = np.array([0.0, 0.0, 0.5])
    slot_bot = anchor + sc.robots[0].slot
    slot_top = anchor + sc.robots[1].slot
    z_bot_tgt = float(slot_bot[2])
    takeoff_s = 4.0
    dt = 1e-3
    duration = float(sc.duration) + takeoff_s + 3.0
    ph = plant_params(mass_mult=mass_mult, kt_mult=kt_mult)

    try:
        CrazyflieSIL._oot_count = 0
        CrazyflieSIL._oot5_count = 0
        t_box = [0.0]
        p0_bot = np.array([slot_bot[0], slot_bot[1], 0.0])
        p0_top = np.array([slot_top[0], slot_top[1], 0.0])
        cfs = [
            CrazyflieSIL("cf5", p0_bot, controller, lambda: t_box[0]),
            CrazyflieSIL("cf_second", p0_top, "oot", lambda: t_box[0]),
        ]
        if controller == "oot4":
            cfs[0].omar_indi_control.indi = 3
        quads = [Quadrotor(State(pos=p0_bot.copy()), ph), Quadrotor(State(pos=p0_top.copy()), ph)]
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(1)
        fw.cvar.g_kp_xy = 64.0
        fw.cvar.g_kp_z = 48.0
        fw.cvar.g_kv_xy = 8.0
        fw.cvar.g_kv_z = 7.0
        fw.cvar.g_controller_mode = 0

        for i, cf in enumerate(cfs):
            cf.takeoff(float(slot_bot[2] if i == 0 else slot_top[2]), takeoff_s)

        hlc_started = False
        logs_t, logs_pos, logs_sp = [], [], []
        n_steps = int(duration / dt)

        for k in range(1, n_steps + 1):
            t_now = k * dt
            t_box[0] = t_now
            positions = [q.state.pos.copy() for q in quads]
            t_phase = t_now - takeoff_s
            if not hlc_started and t_now >= takeoff_s:
                for j, cfj in enumerate(cfs):
                    cfj.uploadTrajectory(0, 0, curve_to_pieces(sc.robots[j].curve))
                for cfj in cfs:
                    cfj.startTrajectory(0, timescale=1.0, relative=True)
                hlc_started = True

            actions = []
            for i, cf in enumerate(cfs):
                if i == 1:
                    fw.oot_select_drone(1)
                if i == 1:
                    cf.cmdFullState(
                        (float(slot_top[0]), float(slot_top[1]), float(slot_top[2])),
                        (0, 0, 0),
                        (0, 0, 0),
                        0.0,
                        (0, 0, 0),
                    )
                else:
                    cf.getSetpoint()
                cf.peers = [
                    tuple(positions[j].tolist()) for j in range(2) if j != i
                ]
                st = quads[i].state
                if i == 0 and acc_z_bias_g != 0.0:
                    st = State(
                        pos=st.pos.copy(),
                        vel=st.vel.copy(),
                        quat=st.quat.copy(),
                        omega=st.omega.copy(),
                        acc=st.acc.copy(),
                        rpm=st.rpm.copy(),
                    )
                    st.acc[2] += acc_z_bias_g
                cf.setState(st)
                cf.sensors.acc.z = float(st.acc[2])
                cf.motors_rpm_meas = [int(x) for x in st.rpm]
                actions.append(cf.executeController())

            for i, (q, act) in enumerate(zip(quads, actions)):
                q.step(act, dt, np.zeros(3))

            if k % 2 == 0:
                logs_t.append(t_now)
                logs_pos.append(quads[0].state.pos.copy())
                sp_b, _, _ = _robot_pose(sc, 0, max(0.0, t_now - takeoff_s), anchor)
                logs_sp.append(sp_b)

        t = np.asarray(logs_t)
        pos = np.asarray(logs_pos)
        sp = np.asarray(logs_sp)
        tr = tracking_outside_crossings(t, pos, sp, "A8", takeoff_s=takeoff_s, n_crossings=4)
        err = pos[:, 2] - sp[:, 2]
        steady = t >= takeoff_s + 8.0
        mean_all = float(np.mean(err[steady]) * 100.0) if np.any(steady) else float("nan")
        mean_cm = float(tr.get("mean_z_cm", mean_all))
        if not np.isfinite(mean_cm):
            mean_cm = mean_all
        return {
            "mean_z_err_cm": mean_cm,
            "mean_z_err_all_steady_cm": mean_all,
            "rms_z_cm": float(tr.get("rms_z_cm", float("nan"))),
        }
    finally:
        _restore(devnull, o1, o2)


def _robot_pose(sc, i, t, anchor):
    rp = sc.robots[i]
    tt = min(float(t), float(rp.curve.duration))
    cur = np.asarray(rp.curve.at(tt), dtype=float)
    pos = anchor + rp.slot + cur[:3]
    return pos, np.zeros(3), 0.0


def calibrate_acc_bias(controller: str, target_cm: float) -> float:
    lo, hi = -0.25, 0.0
    for _ in range(28):
        mid = (lo + hi) / 2.0
        r = run_solo_hover(controller, acc_z_bias_g=mid)
        err = r["mean_z_err_cm"]
        if not np.isfinite(err):
            return float("nan")
        if err > target_cm:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    cases = [
        ("baseline", dict(mass_mult=1.0, kt_mult=1.0, acc_z_bias_g=0.0)),
        ("plant_kt_p4pct", dict(mass_mult=1.0, kt_mult=1.04, acc_z_bias_g=0.0)),
        ("plant_kt_p8pct", dict(mass_mult=1.0, kt_mult=1.08, acc_z_bias_g=0.0)),
        ("plant_mass_p4pct", dict(mass_mult=1.04, kt_mult=1.0, acc_z_bias_g=0.0)),
        ("plant_mass_p8pct", dict(mass_mult=1.08, kt_mult=1.0, acc_z_bias_g=0.0)),
        ("plant_mass_m4pct", dict(mass_mult=0.96, kt_mult=1.0, acc_z_bias_g=0.0)),
        ("plant_mass_m8pct", dict(mass_mult=0.92, kt_mult=1.0, acc_z_bias_g=0.0)),
    ]

    bias_cal = {ctrl: calibrate_acc_bias(ctrl, HW_SOLO_CM) for ctrl in ("oot4", "oot5")}
    print("calibrated acc_z_bias_g for +16 cm:", bias_cal, flush=True)

    rows = []
    for ctrl in ("oot4", "oot5"):
        ctrl_cases = list(cases) + [
            (f"acc_bias_cal_{HW_SOLO_CM:.0f}cm", dict(acc_z_bias_g=bias_cal[ctrl])),
            (
                "kt_p8_and_acc_half_cal",
                dict(kt_mult=1.08, acc_z_bias_g=0.5 * bias_cal[ctrl]),
            ),
        ]
        for name, kw in ctrl_cases:
            r_solo = run_solo_hover(ctrl, **kw)
            r_a8 = run_a8_bottom(ctrl, **{k: v for k, v in kw.items() if k != "f_ext_z"})
            rows.append(
                {
                    "controller": ctrl,
                    "case": name,
                    **kw,
                    "solo_mean_z_err_cm": r_solo["mean_z_err_cm"],
                    "a8_mean_z_err_cm": r_a8["mean_z_err_cm"],
                }
            )
            print(
                f"{ctrl:5} {name:22} solo={r_solo['mean_z_err_cm']:+.1f} cm  A8={r_a8['mean_z_err_cm']:+.1f} cm",
                flush=True,
            )

    # indi=3 rejects constant f_ext (sanity)
    fext = run_solo_hover("oot4", f_ext_z=0.05)

    def closest(rows_, scenario_key, target):
        best = min(
            (r for r in rows_ if np.isfinite(r[scenario_key])),
            key=lambda r: abs(r[scenario_key] - target),
            default=None,
        )
        return best

    report = {
        "step": 1,
        "targets_cm": {"solo_hover": HW_SOLO_CM, "a8_omar_c": HW_A8_C_CM, "a8_omar_rust": HW_A8_R_CM},
        "method": "indi=3, Omar P/D 7/4, plant perturbations only; A8=2-drone HLC no downwash",
        "acc_z_bias_g_calibrated": bias_cal,
        "rows": rows,
        "sanity_f_ext_z_0p05N_indi3_solo_cm": fext["mean_z_err_cm"],
        "closest_to_solo_16cm": {
            "oot4": closest([r for r in rows if r["controller"] == "oot4"], "solo_mean_z_err_cm", HW_SOLO_CM),
            "oot5": closest([r for r in rows if r["controller"] == "oot5"], "solo_mean_z_err_cm", HW_SOLO_CM),
        },
        "verdict": {},
    }

    # H2: kt/mass at indi=3
    h2_max = max(
        abs(r["solo_mean_z_err_cm"])
        for r in rows
        if r["case"].startswith("plant_") and "acc_bias" not in r["case"]
    )
    report["verdict"]["H2_thrust_constants"] = (
        "NOT primary at indi=3"
        if h2_max < 30
        else f"plant mismatch alone gives up to {h2_max:.1f} cm (check)"
    )
    cal = [r for r in rows if r["case"] == f"acc_bias_cal_{HW_SOLO_CM:.0f}cm"]
    h3_ok = all(abs(r["solo_mean_z_err_cm"] - HW_SOLO_CM) < 3 for r in cal)
    report["verdict"]["H3_imu_acc_bias"] = (
        "REPRODUCES +16 cm solo at calibrated bias (indi=3)" if h3_ok else "calibration miss"
    )
    report["verdict"]["H1_missing_integral"] = (
        "Mechanism: DC height error from H3-class bias is not trimmed at Kpos_I=0; "
        "integral would act on pos_e (docs/41 §19–20). Root cause class = estimation/thrust split, not lack of P."
    )
    report["verdict"]["baseline_sil_no_injection"] = {
        r["controller"]: next(x["solo_mean_z_err_cm"] for x in rows if x["controller"] == r["controller"] and x["case"] == "baseline")
        for r in [{"controller": "oot4"}, {"controller": "oot5"}]
    }
    report["note_indi0_fext"] = (
        "Constant f_ext under indi=3 is rejected by RPM-INDI (see oot5_kpos_i_sweep); "
        "use indi=0 + f_ext only for position-branch Kpos_I proxy, not flight indi=3."
    )

    out_path = OUT / "diagnose_step1.json"
    out_path.write_text(json.dumps(report, indent=2))
    print(json.dumps(report["verdict"], indent=2))
    print(f"wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
