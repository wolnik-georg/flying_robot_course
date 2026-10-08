#!/usr/bin/env python3
"""Omar z-integral SIL: H4 cmd_gain actuator mismatch + Kpos_Iz grid (2026-10-08)."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import rowan

REPO = Path(__file__).resolve().parents[2]
ANALYSIS = Path(__file__).resolve().parent
OUT = ANALYSIS / "out" / "omar_z_integral"
HOST = REPO / "flying_drone_stack/firmware_app/host"
FW = Path("/home/georg/Desktop/crazyflie-firmware/build")
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
CS2_EX = Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")
WEIGHTS = REPO / "experiments/analysis/out/c2_e2e_2026-10-01/full_bank_c1_complete.npz"
FIG8_CSV = CS2_EX / "crazyflie_examples/data/figure8_mode1_kt0.05.csv"

for _rn in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_rn, MagicMock())
for p in (HOST, FW, CS2_SIM, CS2_EX, ANALYSIS):
    sys.path.insert(0, str(p))

import cffirmware as fw  # noqa: E402
from indi_sil_package_plant import Quadrotor  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece  # noqa: E402
from crazyflie_sim.sim_data_types import Action, State  # noqa: E402
from crazyflie_examples.formations import scenarios as FSC  # noqa: E402
from crazyflie_examples.formations import poly4d  # noqa: E402
from ns2_closed_loop_sil_plant_bank import BankPlantDisturbance  # noqa: E402
from ns2_closed_loop_sil_metrics import crossing_dip_stats, tracking_outside_crossings  # noqa: E402

MASS = float(fw.oot_omar_mass())
KT = float(fw.oot_omar_kt_equiv())
ARM = float(fw.oot_arm_length())
T2T = float(fw.oot_thrust2torque())
J = [fw.oot_inertia(i) for i in range(3)]
DT = 0.002
TARGET_HOVER = 1.0


def scale_action_rpm(act: Action, cmd_gain: float) -> Action:
    if cmd_gain == 1.0:
        return act
    s = math.sqrt(cmd_gain)
    rpm = np.asarray(act.rpm, float) * s
    return Action(rpm)


def set_omar_kpos_iz(ctrl_name: str, cf, kpos_iz: float) -> None:
    if ctrl_name == "oot4":
        cf.omar_indi_control.Kpos_I = fw.mkvec(0.0, 0.0, float(kpos_iz))
    else:
        fw.omar_indi_rust_set_kpos_iz(float(kpos_iz))


def run_solo_harness(
    controller: str,
    *,
    cmd_gain: float = 1.0,
    kpos_iz: float = 0.0,
    duration: float = 18.0,
) -> dict:
    """Direct Omar plant loop (consistent RPM↔force); cmd_gain scales delivered thrust."""
    devnull = os.open(os.devnull, os.O_WRONLY)
    o1, o2 = os.dup(1), os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
    climb = 2.0
    try:
        if controller == "oot4":
            ctrl = fw.controllerOmarIndi_t()
            fw.controllerOmarIndiInit(ctrl)
            ctrl.indi = 3
            ctrl.Kpos_I = fw.mkvec(0.0, 0.0, kpos_iz)
            def step_fn(sp, sens, st, tick):
                c = fw.control_t()
                fw.controllerOmarIndi(ctrl, c, sp, sens, st, tick)
                return c

        else:
            fw.controllerOutOfTree5Init()
            fw.omar_indi_rust_set_indi(3)
            fw.omar_indi_rust_set_kpos_iz(kpos_iz)

            def step_fn(sp, sens, st, tick):
                c = fw.control_t()
                fw.controllerOutOfTree5(c, sp, sens, st, tick)
                return c

        plant = Quadrotor(
            State(pos=np.zeros(3)),
            dict(mass=MASS, inertia=J, kt=[KT] * 4, arm_length=ARM, t2t=T2T, motor_tau=0.044),
        )
        B0_inv = np.linalg.inv(plant.B0)
        rpm = np.zeros(4)
        zs, gyros = [], []
        for i in range(int(duration / DT)):
            t = i * DT
            tau = min(t / climb, 1.0)
            s = 6 * tau**5 - 15 * tau**4 + 10 * tau**3 if t < climb else 1.0
            sp = fw.setpoint_t()
            sp.position.z = TARGET_HOVER * s
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
            sensors.gyro.x, sensors.gyro.y, sensors.gyro.z = map(
                float, np.degrees(plant.state.omega)
            )
            fw.oot_set_rpm(int(rpm[0]), int(rpm[1]), int(rpm[2]), int(rpm[3]))
            c = step_fn(sp, sensors, st, 2 * i)
            rhs = np.array([c.thrustSi, c.torqueX, c.torqueY, c.torqueZ])
            rpm_cmd = np.sqrt(np.maximum(np.maximum(B0_inv @ rhs, 0.0) / KT, 0.0))
            if cmd_gain != 1.0:
                rpm_cmd = rpm_cmd * math.sqrt(cmd_gain)
            plant.step(Action(rpm_cmd), DT, np.zeros(3))
            rpm = plant.state.rpm
            if t >= 12.0:
                zs.append(plant.state.pos[2])
                gyros.append(np.degrees(plant.state.omega))
    finally:
        os.dup2(o1, 1)
        os.dup2(o2, 2)
        os.close(devnull)
    z = np.asarray(zs)
    g = np.asarray(gyros)
    return {
        "mean_z_err_cm": float((np.mean(z) - TARGET_HOVER) * 100) if len(z) else float("nan"),
        "gyro_rms_deg_s": float(np.sqrt(np.mean(g**2))) if len(g) else float("nan"),
    }


def curve_to_pieces(curve) -> list:
    table = poly4d.compile_curve(curve)
    return [
        TrajectoryPolynomialPiece(
            row[1:9].tolist(),
            row[9:17].tolist(),
            row[17:25].tolist(),
            row[25:33].tolist(),
            float(row[0]),
        )
        for row in table
    ]


def run_formation_sil(
    scenario: str,
    controller: str,
    *,
    cmd_gain: float = 1.0,
    kpos_iz: float = 0.0,
    ns2_plant: bool = False,
    return_trace: bool = False,
) -> dict:
    sc = FSC.A8(dz=0.5, span=1.0, duration=6.0, settle=2.0, passes=4) if scenario == "A8" else FSC.A1(dz=0.5, hold=15.0)
    anchor = np.array([0.0, 0.0, 0.5])
    slot_bot = anchor + sc.robots[0].slot
    slot_top = anchor + sc.robots[1].slot
    takeoff_s = 4.0
    ph = dict(mass=MASS, inertia=J, kt=[KT] * 4, arm_length=ARM, t2t=T2T, motor_tau=0.044)
    bank = None
    if ns2_plant:
        bank = BankPlantDisturbance(WEIGHTS)
        bank.symmetrize = True
    torque_c = 0.0032 if ns2_plant else 0.0

    devnull = os.open(os.devnull, os.O_WRONLY)
    o1, o2 = os.dup(1), os.dup(2)
    os.dup2(devnull, 1)
    os.dup2(devnull, 2)
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
        set_omar_kpos_iz(controller, cfs[0], kpos_iz)
        fw.controllerOutOfTreeInit()
        fw.oot_select_drone(1)
        fw.cvar.g_kp_xy, fw.cvar.g_kp_z = 64.0, 48.0
        fw.cvar.g_kv_xy, fw.cvar.g_kv_z = 8.0, 7.0
        fw.cvar.g_controller_mode = 0
        for i, cf in enumerate(cfs):
            cf.takeoff(float(slot_bot[2] if i == 0 else slot_top[2]), takeoff_s)
        quads = [Quadrotor(State(pos=p0_bot.copy()), ph), Quadrotor(State(pos=p0_top.copy()), ph)]
        hlc = False
        duration = float(sc.duration) + takeoff_s + 3.0
        logs_t, pos_b, sp_b, gyros, quats = [], [], [], [], []
        n_steps = int(duration / 1e-3)
        for k in range(1, n_steps + 1):
            t_now = k * 1e-3
            t_box[0] = t_now
            positions = [q.state.pos.copy() for q in quads]
            vels = [q.state.vel.copy() for q in quads]
            if not hlc and t_now >= takeoff_s:
                for j, cfj in enumerate(cfs):
                    cfj.uploadTrajectory(0, 0, curve_to_pieces(sc.robots[j].curve))
                for cfj in cfs:
                    cfj.startTrajectory(0, timescale=1.0, relative=True)
                hlc = True
            actions = []
            for i, cf in enumerate(cfs):
                if i == 1:
                    fw.oot_select_drone(1)
                    cf.cmdFullState(
                        (float(slot_top[0]), float(slot_top[1]), float(slot_top[2])),
                        (0, 0, 0),
                        (0, 0, 0),
                        0.0,
                        (0, 0, 0),
                    )
                else:
                    set_omar_kpos_iz(controller, cf, kpos_iz)
                    cf.getSetpoint()
                cf.peers = [tuple(positions[j].tolist()) for j in range(2) if j != i]
                cf.setState(quads[i].state)
                cf.motors_rpm_meas = [int(x) for x in quads[i].state.rpm]
                actions.append(cf.executeController())
            for i, (q, act) in enumerate(zip(quads, actions)):
                act_p = scale_action_rpm(act, cmd_gain)
                f_a = np.zeros(3)
                tau_a = None
                if bank is not None:
                    f_a = bank.compute_fa_newtons(i, positions, vels, MASS, scale=1.0)
                    if torque_c and i == 0:
                        tau_a = bank.compute_tau_nm(i, positions, vels, MASS, c_lever2=torque_c)
                q.step(act_p, 1e-3, f_a)
                if tau_a is not None:
                    q.state.omega = q.state.omega + q.inv_J * np.asarray(tau_a, float) * 1e-3
            if k % 2 == 0:
                logs_t.append(t_now)
                pos_b.append(quads[0].state.pos.copy())
                gyros.append(np.degrees(quads[0].state.omega))
                quats.append(quads[0].state.quat.copy())
                t_sc = max(0.0, t_now - takeoff_s)
                tt = min(t_sc, sc.robots[0].curve.duration)
                cur = np.asarray(sc.robots[0].curve.at(tt), float)
                sp_b.append(anchor + sc.robots[0].slot + cur[:3])
        t = np.asarray(logs_t)
        pos = np.asarray(pos_b)
        sp = np.asarray(sp_b)
        sid = "A8" if scenario == "A8" else "A1"
        tr = tracking_outside_crossings(t, pos, sp, sid, takeoff_s=takeoff_s, n_crossings=4 if sid == "A8" else 0)
        dips = crossing_dip_stats(t, pos, sp, n_crossings=4) if sid == "A8" else {}
        g = np.asarray(gyros)
        steady = t >= takeoff_s + 8.0
        mean_steady = float(np.mean((pos[steady, 2] - sp[steady, 2]) * 100)) if np.any(steady) else float("nan")
        return {
            "mean_z_err_cm": float(tr.get("mean_z_cm", mean_steady)),
            "mean_z_steady_cm": mean_steady,
            "max_z_err_cm": float(np.max(np.abs(pos[:, 2] - sp[:, 2])) * 100),
            "gyro_rms_deg_s": float(np.sqrt(np.mean(g**2))),
            "crossing_dip_mean_cm": dips.get("dip_cm_mean"),
            "ns2_plant": ns2_plant,
            "cmd_gain": cmd_gain,
            "kpos_iz": kpos_iz,
            **({"trace": {"t": t[::5].tolist(), "pos": pos[::5].tolist(), "sp": sp[::5].tolist()}} if return_trace else {}),
        }
    finally:
        os.dup2(o1, 1)
        os.dup2(o2, 2)
        os.close(devnull)


def run_figure8_sil(controller: str, *, cmd_gain: float, kpos_iz: float) -> dict:
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "uav_traj", "/home/georg/Desktop/crazyswarm2/crazyflie_py/crazyflie_py/uav_trajectory.py"
    )
    uav = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(uav)
    traj = uav.Trajectory()
    traj.loadcsv(str(FIG8_CSV))
    # Reuse solo-style loop at 1m — abbreviated: call bounded sweep pattern omitted for brevity
    r = run_solo_harness(controller, cmd_gain=cmd_gain, kpos_iz=kpos_iz, duration=12.0)
    r["scenario"] = "figure8_proxy_hover"
    r["note"] = "full figure8 trajectory grid uses FIG8_CSV — extend if needed"
    return r


def h4_table() -> dict:
    rows = []
    for g in (1.0, 1.10, 1.136, 1.14, 1.17, 1.15):
        for ctrl in ("oot4", "oot5"):
            r = run_solo_harness(ctrl, cmd_gain=g)
            rows.append({"controller": ctrl, "cmd_gain": g, **r})
    a8 = []
    for g in (1.10, 1.14, 1.17):
        for ctrl in ("oot4", "oot5"):
            r = run_formation_sil("A8", ctrl, cmd_gain=g, ns2_plant=True)
            a8.append({"controller": ctrl, "cmd_gain": g, **r})
    return {"solo_h4": rows, "a8_ns2_plant": a8}


def grid_run() -> dict:
    gains = [0.0, 0.25, 0.5, 1.0, 2.0]
    rows = []
    for kiz in gains:
        for sc in ("solo", "A8", "A1"):
            if sc == "solo":
                r = run_solo_harness("oot5", cmd_gain=1.14, kpos_iz=kiz)
                r["scenario"] = "solo_hover"
            else:
                r = run_formation_sil(sc, "oot5", cmd_gain=1.14, kpos_iz=kiz, ns2_plant=True)
                r["scenario"] = sc
            rows.append(r)
    return {"grid_cmd_gain_1.14": rows}


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    mode = sys.argv[1] if len(sys.argv) > 1 else "h4"
    if mode == "h4":
        rep = h4_table()
        (OUT / "h4_reproduce.json").write_text(json.dumps(rep, indent=2))
        print(json.dumps(rep, indent=2))
    elif mode == "grid":
        rep = grid_run()
        (OUT / "grid_partial.json").write_text(json.dumps(rep, indent=2))
        print(json.dumps(rep, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
