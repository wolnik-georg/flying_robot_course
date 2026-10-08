#!/usr/bin/env python3
"""Two-drone NS2 closed-loop SIL: geometric + RNN (bottom), NS2 downwash, formation A1/A8."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock

for _rn in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_rn, MagicMock())

import numpy as np
import torch

REPO = Path(__file__).resolve().parents[2]
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
CS2_EX = Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")
FW_BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
HOST = REPO / "flying_drone_stack/firmware_app/host"

for p in (HOST, FW_BUILD, CS2_SIM, str(CS2_EX)):
    sys.path.insert(0, str(p))

import cffirmware as firm  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL, TrajectoryPolynomialPiece  # noqa: E402
from crazyflie_examples.formations import poly4d  # noqa: E402
from crazyflie_sim import sim_data_types  # noqa: E402
from crazyflie_sim.sim_data_types import State  # noqa: E402
from crazyflie_examples.formations import scenarios as FSC  # noqa: E402
from indi_sil_package_ns2 import NeuralSwarm  # noqa: E402
from ns2_closed_loop_sil_plant_bank import BankPlantDisturbance  # noqa: E402
from crazyflie_sim.backend.np import Quadrotor as _CS2Quadrotor  # noqa: E402


class Quadrotor(_CS2Quadrotor):
    """CS2 plant plus an optional external body torque (downwash roll/pitch moment)."""

    def step(self, action, dt, f_a=np.zeros(3), tau_a=None):
        super().step(action, dt, f_a)
        if tau_a is not None:
            # applied after the base update: first-order equivalent of adding tau_a to tau_u
            self.state.omega = self.state.omega + self.inv_J * np.asarray(tau_a, float) * dt
from ns2_closed_loop_sil_metrics import (  # noqa: E402
    corr_pred_a_res,
    crossing_dip_stats,
    gyro_rms_deg,
    max_tilt_deg,
    partner_ok,
    pred_stats,
    tracking_metrics,
    steady_mask,
    tracking_outside_crossings,
)
from test_residual_nn import N_WEIGHTS  # noqa: E402

NS2_DATA = CS2_SIM / "crazyflie_sim/backend/data/neuralswarm2"

# Measurement noise (controller sees noisy state; plant stays truth).
# Source: merged_A8_rnn0_2026-10-05_17-39-27, cf5 outside-crossing std vs ctrl target / gyro.
HW_MEAS_POS_STD_M = np.array([0.0036, 0.0100, 0.0053])
HW_MEAS_GYRO_STD_DEG_S = np.array([25.6, 12.5, 2.3])


def state_for_controller(st: State, rng: np.random.Generator, cfg: dict) -> State:
    if not cfg.get("meas_noise"):
        return st
    sig_p = np.asarray(cfg.get("meas_pos_std_m", HW_MEAS_POS_STD_M), float)
    sig_g = np.asarray(cfg.get("meas_gyro_std_deg_s", HW_MEAS_GYRO_STD_DEG_S), float)
    out = State(
        pos=st.pos.copy(),
        vel=st.vel.copy(),
        quat=st.quat.copy(),
        omega=st.omega.copy(),
        acc=np.asarray(st.acc, float).copy(),
        rpm=np.asarray(st.rpm, float).copy(),
    )
    out.pos = out.pos + rng.normal(0.0, sig_p)
    out.omega = out.omega + np.radians(rng.normal(0.0, sig_g))
    return out

# 2026-10-05 lab meta (cf5 / cf_second, ctrl_mode=0 geometric).
LAB_INDI = dict(
    kr=2400.0,
    kw=170.0,
    kr_z=2400.0,
    kw_z=170.0,
    # kr_geo/kw_geo are firmware defaults (0.01 / 0.0011) — not exposed on host cvar.
    fc_bw=60.0,
    fc_bw_yaw=60.0,
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
    filt_dt_us=1000,
    filt_prewarp=1,
)
LAB_POS = dict(kp_xy=40.0, kp_z=30.0, kv_xy=8.0, kv_z=10.0, ki_z=16.0, ki_z_limit=1.5)
REF_POS = dict(kp_xy=64.0, kp_z=48.0, kv_xy=5.0, kv_z=7.0, ki_z=16.0, ki_z_limit=1.5)

_peer_real = None
_peer_active_drone = [0]
_peer_by_drone: dict[int, dict] = {}

GAIN_SNAPSHOT_KEYS = (
    "g_kp_xy",
    "g_kp_z",
    "g_kv_xy",
    "g_kv_z",
    "g_ki_z",
    "g_ki_z_limit",
    "g_controller_mode",
    "g_indi_res_sign",
)


def snapshot_gains() -> dict[str, float]:
    c = firm.cvar
    return {k: float(getattr(c, k)) for k in GAIN_SNAPSHOT_KEYS}


def snapshot_all_drones() -> list[dict]:
    out = []
    for i in range(2):
        firm.oot_select_drone(i)
        row = snapshot_gains()
        row["drone"] = i
        out.append(row)
    return out


def install_peer_patch(peer_hz: float) -> None:
    """Hold peer packets at ~peer_hz; separate state per receiving drone (oot_select index)."""
    global _peer_real
    if _peer_real is None:
        _peer_real = firm.oot_set_peer
    period_ms = max(1, int(round(1000.0 / float(peer_hz))))

    def _ensure(drone: int) -> dict:
        if drone not in _peer_by_drone:
            _peer_by_drone[drone] = {"last_ms": {}, "held": {}}
        return _peer_by_drone[drone]

    def patched(k, x, y, z, tick):
        drone = int(_peer_active_drone[0])
        st = _ensure(drone)
        tick = int(tick)
        if k not in st["last_ms"]:
            st["last_ms"][k] = -period_ms
            st["held"][k] = (float(x), float(y), float(z), tick)
        if tick - st["last_ms"][k] >= period_ms:
            st["held"][k] = (float(x), float(y), float(z), tick)
            st["last_ms"][k] = tick
        hx, hy, hz, ht = st["held"][k]
        _peer_real(k, hx, hy, hz, ht)

    firm.oot_set_peer = patched


def apply_geometric_gains(c, *, ki_z: float | None = None, res_sign: int = 1, use_ref_fc: bool = False) -> None:
    indi = dict(LAB_INDI)
    if use_ref_fc:
        indi["fc_bw"] = 60.0
        indi["fc_bw_yaw"] = 60.0
        indi.pop("res_fc", None)
        indi.pop("res_clamp", None)
    for k, v in indi.items():
        setattr(c, "g_indi_" + k, v)
    pos = REF_POS if use_ref_fc else LAB_POS
    c.g_kp_xy = pos["kp_xy"]
    c.g_kp_z = pos["kp_z"]
    c.g_kv_xy = pos["kv_xy"]
    c.g_kv_z = pos["kv_z"]
    c.g_ki_z = float(LAB_POS["ki_z"] if ki_z is None else ki_z)
    c.g_ki_z_limit = LAB_POS["ki_z_limit"]
    c.g_controller_mode = 0
    c.g_indi_res_sign = int(res_sign)


def configure_drone(idx: int, *, rnn_en: bool, div: int, res_sign: int, ki_z: float, use_ref_gains: bool = False) -> None:
    firm.oot_select_drone(idx)
    apply_geometric_gains(firm.cvar, ki_z=ki_z, res_sign=res_sign, use_ref_fc=use_ref_gains)
    firm.cvar.g_rnn_div = int(div)
    firm.cvar.g_rnn_en = 1 if (idx == 0 and rnn_en) else 0


def upload_weights(w: np.ndarray) -> None:
    c = firm.cvar
    c.g_rnn_n = int(w.size)
    c.g_rnn_begin = 1
    firm.oot_rnn_service()
    for i, v in enumerate(w):
        c.g_rnn_wi = int(i)
        c.g_rnn_wv = float(v)
        c.g_rnn_wc = 1
        firm.oot_rnn_service()
    c.g_rnn_end = 1
    firm.oot_rnn_service()
    if c.g_rnn_ready != 1:
        raise RuntimeError("RNN upload rejected")


def build_scenario(sid: str, cfg: dict) -> object:
    if sid == "A1":
        return FSC.A1(dz=float(cfg.get("dz", 0.5)), hold=float(cfg.get("hold_s", 15.0)))
    if sid == "A8":
        return FSC.A8(
            dz=float(cfg.get("dz", 0.5)),
            span=float(cfg.get("span", 1.0)),
            duration=float(cfg.get("shuttle_duration", 6.0)),
            settle=float(cfg.get("settle_s", 2.0)),
            passes=int(cfg.get("passes", 4)),
        )
    raise ValueError(sid)


def robot_world_pose(sc, i: int, t: float, anchor: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    pos, vel, _acc, yaw, _omega = robot_world_kinematics(sc, i, t, anchor, vel_mode="fd")
    return pos, vel, yaw


def robot_world_kinematics(
    sc,
    i: int,
    t: float,
    anchor: np.ndarray,
    *,
    vel_mode: str = "fd",
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float, np.ndarray]:
    rp = sc.robots[i]
    tt = min(float(t), float(rp.curve.duration))
    if vel_mode == "analytic":
        d = poly4d._derivs(rp.curve, tt)  # noqa: SLF001 — same stencil as poly4d compile
        off = anchor + rp.slot
        pos = off + d[0, :3]
        vel = d[1, :3].copy()
        acc = d[2, :3].copy()
        yaw = float(d[0, 3])
        omega = np.array([0.0, 0.0, d[1, 3]], dtype=float)
        return pos, vel, acc, yaw, omega
    cur = np.asarray(rp.curve.at(tt), dtype=float)
    pos = anchor + rp.slot + cur[:3]
    yaw = float(cur[3]) if cur.size > 3 else 0.0
    dt = 1e-3
    t2 = min(tt + dt, rp.curve.duration)
    cur2 = np.asarray(rp.curve.at(t2), dtype=float)
    pos2 = anchor + rp.slot + cur2[:3]
    vel = (pos2 - pos) / dt
    acc = np.zeros(3, dtype=float)
    omega = np.zeros(3, dtype=float)
    return pos, vel, acc, yaw, omega


def ground_pos(slot: np.ndarray) -> np.ndarray:
    """Floor at slot xy (anchor + formation slot); matches hardware start pose."""
    return np.array([float(slot[0]), float(slot[1]), 0.0], dtype=float)


def curve_to_pieces(curve) -> list[TrajectoryPolynomialPiece]:
    table = poly4d.compile_curve(curve)
    pieces: list[TrajectoryPolynomialPiece] = []
    for row in table:
        dur = float(row[0])
        pieces.append(
            TrajectoryPolynomialPiece(
                row[1:9].tolist(),
                row[9:17].tolist(),
                row[17:25].tolist(),
                row[25:33].tolist(),
                dur,
            )
        )
    return pieces


def run_episode(cfg: dict) -> dict:
    sid = cfg.get("scenario", "A8")
    sc = build_scenario(sid, cfg)
    anchor = np.array(cfg.get("anchor", [0.0, 0.0, 0.5]), dtype=float)
    dz_stack = float(sc.params.get("dz", 0.5))
    peer_hz = float(cfg.get("peer_hz", 100.0))
    _peer_by_drone.clear()
    if peer_hz < 999.0:
        install_peer_patch(peer_hz)
    elif _peer_real is not None:
        firm.oot_set_peer = _peer_real

    w = None
    if not cfg.get("skip_rnn_upload", False):
        w = np.load(cfg["weights"])["weights"].astype(np.float32)
        if w.size != N_WEIGHTS:
            raise RuntimeError(f"bad weight count {w.size}")

    res_sign = int(cfg.get("res_sign", 1))
    ki_z = float(cfg.get("ki_z", 16.0))
    div = int(cfg.get("div", 10))
    rnn_en = bool(cfg.get("rnn_en", False))
    downwash_scale = float(cfg.get("downwash_scale", 1.0))
    takeoff_s = float(cfg.get("takeoff_s", 4.0))
    log_hz = int(cfg.get("log_hz", 100))

    CrazyflieSIL._oot_count = 0
    t_box = [0.0]

    slot_bot = anchor + sc.robots[0].slot
    slot_top = anchor + sc.robots[1].slot

    use_ref = bool(cfg.get("use_ref_gains", False))
    gain_audit: dict = {}
    for i in range(2):
        configure_drone(i, rnn_en=False, div=div, res_sign=res_sign, ki_z=ki_z, use_ref_gains=use_ref)
    gain_audit["after_configure_before_cfs"] = snapshot_all_drones()

    c0 = firm.cvar
    ph = dict(
        mass=LAB_INDI["mass"],
        kt=[c0.g_indi_kt1, c0.g_indi_kt2, c0.g_indi_kt3, c0.g_indi_kt4],
        arm_length=float(firm.oot_arm_length()),
        t2t=float(firm.oot_thrust2torque()),
        inertia=[firm.oot_inertia(i) for i in range(3)],
        motor_tau=0.044,
    )
    p0_bot = ground_pos(slot_bot)
    p0_top = ground_pos(slot_top)
    cfs = [
        CrazyflieSIL("cf5", p0_bot, "oot", lambda: t_box[0]),
        CrazyflieSIL("cf_second", p0_top, "oot", lambda: t_box[0]),
    ]
    quads = [Quadrotor(State(pos=p0_bot.copy()), ph), Quadrotor(State(pos=p0_top.copy()), ph)]
    dw_plant = str(cfg.get("downwash_plant", "ns2")).lower()
    ns = None
    bank_dw = None
    if cfg.get("downwash", True):
        if dw_plant == "bank":
            wp = cfg.get("plant_weights") or cfg.get("weights")
            if not wp:
                raise ValueError("downwash_plant=bank requires plant_weights or weights path")
            bank_dw = BankPlantDisturbance(wp)
            bank_dw.symmetrize = bool(cfg.get("plant_symmetrize", False))
        else:
            ns = NeuralSwarm(NS2_DATA)
    gain_audit["after_cfs_ctor"] = snapshot_all_drones()

    gain_audit["before_controllerOutOfTreeInit"] = snapshot_all_drones()
    if not cfg.get("skip_controller_init", False):
        firm.controllerOutOfTreeInit()
    gain_audit["after_controllerOutOfTreeInit"] = snapshot_all_drones()

    for i in range(2):
        configure_drone(i, rnn_en=False, div=div, res_sign=res_sign, ki_z=ki_z, use_ref_gains=use_ref)
    gain_audit["after_reconfigure_post_init"] = snapshot_all_drones()

    setpoint_mode = str(cfg.get("setpoint_mode", "hlc" if sid == "A8" else "cmd_fd"))
    legacy_goto = bool(cfg.get("legacy_goto_converge", False))
    converge_s = float(cfg.get("converge_s", 0.0))
    if not legacy_goto:
        converge_s = 0.0
    vel_mode = "analytic" if setpoint_mode == "cmd_analytic" else "fd"
    hlc_started = False
    goto_started = [False, False]

    for i, cf in enumerate(cfs):
        cf.takeoff(float(slot_bot[2] if i == 0 else slot_top[2]), takeoff_s)

    configure_once = not bool(cfg.get("per_step_configure", False))
    if w is not None:
        firm.oot_select_drone(0)
        upload_weights(w)

    scenario_s = float(sc.duration)
    land_s = float(cfg.get("land_s", 3.0))
    duration = float(cfg.get("duration_s", scenario_s + takeoff_s + land_s))
    dt = 1e-3
    n_steps = int(duration / dt)
    log_stride = max(1, int(round(1000.0 / log_hz)))

    logs = {
        "t": [],
        "pos_bot": [],
        "pos_top": [],
        "sp_bot": [],
        "sp_top": [],
        "quat_bot": [],
        "gyro_bot": [],
        "gyro_top": [],
        "pred_z": [],
        "pred_x": [],
        "pred_y": [],
        "a_res_z": [],
        "clamp": [],
    }
    save_trace = bool(cfg.get("save_trace", False))
    trace_top: list[np.ndarray] = []

    n_cross = int(cfg.get("n_crossings", 4 if sid == "A8" else 0))
    rng = np.random.default_rng(int(cfg.get("seed", 0)))
    mass = float(ph["mass"])
    last_fa_bot = np.zeros(3)

    for k in range(1, n_steps + 1):
        t_now = k * dt
        t_box[0] = t_now
        positions = [q.state.pos.copy() for q in quads]
        t_phase = t_now - takeoff_s
        if setpoint_mode == "hlc" and not hlc_started and t_now >= takeoff_s and t_phase >= converge_s:
            for j, cfj in enumerate(cfs):
                cfj.uploadTrajectory(0, 0, curve_to_pieces(sc.robots[j].curve))
            for cfj in cfs:
                cfj.startTrajectory(0, timescale=1.0, relative=True)
            hlc_started = True

        actions = []
        for i, cf in enumerate(cfs):
            _peer_active_drone[0] = i
            firm.oot_select_drone(i)
            if not configure_once:
                configure_drone(
                    i, rnn_en=rnn_en and i == 0, div=div, res_sign=res_sign, ki_z=ki_z, use_ref_gains=use_ref
                )
            elif w is not None:
                firm.cvar.g_rnn_en = 1 if (i == 0 and rnn_en) else 0

            t_phase = t_now - takeoff_s
            if t_now < takeoff_s:
                cf.getSetpoint()
            elif t_phase < converge_s:
                slot_i = slot_bot if i == 0 else slot_top
                if not goto_started[i]:
                    cf.goTo(tuple(slot_i), 0.0, max(converge_s, 0.1))
                    goto_started[i] = True
                cf.getSetpoint()
            elif setpoint_mode == "hlc":
                cf.getSetpoint()
            else:
                t_sc = t_phase - converge_s
                pos_i, vel_i, acc_i, yaw_i, omega_i = robot_world_kinematics(
                    sc, i, t_sc, anchor, vel_mode=vel_mode
                )
                cf.cmdFullState(tuple(pos_i), tuple(vel_i), tuple(acc_i), yaw_i, tuple(omega_i))

            peers = []
            for j in range(2):
                if j == i:
                    continue
                peers.append(tuple(positions[j].tolist()))
            cf.peers = peers
            st = state_for_controller(quads[i].state, rng, cfg)
            cf.setState(st)
            if not cfg.get("skip_rpm_acc_feed", False):
                cf.sensors.acc.x, cf.sensors.acc.y, cf.sensors.acc.z = map(float, st.acc)
                cf.motors_rpm_meas = [int(x) for x in st.rpm]
            actions.append(cf.executeController())

        positions = [q.state.pos.copy() for q in quads]
        velocities = [q.state.vel.copy() for q in quads]
        for i, (q, act) in enumerate(zip(quads, actions)):
            f_a = np.zeros(3)
            if cfg.get("downwash", True):
                if bank_dw is not None:
                    f_a = bank_dw.compute_fa_newtons(
                        i, positions, velocities, mass, scale=downwash_scale
                    )
                elif ns is not None:
                    fa_data = [
                        ("small", torch.hstack((torch.tensor(q.state.pos), torch.tensor(q.state.vel))))
                        for q in quads
                    ]
                    f_a = (
                        ns.compute_Fa(fa_data[i], fa_data[:i] + fa_data[i + 1 :])
                        / 1000.0
                        * 9.81
                        * downwash_scale
                    )
            tau_a = None
            if bank_dw is not None and float(cfg.get("torque_c", 0.0)) != 0.0:
                tau_a = bank_dw.compute_tau_nm(
                    i, positions, velocities, mass, c_lever2=float(cfg["torque_c"])
                )
            if i == 0:
                last_fa_bot = f_a.copy()
                last_tau_bot = tau_a
            q.step(act, dt, f_a, tau_a)

        if not cfg.get("skip_logs", False) and k % log_stride == 0:
            t_log = t_now
            logs["t"].append(t_log)
            logs["pos_bot"].append(quads[0].state.pos.copy())
            logs["pos_top"].append(quads[1].state.pos.copy())
            if t_log < takeoff_s:
                sp_b = np.array([slot_bot[0], slot_bot[1], float(slot_bot[2])])
                sp_t = np.array([slot_top[0], slot_top[1], float(slot_top[2])])
            else:
                t_sc = max(0.0, t_log - takeoff_s - converge_s)
                sp_b, _, _ = robot_world_pose(sc, 0, t_sc, anchor)
                sp_t, _, _ = robot_world_pose(sc, 1, t_sc, anchor)
            logs["sp_bot"].append(sp_b)
            logs["sp_top"].append(sp_t)
            logs["quat_bot"].append(quads[0].state.quat.copy())
            if "quat_top" not in logs:
                logs["quat_top"] = []
            logs["quat_top"].append(quads[1].state.quat.copy())
            logs["gyro_bot"].append(np.degrees(quads[0].state.omega))
            logs["gyro_top"].append(np.degrees(quads[1].state.omega))
            if save_trace:
                trace_top.append(quads[1].state.pos.copy())
            firm.oot_select_drone(0)
            if w is not None and rnn_en:
                logs["pred_x"].append(float(firm.cvar.g_rnn_pred_x))
                logs["pred_y"].append(float(firm.cvar.g_rnn_pred_y))
                logs["pred_z"].append(float(firm.cvar.g_rnn_pred_z))
                logs["clamp"].append(int(firm.cvar.g_rnn_clamped))
            else:
                logs["pred_x"].append(0.0)
                logs["pred_y"].append(0.0)
                logs["pred_z"].append(0.0)
                logs["clamp"].append(0)
            logs["a_res_z"].append(float(firm.oot_get_a_res(2)))
            if cfg.get("downwash", False):
                if "fa_bot_az" not in logs:
                    logs["fa_bot_az"] = []
                logs["fa_bot_az"].append(float(last_fa_bot[2] / mass))

    t = np.asarray(logs["t"])
    pos_bot = np.asarray(logs["pos_bot"])
    pos_top = np.asarray(logs["pos_top"])
    sp_bot = np.asarray(logs["sp_bot"])
    sp_top = np.asarray(logs["sp_top"])
    g_bot = gyro_rms_deg(t, np.asarray(logs["gyro_bot"]))
    g_top = gyro_rms_deg(t, np.asarray(logs["gyro_top"]))
    pred = pred_stats(logs["pred_z"], logs["pred_x"], logs["pred_y"], logs["clamp"])
    pred["vs_a_res"] = corr_pred_a_res(logs["pred_z"], logs["a_res_z"])
    track_b = tracking_outside_crossings(
        t, pos_bot, sp_bot, sid, takeoff_s=takeoff_s, n_crossings=n_cross
    )
    track_t = tracking_outside_crossings(
        t, pos_top, sp_top, sid, takeoff_s=takeoff_s, n_crossings=n_cross
    )
    quat_b = np.asarray(logs["quat_bot"])
    quat_t = np.asarray(logs["quat_top"])
    mb = steady_mask(t, pos_bot, scenario=sid, takeoff_s=takeoff_s, n_crossings=n_cross)
    mt = steady_mask(t, pos_top, scenario=sid, takeoff_s=takeoff_s, n_crossings=n_cross)
    track_b["max_tilt_deg"] = max_tilt_deg(quat_b[mb] if np.any(mb) else quat_b)
    track_t["max_tilt_deg"] = max_tilt_deg(quat_t[mt] if np.any(mt) else quat_t)
    track_b_full = tracking_metrics(t, pos_bot, sp_bot)
    track_t_full = tracking_metrics(t, pos_top, sp_top)

    out = {
        "label": cfg.get("label"),
        "scenario": sid,
        "config": {k: v for k, v in cfg.items() if k != "weights"},
        "bottom_gyro_rms_deg_s": g_bot,
        "top_gyro_rms_deg_s": g_top,
        "partner_ok": partner_ok(g_top, pos_top=pos_top, t=t),
        "prediction": pred,
        "tracking_bottom_cm": track_b,
        "tracking_top_cm": track_t,
        "tracking_bottom_full_cm": track_b_full,
        "tracking_top_full_cm": track_t_full,
        "mean_sep_err_cm": float(np.sqrt(np.mean(np.sum((pos_bot - pos_top) ** 2, axis=1))) * 100),
        "duration_s": duration,
        "n_weights": int(w.size) if w is not None else 0,
        "final_pos_bot": quads[0].state.pos.tolist(),
        "final_pos_top": quads[1].state.pos.tolist(),
        "gain_audit": gain_audit if cfg.get("record_gain_audit", False) else None,
    }
    if sid == "A8" and n_cross > 0:
        out["crossing_dips"] = crossing_dip_stats(t, pos_bot, sp_bot, n_crossings=n_cross)
    if save_trace and trace_top:
        arr = np.stack(trace_top, axis=0)
        out["trace_top_pos_hash"] = float(np.sum(arr * 1e6) % 1e9)
        out["trace_top_n"] = int(arr.shape[0])
        out["trace_top_pos"] = arr.tolist() if cfg.get("embed_trace") else None

    if cfg.get("return_logs"):
        out["logs"] = {k: (v if not isinstance(v, list) or not v or not isinstance(v[0], np.ndarray) else [x.tolist() for x in v]) for k, v in logs.items()}

    return out


def main() -> None:
    cfg = json.loads(sys.argv[1])
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved = os.dup(1)
    os.dup2(devnull, 1)
    try:
        result = run_episode(cfg)
    except Exception as exc:
        result = {"label": cfg.get("label"), "error": str(exc)}
    os.write(saved, (json.dumps(result) + "\n").encode())
    os.dup2(saved, 1)
    os.close(devnull)
    os.close(saved)


if __name__ == "__main__":
    main()
