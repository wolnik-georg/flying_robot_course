#!/usr/bin/env python3
"""Bisect NS2 harness vs ki_z_tradeoff_sweep._sim (2026-10-07). One change per level."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np

REPO = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "out" / "ns2_closed_loop_sil"
HOST = REPO / "flying_drone_stack/firmware_app/host"
FW = Path("/home/georg/Desktop/crazyflie-firmware/build")
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
CS2_EX = Path("/home/georg/Desktop/crazyswarm2/crazyflie_examples")

DURATION = float(os.environ.get("BISECT_DURATION", "8.0"))
TAKEOFF_S = 4.0
MOTOR_TAU = 0.044
ANCHOR = np.array([0.0, 0.0, 0.5])
DZ = 0.5

# ki_z_tradeoff reference gains
REF_INDI = dict(
    kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=60.0, mass=0.041,
    kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10,
    ff_free=0, filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11,
    tau_xy_max=0.045, tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8, notch_en=0,
)
REF_POS = dict(kp_xy=64.0, kp_z=48.0, kv_xy=5.0, kv_z=7.0, ki_z=16.0, ki_z_limit=1.5)

LAB_INDI_EXTRA = dict(
    fc_bw=206.0, fc_bw_yaw=206.0, filt_dt_us=1000, filt_prewarp=1, res_fc=80.0, res_clamp=10.0,
)
LAB_POS = dict(kp_xy=40.0, kp_z=30.0, kv_xy=8.0, kv_z=10.0, ki_z=16.0, ki_z_limit=1.5)


def _mock_ros() -> None:
    for name in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
        sys.modules.setdefault(name, MagicMock())


def _import_stack(use_ref_cffirmware: bool):
    _mock_ros()
    # Clear prior imports when switching
    for mod in list(sys.modules):
        if mod in ("cffirmware", "crazyflie_sim.crazyflie_sil", "crazyflie_sim.backend.np"):
            del sys.modules[mod]
    sys.path[:] = [p for p in sys.path if "cffirmware" not in p and "crazyflie_sim" not in p]
    if use_ref_cffirmware:
        sys.path.insert(0, "/tmp/cffirmware_zint_on/on")
    else:
        sys.path.insert(0, str(HOST))
        sys.path.insert(0, str(FW))
    sys.path.insert(0, str(CS2_SIM))
    sys.path.insert(0, str(CS2_EX))
    import cffirmware as firm  # noqa
    from crazyflie_sim.crazyflie_sil import CrazyflieSIL  # noqa
    from crazyflie_sim import sim_data_types  # noqa
    from crazyflie_sim.backend.np import Quadrotor  # noqa
    return firm, CrazyflieSIL, sim_data_types, Quadrotor


def _setup_gains(firm, *, lab: bool = False) -> None:
    c = firm.cvar
    indi = {**REF_INDI, **(LAB_INDI_EXTRA if lab else {})}
    pos = LAB_POS if lab else REF_POS
    for k, v in indi.items():
        setattr(c, "g_indi_" + k, v)
    c.g_kp_xy, c.g_kp_z = pos["kp_xy"], pos["kp_z"]
    c.g_kv_xy, c.g_kv_z = pos["kv_xy"], pos["kv_z"]
    c.g_ki_z, c.g_ki_z_limit = pos["ki_z"], pos["ki_z_limit"]
    c.g_controller_mode = 0


def _plant_params(firm, Quadrotor, sim_data_types):
    c = firm.cvar
    ph = dict(
        mass=0.041,
        kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
        arm_length=float(firm.oot_arm_length()),
        t2t=float(firm.oot_thrust2torque()),
        inertia=[firm.oot_inertia(i) for i in range(3)],
        motor_tau=MOTOR_TAU,
    )
    return ph


def _metrics_window(t: np.ndarray, pos: np.ndarray, sp: np.ndarray) -> dict:
    m = (t >= TAKEOFF_S) & (t <= DURATION)
    if not np.any(m):
        m = t <= DURATION
    p, s = pos[m], sp[m]
    e = p - s
    return {
        "z_mean_cm": float(np.mean(p[:, 2]) * 100),
        "z_err_mean_cm": float(np.mean(e[:, 2]) * 100),
        "lat_rms_cm": float(np.sqrt(np.mean(np.sum(e[:, :2] ** 2, axis=1))) * 100),
        "lat_max_m": float(np.max(np.abs(p[:, :2]))),
        "pos_y_at_last_sample_m": float(p[-1, 1]) if len(p) else float("nan"),
    }


def run_level(level: int) -> dict:
    use_ref_fw = level == 0
    firm, CrazyflieSIL, sim_data_types, Quadrotor = _import_stack(use_ref_fw)
    _setup_gains(firm, lab=level >= 4)
    if level >= 3:
        firm.controllerOutOfTreeInit()
    CrazyflieSIL._oot_count = 0
    ph = _plant_params(firm, Quadrotor, sim_data_types)
    dt = 1e-3
    n_steps = int(DURATION / dt)
    t_box = [0.0]

    slot_bot = ANCHOR + np.array([0.0, 0.0, 0.0])
    slot_top = ANCHOR + np.array([0.0, 0.0, DZ])
    z_bot, z_top = float(slot_bot[2]), float(slot_top[2])

    n_drones = 1 if level == 0 else 2
    cfs = []
    quads = []
    for i in range(n_drones):
        cfs.append(CrazyflieSIL(f"cf{i}", np.zeros(3), "oot", lambda: t_box[0]))
        quads.append(Quadrotor(sim_data_types.State(pos=np.zeros(3)), ph))
    if level == 0:
        cfs[0].takeoff(1.0, 3.0)
        sp_fn = lambda _i, _t: np.array([0.0, 0.0, 1.0])
    else:
        cfs[0].takeoff(z_bot, TAKEOFF_S)
        cfs[1].takeoff(z_top, TAKEOFF_S)
        sp_fn = lambda i, t_sc: (slot_top if i == 1 else slot_bot).copy()

    logs = {"t": [], "pos": [], "sp": []}

    for k in range(1, n_steps + 1):
        t_now = k * dt
        t_box[0] = t_now
        positions = [q.state.pos.copy() for q in quads]
        actions = []
        for i, cf in enumerate(cfs):
            if level >= 3:
                firm.oot_select_drone(i)
                _setup_gains(firm, lab=level >= 4)
            elif n_drones == 2:
                firm.oot_select_drone(i)

            t_sc = max(0.0, t_now - TAKEOFF_S)
            if level == 0:
                cf.getSetpoint()
                sp_i = sp_fn(i, t_sc)
            elif level == 1 or (level >= 2 and t_now < TAKEOFF_S):
                cf.getSetpoint()
                sp_i = sp_fn(i, t_sc)
            elif level >= 2:
                sp_i = sp_fn(i, t_sc)
                cf.cmdFullState(
                    tuple(sp_i),
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                    0.0,
                    (0.0, 0.0, 0.0),
                )
            else:
                cf.getSetpoint()
                sp_i = sp_fn(i, t_sc)

            if level >= 5:
                peers = [
                    tuple(positions[j].tolist()) for j in range(n_drones) if j != i
                ]
                cf.peers = peers
                _peer_active = i
                # direct peer at 1 kHz (no 100Hz patch for bisect)
                tick = int(t_now * 1000)
                if hasattr(firm, "oot_set_peer") and peers:
                    p0 = peers[0]
                    firm.oot_set_peer(0, p0[0], p0[1], p0[2], tick)
                    firm.oot_set_peer_count(1)

            st = quads[i].state
            cf.setState(st)
            if level >= 6:
                cf.sensors.acc.x, cf.sensors.acc.y, cf.sensors.acc.z = map(float, st.acc)
                cf.motors_rpm_meas = [int(x) for x in st.rpm]
            actions.append(cf.executeController())

        for q, act in zip(quads, actions):
            q.step(act, dt, np.zeros(3))

        if k % 20 == 0:
            logs["t"].append(t_now)
            logs["pos"].append(np.stack([q.state.pos.copy() for q in quads], axis=0))
            sp_row = []
            for i in range(n_drones):
                t_sc = max(0.0, t_now - TAKEOFF_S)
                sp_row.append(sp_fn(i, t_sc))
            logs["sp"].append(np.stack(sp_row, axis=0))

    t = np.asarray(logs["t"])
    pos = np.asarray(logs["pos"])
    sp = np.asarray(logs["sp"])
    stable = True
    per_drone = []
    for i in range(n_drones):
        m = _metrics_window(t, pos[:, i, :], sp[:, i, :])
        m["drone"] = i
        per_drone.append(m)
        if m["lat_rms_cm"] > 500 or m["lat_max_m"] > 3.0:
            stable = False
    return {
        "level": level,
        "stable_heuristic": stable,
        "per_drone": per_drone,
        "n_drones": n_drones,
        "cffirmware": "ref_zint_on" if use_ref_fw else "host_build",
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    first_break = None
    for lv in range(7):
        print(f"level {lv} ...", flush=True)
        try:
            r = run_level(lv)
        except Exception as exc:
            r = {"level": lv, "error": str(exc), "stable_heuristic": False}
        rows.append(r)
        (OUT / "bisect_partial.json").write_text(json.dumps(rows, indent=2))
        if first_break is None and not r.get("stable_heuristic", False):
            first_break = lv
    summary = {"levels": rows, "first_break_level": first_break}
    (OUT / "bisect_summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
