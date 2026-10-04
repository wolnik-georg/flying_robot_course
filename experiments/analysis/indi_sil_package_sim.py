#!/usr/bin/env python3
"""Single A1 two-drone CS2 SIL episode (NeuralSwarm downwash, motor_tau=0.044 s)."""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import rowan
import torch

REPO = Path(__file__).resolve().parents[2]
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
FW_BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
HOST = REPO / "flying_drone_stack/firmware_app/host"

for p in (HOST, FW_BUILD, CS2_SIM):
    sys.path.insert(0, str(p))

import cffirmware as firm  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL  # noqa: E402
from crazyflie_sim.sim_data_types import State  # noqa: E402
from indi_sil_package_ns2 import NeuralSwarm  # noqa: E402
from indi_sil_package_plant import Quadrotor  # noqa: E402

from indi_sil_package_metrics import summarize_trace  # noqa: E402

NS2_DATA = CS2_SIM / "crazyflie_sim/backend/data/neuralswarm2"

BASE_INDI = dict(
    kr=2400.0,
    kw=170.0,
    kr_z=2400.0,
    kw_z=170.0,
    fc_bw=206.0,
    fc_bw_yaw=206.0,
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
    filt_dt_us=1000,
    filt_prewarp=1,
)


def apply_oot_gains(c, cfg: dict) -> None:
    indi = {**BASE_INDI, **cfg.get("indi_overrides", {})}
    for k, v in indi.items():
        setattr(c, "g_indi_" + k, v)
    c.g_kp_xy = float(cfg["kp_xy"])
    c.g_kp_z = float(cfg["kp_z"])
    c.g_kv_xy = float(cfg["kv_xy"])
    c.g_kv_z = float(cfg["kv_z"])
    c.g_ki_z = float(cfg.get("ki_z", 0.0))
    c.g_ki_z_limit = float(cfg.get("ki_z_limit", 1.5))
    c.g_indi_res_sign = int(cfg.get("res_sign", 1))
    c.g_controller_mode = int(cfg.get("ctrl_mode", 3))


def physics_dict(bottom: str) -> dict:
    if bottom == "oot4":
        mass = float(firm.oot_omar_mass())
        kt = float(firm.oot_omar_kt_equiv())
        kt4 = [kt, kt, kt, kt]
    else:
        c = firm.cvar
        mass = float(c.g_indi_mass)
        kt4 = [float(c.g_indi_kt1), float(c.g_indi_kt2), float(c.g_indi_kt3), float(c.g_indi_kt4)]
    J = [firm.oot_inertia(i) for i in range(3)]
    return dict(
        mass=mass,
        kt=kt4,
        arm_length=float(firm.oot_arm_length()),
        t2t=float(firm.oot_thrust2torque()),
        inertia=J,
        motor_tau=0.044,
    )


def run_episode(cfg: dict) -> dict:
    bottom_ctrl = cfg.get("bottom_controller", "oot")
    duration = float(cfg.get("duration_s", 15.0))
    dt = 1e-3
    z_bot = float(cfg.get("z_bottom", 0.5))
    dz = float(cfg.get("dz", 0.5))
    log_every = int(cfg.get("log_decimation", 2))

    firm.controllerOutOfTreeInit()
    CrazyflieSIL._oot_count = 0
    CrazyflieSIL._oot2_count = 0
    CrazyflieSIL._oot3_count = 0

    top_cfg = cfg.get("top_partner")
    if not top_cfg:
        top_cfg = {
            "ctrl_mode": 0,
            "kp_xy": 64.0,
            "kp_z": 48.0,
            "kv_xy": 8.0,
            "kv_z": 7.0,
            "ki_z": 0.0,
            "res_sign": 1,
        }

    if bottom_ctrl == "oot":
        apply_oot_gains(firm.cvar, cfg)

    ph = physics_dict(bottom_ctrl)
    ns = NeuralSwarm(NS2_DATA)

    t_box = [0.0]
    cfs: list[CrazyflieSIL] = []
    targets = (z_bot, z_bot + dz)
    for i, z_tgt in enumerate(targets):
        ctrl = bottom_ctrl if i == 0 else "oot"
        cfs.append(CrazyflieSIL(f"cf{i}", np.zeros(3), ctrl, lambda: t_box[0]))
        if i == 0 and bottom_ctrl == "oot4":
            cfs[0].omar_indi_control.indi = 3
        if i == 1:
            firm.oot_select_drone(1)
            apply_oot_gains(firm.cvar, top_cfg)
        cfs[-1].takeoff(z_tgt, 4.0)

    quads = [Quadrotor(State(pos=np.zeros(3)), ph) for _ in targets]

    logs = {"t": [], "gyro": [], "roll": [], "pitch": [], "pos": []}
    n_steps = int(duration / dt)

    for k in range(1, n_steps + 1):
        t_box[0] = k * dt
        positions = [q.state.pos.copy() for q in quads]
        actions = []
        for i, cf in enumerate(cfs):
            if i == 0 and bottom_ctrl == "oot":
                firm.oot_select_drone(0)
                apply_oot_gains(firm.cvar, cfg)
            elif i == 1:
                firm.oot_select_drone(1)
                apply_oot_gains(firm.cvar, top_cfg)
            if i == 1:
                cf.cmdFullState((0.0, 0.0, z_bot + dz), (0, 0, 0), (0, 0, 0), 0.0, (0, 0, 0))
            else:
                cf.getSetpoint()
            cf.peers = [
                (float(positions[j][0]), float(positions[j][1]), float(positions[j][2]))
                for j in range(len(cfs))
                if j != i
            ]
            cf.setState(quads[i].state)
            actions.append(cf.executeController())

        fa_data = [("small", torch.hstack((torch.tensor(q.state.pos), torch.tensor(q.state.vel)))) for q in quads]
        for i, (q, act) in enumerate(zip(quads, actions)):
            f_a = ns.compute_Fa(fa_data[i], fa_data[0:i] + fa_data[i + 1 :])
            f_a = f_a / 1000.0 * 9.81
            q.step(act, dt, f_a)

        if k % log_every == 0:
            q0 = quads[0].state
            roll, pitch, _yaw = rowan.to_euler(q0.quat)
            logs["t"].append(t_box[0])
            logs["pos"].append(q0.pos.copy())
            logs["gyro"].append(np.degrees(q0.omega))
            logs["roll"].append(np.degrees(roll))
            logs["pitch"].append(np.degrees(pitch))

    t = np.asarray(logs["t"])
    pos = np.asarray(logs["pos"])
    gyro = np.asarray(logs["gyro"])
    roll = np.asarray(logs["roll"])
    pitch = np.asarray(logs["pitch"])
    metrics = summarize_trace(t, gyro, roll, pitch, pos, z_bot)

    def _sanitize(obj):
        if isinstance(obj, dict):
            return {k: _sanitize(v) for k, v in obj.items()}
        if isinstance(obj, float) and not np.isfinite(obj):
            return None
        return obj

    return {
        "label": cfg.get("label", "run"),
        "config": cfg,
        "metrics": _sanitize(metrics),
        "physics": ph,
    }


def main() -> None:
    cfg = json.loads(sys.argv[1])
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved_out = os.dup(1)
    os.dup2(devnull, 1)
    try:
        result = run_episode(cfg)
    finally:
        pass
    payload = (json.dumps(result) + "\n").encode()
    os.write(saved_out, payload)
    os.dup2(saved_out, 1)
    os.close(devnull)
    os.close(saved_out)


if __name__ == "__main__":
    main()
