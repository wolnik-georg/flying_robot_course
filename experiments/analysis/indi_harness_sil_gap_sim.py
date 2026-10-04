#!/usr/bin/env python3
"""Full SIL + switchable harness-gap elements (cmd dead time, RPM age, output hold, plant asymmetry)."""

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
from indi_sil_rpm_delay_metrics import summarize_trace  # noqa: E402

NS2_DATA = CS2_SIM / "crazyflie_sim/backend/data/neuralswarm2"

BASE_INDI = dict(
    kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=206.0, fc_bw_yaw=206.0,
    mass=0.041, kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10,
    ff_free=0, filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11,
    tau_xy_max=0.045, tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8,
    notch_en=0, notch_f0=6.9, notch_bw=3.0, filt_dt_us=1000, filt_prewarp=1,
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
    c.g_indi_res_sign = int(cfg.get("res_sign", 1))
    c.g_controller_mode = int(cfg.get("ctrl_mode", 3))


class GapPlant(Quadrotor):
    """Extends plant with cmd dead-time buffer and asymmetric spool."""

    def __init__(self, state, params, gap: dict):
        super().__init__(state, params)
        self.dead_n = int(gap.get("cmd_dead_ticks", 0))
        self.dead_buf: list = []
        self.spool_asym = float(gap.get("spool_asym", 1.0))

    def step_delayed(self, action, dt, f_a=np.zeros(3)):
        if self.dead_n > 0:
            self.dead_buf.append(action)
            if len(self.dead_buf) <= self.dead_n:
                from crazyflie_sim import sim_data_types
                action = sim_data_types.Action([0, 0, 0, 0])
            else:
                action = self.dead_buf.pop(0)
        if self.spool_asym != 1.0 and self.motor_tau and self._rpm is not None:
            rpm_cmd = np.asarray(action.rpm, dtype=float)
            alpha_up = dt / (self.motor_tau + dt)
            alpha_dn = dt / (self.motor_tau * self.spool_asym + dt)
            dr = rpm_cmd - self._rpm
            alpha = np.where(dr >= 0, alpha_up, alpha_dn)
            self._rpm = self._rpm + alpha * dr
            action = type(action)(self._rpm.tolist())
        self.step(action, dt, f_a)


def run_episode(cfg: dict) -> dict:
    gap = cfg.get("gap", {})
    rpm_lag_samples = int(gap.get("rpm_lag_samples", 0))
    output_hold_500 = bool(gap.get("output_hold_500hz", False))
    bottom_ctrl = cfg.get("bottom_controller", "oot")
    duration = float(cfg.get("duration_s", 15.0))
    dt = 1e-3
    z_bot = float(cfg.get("z_bottom", 0.5))
    dz = float(cfg.get("dz", 0.5))

    firm.controllerOutOfTreeInit()
    CrazyflieSIL._oot_count = 0
    top_cfg = {"ctrl_mode": 0, "kp_xy": 64.0, "kp_z": 48.0, "kv_xy": 8.0, "kv_z": 7.0, "ki_z": 0.0, "res_sign": 1}
    if bottom_ctrl == "oot":
        apply_oot_gains(firm.cvar, cfg)

    ph = dict(mass=0.041, kt=[BASE_INDI["kt1"]] * 4, arm_length=float(firm.oot_arm_length()),
              t2t=float(firm.oot_thrust2torque()), inertia=[firm.oot_inertia(i) for i in range(3)], motor_tau=0.044)
    ns = NeuralSwarm(NS2_DATA)
    t_box = [0.0]
    cfs = []
    for i, z_tgt in enumerate((z_bot, z_bot + dz)):
        ctrl = bottom_ctrl if i == 0 else "oot"
        cfs.append(CrazyflieSIL(f"cf{i}", np.zeros(3), ctrl, lambda: t_box[0]))
        if i == 0 and bottom_ctrl == "oot4":
            cfs[0].omar_indi_control.indi = 3
        if i == 1:
            firm.oot_select_drone(1)
            apply_oot_gains(firm.cvar, top_cfg)
        cfs[-1].takeoff(z_tgt, 4.0)

    quads = [GapPlant(State(pos=np.zeros(3)), ph, gap) for _ in range(2)]
    rpm_hist: list[np.ndarray] = []
    last_action_bottom = None
    logs = {"t": [], "gyro": [], "roll": [], "pitch": [], "pos": []}
    n_steps = int(duration / dt)

    for k in range(1, n_steps + 1):
        t_box[0] = k * dt
        tick = int(t_box[0] * 1000)
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
            cf.peers = [(float(positions[j][0]), float(positions[j][1]), float(positions[j][2]))
                        for j in range(len(cfs)) if j != i]
            cf.setState(quads[i].state)
            true_rpm = np.asarray(quads[i].state.rpm, dtype=float)
            if i == 0 and rpm_lag_samples > 0:
                rpm_hist.append(true_rpm.copy())
                age = rpm_hist[-1 - rpm_lag_samples] if len(rpm_hist) > rpm_lag_samples else true_rpm
                cf.motors_rpm_meas = [int(x) for x in age]
            actions.append(cf.executeController())

        if output_hold_500 and last_action_bottom is not None and tick % 2 != 0:
            actions[0] = last_action_bottom
        last_action_bottom = actions[0]

        fa_data = [("small", torch.hstack((torch.tensor(q.state.pos), torch.tensor(q.state.vel)))) for q in quads]
        for i, (q, act) in enumerate(zip(quads, actions)):
            f_a = ns.compute_Fa(fa_data[i], fa_data[0:i] + fa_data[i + 1 :]) / 1000.0 * 9.81
            q.step_delayed(act, dt, f_a)

        if k % 2 == 0:
            q0 = quads[0].state
            roll, pitch, _ = rowan.to_euler(q0.quat)
            logs["t"].append(t_box[0])
            logs["pos"].append(q0.pos.copy())
            logs["gyro"].append(np.degrees(q0.omega))
            logs["roll"].append(np.degrees(roll))
            logs["pitch"].append(np.degrees(pitch))

    t = np.asarray(logs["t"])
    metrics = summarize_trace(t, np.asarray(logs["gyro"]), np.asarray(logs["roll"]),
                              np.asarray(logs["pitch"]), np.asarray(logs["pos"]), z_bot)
    return {"label": cfg.get("label"), "config": cfg, "metrics": metrics, "gap": gap}


def main() -> None:
    cfg = json.loads(sys.argv[1])
    devnull = os.open(os.devnull, os.O_WRONLY)
    saved = os.dup(1)
    os.dup2(devnull, 1)
    try:
        result = run_episode(cfg)
    finally:
        pass
    os.write(saved, (json.dumps(result) + "\n").encode())
    os.dup2(saved, 1)
    os.close(devnull)
    os.close(saved)


if __name__ == "__main__":
    main()
