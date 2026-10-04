#!/usr/bin/env python3
"""Full SIL: fractional cmd dead time (linear interp on RPM history), sensor LPF/noise/delay."""

from __future__ import annotations

import json
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np
import rowan
import torch
from scipy import signal

REPO = Path(__file__).resolve().parents[2]
CS2_SIM = Path("/home/georg/Desktop/crazyswarm2/crazyflie_sim")
FW_BUILD = Path("/home/georg/Desktop/crazyflie-firmware/build")
HOST = REPO / "flying_drone_stack/firmware_app/host"

for p in (HOST, FW_BUILD, CS2_SIM):
    sys.path.insert(0, str(p))

import cffirmware as firm  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL  # noqa: E402
from crazyflie_sim import sim_data_types  # noqa: E402
from crazyflie_sim.sim_data_types import State  # noqa: E402
from indi_sil_package_ns2 import NeuralSwarm  # noqa: E402
from indi_sil_package_plant import Quadrotor  # noqa: E402
from indi_delay_margin_metrics import summarize_trace  # noqa: E402

NS2_DATA = CS2_SIM / "crazyflie_sim/backend/data/neuralswarm2"

BASE_INDI = dict(
    kr=2400.0, kw=170.0, kr_z=2400.0, kw_z=170.0, fc_bw=206.0, fc_bw_yaw=206.0,
    mass=0.041, kt1=4.1623e-10, kt2=4.0592e-10, kt3=4.1116e-10, kt4=4.0631e-10,
    ff_free=0, filt_order=1, filt_tau=1, j_scale=1.0, clamp_en=11,
    tau_xy_max=0.045, tau_z_max=0.0025, tilt_max_deg=30.0, thrust_max=0.8,
    notch_en=0, filt_dt_us=1000, filt_prewarp=1,
)


class Lpf2p:
    def __init__(self, fc_hz: float, fs_hz: float = 1000.0):
        b, a = signal.butter(2, fc_hz / (0.5 * fs_hz), btype="low")
        self.b, self.a = b, a
        self.zi = signal.lfilter_zi(b, a)

    def update(self, x: float) -> float:
        y, self.zi = signal.lfilter(self.b, self.a, [x], zi=self.zi)
        return float(y[0])


class SensorPath:
    def __init__(self, cfg: dict, rng: np.random.Generator):
        self.gyro_lpf_hz = float(cfg.get("gyro_lpf_hz", 0) or 0)
        self.acc_lpf_hz = float(cfg.get("acc_lpf_hz", 0) or 0)
        self.sensor_delay_s = float(cfg.get("sensor_delay_ms", 0)) * 1e-3
        self.gyro_noise_deg = float(cfg.get("gyro_noise_deg_s", 0))
        self.rng = rng
        self.gyro_f = [Lpf2p(self.gyro_lpf_hz) for _ in range(3)] if self.gyro_lpf_hz > 0 else None
        self.acc_f = [Lpf2p(self.acc_lpf_hz) for _ in range(3)] if self.acc_lpf_hz > 0 else None
        self.hist: deque = deque(maxlen=5000)

    def filter_state(self, t: float, omega_rad: np.ndarray, acc_body: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        self.hist.append((t, omega_rad.copy(), acc_body.copy()))
        t_read = t - self.sensor_delay_s
        om, ac = omega_rad, acc_body
        for ts, o, a in reversed(self.hist):
            if ts <= t_read:
                om, ac = o, a
                break
        if self.gyro_f:
            om = np.array([f.update(v) for f, v in zip(self.gyro_f, om)])
        if self.acc_f:
            ac = np.array([f.update(v) for f, v in zip(self.acc_f, ac)])
        if self.gyro_noise_deg > 0:
            om = om + np.deg2rad(self.rng.normal(0, self.gyro_noise_deg, size=3))
        return om, ac


class CmdDelay:
    """Fractional command dead time: linear interpolation on (t, rpm) history at 1 kHz."""

    METHOD = "linear_interp_on_rpm_history"

    def __init__(self, delay_ms: float):
        self.delay_s = float(delay_ms) * 1e-3
        self.hist: list[tuple[float, np.ndarray]] = []

    def push(self, t: float, rpm: np.ndarray) -> None:
        self.hist.append((t, np.asarray(rpm, dtype=float)))
        if len(self.hist) > 4000:
            self.hist.pop(0)

    def read(self, t: float) -> np.ndarray:
        if self.delay_s <= 0 or not self.hist:
            return self.hist[-1][1] if self.hist else np.zeros(4)
        t_q = t - self.delay_s
        if t_q <= self.hist[0][0]:
            return self.hist[0][1]
        for i in range(len(self.hist) - 1, 0, -1):
            t0, r0 = self.hist[i - 1]
            t1, r1 = self.hist[i]
            if t0 <= t_q <= t1:
                a = (t_q - t0) / (t1 - t0 + 1e-12)
                return (1 - a) * r0 + a * r1
        return self.hist[-1][1]


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


def apply_plant_asym(q: Quadrotor, rpm_applied: np.ndarray, dt: float, spool_asym: float):
    if spool_asym == 1.0 or not q.motor_tau:
        return sim_data_types.Action(rpm_applied.tolist())
    if q._rpm is None:
        q._rpm = rpm_applied.copy()
    alpha_up = dt / (q.motor_tau + dt)
    alpha_dn = dt / (q.motor_tau * spool_asym + dt)
    dr = rpm_applied - q._rpm
    alpha = np.where(dr >= 0, alpha_up, alpha_dn)
    q._rpm = q._rpm + alpha * dr
    return sim_data_types.Action(q._rpm.tolist())


def run_episode(cfg: dict) -> dict:
    lat = cfg.get("latency", {})
    delay_ms = float(lat.get("cmd_dead_ms", 0))
    seed = int(cfg.get("seed", 0))
    sensor_cfg = {
        "gyro_lpf_hz": lat.get("gyro_lpf_hz", 0),
        "acc_lpf_hz": lat.get("acc_lpf_hz", 0),
        "sensor_delay_ms": lat.get("sensor_delay_ms", 0),
        "gyro_noise_deg_s": lat.get("gyro_noise_deg_s", 0),
    }
    spool_asym = float(lat.get("spool_asym", 1.0))
    bottom_ctrl = cfg.get("bottom_controller", "oot")
    duration = float(cfg.get("duration_s", 12.0))
    dt = 1e-3
    z_bot = float(cfg.get("z_bottom", 0.5))
    dz = float(cfg.get("dz", 0.5))

    firm.controllerOutOfTreeInit()
    CrazyflieSIL._oot_count = 0
    top_cfg = {"ctrl_mode": 0, "kp_xy": 64.0, "kp_z": 48.0, "kv_xy": 8.0, "kv_z": 7.0, "ki_z": 0.0, "res_sign": 1}
    if bottom_ctrl == "oot":
        apply_oot_gains(firm.cvar, cfg)

    ph = dict(
        mass=0.041,
        kt=[BASE_INDI["kt1"]] * 4,
        arm_length=float(firm.oot_arm_length()),
        t2t=float(firm.oot_thrust2torque()),
        inertia=[firm.oot_inertia(i) for i in range(3)],
        motor_tau=0.044,
    )
    ns = NeuralSwarm(NS2_DATA)
    cmd_delay = CmdDelay(delay_ms)
    sensor = SensorPath(sensor_cfg, np.random.default_rng(seed))

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

    quads = [Quadrotor(State(pos=np.zeros(3)), ph) for _ in range(2)]
    logs = {"t": [], "gyro": [], "roll": [], "pitch": [], "pos": [], "partner_gyro": []}

    for k in range(1, int(duration / dt) + 1):
        t_now = k * dt
        t_box[0] = t_now
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
            st = quads[i].state
            if i == 0:
                om_f, acc_f = sensor.filter_state(t_now, st.omega, st.acc)
                cf.setState(st)
                cf.sensors.gyro.x = np.degrees(om_f[0])
                cf.sensors.gyro.y = np.degrees(om_f[1])
                cf.sensors.gyro.z = np.degrees(om_f[2])
                cf.sensors.acc.x = acc_f[0]
                cf.sensors.acc.y = acc_f[1]
                cf.sensors.acc.z = acc_f[2]
                cf.motors_rpm_meas = [int(x) for x in st.rpm]
            else:
                cf.setState(st)
            actions.append(cf.executeController())

        fa_data = [("small", torch.hstack((torch.tensor(q.state.pos), torch.tensor(q.state.vel)))) for q in quads]
        for i, (q, act) in enumerate(zip(quads, actions)):
            rpm_cmd = np.asarray(act.rpm, dtype=float)
            if i == 0:
                cmd_delay.push(t_now, rpm_cmd)
                rpm_applied = cmd_delay.read(t_now)
            else:
                rpm_applied = rpm_cmd
            act_d = apply_plant_asym(q, rpm_applied, dt, spool_asym if i == 0 else 1.0)
            f_a = ns.compute_Fa(fa_data[i], fa_data[:i] + fa_data[i + 1 :]) / 1000.0 * 9.81
            q.step(act_d, dt, f_a)

        if k % 2 == 0:
            q0, q1 = quads[0].state, quads[1].state
            roll, pitch, _ = rowan.to_euler(q0.quat)
            logs["t"].append(t_now)
            logs["pos"].append(q0.pos.copy())
            logs["gyro"].append(np.degrees(q0.omega))
            logs["partner_gyro"].append(np.degrees(q1.omega))
            logs["roll"].append(np.degrees(roll))
            logs["pitch"].append(np.degrees(pitch))

    t = np.asarray(logs["t"])
    metrics = summarize_trace(
        t, np.asarray(logs["gyro"]), np.asarray(logs["roll"]), np.asarray(logs["pitch"]), np.asarray(logs["pos"]), z_bot
    )
    pg = np.asarray(logs["partner_gyro"])
    partner = summarize_trace(t, pg, np.zeros_like(t), np.zeros_like(t), np.asarray(logs["pos"]), z_bot + dz)
    partner_ok = bool(np.isfinite(partner.get("gyro_rms_deg_s", float("nan"))))
    return {
        "label": cfg.get("label"),
        "config": cfg,
        "metrics": metrics,
        "partner_metrics": partner,
        "partner_ok": partner_ok,
        "cmd_dead_method": CmdDelay.METHOD,
        "latency": lat,
    }


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
