#!/usr/bin/env python3
"""Closed-loop SIL of controller=11 (Tal & Karaman INDI, tk_indi.rs): one drone, hover or A8 path.

Plant = CS2 numpy Quadrotor (same one the NS2 harness uses), firmware = the compiled Rust controller
through the SIL bindings. System python3:  python3 tk_indi_sil_run.py --scenario hover --seconds 12
Parameters of ctrlTk (gains, filter, jrz, yaw_lit) are overridable:  --set g_tk_fc=30 --set g_tk_jrz=0
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import numpy as np

import ns2_closed_loop_sil_sim as H   # sets up sys.path, mocks rclpy, imports cffirmware / CS2 sim
from ns2_closed_loop_sil_sim import (CrazyflieSIL, Quadrotor, State, firm, LAB_INDI, curve_to_pieces,
                                     build_scenario, ground_pos, robot_world_pose)


MIX_Z = np.array([-1.0, 1.0, -1.0, 1.0])


class RotorQuad(Quadrotor):
    """CS2 plant + the rotor reaction torque of Eq. (5) G2 (yaw row): tau_z += jrz * sum(s_i * domega_i)."""

    jrz = 0.0

    def step(self, action, dt, f_a=np.zeros(3), tau_a=None):
        prev = np.asarray(self.state.rpm, float).copy()
        super().step(action, dt, f_a, tau_a)
        if self.jrz:
            wd = (np.asarray(self.state.rpm, float) - prev) * 2 * math.pi / 60.0 / dt
            tz = self.jrz * float(MIX_Z @ wd)
            self.state.omega = self.state.omega + self.inv_J * np.array([0.0, 0.0, tz]) * dt


def run(scenario="hover", seconds=12.0, takeoff_s=4.0, sets=None, motor_tau=0.044, seed=0,
        noise=False, verbose=True, slot=(0.0, 0.0, 0.5), controller="oot6", plant_jrz=2.5e-8,
        acc_std_g=0.03, rpm_std=60.0, kwargs=None):
    sets = dict(sets or {})
    c = firm.cvar
    c.g_indi_mass = LAB_INDI["mass"]
    c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4 = (LAB_INDI[k] for k in ("kt1", "kt2", "kt3", "kt4"))
    for k, v in sets.items():
        setattr(c, k, v)
    sc = None
    if scenario != "hover":
        sc = build_scenario(scenario, {"scenario_kwargs": dict(kwargs or {})})
    anchor = np.array([0.0, 0.0, 0.5])
    slot = np.asarray(sc.robots[0].slot if sc is not None else slot, float)
    p0 = ground_pos(anchor + slot)
    t_box = [0.0]
    CrazyflieSIL._oot6_count = 0
    CrazyflieSIL._oot7_count = 0
    cf = CrazyflieSIL("cf5", p0, controller, lambda: t_box[0])
    ph = dict(mass=LAB_INDI["mass"], kt=[c.g_indi_kt1, c.g_indi_kt2, c.g_indi_kt3, c.g_indi_kt4],
              arm_length=float(firm.oot_arm_length()), t2t=float(firm.oot_thrust2torque()),
              inertia=[firm.oot_inertia(i) for i in range(3)], motor_tau=motor_tau)
    RotorQuad.jrz = plant_jrz
    quad = RotorQuad(State(pos=p0.copy()), ph)
    z_hover = float((anchor + slot)[2])
    cf.takeoff(z_hover, takeoff_s)
    rng = np.random.default_rng(seed)
    dt = 1e-3
    n = int(seconds / dt) if sc is None else int((takeoff_s + sc.duration + 1.0) / dt)
    started = False
    tl, pl, sl, ql = [], [], [], []
    crashed = None
    for k in range(1, n + 1):
        t = k * dt
        t_box[0] = t
        if sc is not None and not started and t >= takeoff_s:
            cf.uploadTrajectory(0, 0, curve_to_pieces(sc.robots[0].curve))
            cf.startTrajectory(0, timescale=1.0, relative=True)
            started = True
        cf.getSetpoint()
        st = quad.state
        if noise:   # same position / gyro noise as the NS2 harness, plus accelerometer and RPM noise
            st = H.state_for_controller(st, rng, {"meas_noise": True})
            st.acc = np.asarray(st.acc, float) + rng.normal(0.0, acc_std_g, 3)
            st.rpm = np.maximum(0.0, np.asarray(st.rpm, float) + rng.normal(0.0, rpm_std, 4))
        cf.setState(st)
        cf.sensors.acc.x, cf.sensors.acc.y, cf.sensors.acc.z = map(float, st.acc)
        cf.motors_rpm_meas = [int(x) for x in st.rpm]
        act = cf.executeController()
        quad.step(act, dt, np.zeros(3))
        if k % 10 == 0:
            tl.append(t); pl.append(quad.state.pos.copy()); ql.append(quad.state.quat.copy())
            if sc is None:
                sl.append(np.array([anchor[0] + slot[0], anchor[1] + slot[1], z_hover]) if t >= takeoff_s
                          else np.array([p0[0], p0[1], z_hover]))
            else:
                sl.append(robot_world_pose(sc, 0, max(0.0, t - takeoff_s), anchor)[0] if t >= takeoff_s
                          else np.array([p0[0], p0[1], z_hover]))
            w, x, y, z = ql[-1]
            tilt = math.degrees(math.acos(max(-1, min(1, 1 - 2 * (x * x + y * y)))))
            if not np.all(np.isfinite(pl[-1])) or tilt > 80 or (t > takeoff_s + 1 and pl[-1][2] < 0.1):
                crashed = f"t={t:.2f}s tilt={tilt:.0f}deg z={pl[-1][2]:.2f}"
                break
    t = np.array(tl); p = np.array(pl); s = np.array(sl)
    m = t > takeoff_s + 2.0
    err = p[m] - s[m] if m.any() else np.zeros((1, 3))
    q = np.array(ql)
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1)))
    res = dict(scenario=scenario, crashed=crashed, rms_xy_cm=100 * float(np.sqrt((err[:, :2] ** 2).sum(1).mean())),
               mean_dz_cm=100 * float(err[:, 2].mean()), rms_dz_cm=100 * float(np.sqrt((err[:, 2] ** 2).mean())),
               max_tilt_deg=float(tilt[m].max()) if m.any() else float(tilt.max()),
               z_final=float(p[-1, 2]), t_end=float(t[-1]))
    if verbose:
        print(json.dumps(res))
    return res, (t, p, s, q)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="hover")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--set", action="append", default=[])
    ap.add_argument("--controller", default="oot6")
    ap.add_argument("--motor-tau", type=float, default=0.044)
    ap.add_argument("--plant-jrz", type=float, default=2.5e-8)
    ap.add_argument("--noise", action="store_true")
    ap.add_argument("--kwargs", default="{}")
    a = ap.parse_args()
    sets = {kv.split("=")[0]: float(kv.split("=")[1]) for kv in a.set}
    for k in ("g_tk_yaw_lit",):
        if k in sets:
            sets[k] = int(sets[k])
    run(a.scenario, a.seconds, sets=sets, controller=a.controller, motor_tau=a.motor_tau, plant_jrz=a.plant_jrz, noise=a.noise, kwargs=json.loads(a.kwargs))
