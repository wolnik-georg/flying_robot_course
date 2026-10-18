#!/usr/bin/env python3
"""Stage 2: closed loop of the authors' FBL controller (their FBLControl position law + compute_fbl_command + motor allocation, unmodified)
on a rigid-body plant with OUR vehicle constants, bottom vehicle flying A8 (vertical swap) under a scripted top vehicle.
  ~/.pyenv/versions/flying_robots/bin/python experiments/fbl/stage2_sim/s2_closed_loop.py --tag v0
Truth (the downwash that acts on the plant) is one of:  ns2 = NS2 v2 network (data-driven, z only, includes the ground term)
                                                          fit = the Stage 1 ROM+net fit (3-axis, no ground term)
Controller residual variants: none | rom_paper (C_D 0.236) | rom_fit (C_D from Stage 1) | rom_nn (Stage 1 ROM+net_f) | ns2 (NS2 net as the FBL residual).
Everything not measured on our vehicle is a labelled assumption (see ASSUME)."""
from __future__ import annotations

import argparse, json, sys, types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
for _m in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"):
    sys.modules.setdefault(_m, MagicMock())
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_examples")
import residual_providers as RP  # noqa: E402
from common import bundle as B  # noqa: E402
from common.vehicle import OUR  # noqa: E402
from crazyflie_examples.formations import poly4d, scenarios as FSC  # noqa: E402

ASSUME = dict(rotor_speed_max=2300.0,   # rad/s, ~ thrust-to-weight 2.0 (not measured)
              motor_tau=0.02,           # s first-order motor lag (not measured)
              drag_torque_ratio=0.006,  # m, yaw torque per thrust (CS2 default)
              ctrl_hz=500.0)            # as the Omar controller; at 100 Hz the bundle rate gain (238) is marginal in discrete time and a 10 ms position delay already diverges
G = 9.81


def make_controller(res_fn, a):
    b = B.load(); sys.path.insert(0, str(B.BUNDLE))
    from simulator_multi_python.Controllers.fbl_control import FBLControl

    class Ctl(FBLControl):
        def _predict_residual(self, t, state_vec, drones):
            self.last_res = res_fn(t, state_vec[:13], drones[0])
            return self.last_res

    c = Ctl(100, 10, False, False, False, 0.0, 0.0, 0, "bottom", 1)
    L, k = OUR["arm_length"], ASSUME["drag_torque_ratio"]
    c.mass, c.inertia = OUR["mass"], np.diag(OUR["inertia"])
    c.k_thrust, c.arm_length, c.rotor_speed_max = OUR["k_rpm2force"], L, a.rotor_speed_max
    c.ctrl_forces_map = np.array([[1, 1, 1, 1], [0, L, 0, -L], [-L, 0, L, 0], [k, -k, k, -k]])
    c.forces_ctrl_map = np.linalg.inv(c.ctrl_forces_map)
    hover = c.mass * G / 4
    c.forces_old = np.full(4, hover)
    return c


def kin(sc, i, t, anchor):
    rp = sc.robots[i]
    d = poly4d._derivs(rp.curve, min(t, rp.curve.duration))
    off = anchor + rp.slot
    return dict(x=off + d[0, :3], x_dot=d[1, :3], x_ddot=d[2, :3], x_dddot=d[3, :3], yaw=d[0, 3], yaw_dot=d[1, 3])


HW_POS_STD = np.array([0.0036, 0.0100, 0.0053])   # m, measured on hardware (ns2_closed_loop_sil_sim.py, A8 outside crossings)
HW_GYRO_STD = np.radians([25.6, 12.5, 2.3])        # rad/s, same source


def run(truth: str, variant: str, speed: float, a, noise='none', seed=0, delay_ms=0.0):
    span, passes = 1.0, 2
    dur = 1.875 * span / speed
    sc = FSC.A8(dz=a.dz, span=span, duration=dur, settle=2.0, passes=passes)
    anchor = np.array([0.0, 0.0, 0.5])
    T = min(sc.robots[0].curve.duration, sc.robots[1].curve.duration)
    m, J = OUR["mass"], np.diag(OUR["inertia"])
    if variant == "none": res = RP.Zero()
    elif variant == "rom_paper": res = RP.FBLResidual(0.236, False)
    elif variant == "rom_fit": res = RP.FBLResidual(a.cd_fit, False)
    elif variant == "rom_nn": res = RP.FBLResidual(a.cd_nn, True)
    elif variant == "ns2": res = RP.NS2Residual()
    else: raise ValueError(variant)
    if truth == "ns2": tr = RP.NS2Residual()
    else: tr = RP.FBLResidual(a.cd_nn, True)

    def top13(t):
        k = kin(sc, 1, t, anchor); return np.r_[k["x"], k["x_dot"], 0, 0, 0, 1, 0, 0, 0]

    holder = {}
    ctl = make_controller(lambda t, ego, _d: res(t, ego, holder["top"]), a)
    k0 = kin(sc, 0, 0.0, anchor)
    x, v, q, w = k0["x"].copy(), np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3)
    rpm = np.full(4, np.sqrt(m * G / 4 / OUR["k_rpm2force"]))
    dt, nctl = 1e-3, int(round(1 / (a.ctrl_hz * 1e-3))); cmd_rpm = rpm.copy()
    rng = np.random.default_rng(seed); hist = []
    n = int(T / dt)
    rec = np.zeros((n, 15)); f_ext = np.zeros(3); sat = 0
    for i in range(n):
        t = i * dt
        top = top13(t); holder["top"] = top
        if i % nctl == 0:
            hist.append((x.copy(), v.copy(), q.copy(), w.copy()))
            xd, vd, _q, _w = hist[max(0, len(hist) - 1 - int(round(delay_ms / (1000.0 / a.ctrl_hz))))]   # position / velocity (mocap + EKF) are delayed; attitude and gyro are onboard, undelayed
            qd, wd = q, w
            xm = xd + (rng.normal(0, HW_POS_STD) if noise in ('pos', 'both') else 0)
            wm = wd + (rng.normal(0, HW_GYRO_STD) if noise in ('gyro', 'both') else 0)
            state = dict(x=xm, v=vd, q=qd, w=wm, u=0.0)
            out = ctl.update(t, state, kin(sc, 0, t, anchor), [{"name": "bottom", "state": np.zeros(14)}, {"name": "top", "state": np.r_[top, 0]}], None, None, 1)
            cmd_rpm = out[0][:4]
            sat += int(np.any(cmd_rpm >= a.rotor_speed_max - 1e-6))
        if i % 2 == 0:  # truth downwash every 2 ms
            ego = np.r_[x, v, q, w]
            acc_t = tr(t, ego, top)[0] if truth == "fit" else tr.acc(ego, top)
            f_ext = acc_t * m if (truth == "ns2" or top[2] > x[2]) else np.zeros(3)
        rpm = rpm + dt / (ASSUME["motor_tau"] + dt) * (cmd_rpm - rpm)
        f = OUR["k_rpm2force"] * rpm ** 2
        Mmap = ctl.ctrl_forces_map; eta = Mmap @ f
        R = Rot.from_quat(q).as_matrix()
        v = v + (np.array([0, 0, -G]) + R[:, 2] * eta[0] / m + f_ext / m) * dt
        x = x + v * dt
        w = w + np.linalg.solve(J, eta[1:] - np.cross(w, J @ w)) * dt
        q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        rec[i] = np.r_[t, x, v, top[:3], np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), eta[0], f_ext]
    return sc, rec, sat / (n / nctl)


def metrics(sc, rec, anchor=np.array([0, 0, 0.5])):
    t = rec[:, 0]; ref = np.array([kin(sc, 0, tt, anchor)["x"] for tt in t[::10]]); r = rec[::10]
    err = r[:, 1:4] - ref
    rel = r[:, 7:10] - r[:, 1:4]
    gate = (np.abs(rel[:, 0]) < 0.2) & (np.abs(rel[:, 1]) < 0.2) & (rel[:, 2] > 0)
    cross = np.abs(rel[:, 1]) < 0.1
    ez = err[:, 2]
    return dict(z_rms_cm=100 * float(np.sqrt(np.mean(ez ** 2))), xy_rms_cm=100 * float(np.sqrt(np.mean(err[:, :2] ** 2))),
                z_dip_cm=100 * float(ez[gate].min()) if gate.any() else 0.0, z_peak_up_cm=100 * float(ez[gate].max()) if gate.any() else 0.0,
                z_dip_cross_cm=100 * float(ez[cross & gate].min()) if (cross & gate).any() else 0.0,
                gate_s=float(gate.sum() * 0.01), max_tilt_deg=float(r[:, 10].max()), min_z_m=float(r[:, 3].min()), diverged=bool(np.abs(err).max() > 0.5))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--tag", required=True); p.add_argument("--dz", type=float, default=0.5)
    p.add_argument("--speeds", type=float, nargs="+", default=[0.1, 0.3, 0.5])
    p.add_argument("--truths", nargs="+", default=["ns2", "fit"])
    p.add_argument("--variants", nargs="+", default=["none", "rom_paper", "rom_fit", "rom_nn", "ns2"])
    p.add_argument("--rotor-speed-max", type=float, default=ASSUME["rotor_speed_max"], dest="rotor_speed_max")
    p.add_argument("--ctrl-hz", type=float, default=ASSUME["ctrl_hz"], dest="ctrl_hz")
    p.add_argument("--noise", choices=["none", "pos", "gyro", "both"], default="none", help="hardware position / gyro noise on the controller inputs")
    p.add_argument("--delay-ms", type=float, default=0.0, dest="delay_ms", help="measurement delay of the controller inputs (assumption)")
    a = p.parse_args()
    c = __import__("torch").load(RP.CKPT, map_location="cpu", weights_only=False); a.cd_nn = c["c_d"]
    a.cd_fit = 0.43   # ROM-only drag coefficient, median over the Stage 1 folds (range 0.426-0.434)
    out = HERE.parent / "out" / "stage2" / a.tag
    if out.exists(): sys.exit(f"{out} exists; choose a new --tag")
    out.mkdir(parents=True)
    res = {"args": vars(a), "assume": ASSUME, "runs": {}}
    for tr in a.truths:
        for sp in a.speeds:
            for var in a.variants:
                sc, rec, sat = run(tr, var, sp, a, a.noise, delay_ms=a.delay_ms)
                mm = metrics(sc, rec); mm["sat_frac"] = sat
                res["runs"][f"{tr}|{sp}|{var}"] = mm
                print(f"truth {tr:3s} v {sp:.1f} {var:9s} z_rms {mm['z_rms_cm']:5.2f} dip {mm['z_dip_cm']:6.2f} cross {mm['z_dip_cross_cm']:6.2f} up {mm['z_peak_up_cm']:5.2f} tilt {mm['max_tilt_deg']:4.1f} sat {sat:.2f} div {mm['diverged']}", flush=True)
                np.save(out / f"{tr}_{sp}_{var}.npy", rec[::10].astype(np.float32))
    json.dump(res, open(out / "results.json", "w"), indent=1)


if __name__ == "__main__":
    main()
