#!/usr/bin/env python3
"""Does a constant body-torque bias (as measured on cf5) reproduce the 2026-10-24 hardware behaviour of FBL (yaw drift, slow lateral error), and does an inner-loop
integral (as in the stock Crazyflie rate PID the authors flew through) remove it? Real firmware controller (controller_omar_indi.c, fbl.en 1, via host bindings),
plant of c_in_the_loop_real.py (delay 4 ms, motor lag 44 ms, noise), NS2 v2 as downwash truth, _kp gains (the frozen proposal), C_D 0.55.
Torque bias: the plant gets d = -tau_hold, where tau_hold is the mean torque the controllers had to supply on cf5 (S0 / FBL logs 2026-10-23/24, docs/fbl/HARDWARE_REVIEW_2026-10-24.md).
Inner-loop variants are emulated in this wrapper (added torque after the controller), not in the C code:
  rateI : torque += J * kr * KI_RATE * integral(w_des - w)        (stock CF2.1 rate PID: ki/kp = 500/250 = 2 /s roll/pitch; yaw 16.7/120)
  yawA  : torque_z += J_z * kr_z * KP_YAW * (psi_des - psi)        (stock: yaw-rate command integrated into a yaw setpoint, attitude kp 6)
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/torque_bias_sim.py"""
import json, os, sys
from pathlib import Path
from unittest.mock import MagicMock
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
fw = C.fw; M, G, J = C.M, C.G, C.J
GAINS = dict(kp_xy=9.8, kp_z=11.25, kv_xy=6.65, kv_z=5.4, ki_xy=8.4, ki_z=12.0, kr_xy=59.5, kr_z=35.0)
KI_RATE = np.array([2.0, 2.0, 16.7 / 120.0]); KP_YAW = 6.0
TAU_HOLD = np.array([80e-6, -300e-6, -30e-6])          # N m, typical mean torque supplied on cf5 (FBL A1/A4 preset 1: +81 / -296 / -36; S0 A4: tau_y -320..-490)

def run(scen, bias=False, rateI=False, yawA=False, seed=0):
    rng = np.random.default_rng(seed); real = dict(delay_ms=4.0, motor_tau_ms=44.0, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005)
    sc = C.FSC.A1(dz=0.45, hold=15.0) if scen == "A1" else C.FSC.A4(dz=0.4, offset=0.125, axis='y', motion='line', speed=0.35); anchor = np.array([0, 0, 0.5]); T = min(r.curve.duration for r in sc.robots)
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = M; omar.indi = 0
    for k, v in GAINS.items(): setattr(omar.fbl, k, v)
    fw.oot_set_peer_count(1)       # host peer feed (was missing before 2026-10-25: residual off)
    omar.fbl.inner = 0; omar.fbl.nn_en = 0   # this script emulates its variants on top of the flown Eq. 19 loop (firmware default is inner 1 since 2026-10-25)
    omar.fbl.en = 1; fw.cvar.g_rnn_div = 10; fw.cvar.g_rnn_en = 0; omar.fbl.c_d = 0.55
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    p0 = C.kin(sc, 0, 0.0, anchor)[0]; x, v, q, w = p0.copy(), np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3)
    kr = np.array([GAINS["kr_xy"], GAINS["kr_xy"], GAINS["kr_z"]]); Jd = np.diag(J)
    dt = 1e-3; n = int(T / dt); thrust = M * G; tau = np.zeros(3); cmd_q = []; thr_c = M * G; tau_c = np.zeros(3); fext = np.zeros(3); ierr = np.zeros(3); rec = []
    for i in range(n):
        t = i * dt; tp, tv, *_ = C.kin(sc, 1, t, anchor)
        if i % C.PEER_EVERY == 0: fw.oot_set_peer(0, float(tp[0]), float(tp[1]), float(tp[2]), int(i))
        pr, vr, ar, jr, _ = C.kin(sc, 0, t, anchor)
        sp.position.x, sp.position.y, sp.position.z = map(float, pr); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vr)
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ar); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jr)
        xm = x + rng.normal(0, real['pos_noise_m'], 3); vm = v + rng.normal(0, real['vel_noise'], 3)
        st.position.x, st.position.y, st.position.z = map(float, xm); st.velocity.x, st.velocity.y, st.velocity.z = map(float, vm)
        st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
        gm = w + np.radians(rng.normal(0, real['gyro_noise_dps'], 3)); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, np.degrees(gm))
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
        u = np.array([ctl.torqueX, ctl.torqueY, ctl.torqueZ])
        if omar.fbl.active:
            wd = np.array([omar.fbl.w_des.x, omar.fbl.w_des.y, omar.fbl.w_des.z])
            if rateI: ierr += (wd - gm) * dt; u = u + Jd * kr * KI_RATE * ierr
            if yawA: u[2] += Jd[2] * kr[2] * KP_YAW * (0.0 - Rot.from_quat(q).as_euler("xyz")[2])
        else: ierr[:] = 0
        if i % 2 == 0 and np.isfinite([ctl.thrustSi, *u]).all():
            cmd_q.append((t + real['delay_ms'] * 1e-3, float(np.clip(ctl.thrustSi, 0, C.FMAX)), np.clip(u, -C.TAUMAX, C.TAUMAX)))
        while cmd_q and cmd_q[0][0] <= t: _, thr_c, tau_c = cmd_q.pop(0)
        a_m = dt / (real['motor_tau_ms'] * 1e-3 + dt); thrust += a_m * (thr_c - thrust); tau = tau + a_m * (tau_c - tau)
        if i % 2 == 0:
            rel = [(tp - x, tv - v)] if tp[2] > x[2] else []
            a_nn, _ = C.reference(C.W, rel, float(x[2]), v, M); fext = np.asarray(a_nn, float) * M
        R = Rot.from_quat(q).as_matrix(); tq = tau - (TAU_HOLD if bias else 0.0)
        v = v + (np.array([0, 0, -G]) + R[:, 2] * thrust / M + fext / M) * dt; x = x + v * dt
        w = w + np.linalg.solve(J, tq - np.cross(w, J @ w)) * dt; q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        if i % 10 == 0: rec.append([t, *(x - pr), np.degrees(Rot.from_quat(q).as_euler("xyz")[2]), np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), omar.fbl.tripped])
    r = np.array(rec); win = r[:, 0] > 2.0; rms = lambda a: 100 * float(np.sqrt(np.mean(a[win] ** 2)))
    return dict(x_rms_cm=rms(r[:, 1]), y_rms_cm=rms(r[:, 2]), z_rms_cm=rms(r[:, 3]), yaw_end_deg=float(r[-1, 4]), yaw_absmax_deg=float(np.abs(r[:, 4]).max()),
                tilt_rms_deg=float(np.sqrt(np.mean(r[win, 5] ** 2))), tripped=bool(r[:, 6].max())), r

if __name__ == "__main__":
    out = {}; traces = {}
    for scen in ("A4", "A1"):
        for name, kw in (("no bias", {}), ("bias", dict(bias=True)), ("bias + rate I", dict(bias=True, rateI=True)), ("bias + rate I + yaw angle", dict(bias=True, rateI=True, yawA=True))):
            m, r = run(scen, **kw); out[f"{scen}|{name}"] = m; traces[f"{scen}|{name}"] = r
            print(f"{scen} {name:26s} " + "  ".join(f"{k} {v:.2f}" if isinstance(v, float) else f"{k} {v}" for k, v in m.items()), flush=True)
    d = Path(__file__).parents[1] / "out/stage3"; json.dump(out, open(d / "torque_bias_sim.json", "w"), indent=1); np.savez(d / "torque_bias_sim_traces.npz", **{k.replace("|", "__").replace(" ", "_"): v for k, v in traces.items()})
