#!/usr/bin/env python3
"""[REALISTIC PLANT VARIANT of c_in_the_loop.py: command delay, motor lag, sensor noise, thrust efficiency, gain overrides]
C-in-the-loop check of the integrated FBL branch (G5 first step): the REAL firmware controller (controller_omar_indi.c with fbl.en, via the host bindings)
in a rigid-body plant of our vehicle on the A8 swap under a scripted top vehicle; downwash truth = NS2 v2 network. Same plant and metrics as the Python Stage 2
(experiments/fbl/stage2_sim) but the controller is the C code that will fly. Compares: Omar geometric (fbl.en 0), FBL without residual (c_d 0), FBL with the ROM residual (c_d 0.43).
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/c_in_the_loop.py [--speeds 0.1 0.3 0.5] [--tag NAME]
System python3 (3.10: the host bindings are a 3.10 extension)."""
import argparse, json, os, sys
from pathlib import Path
from unittest.mock import MagicMock
import numpy as np
from scipy.spatial.transform import Rotation as Rot
for _m in ("rclpy", "rclpy.node", "rclpy.time", "rosgraph_msgs", "rosgraph_msgs.msg"): sys.modules.setdefault(_m, MagicMock())
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, os.environ.get("CFFIRMWARE_DIR", "/home/georg/Desktop/crazyflie-firmware/build")); sys.path.insert(0, str(REPO / "flying_drone_stack/firmware_app/host"))
sys.path.insert(0, "/home/georg/Desktop/crazyswarm2/crazyflie_examples")
import cffirmware as fw  # noqa: E402
from crazyflie_examples.formations import poly4d, scenarios as FSC  # noqa: E402
from test_residual_nn import reference  # noqa: E402
KICK = 12.0
PEER_EVERY = 10
FEEDFWD = True      # acceleration/jerk feedforward on the handover test ramp (the HLC trajectories of real flights have it)
M, G = 0.0427, 9.81; J = np.diag([2.395e-5, 2.395e-5, 3.235e-5]); FMAX = 2.0 * M * G; TAUMAX = 0.02
W = np.asarray(np.load(REPO / "flying_drone_stack/tools/residual/weights/ns2_v2.npz")["weights"], np.float32)

def kin(sc, i, t, anchor):
    rp = sc.robots[i]; d = poly4d._derivs(rp.curve, min(t, rp.curve.duration)); return anchor + rp.slot + d[0, :3], d[1, :3], d[2, :3], d[3, :3], d[0, 3]

def run(mode, speed, dz=0.5, real=None, gains=None, seed=0, handover=False, fault=None, scen='A8', offset=0.1, fscale=1.0, peer=None):
    real = {**dict(delay_ms=0.0, motor_tau_ms=0.0, gyro_noise_dps=0.0, pos_noise_m=0.0, vel_noise=0.0, thrust_eff=1.0), **(real or {})}; rng = np.random.default_rng(seed)
    sc = (FSC.A8(dz=dz, span=1.0, duration=1.875 / speed, settle=2.0, passes=2) if scen == 'A8' else FSC.A1(dz=dz, hold=15.0) if scen == 'A1' else FSC.A4(dz=dz, offset=offset, axis='y', motion='line', speed=speed)); anchor = np.array([0, 0, 0.5]); T = min(r.curve.duration for r in sc.robots)
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = M; omar.indi = 0
    [setattr(omar.fbl, k, v) for k, v in (gains or {}).items()]
    omar.fbl.en = 0 if mode in ("omar", "s2") else 1;
    fw.cvar.g_rnn_div = 10; fw.cvar.g_rnn_en = 1 if mode == "s2" else 0; omar.fbl.c_d = 0.43 if mode == "fbl_rom" else 0.0
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    fw.oot_set_peer_count(0 if handover else 1)
    p0, *_ = kin(sc, 0, 0.0, anchor)
    if handover: p0 = np.array([0.0, 0.0, 0.02]); T = 14.0
    x, v, q, w = p0.copy(), np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3)
    PEMP = np.load(Path(__file__).parent / 'peer_intervals.npy') if (peer and peer.get('emp')) else None
    tp_hist = []; peer_next = 0; peer_pos = None
    n = int(T / 1e-3); rec = np.zeros((n, 11)); dt = 1e-3; thrust = M * G; tau = np.zeros(3); cmd_q = []; thr_c = M * G; tau_c = np.zeros(3); fext = np.zeros(3); sat = 0; faults = 0
    for i in range(n):
        t = i * dt; tp, tv, ta, tj, _ = kin(sc, 1, t, anchor)
        tp_hist.append(tp.copy())
        if peer is None:
            if (i % PEER_EVERY == 0) and not handover: fw.oot_set_peer(0, float(tp[0]), float(tp[1]), float(tp[2]), int(i))
        elif not handover and i >= peer_next:      # measured peer feed: irregular updates (empirical intervals), latency, noise; the timestamp is the reception time [ms]
            src = tp_hist[max(0, i - int(peer.get('lat_ms', 0)))] + rng.normal(0, peer.get('noise_m', 0.0), 3)
            fw.oot_set_peer(0, float(src[0]), float(src[1]), float(src[2]), int(i))
            peer_next = i + (max(1, int(round(1000 * rng.choice(PEMP)))) if PEMP is not None else 10)
        pr, vr, ar, jr, yw = kin(sc, 0, t, anchor)
        if handover:   # smooth takeoff 0.02 -> 0.5 m (t 1..4 s), hold, smooth landing to 0.02 m (t 9..13 s)
            def sm(u): u = min(max(u, 0.0), 1.0); return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5, (30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4)
            z_up, d_up = sm((t - 1.0) / 3.0); z_dn, d_dn = sm((t - 9.0) / 4.0)
            zc = 0.02 + 0.48 * z_up - 0.48 * z_dn; vz = 0.48 * d_up / 3.0 - 0.48 * d_dn / 4.0
            def sm2(u, T):    # second and third derivative of the smoothstep (for the acceleration / jerk feedforward)
                if u <= 0.0 or u >= 1.0: return 0.0, 0.0
                return (60 * u - 180 * u ** 2 + 120 * u ** 3) / T ** 2, (60 - 360 * u + 360 * u ** 2) / T ** 3
            a_up, j_up = sm2((t - 1.0) / 3.0, 3.0); a_dn, j_dn = sm2((t - 9.0) / 4.0, 4.0)
            pr = np.array([0.0, 0.0, zc]); vr = np.array([0.0, 0.0, vz])
            ar = np.array([0.0, 0.0, 0.48 * a_up - 0.48 * a_dn]) if FEEDFWD else np.zeros(3); jr = np.array([0.0, 0.0, 0.48 * j_up - 0.48 * j_dn]) if FEEDFWD else np.zeros(3)
        sp.position.x, sp.position.y, sp.position.z = map(float, pr); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vr)
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ar); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jr)
        xm = x + rng.normal(0, real['pos_noise_m'], 3); vm = v + rng.normal(0, real['vel_noise'], 3)
        st.position.x, st.position.y, st.position.z = map(float, xm); st.velocity.x, st.velocity.y, st.velocity.z = map(float, vm)
        st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
        if fault and abs(t - 6.0) < dt / 2:
            if fault == 'kick': w = w + np.array([KICK, 0.0, 0.0])      # sudden roll rate: tilts the vehicle past the latch
        nanf = fault == 'nan' and 6.0 <= t < 6.002
        if nanf: st.position.x = float('nan')
        gd = np.degrees(w) + rng.normal(0, real['gyro_noise_dps'], 3); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, gd)
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
        if i % 2 == 0 and np.isfinite([ctl.thrustSi, ctl.torqueX, ctl.torqueY, ctl.torqueZ]).all():
            cmd_q.append((t + real['delay_ms'] * 1e-3, float(np.clip(ctl.thrustSi, 0, FMAX)), np.clip([ctl.torqueX, ctl.torqueY, ctl.torqueZ], -TAUMAX, TAUMAX))); sat += int(ctl.thrustSi > FMAX)
        while cmd_q and cmd_q[0][0] <= t: _, thr_c, tau_c = cmd_q.pop(0)
        a_m = 1.0 if real['motor_tau_ms'] <= 0 else dt / (real['motor_tau_ms'] * 1e-3 + dt)
        thrust += a_m * (thr_c * real['thrust_eff'] - thrust); tau = tau + a_m * (tau_c - tau)
        if i % 2 == 0:
            rel = [(tp - x, tv - v)] if (tp[2] > x[2] and not handover) else []
            a_nn, _ = reference(W, rel, float(x[2]), v, M); fext = np.asarray(a_nn, float) * M * fscale
        R = Rot.from_quat(q).as_matrix()
        if handover and x[2] <= 0.0: x[2] = 0.0; v[2] = max(v[2], 0.0)
        v = v + (np.array([0, 0, -G]) + R[:, 2] * thrust / M + fext / M) * dt; x = x + v * dt
        w = w + np.linalg.solve(J, tau - np.cross(w, J @ w)) * dt; q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        rec[i] = [t, *x, *tp, np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), thrust, omar.fbl.active, omar.fbl.tripped]
    if handover:
        zr = lambda t: 0.02 + 0.48 * (lambda u: 10*u**3-15*u**4+6*u**5)(min(max((t-1)/3,0),1)) - 0.48 * (lambda u: 10*u**3-15*u**4+6*u**5)(min(max((t-9)/4,0),1))
        ez = np.array([rec[k, 3] - zr(rec[k, 0]) for k in range(0, n, 10)]); r = rec[::10]; np.save(str(Path(__file__).parent / '../out/stage3') + '/handover_' + mode + ('_' + fault if fault else '') + '.npy', np.c_[r[:, 0], r[:, 3], np.array([zr(t) for t in r[:, 0]]), r[:, 7], r[:, 8], r[:, 9], r[:, 10]])
    else:
        ez = np.array([rec[k, 3] - kin(sc, 0, rec[k, 0], anchor)[0][2] for k in range(0, n, 10)]); r = rec[::10]
    rel = r[:, 4:7] - r[:, 1:4] if not handover else np.tile([5.0, 5.0, -1.0], (len(r), 1)); gate = (np.abs(rel[:, 0]) < 0.2) & (np.abs(rel[:, 1]) < 0.2) & (rel[:, 2] > 0)
    win = np.arange(len(ez)) * 0.01 > 2.0
    return dict(z_mean_cm=100 * float(ez[win].mean()), z_rms_cm=100 * float(np.sqrt(np.mean(ez ** 2))), z_dip_cm=100 * float(ez[gate].min()) if gate.any() else 0.0, tilt_max_deg=float(r[:, 7].max()), fbl_active_pct=100 * float(r[:, 9].mean()),
                tripped=bool(r[:, 10].max()), sat_frac=sat / (n / 2), gate_s=float(gate.sum() * 0.01), motor_note='delay+lag' , diverged=bool(np.abs(ez).max() > 0.5 or r[:, 7].max() > 80))

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--speeds", type=float, nargs="+", default=[0.1, 0.3, 0.5]); ap.add_argument("--modes", nargs="+", default=["omar", "fbl_none", "fbl_rom"]); ap.add_argument("--delay", type=float, default=4.0); ap.add_argument("--lag", type=float, default=20.0); ap.add_argument("--eff", type=float, default=1.0); ap.add_argument("--tag", default=None); a = ap.parse_args()
    res = {}
    for sp_ in a.speeds:
        for m in a.modes:
            r = run(m, sp_, real=dict(delay_ms=a.delay, motor_tau_ms=a.lag, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=a.eff)); res[f"{m}|{sp_}"] = r
            print(f"v {sp_:.1f} {m:9s} z_rms {r['z_rms_cm']:6.2f} dip {r['z_dip_cm']:7.2f} cm  tilt {r['tilt_max_deg']:5.1f}  fbl active {r['fbl_active_pct']:5.1f} %  tripped {r['tripped']}  sat {r['sat_frac']:.2f}  div {r['diverged']}", flush=True)
    if a.tag:
        out = REPO / "experiments/fbl/out/stage3" / f"c_in_the_loop_{a.tag}.json"; out.write_text(json.dumps(res, indent=1)); print("->", out)
