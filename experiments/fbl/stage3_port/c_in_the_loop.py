#!/usr/bin/env python3
"""C-in-the-loop check of the integrated FBL branch (G5 first step): the REAL firmware controller (controller_omar_indi.c with fbl.en, via the host bindings)
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
M, G = 0.0427, 9.81; J = np.diag([2.395e-5, 2.395e-5, 3.235e-5]); FMAX = 2.0 * M * G; TAUMAX = 0.02
W = np.asarray(np.load(REPO / "flying_drone_stack/tools/residual/weights/ns2_v2.npz")["weights"], np.float32)

def kin(sc, i, t, anchor):
    rp = sc.robots[i]; d = poly4d._derivs(rp.curve, min(t, rp.curve.duration)); return anchor + rp.slot + d[0, :3], d[1, :3], d[2, :3], d[3, :3], d[0, 3]

def run(mode, speed, dz=0.5):
    sc = FSC.A8(dz=dz, span=1.0, duration=1.875 / speed, settle=2.0, passes=2); anchor = np.array([0, 0, 0.5]); T = min(r.curve.duration for r in sc.robots)
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.mass = M; omar.indi = 0
    omar.fbl.en = 0 if mode == "omar" else 1; omar.fbl.c_d = 0.43 if mode == "fbl_rom" else 0.0
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    fw.oot_set_peer_count(1)
    p0, *_ = kin(sc, 0, 0.0, anchor); x, v, q, w = p0.copy(), np.zeros(3), np.array([0, 0, 0, 1.0]), np.zeros(3)
    n = int(T / 1e-3); rec = np.zeros((n, 11)); dt = 1e-3; thrust = M * G; tau = np.zeros(3); fext = np.zeros(3); sat = 0; faults = 0
    for i in range(n):
        t = i * dt; tp, tv, ta, tj, _ = kin(sc, 1, t, anchor)
        if i % 10 == 0: fw.oot_set_peer(0, float(tp[0]), float(tp[1]), float(tp[2]), int(i))
        pr, vr, ar, jr, yw = kin(sc, 0, t, anchor)
        sp.position.x, sp.position.y, sp.position.z = map(float, pr); sp.velocity.x, sp.velocity.y, sp.velocity.z = map(float, vr)
        sp.acceleration.x, sp.acceleration.y, sp.acceleration.z = map(float, ar); sp.jerk.x, sp.jerk.y, sp.jerk.z = map(float, jr)
        st.position.x, st.position.y, st.position.z = map(float, x); st.velocity.x, st.velocity.y, st.velocity.z = map(float, v)
        st.attitudeQuaternion.x, st.attitudeQuaternion.y, st.attitudeQuaternion.z, st.attitudeQuaternion.w = map(float, q)
        gd = np.degrees(w); sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, gd)
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, i)
        if i % 2 == 0:
            thrust = float(np.clip(ctl.thrustSi, 0, FMAX)); tau = np.clip([ctl.torqueX, ctl.torqueY, ctl.torqueZ], -TAUMAX, TAUMAX); sat += int(ctl.thrustSi > FMAX)
        if i % 2 == 0:
            rel = [(tp - x, tv - v)] if tp[2] > x[2] else []
            a_nn, _ = reference(W, rel, float(x[2]), v, M); fext = np.asarray(a_nn, float) * M
        R = Rot.from_quat(q).as_matrix()
        v = v + (np.array([0, 0, -G]) + R[:, 2] * thrust / M + fext / M) * dt; x = x + v * dt
        w = w + np.linalg.solve(J, tau - np.cross(w, J @ w)) * dt; q = (Rot.from_rotvec(R @ w * dt) * Rot.from_quat(q)).as_quat()
        rec[i] = [t, *x, *tp, np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))), thrust, omar.fbl.active, omar.fbl.tripped]
    ez = np.array([rec[k, 3] - kin(sc, 0, rec[k, 0], anchor)[0][2] for k in range(0, n, 10)]); r = rec[::10]
    rel = r[:, 4:7] - r[:, 1:4]; gate = (np.abs(rel[:, 0]) < 0.2) & (np.abs(rel[:, 1]) < 0.2) & (rel[:, 2] > 0)
    return dict(z_rms_cm=100 * float(np.sqrt(np.mean(ez ** 2))), z_dip_cm=100 * float(ez[gate].min()) if gate.any() else 0.0, tilt_max_deg=float(r[:, 7].max()), fbl_active_pct=100 * float(r[:, 9].mean()),
                tripped=bool(r[:, 10].max()), sat_frac=sat / (n / 2), gate_s=float(gate.sum() * 0.01), diverged=bool(np.abs(ez).max() > 0.5 or r[:, 7].max() > 80))

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--speeds", type=float, nargs="+", default=[0.1, 0.3, 0.5]); ap.add_argument("--modes", nargs="+", default=["omar", "fbl_none", "fbl_rom"]); ap.add_argument("--tag", default=None); a = ap.parse_args()
    res = {}
    for sp_ in a.speeds:
        for m in a.modes:
            r = run(m, sp_); res[f"{m}|{sp_}"] = r
            print(f"v {sp_:.1f} {m:9s} z_rms {r['z_rms_cm']:6.2f} dip {r['z_dip_cm']:7.2f} cm  tilt {r['tilt_max_deg']:5.1f}  fbl active {r['fbl_active_pct']:5.1f} %  tripped {r['tripped']}  sat {r['sat_frac']:.2f}  div {r['diverged']}", flush=True)
    if a.tag:
        out = REPO / "experiments/fbl/out/stage3" / f"c_in_the_loop_{a.tag}.json"; out.write_text(json.dumps(res, indent=1)); print("->", out)
