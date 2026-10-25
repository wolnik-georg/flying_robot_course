#!/usr/bin/env python3
"""Gate 1 (second part) of the FBL stock inner loop: with ctrlFbl.inner = 0, nn_en = 0 (the 10-24 controller; set explicitly, newer builds default to 1) FBL and the S0-S3 paths of controller 9 must be BIT-IDENTICAL
to the build before the inner-loop change. Deterministic input sequences (state, gyro, setpoint, one peer above for the FBL residual and NS2) go into
controllerOmarIndi; every tick's control output (mode, thrustSi, torque, legacy roll / pitch / yaw / thrust) and the FBL / Omar log state are hashed.
Each build runs in its own process (two same-named cffirmware modules must not share a process).

  python3 flying_drone_stack/firmware_app/host/test_fbl_inner0_identical.py --old-dir <bindings before> [--new-dir ~/Desktop/crazyflie-firmware/build]
  (system python3, 3.10)"""
import argparse, hashlib, json, os, struct, subprocess, sys

CASES = {"S0 (indi 0, rnn 0)": dict(fbl=0, indi=0, rnn=0), "S1 (indi 3, rnn 0)": dict(fbl=0, indi=3, rnn=0),
         "S2 (indi 0, rnn 1)": dict(fbl=0, indi=0, rnn=1), "S3 (indi 3, rnn 1)": dict(fbl=0, indi=3, rnn=1),
         "FBL inner 0 (cd 0.55)": dict(fbl=1, indi=0, rnn=0), "FBL inner 0, preset 0": dict(fbl=1, indi=0, rnn=0, preset=0)}


def worker(build, case):
    sys.path.insert(0, build)
    import numpy as np
    import cffirmware as fw
    c = CASES[case]; rng = np.random.default_rng(7); n = 4000
    omar = fw.controllerOmarIndi_t(); fw.controllerOmarIndiInit(omar); omar.indi = c["indi"]
    fw.cvar.g_rnn_div = 10; fw.cvar.g_rnn_en = c["rnn"]; fw.oot_set_peer_count(1)   # peer feed active (FBL residual and NS2 see the peer)
    omar.fbl.en = c["fbl"]; omar.fbl.c_d = 0.55
    for k in ("inner", "nn_en", "abg_on_new"):                # the 10-24 controller: newer builds default to inner 1 / nn_en 1, set the old path explicitly
        if hasattr(omar.fbl, k): setattr(omar.fbl, k, 0)
    omar.fbl.z_on, omar.fbl.z_off, omar.fbl.tilt_max_deg, omar.fbl.w_max = 0.15, 0.10, 60.0, 20.0   # the 10-24 handover / latch / limit (defaults changed 2026-10-25)
    if "preset" in c:
        omar.fbl.kp_xy, omar.fbl.kp_z, omar.fbl.kv_xy, omar.fbl.kv_z, omar.fbl.ki_xy, omar.fbl.ki_z, omar.fbl.kr_xy, omar.fbl.kr_z = 56.0, 45.0, 38.0, 21.6, 48.0, 3.0, 238.0, 140.0
    sp, sens, st, ctl = fw.setpoint_t(), fw.sensorData_t(), fw.state_t(), fw.control_t()
    sp.mode.x = sp.mode.y = sp.mode.z = fw.modeAbs; sp.mode.yaw = fw.modeAbs; sp.attitude.yaw = 0.0
    h = hashlib.sha256(); z = 0.05; vz = 0.0; att = np.zeros(3); g = np.zeros(3); p = np.zeros(2)
    for k in range(n):
        t = k * 0.002
        z = min(0.55, z + 0.0006) if k < 1500 else 0.55 + 0.03 * np.sin(t)          # climb through z_on (handover), then hold
        att = 0.995 * att + rng.normal(0, 0.3, 3); g = 0.9 * g + rng.normal(0, 3.0, 3); p = 0.999 * p + rng.normal(0, 0.002, 2)
        r_, p_, y_ = np.radians(att); cr, sr, cp, spp, cy, sy = np.cos(r_ / 2), np.sin(r_ / 2), np.cos(p_ / 2), np.sin(p_ / 2), np.cos(y_ / 2), np.sin(y_ / 2)
        st.attitudeQuaternion.w = float(cr * cp * cy + sr * spp * sy); st.attitudeQuaternion.x = float(sr * cp * cy - cr * spp * sy)
        st.attitudeQuaternion.y = float(cr * spp * cy + sr * cp * sy); st.attitudeQuaternion.z = float(cr * cp * sy - sr * spp * cy)
        st.attitude.roll, st.attitude.pitch, st.attitude.yaw = map(float, att)
        st.position.x, st.position.y, st.position.z = float(p[0]), float(p[1]), float(z)
        st.velocity.x, st.velocity.y, st.velocity.z = float(rng.normal(0, 0.05)), float(rng.normal(0, 0.05)), float(rng.normal(0, 0.05))
        st.acc.x, st.acc.y, st.acc.z = float(rng.normal(0, 0.1)), float(rng.normal(0, 0.1)), float(rng.normal(0, 0.1))
        sens.gyro.x, sens.gyro.y, sens.gyro.z = map(float, g)
        sens.acc.x, sens.acc.y, sens.acc.z = float(rng.normal(0, 0.02)), float(rng.normal(0, 0.02)), float(1 + rng.normal(0, 0.02))
        sp.position.x, sp.position.y, sp.position.z = 0.0, float(0.1 * np.sin(0.5 * t)), 0.55
        sp.velocity.y = float(0.05 * np.cos(0.5 * t)); sp.acceleration.y = float(-0.025 * np.sin(0.5 * t))
        if k % 5 == 0: fw.oot_set_peer(0, float(0.02 * np.sin(t)), float(0.1 * np.sin(0.5 * t)), 1.0, int(2 * k))
        fw.controllerOmarIndi(omar, ctl, sp, sens, st, 2 * k)
        h.update(struct.pack("<i4f3hf", int(ctl.controlMode), ctl.thrustSi, ctl.torqueX, ctl.torqueY, ctl.torqueZ, ctl.roll, ctl.pitch, ctl.yaw, ctl.thrust))
        h.update(struct.pack("<2B2I4f", omar.fbl.active, omar.fbl.tripped, omar.fbl.steps, omar.fbl.faults, omar.fbl.res_az, omar.fbl.thrust, omar.fbl.w_des.x, omar.fbl.w_des.z))
        h.update(struct.pack("<4f", omar.thrustSi, omar.u.x, omar.u.y, omar.u.z))
    print(json.dumps(dict(sha=h.hexdigest(), steps=int(omar.fbl.steps))))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--old-dir"); ap.add_argument("--new-dir", default=os.path.expanduser("~/Desktop/crazyflie-firmware/build"))
    ap.add_argument("--worker", nargs=2); a = ap.parse_args()
    if a.worker: worker(*a.worker); sys.exit(0)
    ok = True
    for case in CASES:
        res = [json.loads([l for l in subprocess.run([sys.executable, __file__, "--worker", d, case], capture_output=True, text=True, check=True).stdout.splitlines() if l.startswith("{")][-1]) for d in (a.old_dir, a.new_dir)]
        same = res[0]["sha"] == res[1]["sha"]; ok &= same
        print(f"  {'PASS' if same else 'FAIL'}  {case:24s} sha256 old {res[0]['sha'][:12]} new {res[1]['sha'][:12]}  FBL steps {res[0]['steps']} / {res[1]['steps']}")
    print("RESULT: ALL IDENTICAL" if ok else "RESULT: DIFFERENT"); sys.exit(0 if ok else 1)
