#!/usr/bin/env python3
"""FBL against S0 and S2 on more scenarios (A8 speeds x dz, A2, A3, A5, R1 seeds) in the calibrated plant (thrust 97.7 %, downwash 1.3x, measured peer feed). out/stage3/scenario_matrix.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/scenario_matrix.py"""
import json, sys, multiprocessing as mp
from pathlib import Path
HERE = Path(__file__).parent
CASES = [("A8 dz0.25 v0.1", "A8", dict(dz=0.25, speed=0.1)), ("A8 dz0.25 v0.3", "A8", dict(dz=0.25, speed=0.3)), ("A8 dz0.25 v0.5", "A8", dict(dz=0.25, speed=0.5)),
         ("A8 dz0.5 v0.1", "A8", dict(dz=0.5, speed=0.1)), ("A8 dz0.5 v0.3", "A8", dict(dz=0.5, speed=0.3)), ("A8 dz0.5 v0.5", "A8", dict(dz=0.5, speed=0.5)),
         ("A8 dz0.75 v0.1", "A8", dict(dz=0.75, speed=0.1)), ("A8 dz0.75 v0.3", "A8", dict(dz=0.75, speed=0.3)), ("A8 dz0.75 v0.5", "A8", dict(dz=0.75, speed=0.5)),
         ("A2 circle v0.3", "A2", dict(speed=0.3)), ("A2 circle v0.5", "A2", dict(speed=0.5)), ("A3 v0.3", "A3", dict(speed=0.3)), ("A3 v0.5", "A3", dict(speed=0.5)), ("A5 circle", "A5", {})] \
        + [(f"R1 seed {sd}", "R1", dict(seed=sd)) for sd in (101, 102, 103, 104, 105)]
def build(kind, kw):
    sys.argv = sys.argv[:1]; sys.path.insert(0, str(HERE)); import c_in_the_loop_real as C
    FSC = C.FSC
    if kind == "A8": return FSC.A8(dz=kw["dz"], span=1.0, duration=1.875 / kw["speed"], settle=2.0, passes=2)
    if kind == "A2": return FSC.A2(dz=0.5, path="circle", speed=kw["speed"], radius=0.4, period=2 * 3.14159 * 0.4 / kw["speed"], laps=2.0)
    if kind == "A3": return FSC.A3(dz=0.5, speed=kw["speed"], length=1.2, passes=4)
    if kind == "A5": return FSC.A5(dz=0.5, radius=0.4, period=7.5, laps=2.0)
    if kind == "R1": return FSC.R1(seed=kw["seed"])
def job(a):
    name, kind, kw, mode = a
    sys.argv = sys.argv[:1]; sys.path.insert(0, str(HERE)); import c_in_the_loop_real as C
    R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977)
    try:
        r = C.run(mode, kw.get("speed", 0.5), real=R, fscale=1.3, peer=dict(emp=1, noise_m=0.001, lat_ms=0), sc_obj=build(kind, kw), dz=kw.get("dz", 0.5))
        return name, mode, dict(z_rms=r["z_rms_cm"], z_dip=r["z_dip_cm"], tilt=r["tilt_max_deg"], tripped=r["tripped"], diverged=r["diverged"], gate_s=r["gate_s"])
    except Exception as e: return name, mode, dict(error=repr(e)[:200])
if __name__ == "__main__":
    jobs = [(n, k, kw, m) for n, k, kw in CASES for m in ("omar", "s2", "fbl_rom")]
    out = {}
    with mp.get_context("spawn").Pool(6) as pool:
        for name, mode, r in pool.imap_unordered(job, jobs):
            out[f"{name}|{mode}"] = r; print(name, mode, {k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()}, flush=True)
    json.dump(out, open(HERE.parent / "out/stage3/scenario_matrix.json", "w"), indent=1)
