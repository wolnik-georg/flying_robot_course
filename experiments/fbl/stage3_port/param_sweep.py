#!/usr/bin/env python3
"""Parameter-uncertainty sweep (Monte Carlo) around the first flight: plant mass +-5 %, inertia +-20 %, motor lag 30-60 ms, command delay 2-8 ms, thrust factor 0.95-1.04, downwash 1.0-1.5x the prediction,
FBL's own C_D 0.3-0.8; same draws for FBL and S2, scenarios A8 (dz 0.5, 0.5 m/s) and A1. out/stage3/param_sweep.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/param_sweep.py [N]"""
import json, sys, multiprocessing as mp
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent; N = int(sys.argv[1]) if len(sys.argv) > 1 else 60
def draw(i):
    r = np.random.default_rng(1000 + i)
    return dict(mass=r.uniform(0.95, 1.05), J=r.uniform(0.8, 1.2), lag=r.uniform(30, 60), delay=r.uniform(2, 8), eff=r.uniform(0.95, 1.04), fscale=r.uniform(1.0, 1.5), cd=r.uniform(0.3, 0.8))
def job(a):
    i, scen, mode = a; p = draw(i)
    sys.argv = sys.argv[:1]; sys.path.insert(0, str(HERE)); import c_in_the_loop_real as C
    C.CD_FIT = p["cd"]
    R = dict(delay_ms=p["delay"], motor_tau_ms=p["lag"], gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=p["eff"])
    try:
        r = C.run(mode, 0.5, real=R, fscale=p["fscale"], scen=scen, peer=dict(emp=1, noise_m=0.001, lat_ms=0), plant=dict(mass_scale=p["mass"], J_scale=p["J"]), seed=i)
        return i, scen, mode, dict(p, z_rms=r["z_rms_cm"], z_dip=r["z_dip_cm"], z_mean=r["z_mean_cm"], tilt=r["tilt_max_deg"], tripped=r["tripped"], diverged=r["diverged"])
    except Exception as e: return i, scen, mode, dict(p, error=repr(e)[:200])
if __name__ == "__main__":
    jobs = [(i, s, m) for i in range(N) for s in ("A8", "A1") for m in ("s2", "fbl_rom")]; out = []
    with mp.get_context("spawn").Pool(6) as pool:
        for k, (i, s, m, r) in enumerate(pool.imap_unordered(job, jobs)):
            out.append(dict(i=i, scen=s, mode=m, **r))
            if k % 20 == 0: print(k, "of", len(jobs), flush=True)
    json.dump(out, open(HERE.parent / "out/stage3/param_sweep.json", "w"), indent=1); print("done", len(out))
