#!/usr/bin/env python3
"""Battery sag in the calibrated plant: (a) thrust per command fades linearly over the flight by 0 / 5 / 10 / 15 %, (b) load-dependent drop (0.15 per unit of thrust above weight);
A1, A8 and R1 (seed 101, 30 s), S0 / S2 / FBL. out/stage3/battery_sag.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/battery_sag.py"""
import json, sys, multiprocessing as mp
from pathlib import Path
HERE = Path(__file__).parent
def job(a):
    kind, val, scen, mode = a
    sys.argv = sys.argv[:1]; sys.path.insert(0, str(HERE)); import c_in_the_loop_real as C
    R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977, **({"sag_t": val} if kind == "fade" else {"sag_load": val}))
    sc = C.FSC.R1(seed=101) if scen == "R1" else None
    r = C.run(mode, 0.5, real=R, fscale=1.3, scen=scen, sc_obj=sc, peer=dict(emp=1, noise_m=0.001, lat_ms=0))
    return kind, val, scen, mode, dict(z_rms=r["z_rms_cm"], z_mean=r["z_mean_cm"], z_dip=r["z_dip_cm"], tilt=r["tilt_max_deg"], tripped=r["tripped"], diverged=r["diverged"])
if __name__ == "__main__":
    jobs = [("fade", v, s, m) for v in (0.0, 0.05, 0.10, 0.15) for s in ("A1", "A8", "R1") for m in ("omar", "s2", "fbl_rom")] + [("load", 0.15, s, m) for s in ("A1", "A8", "R1") for m in ("omar", "s2", "fbl_rom")]
    out = {}
    with mp.get_context("spawn").Pool(6) as pool:
        for kind, val, scen, mode, r in pool.imap_unordered(job, jobs): out[f"{kind}|{val}|{scen}|{mode}"] = r
    json.dump(out, open(HERE.parent / "out/stage3/battery_sag.json", "w"), indent=1); print("done", len(out))
