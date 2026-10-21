#!/usr/bin/env python3
"""Lower-gain fallback for the case that the first flights show tilt bursts: the draws of param_sweep.json where FBL reached > 8 deg (lag and delay both high) plus the nominal plant,
run with scaled gains (rate loop kr, xy position loop kp/kv/ki). out/stage3/fallback_gains.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/fallback_gains.py"""
import json, sys, multiprocessing as mp
from pathlib import Path
import numpy as np
HERE = Path(__file__).parent
P1 = dict(kp_xy=14.0, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12.0, ki_z=12.0, kr_xy=59.5, kr_z=35.0)
VARIANTS = {"preset 1 (default)": (1, 1), "kr x0.7": (0.7, 1), "kr x0.5": (0.5, 1), "xy position x0.7": (1, 0.7), "kr x0.7 + xy pos x0.7": (0.7, 0.7), "kr x0.5 + xy pos x0.5": (0.5, 0.5)}
def gains(kr, pos): return {**P1, "kr_xy": P1["kr_xy"] * kr, "kr_z": P1["kr_z"] * kr, "kp_xy": P1["kp_xy"] * pos, "kv_xy": P1["kv_xy"] * pos, "ki_xy": P1["ki_xy"] * pos}
def job(a):
    var, tag, scen, p = a; kr, pos = VARIANTS[var]
    sys.argv = sys.argv[:1]; sys.path.insert(0, str(HERE)); import c_in_the_loop_real as C
    C.CD_FIT = p["cd"]
    R = dict(delay_ms=p["delay"], motor_tau_ms=p["lag"], gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=p["eff"])
    r = C.run("fbl_rom", 0.5, real=R, fscale=p["fscale"], scen=scen, gains=gains(kr, pos), peer=dict(emp=1, noise_m=0.001, lat_ms=0), plant=dict(mass_scale=p["mass"], J_scale=p["J"]), seed=1)
    return var, tag, scen, dict(z_rms=r["z_rms_cm"], z_dip=r["z_dip_cm"], z_mean=r["z_mean_cm"], tilt=r["tilt_max_deg"], tripped=r["tripped"], diverged=r["diverged"])
if __name__ == "__main__":
    sw = json.load(open(HERE.parent / "out/stage3/param_sweep.json")); bad = {x["i"]: x for x in sw if x["mode"] == "fbl_rom" and x["tilt"] > 8}
    draws = {f"draw {i}": {k: x[k] for k in ("mass", "J", "lag", "delay", "eff", "fscale", "cd")} for i, x in sorted(bad.items())}
    draws["nominal"] = dict(mass=1.0, J=1.0, lag=44, delay=4, eff=0.977, fscale=1.3, cd=0.43)
    jobs = [(v, t, s, p) for v in VARIANTS for t, p in draws.items() for s in ("A8", "A1")]; out = {}
    with mp.get_context("spawn").Pool(6) as pool:
        for v, t, s, r in pool.imap_unordered(job, jobs): out[f"{v}|{t}|{s}"] = r
    json.dump(dict(draws=draws, results=out), open(HERE.parent / "out/stage3/fallback_gains.json", "w"), indent=1)
    for v in VARIANTS:
        rr = [out[f"{v}|{t}|{s}"] for t in draws if t != "nominal" for s in ("A8", "A1")]; nm = [out[f"{v}|nominal|{s}"] for s in ("A8", "A1")]
        print(f"{v:26s} worst draws: tilt max {max(r['tilt'] for r in rr):5.1f} deg, z_rms median {np.median([r['z_rms'] for r in rr]):.1f} | nominal: A8 dip {nm[0]['z_dip']:.2f} z_rms {nm[0]['z_rms']:.2f}, A1 mean {nm[1]['z_mean']:.2f} z_rms {nm[1]['z_rms']:.2f}, tilt {max(n['tilt'] for n in nm):.1f}")
