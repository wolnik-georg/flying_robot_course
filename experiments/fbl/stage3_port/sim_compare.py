#!/usr/bin/env python3
"""Sim-only comparison in ONE plant (c_in_the_loop_real.py: delay 4 ms, motor lag 44 ms, noise; truth downwash = NS2 network): S0 geometric, S2 geometric + NS2,
FBL without / with the downwash model (tuned preset). INDI strategies (S1, S3) need the RPM-based plant of the two-drone harness and are NOT in this plant.
Two variants: 'ideal' (force scale 1.0, all thrust arrives) and 'stress' (downwash 1.3x what the models predict, 12 % thrust shortfall for every strategy).
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/sim_compare.py      -> out/stage3/sim_compare.json"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
MODES = [("omar", "S0 geometric"), ("s2", "S2 geometric + NS2"), ("fbl_none", "FBL, no downwash model"), ("fbl_rom", "FBL + downwash model")]
CASES = [("A8", dict(scen="A8"), "z_dip_cm"), ("A1", dict(scen="A1"), "z_mean_cm"), ("A4 0.05", dict(scen="A4", offset=0.05), "z_rms_cm"), ("A4 0.10", dict(scen="A4", offset=0.10), "z_rms_cm"), ("A4 0.15", dict(scen="A4", offset=0.15), "z_rms_cm")]
VAR = {"ideal": dict(fscale=1.0, eff=1.0), "stress": dict(fscale=1.3, eff=0.88),
       "measured": dict(fscale=1.0, eff=0.977),           # thrust command / weight at hover without downwash = 1.023 on the bottom drone (uSD logs, 8 A8 flights, range 1.006-1.035)
       "measured_1p3": dict(fscale=1.3, eff=0.977)}      # plus downwash 1.3x the prediction (the factor that reproduces the hardware S0 dip in the two-drone harness)
import os
ONLY = os.environ.get("VARIANTS"); TAG = os.environ.get("TAG", "")
out = {}
for vn, v in VAR.items():
    if ONLY and vn not in ONLY.split(','): continue
    real = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=v["eff"])
    for cn, kw, metric in CASES:
        for m, lab in MODES:
            r = C.run(m, 0.5, real=real, fscale=v["fscale"], **kw); out[f"{vn}|{cn}|{m}"] = dict(metric=metric, value=r[metric], z_rms=r["z_rms_cm"], tripped=r["tripped"], tilt=r["tilt_max_deg"])
            print(f"{vn:6s} {cn:8s} {lab:26s} {metric} {r[metric]:7.2f}  z_rms {r['z_rms_cm']:6.2f} tripped {r['tripped']}", flush=True)
json.dump(out, open(Path(__file__).parents[1] / f"out/stage3/sim_compare{TAG}.json", "w"), indent=1)
