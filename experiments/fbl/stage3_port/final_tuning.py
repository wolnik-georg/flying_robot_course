#!/usr/bin/env python3
"""Last desk tuning round: thrust-error robustness (A1, A8) x ki_z, estimator beta/gamma, motor lag, with the tuned preset.  Output out/stage3/final_tuning.json
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/final_tuning.py"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
P = dict(kp_xy=14, kp_z=11.25, kv_xy=9.5, kv_z=5.4, ki_xy=12, ki_z=8, kr_xy=59.5, kr_z=35)
def R(lag=44, eff=1.0): return dict(delay_ms=4, motor_tau_ms=lag, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=eff)
out = {}
def go(key, scen, g, speed=0.5, **rk):
    r = C.run("fbl_rom", speed, real=R(**rk), gains={**P, **g}, scen=scen); out[key] = r
    print(f"{key:44s} z_mean {r['z_mean_cm']:7.2f} z_rms {r['z_rms_cm']:6.2f} dip {r['z_dip_cm']:7.2f} tilt {r['tilt_max_deg']:5.1f} tripped {r['tripped']}", flush=True)
for scen in ("A1", "A8"):
    for eff in (1.0, 0.9, 0.8, 0.7):
        for kz in (3, 8, 16, 30): go(f"{scen}|eff{eff}|kiz{kz}", scen, {"ki_z": kz}, eff=eff)
for bg in ((0.4, 0.1), (0.2, 0.05), (0.8, 0.3), (1.0, 0.5)):
    go(f"A8|beta{bg[0]}|gamma{bg[1]}", "A8", {"beta": bg[0], "gamma": bg[1]})
for lag in (20, 30, 60, 80): go(f"A8|lag{lag}", "A8", {}, lag=lag)
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/final_tuning.json", "w"), indent=1)
