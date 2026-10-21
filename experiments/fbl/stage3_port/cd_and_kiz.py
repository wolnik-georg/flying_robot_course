#!/usr/bin/env python3
"""(a) sensitivity of FBL to the downwash coefficient C_D (0.43 fitted) and (b) the z-integral gain ki_z, in the calibrated plant (thrust 97.7 %, downwash 1.3x). Output out/stage3/cd_and_kiz.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/cd_and_kiz.py"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977)
CASES = (("A8", dict(scen="A8"), "z_dip_cm"), ("A1", dict(scen="A1"), "z_mean_cm"), ("A4 0.05", dict(scen="A4", offset=0.05), "z_rms_cm"))
out = {}
def go(key, mode, gains=None):
    for cn, kw, met in CASES:
        r = C.run(mode, 0.5, real=R, fscale=1.3, gains=gains, **kw); out[f"{key}|{cn}"] = dict(metric=met, value=r[met], z_rms=r["z_rms_cm"], tripped=r["tripped"])
    print(key, {cn: round(out[f"{key}|{cn}"]["value"], 2) for cn, _, _ in CASES}, flush=True)
# (a) C_D via the module constant CD_FIT
for cd in (0.0, 0.2, 0.3, 0.43, 0.6, 0.8):
    C.CD_FIT = cd; go(f"cd {cd}", "fbl_rom")
C.CD_FIT = 0.43
for kz in (3, 8, 12, 16, 24): go(f"ki_z {kz}", "fbl_rom", {"ki_z": kz})
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/cd_and_kiz.json", "w"), indent=1)
