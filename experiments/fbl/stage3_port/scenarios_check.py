#!/usr/bin/env python3
"""FBL (tuned preset) vs Omar in the realistic plant on A1 and A4 (and the authors' gains for reference). Output experiments/fbl/out/stage3/scenarios_check.json
  CFFIRMWARE_DIR=<firmware>/build python3 experiments/fbl/stage3_port/scenarios_check.py"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005)
OFF = dict(kp_xy=56, kp_z=45, kv_xy=38, kv_z=21.6, ki_xy=48, ki_z=3, kr_xy=238, kr_z=140)
out = {}
for scen, kw in (("A1", {}), ("A4", dict(offset=0.05)), ("A4", dict(offset=0.10)), ("A4", dict(offset=0.15))):
    for m, g, lab in (("omar", None, "Omar geometric"), ("fbl_none", None, "FBL, no downwash model"), ("fbl_rom", None, "FBL + downwash model (ours)"), ("fbl_rom", OFF, "FBL + downwash model (authors' gains)")):
        r = C.run(m, 0.5, real=R, gains=g, scen=scen, **kw); key = f"{scen}{kw.get('offset', '')}|{lab}"; out[key] = r
        print(f"{key:55s} z_mean {r['z_mean_cm']:7.2f} z_rms {r['z_rms_cm']:6.2f} tilt {r['tilt_max_deg']:5.1f} tripped {r['tripped']}", flush=True)
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/scenarios_check.json", "w"), indent=1)
