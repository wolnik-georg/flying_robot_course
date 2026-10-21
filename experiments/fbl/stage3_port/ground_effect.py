#!/usr/bin/env python3
"""Ground effect near the floor: takeoff / hold / landing handover test with a Cheeseman-Bennett ground-effect thrust gain (ge = 0 none, 1 nominal, 3 stress) for Omar (geometric) and FBL; also the A8 at the
lowest gap with the ground effect on. out/stage3/ground_effect.json   CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/ground_effect.py"""
import json, sys
from pathlib import Path
import numpy as np
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
base = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977); out = {}
for ge in (0, 1, 3):
    for m in ("omar", "fbl_rom"):
        r = C.run(m, 0.5, real={**base, "ge": ge}, handover=True); d = np.load(Path(__file__).parents[1] / f"out/stage3/handover_{m}.npy"); t = d[:, 0]
        e = 100 * (d[:, 1] - d[:, 2]); up = (t > 1) & (t < 4.5); dn = (t > 9) & (t < 13.5)
        out[f"{m}|ge{ge}"] = dict(tilt=r["tilt_max_deg"], tripped=r["tripped"], takeoff_err_min=float(e[up].min()), takeoff_err_max=float(e[up].max()), landing_err_min=float(e[dn].min()), landing_err_max=float(e[dn].max()), touchdown_vz=C.TD[0], hold_err_max=float(np.abs(e[(t > 5) & (t < 9)]).max()))
        print(m, ge, {k: round(v, 2) if isinstance(v, float) else v for k, v in out[f"{m}|ge{ge}"].items()}, flush=True)
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/ground_effect.json", "w"), indent=1)
