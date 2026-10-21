#!/usr/bin/env python3
"""Peer-feed faults in the calibrated plant (A8 0.5 m/s, thrust 97.7 %, downwash 1.3x): dropouts of 0.3 / 1 / 3 s around the crossing and a single NaN / Inf position. out/stage3/peer_faults.json
  CFFIRMWARE_DIR=<fw>/build python3 experiments/fbl/stage3_port/peer_faults.py"""
import json, sys
from pathlib import Path
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977); base = dict(emp=1, noise_m=0.001, lat_ms=0)
cases = {"no fault": base, "dropout 0.3 s": {**base, "drop": (8.2, 8.5)}, "dropout 1 s": {**base, "drop": (7.8, 8.8)}, "dropout 3 s": {**base, "drop": (6.5, 9.5)}, "one NaN": {**base, "nan_at": 8.0}, "one Inf": {**base, "nan_at": 8.0, "nan_kind": "inf"}}
out = {}
for m in ("s2", "fbl_rom"):
    for k, pk in cases.items():
        r = C.run(m, 0.5, real=R, fscale=1.3, peer=pk); out[f"{m}|{k}"] = dict(dip=r["z_dip_cm"], z_rms=r["z_rms_cm"], tripped=r["tripped"]); print(m, k, round(r["z_dip_cm"], 2), r["tripped"], flush=True)
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/peer_faults.json", "w"), indent=1)
