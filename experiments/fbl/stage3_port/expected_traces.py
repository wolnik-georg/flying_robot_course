#!/usr/bin/env python3
"""What the ctrlFbl.* channels should look like in the first FBL flights (A1 hold, A8 swap), from the calibrated plant (thrust 97.7 %, downwash 1.3x, measured peer feed). Reference for the lab:
compare the uSD log of the first flight with this figure.   CFFIRMWARE_DIR=<fw>/build ~/..python3 experiments/fbl/stage3_port/expected_traces.py  (system python3, then plot with robustness env)"""
import json, sys
from pathlib import Path
import numpy as np
sys.argv = sys.argv[:1]; sys.path.insert(0, str(Path(__file__).parent)); import c_in_the_loop_real as C  # noqa: E402
import cffirmware as fw  # noqa: E402
R = dict(delay_ms=4, motor_tau_ms=44, gyro_noise_dps=0.5, pos_noise_m=0.0005, vel_noise=0.005, thrust_eff=0.977); PEER = dict(emp=1, noise_m=0.001, lat_ms=0)
orig = fw.controllerOmarIndi; rec = []
def wrap(omar, ctl, sp, sens, st, i):
    orig(omar, ctl, sp, sens, st, i)
    if i % 10 == 0: rec.append([i * 1e-3, st.position.z, sp.position.z, omar.fbl.active, omar.fbl.tripped, omar.fbl.steps, omar.fbl.res_az, omar.fbl.thrust, omar.fbl.acc_cmd.z, omar.fbl.w_des.x])
fw.controllerOmarIndi = wrap; out = {}
for scen in ("A1", "A8"):
    del rec[:]; r = C.run("fbl_rom", 0.5, real=R, fscale=1.3, scen=scen, peer=PEER); out[scen] = np.array(rec).tolist(); print(scen, {k: round(v, 2) if isinstance(v, float) else v for k, v in r.items() if k in ("z_rms_cm", "tilt_max_deg", "tripped")}, len(rec))
json.dump(out, open(Path(__file__).parents[1] / "out/stage3/expected_traces.json", "w"))
