#!/usr/bin/env python3
"""Independent review 2026-10-24: yaw budget of the FBL flights (cf5, FBL-active part, raw uSD). Over gap-free log steps (dt < 5 ms) only:
  dpsi_obs  = sum of the yaw change (EKF quaternion, ZYX)
  dpsi_kin  = sum of the Euler yaw rate from the gyro, (sin(phi) w_y + cos(phi) w_z) / cos(theta)   (kinematic check of gyro vs estimate)
  dpsi_cmd  = same with the commanded rates ctrlFbl.w_des (Eq. 9/10); ~0 if Eq. 10 holds the heading
  dpsi_err  = dpsi_kin - dpsi_cmd = yaw change caused by the rate tracking error
Also: mean body z rate vs mean Euler yaw rate (coning), and S0 mean torque sign per axis.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_yaw_budget.py"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as Rot
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D  # noqa: E402
rows = []
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]:
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"]))
        on = (d["fbl_active"] > 0) & (d["fbl_tripped"] == 0); n = int(np.argmax(~on)) if (~on).any() else len(on)
        t = d["t"][:n]; dt = np.diff(t); ok = dt < 0.005
        eul = Rot.from_quat(np.c_[d["est_qx"], d["est_qy"], d["est_qz"], d["est_qw"]][:n]).as_euler("ZYX"); psi = np.unwrap(eul[:, 0]); th, ph = eul[:, 1], eul[:, 2]
        g = np.radians(np.c_[d["gyro_x"], d["gyro_y"], d["gyro_z"]][:n]); wd = np.c_[d["fbl_w_des_x"], d["fbl_w_des_y"], d["fbl_w_des_z"]][:n]
        er = lambda w: (np.sin(ph) * w[:, 1] + np.cos(ph) * w[:, 2]) / np.cos(th)
        k = lambda x: np.degrees(np.sum((x[:-1] * dt)[ok]))
        dobs = np.degrees(np.sum(np.diff(psi)[ok])); dkin = k(er(g)); dcmd = k(er(wd))
        rows.append(dict(stamp=fl["stamp"], scen=fl["scenario"], dur=t[-1] - t[0], gap_s=float(dt[~ok].sum()), dpsi_obs=dobs, dpsi_kin=dkin, dpsi_cmd=dcmd, dpsi_err=dkin - dcmd,
                         mean_wz_body_dps=np.degrees(g[:, 2].mean()), mean_yawrate_dps=np.degrees(er(g).mean()), mean_rate_err_z_dps=np.degrees((wd[:, 2] - g[:, 2]).mean())))
T = pd.DataFrame(rows); pd.set_option("display.width", 220); print(T.round(2).to_string(index=False))
