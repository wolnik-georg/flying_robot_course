#!/usr/bin/env python3
"""Independent review 2026-10-24: (1) the mass the flown FBL used, recovered from thrust = m (a_cmd - a_res + g) . z_b with a_cmd from ctrlOmarIndi.rpyd + ctrlFbl.acc_z;
(2) the xy-coupling term m a_cmd,xy . z_b,xy of Eq. 15 (size, correlation with the logged thrust); (3) kiI at log start. cf5, FBL-active part, raw uSD.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_thrust_coupling.py"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as Rot
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D  # noqa: E402
GA = {"preset 1": (14, 9.5, 12), "_kr": (14, 9.5, 12), "_kp": (9.8, 6.65, 8.4)}
BLOCK = {**{s: "preset 1" for s in ("14-25-32", "14-27-01", "14-29-07", "14-30-39", "14-44-24")}, **{s: "_kr" for s in ("14-46-44", "14-50-08", "14-54-00")},
         **{s: "_kp" for s in ("14-55-45", "14-57-11", "14-59-53", "15-01-21")}}
rows = []
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]:
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"])); on = (d["fbl_active"] > 0) & (d["fbl_tripped"] == 0); n = int(np.argmax(~on)) if (~on).any() else len(on)
        zd = Rot.from_euler("ZYX", np.c_[d["omar_rpyd_z"], d["omar_rpyd_y"], d["omar_rpyd_x"]][:n]).as_matrix()[:, :, 2]
        Fz = d["fbl_acc_z"][:n] - d["fbl_res_az"][:n] + 9.81; F = np.c_[zd[:, 0] / zd[:, 2] * Fz, zd[:, 1] / zd[:, 2] * Fz, Fz]
        zb = Rot.from_quat(np.c_[d["est_qx"], d["est_qy"], d["est_qz"], d["est_qw"]][:n]).as_matrix()[:, :, 2]
        th = d["fbl_thrust"][:n]; ok = (th > 0.05) & (th < 0.5); m = float(np.median(th[ok] / np.sum(F * zb, 1)[ok]))
        cpl = m * np.sum(F[:, :2] * zb[:, :2], 1); rec = m * np.sum(F * zb, 1)
        kp, kv, ki = GA[BLOCK[fl["stamp"]]]
        e = np.c_[d["x"] - d["ctrltarget_x"], d["y"] - d["ctrltarget_y"]][:n]; ev = np.c_[d["vx"] - d["ctrltarget_vx"], d["vy"] - d["ctrltarget_vy"]][:n]
        kiI = np.c_[d["ctrltarget_ax"], d["ctrltarget_ay"]][:n] - kp * e - kv * ev - F[:, :2]
        rows.append(dict(stamp=fl["stamp"], scen=fl["scenario"], block=BLOCK[fl["stamp"]], tripped=bool((d["fbl_tripped"] > 0).any()), mass=m,
                         rec_err_mN=1e3 * float(np.abs(rec - th)[ok].max()), thrust_std_mN=1e3 * th.std(), cpl_std_mN=1e3 * cpl.std(), corr=float(np.corrcoef(cpl, th)[0, 1]),
                         kiI_start=float(np.hypot(*kiI[0])), kiI_end=float(np.hypot(*kiI[-1]))))
T = pd.DataFrame(rows); pd.set_option("display.width", 220); print(T.round(4).to_string(index=False))
