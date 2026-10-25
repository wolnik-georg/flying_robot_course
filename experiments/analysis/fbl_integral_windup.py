#!/usr/bin/env python3
"""FBL flights 2026-10-24 (cf5, raw uSD, FBL-active part): the commanded acceleration of Eq. 17 reconstructed from the logged desired attitude (ctrlOmarIndi.rpyd = thrust
direction of a_cmd - a_res + g, built in the FBL branch) and fbl_acc_z. Shows the xy integral winding up (ki * integral of the position error grows while the drone flies level)
and checks the reconstruction: the integral state recovered from it must satisfy dI/dt = position error.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_integral_windup.py --out experiments/results/fbl_replay_2026-10-24"""
import argparse, json, sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy.spatial.transform import Rotation as Rot
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D  # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); ap.add_argument("--fbl-banks", nargs="*", default=[]); a = ap.parse_args(); out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True)
G = 9.81; M = 0.043
GA = {"preset 1": (14, 9.5, 12), "rate x0.5 (_kr)": (14, 9.5, 12), "xy x0.7 (_kp)": (9.8, 6.65, 8.4)}
BLOCK = {**{s: "preset 1" for s in ("14-25-32", "14-27-01", "14-29-07", "14-30-39", "14-44-24")}, **{s: "rate x0.5 (_kr)" for s in ("14-46-44", "14-50-08", "14-54-00")},
         **{s: "xy x0.7 (_kp)" for s in ("14-55-45", "14-57-11", "14-59-53", "15-01-21")}}   # identified from the logs (fbl_replay_check.py)
rows = []; tr = {}
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"] + list(a.fbl_banks):
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); cell = f"A4 {meta['params'].get('offset'):g}" if fl["scenario"] == "A4" else fl["scenario"]
        d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"])); on = (d["fbl_active"] > 0) & (d["fbl_tripped"] == 0); n = int(np.argmax(~on)) if (~on).any() else len(on)
        if fl["stamp"] not in BLOCK:                               # later flights: gains and label from meta.json controller_variant (run_formation, since 2026-10-25)
            cv = (meta.get("controller_variant") or {}).get("cf5") or {}
            BLOCK[fl["stamp"]] = f"inner {int(cv['ctrlFbl_inner'])} p{int(cv['ctrlFbl_preset'])} nn{int(cv['ctrlFbl_nn_en'])}"
            GA[BLOCK[fl["stamp"]]] = (float(cv["ctrlFbl_kp_xy"]), float(cv["ctrlFbl_kv_xy"]), float(cv["ctrlFbl_ki_xy"]))
        kp, kv, ki = GA[BLOCK[fl["stamp"]]]; t = d["t"][:n]
        zd = Rot.from_euler("ZYX", np.c_[d["omar_rpyd_z"], d["omar_rpyd_y"], d["omar_rpyd_x"]][:n]).as_matrix()[:, :, 2]        # quat2rpy: R = Rz Ry Rx
        Fz = d["fbl_acc_z"][:n] - d["fbl_res_az"][:n] + G; acc = np.c_[zd[:, 0] / zd[:, 2] * Fz, zd[:, 1] / zd[:, 2] * Fz]
        e = np.c_[d["x"] - d["ctrltarget_x"], d["y"] - d["ctrltarget_y"]][:n]; ev = np.c_[d["vx"] - d["ctrltarget_vx"], d["vy"] - d["ctrltarget_vy"]][:n]; ar = np.c_[d["ctrltarget_ax"], d["ctrltarget_ay"]][:n]
        I = (ar - kp * e - kv * ev - acc) / ki; Ip = I[0] + np.r_[np.zeros((1, 2)), np.cumsum(e[1:] * np.diff(t)[:, None], axis=0)]
        zb = Rot.from_quat(np.c_[d["est_qx"], d["est_qy"], d["est_qz"], d["est_qw"]][:n]).as_matrix()[:, :, 2]
        tilt_cmd = np.degrees(np.arccos(np.clip(zd[:, 2], -1, 1))); tilt = np.degrees(np.arccos(np.clip(zb[:, 2], -1, 1)))
        rows.append(dict(cell=cell, stamp=fl["stamp"], block=BLOCK[fl["stamp"]], tilt_cmd_mean=tilt_cmd.mean(), tilt_mean=tilt.mean(), kiI_abs_max=float(np.abs(ki * I).max()),
                         acc_cmd_xy_rms=float(np.sqrt(np.mean(np.sum(acc ** 2, 1)))), I_check_corr_x=float(np.corrcoef(I[:, 0], Ip[:, 0])[0, 1]), I_check_corr_y=float(np.corrcoef(I[:, 1], Ip[:, 1])[0, 1]),
                         I_check_rms=float(np.sqrt(np.mean((I - Ip) ** 2)))))
        tr[(cell, fl["stamp"])] = (t - t[0], tilt_cmd, tilt, ki * I)
T = pd.DataFrame(rows).sort_values(["cell", "stamp"]); T.to_csv(out / "integral_windup.csv", index=False); print(T.round(3).to_string(index=False))
sel = [("A1", "14-30-39"), ("A4 0.075", "14-59-53"), ("A4 0.125", "15-01-21"), ("A8", "14-55-45")]
fig, axs = plt.subplots(2, 4, figsize=(15, 6), sharex="col")
for j, k in enumerate(sel):
    t, tc, ta, kI = tr[k]
    axs[0, j].plot(t, tc, c="#9467bd", lw=1, label="thrust direction commanded by Eq. 17"); axs[0, j].plot(t, ta, c="k", lw=0.8, label="actual tilt"); axs[0, j].set_title(f"{k[0]} {k[1][:5]} ({BLOCK[k[1]]})", fontsize=8)
    axs[1, j].plot(t, kI[:, 0], c="#1f77b4", lw=1, label="x"); axs[1, j].plot(t, kI[:, 1], c="#d62728", lw=1, label="y"); axs[1, j].axhline(0, c="k", lw=0.5)
    axs[1, j].set_xlabel("time since log start [s]"); [x.grid(alpha=0.3) for x in axs[:, j]]
axs[0, 0].set_ylabel("tilt from vertical [deg]", fontsize=8); axs[1, 0].set_ylabel("ki x integral of position error [m/s²]", fontsize=8); axs[0, 0].legend(fontsize=7); axs[1, 0].legend(fontsize=7)
fig.suptitle("The xy integral of Eq. 17 winds up: the commanded acceleration points 30-65 deg off vertical while the drone flies level (attitude follows only the jerk path, Eq. 18)", fontsize=9)
fig.tight_layout(); fig.savefig(out / "integral_windup.png", dpi=120)
