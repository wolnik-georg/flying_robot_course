#!/usr/bin/env python3
"""Independent review 2026-10-24: does the inner-loop (rate) tracking error explain the FBL lateral error? (cf5, FBL-active part, raw uSD)
Jerk deficit of the inner loop, world frame: d(t) = (T/m) R (-werr_y, werr_x, 0), werr = w_des - gyro, T = logged FBL thrust, m = 0.0427 kg (recovered from the logs).
With an ideal acceleration estimate the xy error then obeys e''' + kv e'' + kp e' + ki e = d  (Eq. 17/18 closed loop). We simulate that from the logged d
(initial state from the log) and compare with the measured error: full d, and d low-passed at 0.3 Hz (the 'steady' part = torque-bias hypothesis).
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/review_fbl_lateral_chain.py"""
import json, sys
from pathlib import Path
import numpy as np, pandas as pd
from scipy import signal
from scipy.spatial.transform import Rotation as Rot
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D  # noqa: E402
M = 0.0427; DT = 0.002
GA = {"preset 1": (14, 9.5, 12), "_kr": (14, 9.5, 12), "_kp": (9.8, 6.65, 8.4)}
BLOCK = {**{s: "preset 1" for s in ("14-25-32", "14-27-01", "14-29-07", "14-30-39", "14-44-24")}, **{s: "_kr" for s in ("14-46-44", "14-50-08", "14-54-00")},
         **{s: "_kp" for s in ("14-55-45", "14-57-11", "14-59-53", "15-01-21")}}
rows = []
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]:
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json")); cell = f"A4 {meta['params'].get('offset'):g}" if fl["scenario"] == "A4" else fl["scenario"]
        d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"])); on = (d["fbl_active"] > 0) & (d["fbl_tripped"] == 0); n = int(np.argmax(~on)) if (~on).any() else len(on)
        t0 = d["t"][:n]; tu = np.arange(t0[0], t0[-1], DT); I = lambda x: np.interp(tu, t0, x[:n])
        q = np.c_[I(d["est_qx"]), I(d["est_qy"]), I(d["est_qz"]), I(d["est_qw"])]; R = Rot.from_quat(q / np.linalg.norm(q, axis=1)[:, None]).as_matrix()
        we = np.c_[I(d["fbl_w_des_x"]) - np.radians(I(d["gyro_x"])), I(d["fbl_w_des_y"]) - np.radians(I(d["gyro_y"]))]
        T = I(d["fbl_thrust"]); dw = np.einsum("nij,nj->ni", R, np.c_[-we[:, 1], we[:, 0], np.zeros(len(tu))] * (T / M)[:, None])[:, :2]
        e = np.c_[I(d["x"]) - I(d["ctrltarget_x"]), I(d["y"]) - I(d["ctrltarget_y"])]; ev = np.c_[I(d["vx"]) - I(d["ctrltarget_vx"]), I(d["vy"]) - I(d["ctrltarget_vy"])]
        kp, kv, ki = GA[BLOCK[fl["stamp"]]]
        sys3 = signal.StateSpace([[0, 1, 0], [0, 0, 1], [-ki, -kp, -kv]], [[0], [0], [1]], [[1, 0, 0]], [[0]])
        bl, al = signal.butter(1, 0.3, fs=1 / DT); r = dict(cell=cell, stamp=fl["stamp"], block=BLOCK[fl["stamp"]], dur=tu[-1] - tu[0])
        for tag, dd in (("full", dw), ("lp0.3Hz", signal.filtfilt(bl, al, dw, axis=0))):
            for c, ax in enumerate("xy"):
                ea0 = 0.0
                _, yp, _ = signal.lsim(sys3, dd[:, c], tu - tu[0], X0=[e[0, c], ev[0, c], ea0])
                r[f"{tag}_corr_{ax}"] = np.corrcoef(yp, e[:, c])[0, 1]; r[f"{tag}_rmsratio_{ax}"] = np.sqrt(np.mean(yp ** 2)) / np.sqrt(np.mean(e[:, c] ** 2))
        r["e_rms_x_cm"] = 100 * np.sqrt(np.mean(e[:, 0] ** 2)); r["e_rms_y_cm"] = 100 * np.sqrt(np.mean(e[:, 1] ** 2))
        r["werr_mean_x_dps"] = np.degrees(we[:, 0].mean()); r["werr_mean_y_dps"] = np.degrees(we[:, 1].mean())
        rows.append(r)
T = pd.DataFrame(rows).sort_values(["cell", "stamp"]); pd.set_option("display.width", 250); print(T.round(2).to_string(index=False))
