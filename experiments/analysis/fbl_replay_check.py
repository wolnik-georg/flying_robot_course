#!/usr/bin/env python3
"""FBL flights 2026-10-24 (bottom drone cf5, raw uSD, ~500 Hz, FBL-active part up to a trip): what the firmware really did.
1. Gains flown, identified from the logs: rate gains from torque = J kr (w_des - w) (omar_tau = FBL torque under FBL); xy position gains (ki, kp, kv) by least squares
   from the logged body-rate command (Eq. 9: w_xy = e3 x R^T m (j - j_res) / thrust, jerk of Eq. 18 with the alpha-beta-gamma estimate).
2. Replay check: Eq. 18 + Eq. 9 with the flown gains against the logged ctrlFbl.w_des_x/y.
3. Torque bias and the P-only rate loop: mean torque the loop supplied, the steady rate error it needed (gyro - w_des), and the yaw change it produced
   (Eq. 10 has no yaw-angle feedback; the stock firmware the authors flew through integrates a yaw-rate command into a yaw setpoint).
   The yaw change per flight is in the csv but not plotted against the rate error: Eq. 10 also commands a yaw rate and yaw drifts before the log starts.
  ~/.pyenv/versions/flying_robots/bin/python experiments/analysis/fbl_replay_check.py --out experiments/results/fbl_replay_2026-10-24"""
import argparse, json, sys
from pathlib import Path
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "flying_drone_stack/tools")); import decode_usd_log as D  # noqa: E402
ap = argparse.ArgumentParser(); ap.add_argument("--out", type=Path, required=True); a = ap.parse_args(); out = a.out if a.out.is_absolute() else REPO / a.out; out.mkdir(parents=True, exist_ok=True)
J = np.array([2.395e-5, 2.395e-5, 3.235e-5]); M = 0.043; Z = dict(kp=11.25, kv=5.4, ki=12.0)

def quat_R(qw, qx, qy, qz):
    R = np.empty((len(qw), 3, 3))
    R[:, 0, 0] = 1 - 2 * (qy ** 2 + qz ** 2); R[:, 0, 1] = 2 * (qx * qy - qz * qw); R[:, 0, 2] = 2 * (qx * qz + qy * qw)
    R[:, 1, 0] = 2 * (qx * qy + qz * qw); R[:, 1, 1] = 1 - 2 * (qx ** 2 + qz ** 2); R[:, 1, 2] = 2 * (qy * qz - qx * qw)
    R[:, 2, 0] = 2 * (qx * qz - qy * qw); R[:, 2, 1] = 2 * (qy * qz + qx * qw); R[:, 2, 2] = 1 - 2 * (qx ** 2 + qy ** 2)
    return R

def analyse(fl):
    d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"]))
    on = (d["fbl_active"] > 0) & (d["fbl_tripped"] == 0); n = int(np.argmax(~on)) if (~on).any() else len(on); s = slice(200, n)   # skip 0.4 s: estimator start-up of the replay
    t = d["t"][:n]; v = np.c_[d["vx"], d["vy"], d["vz"]][:n]; ve = v[0].copy(); ae = np.zeros(3); A = np.zeros((n, 3))
    for k in range(n):                                          # alpha-beta-gamma (0.4 / 0.1), as fbl_law.c
        dt = t[k] - t[k - 1] if k else 0.0
        if dt > 0: vp = ve + dt * ae; r = v[k] - vp; ve = vp + 0.4 * r; ae = ae + (0.1 / dt) * r
        A[k] = ae
    R = quat_R(*[d[f"est_q{c}"][:n] for c in "wxyz"])
    e = np.c_[d["x"] - d["ctrltarget_x"], d["y"] - d["ctrltarget_y"]][:n]; ev = np.c_[d["vx"] - d["ctrltarget_vx"], d["vy"] - d["ctrltarget_vy"]][:n]
    ea = A[:, :2] - np.c_[d["ctrltarget_ax"], d["ctrltarget_ay"]][:n]
    jz = -Z["ki"] * (d["z"] - d["ctrltarget_z"])[:n] - Z["kp"] * (d["vz"] - d["ctrltarget_vz"])[:n] - Z["kv"] * (A[:, 2] - d["ctrltarget_az"][:n]) - d["fbl_res_jz"][:n]
    th = d["fbl_thrust"][:n]; jb = [d["fbl_w_des_y"][:n] * th / M, -d["fbl_w_des_x"][:n] * th / M]       # body jerk x, y implied by the logged rate command
    X = np.vstack([-np.c_[R[:, 0, c] * e[:, 0] + R[:, 1, c] * e[:, 1], R[:, 0, c] * ev[:, 0] + R[:, 1, c] * ev[:, 1], R[:, 0, c] * ea[:, 0] + R[:, 1, c] * ea[:, 1]][s] for c in (0, 1)])
    y = np.concatenate([(jb[c] - R[:, 2, c] * jz)[s] for c in (0, 1)])
    g, *_ = np.linalg.lstsq(X, y, rcond=None); fit_corr = float(np.corrcoef(X @ g, y)[0, 1])
    werr = np.c_[[d["fbl_w_des_" + c][:n] - np.radians(d["gyro_" + c][:n]) for c in "xyz"]].T; tau = np.c_[[d["omar_tau_" + c][:n] for c in "xyz"]].T
    kr = [float(np.dot(werr[s, i], tau[s, i] / J[i]) / np.dot(werr[s, i], werr[s, i])) for i in range(3)]
    return dict(kr_xy=kr[0], kr_y=kr[1], kr_z=kr[2], ki_xy=g[0], kp_xy=g[1], kv_xy=g[2], replay_corr=fit_corr,
                tau_x_uNm=1e6 * tau[s, 0].mean(), tau_y_uNm=1e6 * tau[s, 1].mean(), tau_z_uNm=1e6 * tau[s, 2].mean(),
                rate_err_x_dps=np.degrees(werr[s, 0].mean()), rate_err_y_dps=np.degrees(werr[s, 1].mean()), rate_err_z_dps=np.degrees(werr[s, 2].mean()),
                yaw_start=float(d["yaw_deg"][0]), yaw_change=float(d["yaw_deg"][n - 1] - d["yaw_deg"][0]), dur_s=float(t[-1] - t[0]))

def s0_bias(fl):
    d = D.load(str(REPO / "experiments/logs/usd_raw" / fl["cf5"])); m = d["z"] > 0.3
    return {f"tau_{c}_uNm": 1e6 * float(d["omar_tau_" + c][m].mean()) for c in "xyz"}

rows = []
for b in ["bank_fbl2_2026-10-24", "bank_fbl3_2026-10-24"]:
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        meta = json.load(open(REPO / "experiments/logs" / f"{fl['scenario']}_{fl['date']}_{fl['stamp']}.meta.json"))
        cell = f"A4 {meta['params'].get('offset'):g}" if fl["scenario"] == "A4" else fl["scenario"]
        r = analyse(fl); r["block"] = "rate x0.5 (_kr)" if r["kr_z"] < 26 else "xy x0.7 (_kp)" if r["kp_xy"] < 11.8 else "preset 1"
        rows.append(dict(kind="FBL", cell=cell, stamp=fl["stamp"], **r))
for b in ["bank_fbl_2026-10-24", "bank_unseen_S0_2026-10-23"]:
    for fl in json.load(open(REPO / "experiments/logs" / b / "training_manifest.json"))["flights"]:
        rows.append(dict(kind="S0", cell=fl["scenario"], stamp=fl["date"][5:] + " " + fl["stamp"], **s0_bias(fl)))
T = pd.DataFrame(rows); T.to_csv(out / "replay_check.csv", index=False); pd.set_option("display.width", 250); print(T.round(2).to_string(index=False))

F = T[T.kind == "FBL"]; S = T[T.kind == "S0"]
fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
for i, c in enumerate("xyz"):
    axs[0].scatter(np.full(len(S), i - 0.12), S[f"tau_{c}_uNm"], c="#7f7f7f", s=18, label="S0 (Omar geometric)" if i == 0 else None)
    axs[0].scatter(np.full(len(F), i + 0.12), F[f"tau_{c}_uNm"], c="#9467bd", s=18, label="FBL" if i == 0 else None)
axs[0].set_xticks(range(3)); axs[0].set_xticklabels(["roll (x)", "pitch (y)", "yaw (z)"]); axs[0].axhline(0, c="k", lw=0.5); axs[0].legend(fontsize=7); axs[0].grid(alpha=0.3)
axs[0].set_ylabel("mean body torque supplied [µN m]", fontsize=8); axs[0].set_title("cf5 needs a steady torque under every controller", fontsize=9)
for i, c in enumerate("xyz"): axs[1].scatter(F[f"tau_{c}_uNm"], F[f"rate_err_{c}_dps"], s=22, label=["roll", "pitch", "yaw"][i])
axs[1].axhline(0, c="k", lw=0.5); axs[1].axvline(0, c="k", lw=0.5); axs[1].grid(alpha=0.3); axs[1].legend(fontsize=7)
axs[1].set_xlabel("mean torque [µN m]", fontsize=8); axs[1].set_ylabel("mean rate error w_des - w [deg/s]", fontsize=8); axs[1].set_title("FBL's P-only rate loop pays for it with a steady rate error", fontsize=9)
fig.tight_layout(); fig.savefig(out / "torque_bias_rate_error.png", dpi=120); plt.close(fig)
