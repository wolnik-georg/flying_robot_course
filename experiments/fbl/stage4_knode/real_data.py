"""Real-flight data for the KNODE training (Task 1, step 3): merged uSD banks -> ego (bottom, cf5) / neighbour (top, cf_second) 13-states, thrust input, label.

State = [p(3), v(3), q_xyzw(4), body rates rad/s (3)] from the logged estimator state (x/y/z, vx/vy/vz, est_q*, gyro deg/s). Thrust input = controller thrust command
`omar_thrust_si` [N] (option 'cmd') or the rotor-speed force `omar_f_rpm` (option 'rpm'), held (mean) over the step. Torque input = 0 (force-only training, rates weighted 0).
`label` = the logged residual a_res (a_imu - a_rpm, world frame) of the bottom vehicle, used only for evaluation."""
from __future__ import annotations

import glob, json
from pathlib import Path
import numpy as np, pandas as pd

REPO = Path(__file__).resolve().parents[3]
MASS = 0.0427
K_RPM2FORCE = 3.91194027330774e-08          # N/(rad/s)^2, MOTORRPM2FORCE of the platform


def _drone(df, pre):
    b = df[[c for c in df.columns if c.startswith(pre + ".")]].rename(columns=lambda c: c.split(".", 1)[1])
    X = np.c_[b.x, b.y, b.z, b.vx, b.vy, b.vz, b.est_qx, b.est_qy, b.est_qz, b.est_qw, np.deg2rad(b[["gyro_x", "gyro_y", "gyro_z"]].values)]
    return b, X


def load_flight(csv):
    df = pd.read_csv(csv, comment="#")
    pb = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf5_")})[0]; pt = sorted({c.split(".")[0] for c in df.columns if c.startswith("cf_second_")})[0]
    b, X = _drone(df, pb); t_, XT = _drone(df, pt)
    rp = np.c_[[b[f"rpm_m{i}"].values for i in range(1, 5)]].T.astype(float)                  # deck rpm; the deck channel has invalid 65535 spikes -> interpolate over them
    bad = rp > 30000
    for k in range(4):
        if bad[:, k].any(): rp[bad[:, k], k] = np.interp(np.flatnonzero(bad[:, k]), np.flatnonzero(~bad[:, k]), rp[~bad[:, k], k])
    f_rpm = (K_RPM2FORCE * (rp * 2 * np.pi / 60) ** 2).sum(1)                                  # N, rotor-speed force (omar_f_rpm is 0 when indi = 0)
    return dict(t=df.t.values, X=X, XT=XT, thrust_cmd=b.omar_thrust_si.values, thrust_rpm=f_rpm, label=b[["a_res_x", "a_res_y", "a_res_z"]].values, vbat=b.vbat.values, nn=b[["rnn_pred_x", "rnn_pred_y", "rnn_pred_z"]].values, name=Path(csv).parent.name)


def make_pairs(fl, stride=5, thrust="cmd", thrust_scale=1.0, max_r=0.5, require_above=True, z_min=0.15, subsample=None):
    """Pairs i -> i+stride with the thrust held (mean over the step), neighbour state held at row i. Returns dict of arrays (y0 rows (n,30), target (n,13), label (n,3), r, dz)."""
    X, XT, t = fl["X"], fl["XT"], fl["t"]; n = len(t); dt = float(np.median(np.diff(t))) * stride
    th = fl["thrust_cmd"] if thrust == "cmd" else fl["thrust_rpm"]
    cs = np.cumsum(np.r_[0, th]); thm_all = (cs[stride:] - cs[:-stride])[: n - stride] / stride     # mean thrust over [i, i+stride)
    i = np.arange(0, n - stride, subsample or stride)                       # non-overlapping steps by default (every `stride`-th row)
    r = np.hypot(X[i, 0] - XT[i, 0], X[i, 1] - XT[i, 1]); dz = XT[i, 2] - X[i, 2]
    keep = (X[i, 2] > z_min) & (r < max_r) & (r >= 2e-3) & ((dz > 0) if require_above else True)
    U = np.zeros((len(i), 4)); U[:, 0] = thm_all[i] * thrust_scale
    y0 = np.concatenate([X[i], U, XT[i]], axis=1)[keep]
    return dict(y0=y0.astype(np.float32), target=X[i + stride][keep].astype(np.float32), label=fl["label"][i][keep], nn=fl["nn"][i][keep], r=r[keep], dz=dz[keep], dt=dt, name=fl["name"])
