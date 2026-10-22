"""Synthetic ground-truth flights for checking the KNODE training pipeline (Task 1, step 2).

Ground truth = the bundle's own training dynamics (rigid body + downwash ROM, neighbour attitude used as in the bundle's training wrapper) with the ROM drag
coefficient C_d_GT (default 0.55, the bundle's `C_d_GT`; the paper's simulation uses 0.7 for the truth and 0.3 for the ROM). The model under training uses C_d = 0.3, so
the network has to learn the ROM-to-truth correction. A geometric tracking controller without any downwash compensation flies the bottom vehicle ("nominal controller", like the
paper's data collection); the top vehicle is kinematic (upright, on its reference). Vehicle = the bundle's 30 g Crazyflie (this is a check of the pipeline, not of our platform).
Scenarios: stacked (top moves with the bottom, vertical separation dz) and static top (top hovers, bottom shuttles beneath it)."""
from __future__ import annotations

import numpy as np
import torch
from scipy.spatial.transform import Rotation as Rot

from knode_train import rk4_step, setup_ode
from common import bundle as B

G = 9.81


def reference(t, kind, speed, length):
    """Bottom reference x(t), v(t), a(t): sinusoidal shuttle along x with peak speed `speed` (amplitude length/2)."""
    A = length / 2.0; w = speed / A
    return A * np.sin(w * t), A * w * np.cos(w * t), -A * w * w * np.sin(w * t)


def simulate(dz=0.4, kind="stacked", speed=0.4, length=1.0, duration=9.0, dt=0.005, c_d_truth=0.55, c_d_rom=0.3, z0=0.5, seed=0):
    old_cd = B.load().nominal.C_d                                         # the bundle's C_d is a module global: restore it after the truth simulation
    try:
        return _simulate(dz, kind, speed, length, duration, dt, c_d_truth, z0, seed)
    finally:
        B.load().nominal.C_d = old_cd


def _simulate(dz, kind, speed, length, duration, dt, c_d_truth, z0, seed):
    b, ode = setup_ode(c_d_truth)                                         # truth plant: ROM with c_d_truth inside the nominal model
    nom = ode.nominal_model; m = nom.mass; J = np.diag([nom.Ixx, nom.Iyy, nom.Izz])
    N = int(duration / dt) + 1
    X = np.zeros((N, 13)); U = np.zeros((N, 4)); XT = np.zeros((N, 13))
    x0, _, _ = reference(0.0, kind, speed, length)
    X[0, :3] = [x0, 0.0, z0]; X[0, 9] = 1.0
    rng = np.random.default_rng(seed)
    kp, kd, kR, kw, kiz = 16.0, 6.0, 200.0, 20.0, 8.0
    iz = 0.0                                                              # z integral of the nominal controller (keeps the sag moderate; no downwash model)
    for i in range(N):
        t = i * dt; x = X[i]
        xr, vr, ar = reference(t, kind, speed, length)
        p_ref = np.array([xr, 0.0, z0]); v_ref = np.array([vr, 0.0, 0.0]); a_ref = np.array([ar, 0.0, 0.0])
        if kind == "stacked": top = np.array([xr, 0.0, z0 + dz]); vt = v_ref
        else: top = np.array([0.0, 0.0, z0 + dz]); vt = np.zeros(3)
        XT[i, :3] = top; XT[i, 3:6] = vt; XT[i, 9] = 1.0
        R = Rot.from_quat(x[6:10]).as_matrix(); zb = R[:, 2]
        iz += (z0 - x[2]) * dt
        a_d = a_ref + kd * (v_ref - x[3:6]) + kp * (p_ref - x[:3]) + np.array([0, 0, G + kiz * iz])
        thrust = max(m * float(a_d @ zb), 0.0)
        zd = a_d / np.linalg.norm(a_d); yd = np.cross(zd, [1, 0, 0]); yd /= np.linalg.norm(yd); xd = np.cross(yd, zd); Rd = np.stack([xd, yd, zd], 1)
        eR_m = 0.5 * (Rd.T @ R - R.T @ Rd); eR = np.array([eR_m[2, 1], eR_m[0, 2], eR_m[1, 0]])
        w = x[10:13]; tau = J @ (-kR * eR - kw * w) + np.cross(w, J @ w)
        U[i] = [thrust, *tau]
        if i == N - 1: break
        y = torch.tensor(np.concatenate([x, U[i], XT[i]]), dtype=torch.float32).reshape(1, 1, 30)
        with torch.no_grad(): yn = rk4_step(ode, y, dt)[0, 0, :13].numpy().astype(np.float64)
        yn[6:10] /= np.linalg.norm(yn[6:10]); X[i + 1] = yn
    return dict(t=np.arange(N) * dt, X=X, U=U, XT=XT, dt=dt, dz=dz, kind=kind, speed=speed)
