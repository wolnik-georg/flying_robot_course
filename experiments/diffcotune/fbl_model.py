#!/usr/bin/env python3
"""Differentiable FBL model on the frsim plant, for DiffCoTune (Stage F1, 17 Nov 2026). Same interface as s0_model.py.

Controller law, ROM, 4-neuron net, stock rate / yaw PIDs and legacy mixer: copied from the closest old torch copy,
lab:experiments/tuning/diff/fbl_plant_torch_v2o.py (authors-aligned firmware, Omar inertia; checked against the firmware by
check_fbl_equiv_v2o.py in the old stage). Plant, beta, reference, 4 ms command delay with float jitter, top-vehicle downwash and
plume: as s0_model.py (the frsim-calibrated plant; thrust-law gain, motor lag, downwash scale, plume scale as inputs).
Differences to v2o, all required to follow frsim (simulator/frsim/episode.py:167-171, firmware/host/oot_host.c:337):
  * timing, found by comparing the model's commands with the firmware's on frsim states (fbl/diag_law.py, diag_w.py): frsim calls the FBL
    firmware on every 1 ms tick (episode.py:167), but the controller state (alpha-beta-gamma estimator, position integral, yaw set point and
    rate-PID integrators) advances only on the even ticks, with dt = 2 ms (the odd-tick call changes no state and its output is not used);
    the model therefore has ONE controller step per 2 ms block, as v2o had. (The host source oot_host.c:337 adds 2000 us to the clock per call;
    why the odd calls do not advance the state was not traced further.) Command of the even tick: queued, 4 ms delay with float jitter.
  * the plant is the frsim-calibrated one of s0_model (v2o: old plant_core plant with its own constants).
  * theta = 6 log-factors relative to preset 1 (FBL_DEFAULT_GAINS); kr_xy 59.5 / kr_z 35.0 are not tuned and unused with inner 1.
  * variants (env DIFFCOTUNE_FBL_VARIANT, or the `variant` argument): base = no residual (dw_en 0); rom = ROM only (dw_en 1, nn_en 0);
    both = ROM + net (dw_en 1, nn_en 1, the frsim default); net = network without ROM (NOT a firmware mode, no frsim reference; defined as
    "ROM off + network on", an open decision of the spec, item 10).
  * solo scenarios (no top vehicle): no peer, residual zero (frsim: oot_set_peer_count(0)).
No sensor noise (optional noise array (n_ticks, 9) per 1 ms tick: position, velocity, gyro; not checked in Stage F1), no state delay.
Not modelled: nothing beyond v2o's list (hand-over / latch / trip do not exist in this firmware).
"""
import math
import os
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import reference as R  # noqa: E402
import s0_model as S0  # noqa: E402
import s0_torch as T  # noqa: E402
from frsim import config, controllers  # noqa: E402

M, G = S0.M, S0.G
J_PLANT = S0.J_PLANT
HOVER_CMD, FMAX = S0.HOVER_CMD, S0.FMAX
GAIN_NAMES = controllers.FBL_GAIN_NAMES                                   # kp_xy kv_xy ki_xy kp_z kv_z ki_z
GAIN_DEFAULT = np.array([controllers.FBL_DEFAULT_GAINS[k] for k in GAIN_NAMES])
BETA_NAMES, BETA_START = S0.BETA_NAMES, S0.BETA_START
N_GAINS, N_BETA = 6, 4
PLUME_AMP_FIT, PLUME_WIDTH, PLUME_EXP, ARMS = S0.PLUME_AMP_FIT, S0.PLUME_WIDTH, S0.PLUME_EXP, S0.ARMS
TILT_CRASH_DEG = config.CRASH_TILT_DEG['fbl']
VARIANTS = ('base', 'rom', 'net', 'both')
VARIANT = os.environ.get('DIFFCOTUNE_FBL_VARIANT', 'both')
qmul, qexp = S0.qmul, S0.qexp

DT_LAW = 2e-3                                                             # dt of the FBL law and of the stock PIDs: the controller advances once per 2 ms (even ticks)
DT_PID = 2e-3                                                             # dt of the stock rate / yaw PIDs
ROM_ARM, ROM_RHO, ROM_G, ROM_BD, ROM_Z0C, ROM_ALPHA, ROM_S, ROM_RPROP, ROM_NG = 0.050, 1.293, 9.81, 6.84, 3.08, 0.414213562, 0.0997, 0.0225, 10
ROM_COS = [1.0, 7.660444431e-01, 1.736481777e-01, -5.0e-01, -9.396926208e-01, -9.396926208e-01, -5.0e-01, 1.736481777e-01, 7.660444431e-01, 1.0]
BETA_ABG, GAMMA_ABG, C_D = 0.4, 0.1, 0.55          # firmware beta, gamma (fbl.beta, fbl.gamma defaults); C_D as controllers.configure sets it
THRUST_MAX_N = 0.2; MAX_THRUST = 4 * THRUST_MAX_N; U16 = 65535.0; IDLE_COUNTS = 7000.0
RATE_KP, RATE_KI, RATE_KD, RATE_IL = (200.0, 200.0, 120.0), (400.0, 400.0, 16.7), (2.5, 2.5, 0.0), (33.3, 33.3, 166.7)
YAW_KP, YAW_KI, YAW_KD, YAW_IL = 6.0, 1.0, 0.35, 360.0
SG_R = float(os.environ.get('FBL_SG_R', '0'))                             # Stage F2 decision D1 (17 Nov): stop-gradient radius off by default (exact gradient, F1: 18 of 18 rows PASS); v2o used 0.03 (env FBL_SG_R)
QUANT = bool(int(os.environ.get('FBL_QUANT', '0')))                       # Stage F2 decision D5: quantisation off in the fit and the tuning step (as S0-S3, s0_model.py: not modelled); FBL_QUANT=1 = integer truncations of the firmware in the forward pass, straight-through gradient
GYRO_LPF = os.environ.get('FBL_GYRO_LPF', '1') != '0'                      # Stage F2c (E1): the drone's gyro driver low-pass (2nd order, 80 Hz at 1 kHz, hal/src/sensors_bmi088_bmp3xx.c:140, :339, :555) on the measured gyro, as frsim/sensors.py (FRSIM_FBL_GYRO_LPF)
_FR = 1000.0 / 80.0; _OHM = math.tan(math.pi / _FR); _C = 1.0 + 2.0 * math.cos(math.pi / 4.0) * _OHM + _OHM * _OHM
LPF_B0 = _OHM * _OHM / _C; LPF_B1 = 2.0 * LPF_B0; LPF_A1 = 2.0 * (_OHM * _OHM - 1.0) / _C; LPF_A2 = (1.0 - 2.0 * math.cos(math.pi / 4.0) * _OHM + _OHM * _OHM) / _C   # filter.c lpf2pSetCutoffFreq
NFS = 29                                                                  # fst: vel_est 0:3, acc_est 3:6, int_err 6:9, init 9, peer_prev 10:13, peer_vel 13:16, peer_pos 16:19, peer_cnt 19, yaw_sp 20, rate_int 21:24, rate_prev 24:27, yaw_int 27, yaw_prev 28
NN = dict(w1=[[0.869367301, -0.0794358477], [-2.96447873, 0.502544641], [0.36294508, -0.348767281], [-0.80364579, -0.399012148]], b1=[-0.0260919183, -0.538594186, 0.26444912, 0.0790786892],
          w2=[[-0.278275579, -0.596521318, -0.445636719, -0.226835236], [0.238675922, 0.184005216, -0.162619084, -0.501317501], [-0.832299173, -0.86985296, -0.501201808, 0.467404276]], b2=[-0.35890916, -0.0474214554, -0.237855569])


_DBG = None                                                               # diagnosis hook (Stage F1): list that fbl_step appends the rate-PID internals to


def flags(variant):
    """(residual on, ROM on, net on) of a variant name."""
    assert variant in VARIANTS, 'variant must be one of %s' % (VARIANTS,)
    return variant != 'base', variant in ('rom', 'both'), variant in ('net', 'both')


def gains_from(phi, dtype, device):
    """phi (B, 6) log-factors -> dict of (B,) gains around preset 1."""
    p = T.K(GAIN_DEFAULT, dtype, device) * torch.exp(phi)
    return {k: p[..., i] for i, k in enumerate(GAIN_NAMES)}


def beta_from(psi, dtype, device):
    return S0.beta_from(psi, dtype, device)


def euler_deg(q):
    x, y, z, w = q.unbind(-1)
    return torch.rad2deg(torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap180(a): return a - 360.0 * torch.round(a / 360.0)
def _sqrt0(x): return torch.sqrt(torch.clamp(x, min=1e-30))
def _q(x): return x + (torch.trunc(x) - x).detach()


def nn_pair(p, v, pt, vt, dt_, dev):
    c = lambda a: T.K(a, dt_, dev)
    w1, b1, w2, b2 = c(NN['w1']), c(NN['b1']), c(NN['w2']), c(NN['b2'])
    d = p - pt; dd = v - vt; z0 = d[..., 2].abs(); dz0 = torch.sign(d[..., 2]) * dd[..., 2]
    r0 = _sqrt0(d[..., 0] ** 2 + d[..., 1] ** 2); dr0 = (d[..., 0] * dd[..., 0] + d[..., 1] * dd[..., 1]) / torch.clamp(r0, min=1e-12)
    h = torch.tanh(w1[:, 0] * r0[..., None] + w1[:, 1] * z0[..., None] + b1)
    gk = (1 - h * h) * (w1[:, 0] * dr0[..., None] + w1[:, 1] * dz0[..., None])
    f = h @ w2.T + b2; df = gk @ w2.T
    rh = -d; rhd = -dd; nrm = torch.linalg.norm(rh, dim=-1, keepdim=True) + 1e-8
    x0 = rh / nrm; x0p = (x0 * rhd).sum(-1, keepdim=True); dx0 = (rhd - x0 * x0p) / nrm
    y0 = torch.stack([-x0[..., 1], x0[..., 0]], -1); dy0 = torch.stack([-dx0[..., 1], dx0[..., 0]], -1)
    yn0 = torch.clamp(torch.linalg.norm(y0, dim=-1, keepdim=True), min=1e-8); yn = y0 / yn0; ynp = (yn * dy0).sum(-1, keepdim=True)
    dyn = (dy0 - yn * ynp) / yn0
    x1 = torch.stack([yn[..., 1], -yn[..., 0]], -1); dx1 = torch.stack([dyn[..., 1], -dyn[..., 0]], -1)
    xn0 = torch.clamp(torch.linalg.norm(x1, dim=-1, keepdim=True), min=1e-8); xn = x1 / xn0; xnp = (xn * dx1).sum(-1, keepdim=True)
    dxn = (dx1 - xn * xnp) / xn0
    f0, f1, f2 = f[..., 0:1], f[..., 1:2], f[..., 2]; g0, g1, g2 = df[..., 0:1], df[..., 1:2], df[..., 2]
    acc_xy = xn * f0 + yn * f1; jerk_xy = dxn * f0 + dyn * f1 + xn * g0 + yn * g1
    return torch.cat([acc_xy, f2[..., None]], -1), torch.cat([jerk_xy, g2[..., None]], -1)


def rom_force(z0, r0, dt_, dev):
    """fbl_rom_force: F / mass of the authors' ROM and its partials d/dz0, d/dr0 (analytic), 10 x 10 grid; z0, r0 (B,)."""
    c = lambda a: T.K(a, dt_, dev)
    cth = c(ROM_COS)
    l = 2.0 * (ROM_ARM + ROM_RPROP)
    K = (G / (2.0 * ROM_RHO * math.pi * ROM_RPROP ** 2 * 4.0)) * ROM_BD ** 2
    quad = ROM_ARM + 0.025 + 0.001; j = c(list(range(ROM_NG))); rj = quad * j / (ROM_NG - 1)
    mid = 0.5 * (rj[:-1] + rj[1:]); wb = torch.cat([torch.zeros(1, dtype=dt_, device=dev), mid]) + torch.cat([mid, torch.zeros(1, dtype=dt_, device=dev)])
    scale = 0.25 * (quad / (ROM_NG - 1)) * (2.0 * math.pi / (ROM_NG - 1))
    rowf = torch.ones(ROM_NG, dtype=dt_, device=dev); rowf[1:-1] = 2.0
    zeta = (z0 / l + ROM_Z0C)[..., None, None]; lSz = l * ROM_S * zeta
    r0_ = r0[..., None, None]; r = rj[None, None, :]; ct = cth[None, :, None]
    rp2 = torch.clamp(r0_ ** 2 + r ** 2 + 2.0 * r0_ * r * ct, min=0.0); Rpt = _sqrt0(rp2)
    eta = Rpt / lSz; e2 = eta * eta; f = 1.0 + ROM_ALPHA * e2; f2 = f * f
    Vsq = K / (zeta * zeta * f2 * f2)
    dVz = Vsq / (zeta * l) * (-2.0 + 8.0 * ROM_ALPHA * e2 / f)
    dR = (r0_ + r * ct) / torch.clamp(Rpt, min=1e-15)
    dVr = Vsq * (-8.0 * ROM_ALPHA * eta / f) / lSz * dR
    w = rowf[None, :, None] * wb[None, None, :]
    coeff = 0.5 * C_D * ROM_RHO * scale
    return coeff * (w * Vsq).sum((-1, -2)), coeff * (w * dVz).sum((-1, -2)), coeff * (w * dVr).sum((-1, -2))


def residual(p, v, pt, vt, dt_, dev, rom_on, net_on):
    """acc, jerk residual (B, 3) of one peer: ROM (z only) and / or net, no gate, no fade (v2o)."""
    d = p - pt; dd = v - vt
    z0 = d[..., 2].abs(); dz0 = torch.sign(d[..., 2]) * dd[..., 2]
    r0 = _sqrt0(d[..., 0] ** 2 + d[..., 1] ** 2); dr0 = (d[..., 0] * dd[..., 0] + d[..., 1] * dd[..., 1]) / torch.clamp(r0, min=1e-12)
    acc = torch.zeros_like(p); jerk = torch.zeros_like(p)
    if rom_on:
        F, dFz, dFr = rom_force(z0, r0, dt_, dev)
        zero = torch.zeros_like(F)
        acc = acc + torch.stack([zero, zero, -F], -1); jerk = jerk + torch.stack([zero, zero, -(dFz * dz0 + dFr * dr0)], -1)
    if net_on:
        an, jn = nn_pair(p, v, pt, vt, dt_, dev)
        acc, jerk = acc + an, jerk + jn
    if SG_R > 0:
        m = (r0 < SG_R)[..., None]; acc = torch.where(m, acc.detach(), acc); jerk = torch.where(m, jerk.detach(), jerk)
    return acc, jerk


def fbl_step(fst, g, ref, xm, vm, q, gyro_dps, tp, feed, has_top, res_on, rom_on, net_on):
    """One FBL controller call (dt 2 ms). fst (B, NFS); g dict of (B,) gains; ref = (pos, vel, acc, jerk); tp top position (fed to the peer estimator when `feed`).
    Returns the new fst and the motor command in N (B, 4) (the caller uses it only on even ticks)."""
    dt_, dev = xm.dtype, xm.device
    pos_d, vel_d, acc_d, jerk_d = ref
    first = (fst[..., 9] < 0.5)
    vel_est, acc_est, int_err = fst[..., 0:3], fst[..., 3:6], fst[..., 6:9]
    fm = first[..., None]
    vel_est = torch.where(fm, vm, vel_est); acc_est = torch.where(fm, torch.zeros_like(acc_est), acc_est)
    d = torch.where(first, torch.zeros_like(xm[..., 0]), torch.full_like(xm[..., 0], DT_LAW))[..., None]
    dsafe = torch.clamp(d, min=1e-12)
    vel_pred = vel_est + d * acc_est; resd = vm - vel_pred
    upd = (d > 0)
    vel_est = torch.where(upd, vel_pred + BETA_ABG * resd, vel_est); acc_est = torch.where(upd, acc_est + (GAMMA_ABG / dsafe) * resd, acc_est)
    e = xm - pos_d; ev = vm - vel_d
    int_err = int_err + e * d
    kp = torch.stack([g['kp_xy'], g['kp_xy'], g['kp_z']], -1); kv = torch.stack([g['kv_xy'], g['kv_xy'], g['kv_z']], -1); ki = torch.stack([g['ki_xy'], g['ki_xy'], g['ki_z']], -1)
    acc_cmd = acc_d - ki * int_err - kp * e - kv * ev
    jerk_cmd = jerk_d - ki * e - kp * ev - kv * (acc_est - acc_d)
    peer_prev, peer_vel, peer_pos, cnt = fst[..., 10:13], fst[..., 13:16], fst[..., 16:19], fst[..., 19]
    if has_top:
        tp = tp.expand_as(xm)
        if feed:
            c0 = (cnt < 0.5)[..., None]; c1 = ((cnt >= 0.5) & (cnt < 1.5))[..., None]
            newv = torch.where(c0 | c1, torch.zeros_like(peer_vel), (tp - peer_prev) * 100.0)
            peer_vel = newv; peer_prev = tp; peer_pos = tp; cnt = cnt + 1.0
        if res_on:
            res_acc, res_jerk = residual(xm, vm, peer_pos, peer_vel, dt_, dev, rom_on, net_on)
        else:
            res_acc = torch.zeros_like(xm); res_jerk = torch.zeros_like(xm)
    else:
        res_acc = torch.zeros_like(xm); res_jerk = torch.zeros_like(xm)
    Rm = T.quat2rot(q); xb, yb, zb = Rm[..., :, 0], Rm[..., :, 1], Rm[..., :, 2]
    f = acc_cmd - res_acc + T.K([0., 0., G], dt_, dev)
    th = M * (f * zb).sum(-1)
    jw = M * (jerk_cmd - res_jerk)
    jb = (Rm.transpose(-1, -2) @ jw[..., None])[..., 0]
    inv = 1.0 / torch.clamp(th, min=0.01)
    wx = -jb[..., 1] * inv; wy = jb[..., 0] * inv
    den = xb[..., 0]
    wz = torch.where(den.abs() >= 1e-6, wx * zb[..., 0] / torch.where(den.abs() >= 1e-6, den, torch.ones_like(den)), torch.zeros_like(wx))
    thr = torch.clamp(th, 0.0, MAX_THRUST)
    yaw = euler_deg(q); gx, gy, gz = gyro_dps.unbind(-1)
    yaw_sp = torch.where(first, yaw, fst[..., 20])
    yaw_sp = wrap180(yaw_sp + torch.rad2deg(wz) * DT_PID)
    meas = torch.stack([gx, -gy, gz], -1); des = torch.stack([torch.rad2deg(wx), -torch.rad2deg(wy), torch.zeros_like(wx)], -1)
    yint0 = torch.where(first, torch.zeros_like(yaw), fst[..., 27]); yprev = torch.where(first, yaw, fst[..., 28])
    yerr = wrap180(yaw_sp - yaw); ydelta = wrap180(-(yaw - yprev))
    yint = torch.clamp(yint0 + yerr * DT_PID, -YAW_IL, YAW_IL)
    ry = YAW_KP * yerr + YAW_KD * ydelta / DT_PID + YAW_KI * yint
    des = torch.cat([des[..., :2], ry[..., None]], -1)
    rint0 = torch.where(fm, torch.zeros_like(meas), fst[..., 21:24]); rprev = torch.where(fm, meas, fst[..., 24:27])
    err = des - meas
    kpR, kiR, kdR, ilR = T.K(RATE_KP, dt_, dev), T.K(RATE_KI, dt_, dev), T.K(RATE_KD, dt_, dev), T.K(RATE_IL, dt_, dev)
    rint = torch.maximum(torch.minimum(rint0 + err * DT_PID, ilR), -ilR)
    out = kpR * err + kdR * (-(meas - rprev)) / DT_PID + kiR * rint
    out = torch.clamp(out, -32767.0, 32767.0)
    if QUANT: out = _q(out)
    roll, pitch, yawo = out[..., 0], out[..., 1], -out[..., 2]
    if _DBG is not None: _DBG.append(dict(out=out.detach().clone(), des=des.detach().clone(), meas=meas.detach().clone(), rint=rint.detach().clone(), w=torch.stack([wx, wy, wz], -1).detach().clone(), acc_cmd=acc_cmd.detach().clone(), jerk_cmd=jerk_cmd.detach().clone(), acc_est=acc_est.detach().clone(), res_jerk=res_jerk.detach().clone(), res_acc=res_acc.detach().clone()))
    thrust_c = thr / MAX_THRUST * U16
    r_, p_ = roll / 2.0, pitch / 2.0
    if QUANT: r_, p_ = _q(r_), _q(p_)
    mo = torch.stack([thrust_c - r_ + p_ + yawo, thrust_c - r_ - p_ - yawo, thrust_c + r_ - p_ + yawo, thrust_c + r_ + p_ - yawo], -1)
    if QUANT: mo = _q(mo)
    red = (mo.max(-1).values - U16).clamp(min=0.0)
    cmd = (mo - red[..., None]).clamp(min=IDLE_COUNTS) / U16 * THRUST_MAX_N
    new = torch.cat([vel_est, acc_est, int_err, torch.ones_like(yaw)[..., None], peer_prev, peer_vel, peer_pos, cnt[..., None], yaw_sp[..., None], rint, meas, yint[..., None], yaw[..., None]], -1)
    return new, cmd


def _physics(x, v, q, w, F, tau_pl, fext_held, sub, cm2, cm3, jit, pr, tpk, act_top, nets, Jp, gvec, a_m):
    F_c = torch.where(jit, cm3, cm2) if sub == 0 else cm2
    F_t = torch.minimum(pr['gain'][..., None] * (F_c / HOVER_CMD) ** 2, T.K(FMAX, x.dtype, x.device))
    F = F + a_m[..., None] * (F_t - F); thrust = F.sum(-1)
    tau = torch.stack([T.ARM * (-F[..., 0] - F[..., 1] + F[..., 2] + F[..., 3]), T.ARM * (-F[..., 0] + F[..., 1] + F[..., 2] - F[..., 3]), T.T2T * (-F[..., 0] + F[..., 1] - F[..., 2] + F[..., 3])], -1)
    Rm = T.quat2rot(q)
    tau = tau + tau_pl
    if sub == 0:
        tp, tv = tpk
        act = (tp[..., 2] > x[..., 2]) if act_top else torch.zeros(x.shape[:-1], dtype=torch.bool, device=x.device)
        a_z = S0.NS2.faz(nets, tp - x, tv - v, x[..., 2], v, M, act)
        fz = a_z * M * pr['dws']
        fext_held = torch.cat([torch.zeros_like(x[..., :2]), fz[..., None]], -1)
        tq = torch.zeros_like(x)
        for rk in ARMS:
            pk = x + Rm[..., :, 0] * rk[0] + Rm[..., :, 1] * rk[1]
            rho2 = (pk[..., 0] - tp[..., 0]) ** 2 + (pk[..., 1] - tp[..., 1]) ** 2
            dFk = -pr['plume_amp'] * (0.4 / torch.clamp(tp[..., 2] - pk[..., 2], min=0.05)) ** PLUME_EXP * torch.exp(-rho2 / (2 * PLUME_WIDTH ** 2))
            tq = tq + torch.stack([rk[1] * dFk, -rk[0] * dFk, torch.zeros_like(dFk)], -1)
        tau_pl = tq * act[..., None].to(tq.dtype)
    v = v + (gvec + Rm[..., :, 2] * (thrust / M)[..., None] + fext_held / M) * 1e-3
    x = x + v * 1e-3
    under = x[..., 2] <= 0.0
    vz = torch.where(under, torch.clamp(v[..., 2], min=0.0), v[..., 2]); hold = (under & (thrust < M * G))[..., None]
    v = torch.cat([torch.where(hold, torch.zeros_like(v[..., :2]), v[..., :2]), vz[..., None]], -1)
    x = torch.cat([x[..., :2], torch.where(under, torch.zeros_like(x[..., 2]), x[..., 2])[..., None]], -1)
    w = w + (tau - T.vcross(w, w * Jp)) / Jp * 1e-3
    q = qmul(qexp((Rm @ w[..., None])[..., 0] * 1e-3), q); q = q / torch.linalg.norm(q, dim=-1, keepdim=True)
    return x, v, q, w, F, tau_pl, fext_held


NFS_ALL = NFS + 6                                                         # fst + the two delay elements of the gyro low-pass (3 + 3)


def _lpf(d1, d2, s):
    """filter.c lpf2pApply on a (B, 3) sample: returns (output, new d1, new d2)."""
    d0 = s - d1 * LPF_A1 - d2 * LPF_A2
    return d0 * LPF_B0 + d1 * LPF_B1 + d2 * LPF_B0, d0, d1


def _block(st, spk0, tpk, jit, gains, pr, act_top, nets, feed, flg, nz=None):
    """One block = frsim ticks 2k and 2k+1: peer feed (every 5th block), FBL step on the state at the start of tick 2k, command queued (4 ms delay), two 1 ms physics steps
    (downwash force and plume torque on the first)."""
    x, v, q, w, F, fst, cm1, cm2, cm3, tau_pl = st
    res_on, rom_on, net_on = flg
    dtype, dev = x.dtype, x.device
    a_m = 1e-3 / (pr['lag'] * 1e-3 + 1e-3)
    Jp = T.K(J_PLANT, dtype, dev); gvec = T.K([0, 0, -G], dtype, dev)
    xm, vm, gy = x, v, torch.rad2deg(w)
    if nz is not None: xm, vm, gy = x + nz[..., 0:3], v + nz[..., 3:6], gy + nz[..., 6:9]
    fst, gl = fst[..., :NFS], fst[..., NFS:]
    if GYRO_LPF: gy, d1, d2 = _lpf(gl[..., 0:3], gl[..., 3:6], gy)                       # tick 2k: the gyro the controller reads
    fst, cmd = fbl_step(fst, gains, spk0, xm, vm, q, gy, tpk[0], feed, act_top, res_on, rom_on, net_on)
    fext_held = torch.zeros_like(x)
    x, v, q, w, F, tau_pl, fext_held = _physics(x, v, q, w, F, tau_pl, fext_held, 0, cm2, cm3, jit, pr, tpk, act_top, nets, Jp, gvec, a_m)
    if GYRO_LPF:                                                                          # tick 2k + 1: the filter runs at 1 kHz, the controller does not read this sample
        gy1 = torch.rad2deg(w)
        if nz is not None: gy1 = gy1 + nz[..., 15:18]
        _, d1, d2 = _lpf(d1, d2, gy1); fst = torch.cat([fst, d1, d2], -1)
    else: fst = torch.cat([fst, gl], -1)
    x, v, q, w, F, tau_pl, fext_held = _physics(x, v, q, w, F, tau_pl, fext_held, 1, cm2, cm3, jit, pr, tpk, act_top, nets, Jp, gvec, a_m)
    return (x, v, q, w, F, fst, cmd, cm1, cm2, tau_pl)


_COMPILED = {}


def rollout(ref, phi, psi, dtype=torch.float64, device='cpu', keep_blocks=5, compile=False, n_blocks=None, noise=None, variant=None):
    """Closed-loop FBL flight of one reference for B parameter rows (same return as s0_model.rollout).
    phi (B, 6), psi (B, 4): log-factors of the gains (preset 1) / of beta (calibration). noise: None (not used in Stage F1)."""
    variant = variant or VARIANT; flg = flags(variant)
    phi = torch.as_tensor(phi, dtype=dtype, device=device); psi = torch.as_tensor(psi, dtype=dtype, device=device)
    B = phi.shape[0]; nb = (ref.n_ticks // 2) if n_blocks is None else n_blocks
    t = lambda a: torch.as_tensor(np.asarray(a), dtype=dtype, device=device)
    sp = {k: t(getattr(ref, k)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    gains = gains_from(phi, dtype, device); pr = beta_from(psi, dtype, device); nets = S0.net(dtype, device)
    jit = torch.as_tensor(R.command_jitter(nb), device=device)
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=device)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, device)).expand(B, 3).clone()
    q = T.K([0., 0., 0., 1.], dtype, device).expand(B, 4).clone()
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=device)
    st = (x, zeros3.clone(), q, zeros3.clone(), F, torch.zeros(B, NFS_ALL, dtype=dtype, device=device), F.clone(), F.clone(), F.clone(), zeros3.clone())
    fn = _block
    if compile:
        if B > 1:
            S0._patch_inductor_loop_split()          # the CPU Inductor crashes on batches of rows > 1 in try_loop_split (Stage 3d, s0_model)
        key = (flg, ref.has_top)
        if key not in _COMPILED:
            _COMPILED[key] = torch.compile(_block, dynamic=False)
        fn = _COMPILED[key]
    nzt = None
    if noise is not None:          # (n_blocks, 18) shared by all rows, or (n_blocks, B, 18) one sequence per row (frsim_noise)
        nzt = torch.as_tensor(np.asarray(noise)[:nb], dtype=dtype, device=device)
        nzt = nzt[:, None, :].expand(nb, B, 18) if nzt.ndim == 2 else nzt
    rows = {'x': [], 'v': [], 'q': [], 'w': []}; idx = []
    for k in range(nb):
        i = 2 * k
        if k % keep_blocks == 0:
            rows['x'].append(st[0]); rows['v'].append(st[1]); rows['q'].append(st[2]); rows['w'].append(st[3]); idx.append(i)
        st = fn(st, (sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jit[k], gains, pr, ref.has_top, nets, k % 5 == 0, flg, None if nzt is None else nzt[k])
    idx = np.array(idx)
    check_finite(ref, rows, 'rollout')
    return dict(t=ref.time_s[idx], idx=idx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']),
                ref_pos=sp['pos'][idx], ref_vel=sp['vel'][idx], ref_q=S0._flat_rows(ref, idx, dtype, device), final=st)


def frsim_noise(seed, n_ticks, plant=None):
    """The sensor noise sequence of frsim's FBL for one seed as the (n_ticks // 2, 9) array the model needs (position 3 [m], velocity 3 [m/s], gyro 3 [deg/s]).
    frsim/sensors.py: one np.random.default_rng(seed) per episode; per 1 ms tick Sensors.state() draws position (3, std STATE_NOISE_POSITION_M) and velocity (3, STATE_NOISE_VELOCITY_M_S),
    then Sensors.gyro() draws 3 (std plant.gyro_noise_deg_s * variant scale, 1 for 'nominal'); accelerometer and rpm are drawn only for the tree controllers (episode.py: `if controller.tree`),
    so FBL draws 9 per tick. Generator.normal(0, std, n) = std * standard_normal, so the stream is reproduced exactly. The controller advances on even ticks only (episode.py:167, controllerOmarIndi
    rate gate), so only the even ticks are kept (as s0_model.frsim_noise does for S0)."""
    plant = config.CALIBRATED_PLANT if plant is None else plant
    z = np.random.default_rng(seed).standard_normal((n_ticks, 9))
    std = np.array([config.STATE_NOISE_POSITION_M] * 3 + [config.STATE_NOISE_VELOCITY_M_S] * 3 + [plant.gyro_noise_deg_s] * 3)
    nb = n_ticks // 2; z = z[:2 * nb].reshape(nb, 18)
    return z * np.concatenate([std, std])          # Stage F2c: (n_blocks, 18) = tick 2k (9) and tick 2k+1 (9); the gyro low-pass needs the gyro noise of the odd ticks too (columns 15:18)


def r0_min(ref, x_rows):
    """Smallest horizontal distance [m] between the vehicle and the top vehicle over the recorded rows (None for solo scenarios)."""
    if not ref.has_top: return None
    n = x_rows.shape[0]; tp = torch.as_tensor(np.asarray(ref.top_pos)[10 * np.arange(n)], dtype=x_rows.dtype)          # rows are every 10 ticks (keep_blocks 5)
    d = x_rows - tp[:, None, :]
    return float(torch.sqrt((d[..., 0] ** 2 + d[..., 1] ** 2)).min())


def check_finite(ref, rows, where, label=''):
    """Decision D1 (F2): a non-finite state stops the run with scenario, time of the first bad row and the smallest r0; no silent clamp."""
    xs = torch.stack(rows['x']).detach()
    bad = ~torch.isfinite(xs).all(-1).all(-1)
    if bad.any():
        k = int(torch.nonzero(bad)[0]); good = xs[~bad]
        raise FloatingPointError('FBL model: non-finite state in %s (%s, scenario %s, first bad row %d, scenario time %.3f s, r0_min over the finite rows %s)' % (where, label, getattr(ref, 'scenario', '?'), k, float(ref.time_s[min(2 * 5 * k, len(ref.time_s) - 1)]), None if (not ref.has_top or len(good) == 0) else '%.4f m' % r0_min(ref, good)))


def tilt_deg(q):
    return S0.tilt_deg(q)
