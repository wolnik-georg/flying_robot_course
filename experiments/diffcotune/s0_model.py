#!/usr/bin/env python3
"""Differentiable S0 model (geometric controller, no NS2) on the frsim plant, for DiffCoTune (Stage 2, 2026-11-11).

Derived from experiments/tuning/diff/s0_plant_torch.py (`_block`, `rollout_blocks`) and s0_torch.py (controller); the old
files are untouched. Differences to the old copy, all required by docs/plan_diffcotune_2026-11-11.md section 5b:
  * plant inertia = the exact Omar values of frsim (the old copy used rounded values), mass 0.0427 kg;
  * the thrust-law gain (THRUST_LAW_GAIN_N) is an input and differentiable, the force cap stays fixed;
  * beta = (thrust-law gain, motor lag, downwash scale, plume amplitude scale), all differentiable, parametrised as
    log-factors relative to the frozen calibration (psi = 0 is the calibration);
  * theta = the 10 S0 gains of simulator/gains/defaults.yaml, parametrised as log-factors relative to the defaults
    (phi = 0 are the defaults); positivity by construction;
  * no sensor noise, no state delay (the tuning model of the spec), no robustness variants;
  * the reference comes from frsim's FlightPlan (reference.py), so every frsim scenario (A1, A4, A8, CIRCLE, FIG8, OVAL)
    is available; solo scenarios have no top vehicle (ground effect only);
  * the 4 ms command delay reproduces frsim's float-rounding jitter of the release time (reference.command_jitter);
  * the output is the full state (position, velocity, quaternion, body rates) every `keep_blocks` * 2 ms.
Not modelled (as in the old copy, and not needed for the cost): uint16 quantisation of the motor command.

State convention of the output = frsim's state trace (EpisodeResult.state_trace): state at the START of tick i, rows
every 10 ms (i = 0, 10, 20, ...), quaternion (x, y, z, w), body rates in rad/s.
"""
import math
import sys
from pathlib import Path

import numpy as np
import torch
import torch._dynamo
torch._dynamo.config.cache_size_limit = 64   # one compiled variant per (requires_grad pattern, batch size); the default 8 is exceeded in the gradient check

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import ns2_torch as NS2  # noqa: E402
import reference as R  # noqa: E402
import s0_torch as T  # noqa: E402
from frsim import config, controllers  # noqa: E402

M, G = config.MASS_KG, config.GRAVITY
J_PLANT = tuple(float(v) for v in config.INERTIA_DIAG)           # exact Omar inertia, the same as frsim
HOVER_CMD = config.THRUST_LAW_HOVER_COMMAND                       # 0.2 N * 0.618
FMAX = config.CALIBRATED_PLANT.max_motor_force_n                  # force cap, fixed
GAIN_NAMES = controllers.TREE_GAIN_NAMES                          # Pxy Pz Dxy Dz Iz KRxy KRz Kwxy Kwz KIxy
GAIN_DEFAULT = np.array([controllers.TREE_DEFAULT_GAINS[k] for k in GAIN_NAMES])
BETA_NAMES = ('thrust_gain', 'motor_lag_ms', 'downwash_scale', 'plume_scale')
_P = config.CALIBRATED_PLANT
BETA_START = np.array([_P.thrust_law_gain_n, _P.motor_lag_ms, _P.downwash_scale, _P.plume_amplitude_n / config.PLUME_FIT_AMPLITUDE_N])
PLUME_AMP_FIT, PLUME_WIDTH, PLUME_EXP = config.PLUME_FIT_AMPLITUDE_N, _P.plume_width_m, _P.plume_exponent
ARMS = config.ROTOR_POSITIONS_M
TILT_CRASH_DEG = config.CRASH_TILT_DEG['tree']

_NET = {}


def net(dtype, device):
    key = (dtype, str(device))
    if key not in _NET:
        _NET[key] = NS2.load(NS2.DEFAULT_WEIGHTS, dtype, device)
    return _NET[key]


def qmul(a, b):
    ax, ay, az, aw = a.unbind(-1); bx, by, bz, bw = b.unbind(-1)
    return torch.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


def qexp(r):
    th2 = (r * r).sum(-1, keepdim=True); small = th2 < 1e-8
    th = torch.sqrt(torch.where(small, torch.full_like(th2, 1e-8), th2))           # safe argument: no 0/0 or inf in the backward
    s = torch.where(small, 0.5 - th2 / 48.0, torch.sin(th / 2) / th)
    return torch.cat([s * r, torch.where(small, 1 - th2 / 8.0, torch.cos(th / 2))], -1)


def gains_from(phi, dtype, device):
    """phi (B, 10) log-factors -> gain dict of the S0 controller (rows = batch), as train_s0.gains_from."""
    p = T.K(GAIN_DEFAULT, dtype, device) * torch.exp(phi)
    P, Pz, D, Dz, Iz, KRxy, KRz, Kwxy, Kwz, KI = p.unbind(-1)
    e = lambda a, b, c: torch.stack([a, b, c], -1)
    ki_z = T.K(0.03, dtype, device).expand_as(KI)
    return dict(Kpos_P=e(P, P, Pz), Kpos_D=e(D, D, Dz), Kpos_Iz=Iz, KR=e(KRxy, KRxy, KRz), Komega=e(Kwxy, Kwxy, Kwz), KI=e(KI, KI, ki_z), Kpos_I_limit=T.K(2.0, dtype, device))


def beta_from(psi, dtype, device):
    """psi (B, 4) log-factors -> dict of plant values (rows = batch): gain (N), lag (ms), dws, plume amplitude (N)."""
    b = T.K(BETA_START, dtype, device) * torch.exp(psi)
    return dict(gain=b[..., 0], lag=b[..., 1], dws=b[..., 2], plume_amp=PLUME_AMP_FIT * b[..., 3])


def _ctrl_step(g, i_pos, i_att, pos_d, vel_d, acc_d, jerk_d, pos, vel, q, gyro_dps):
    """Functional copy of s0_torch.S0.step (same arithmetic; a pure function so that torch.compile has no graph break).
    Returns thrust, moments and the new integrator states."""
    m = M; DT = T.DT
    pos_e, vel_e = pos_d - pos, vel_d - vel
    i_pos = i_pos + DT * pos_e
    iz = torch.clamp(i_pos[..., 2], -g['Kpos_I_limit'], g['Kpos_I_limit']); i_pos = torch.cat([i_pos[..., :2], iz[..., None]], -1)
    i_term = torch.cat([torch.zeros_like(iz)[..., None], torch.zeros_like(iz)[..., None], (g['Kpos_Iz'] * iz)[..., None]], -1)
    a_d = acc_d + T.K([0, 0, T.G], pos.dtype, pos.device) + g['Kpos_D'] * vel_e + g['Kpos_P'] * pos_e + i_term
    Rm = T.quat2rot(q); zw = Rm[..., :, 2]
    thrust = m * T.vdot(a_d, zw)
    on = thrust >= 0.01
    i_pos = i_pos * on[..., None]; i_att = i_att * on[..., None]
    yc = T.K([0., 1., 0.], pos.dtype, pos.device).expand_as(a_d)
    xb = T.vnorm(T.vcross(yc, a_d)); yb = T.vnorm(T.vcross(a_d, xb)); zb = T.vcross(xb, yb)
    Rd = torch.stack([xb, yb, zb], -1)
    eRM = Rd.transpose(-1, -2) @ Rm - Rm.transpose(-1, -2) @ Rd
    eR = 0.5 * torch.stack([eRM[..., 2, 1], eRM[..., 0, 2], eRM[..., 1, 0]], -1)
    om = torch.deg2rad(gyro_dps)
    c = thrust / m; B1 = c; B3 = -T.vdot(yc, zb); C3 = torch.linalg.norm(T.vcross(yc, zb), dim=-1)
    D1 = T.vdot(xb, jerk_d); D2 = -T.vdot(yb, jerk_d)
    od = torch.stack([D2 / B1, D1 / B1, (-B3 * D1) / (B1 * C3)], -1)
    snap = torch.zeros_like(a_d)
    c_dot = T.vdot(zb, jerk_d)
    E1 = T.vdot(xb, snap) - 2 * c_dot * od[..., 1] - c * od[..., 0] * od[..., 2]
    E2 = -T.vdot(yb, snap) - 2 * c_dot * od[..., 0] + c * od[..., 1] * od[..., 2]
    E3 = -od[..., 0] * od[..., 1] * T.vdot(yc, yb) - od[..., 0] * od[..., 2] * T.vdot(yc, zb)
    odd = torch.stack([E2 / B1, E1 / B1, (-B3 * E1 + B1 * E3) / (B1 * C3)], -1)
    RtRd = Rm.transpose(-1, -2) @ Rd
    om_r = (RtRd @ od[..., None])[..., 0]
    i_att = i_att + DT * eR
    Jc = T.K(T.J_CTRL, pos.dtype, pos.device); Jo = Jc * om
    u = -g['KR'] * eR - g['Komega'] * (om - om_r) - g['KI'] * i_att + T.vcross(om, Jo) - Jc * (T.vcross(om, om_r) - (RtRd @ odd[..., None])[..., 0])
    return thrust, u, i_pos, i_att


def _block(st, spk, tpk, jit, gains, pr, act_top, nets, nz=None):
    """One control period (2 ms = two 1 ms physics steps), the order of one frsim tick pair (frsim/episode.py).
    nz (Stage 3d): None = no sensor noise (as before); else a fixed noise tensor (..., 9) = position [m], velocity [m/s], gyro [deg/s] added to the
    measurements the controller sees (the plant state is not noisy); it is data, not a parameter."""
    x, v, q, w, F, i_pos, i_att, cm1, cm2, cm3, tau_pl = st
    pos_d, vel_d, acc_d, jerk_d = spk
    tp, tv = tpk
    dtype, dev = x.dtype, x.device
    if nz is None:
        th, u, i_pos, i_att = _ctrl_step(gains, i_pos, i_att, pos_d, vel_d, acc_d, jerk_d, x, v, q, torch.rad2deg(w))
    else:
        th, u, i_pos, i_att = _ctrl_step(gains, i_pos, i_att, pos_d, vel_d, acc_d, jerk_d, x + nz[..., 0:3], v + nz[..., 3:6], q, torch.rad2deg(w) + nz[..., 6:9])
    cmd = T.mixer(th, u)
    a_m = 1e-3 / (pr['lag'] * 1e-3 + 1e-3)
    Jp = T.K(J_PLANT, dtype, dev); gvec = T.K([0, 0, -G], dtype, dev)
    fext_held = torch.zeros_like(x)
    for sub in range(2):
        F_c = torch.where(jit, cm3, cm2) if sub == 0 else cm2          # command released by now (4 ms delay, float jitter)
        F_t = torch.minimum(pr['gain'][..., None] * (F_c / HOVER_CMD) ** 2, T.K(FMAX, dtype, dev))
        F = F + a_m[..., None] * (F_t - F); thrust = F.sum(-1)
        tau = torch.stack([T.ARM * (-F[..., 0] - F[..., 1] + F[..., 2] + F[..., 3]), T.ARM * (-F[..., 0] + F[..., 1] + F[..., 2] - F[..., 3]), T.T2T * (-F[..., 0] + F[..., 1] - F[..., 2] + F[..., 3])], -1)
        Rm = T.quat2rot(q)
        tau = tau + tau_pl                                              # plume torque of the previous even tick (frsim: `plume`)
        if sub == 0:                                                    # downwash force and plume torque, every 2 ms (even tick)
            if isinstance(act_top, torch.Tensor):                       # per-row top-vehicle flag (scenario batch, s0_batch.py)
                act = (tp[..., 2] > x[..., 2]) & act_top
            else:
                act = (tp[..., 2] > x[..., 2]) if act_top else torch.zeros(x.shape[:-1], dtype=torch.bool, device=dev)
            a_z = NS2.faz(nets, tp - x, tv - v, x[..., 2], v, M, act)
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
    return (x, v, q, w, F, i_pos, i_att, cmd, cm1, cm2, tau_pl)


_COMPILED = {}


def _patch_inductor_loop_split():
    """Stage 3d: the torch 2.x CPU Inductor crashes on batches of rows > 1 in `CppScheduling.try_loop_split` ('int' object has no attribute 'count').
    That pass is an optional loop-splitting optimisation; it is switched off here (identity), which changes the loop structure of the generated code only.
    Applied only when a compiled rollout has more than one row."""
    from torch._inductor.codegen import cpp as _cpp
    if not getattr(_cpp.CppScheduling, '_split_off', False):
        _cpp.CppScheduling.try_loop_split = lambda self, nodes: nodes
        _cpp.CppScheduling._split_off = True


def frsim_noise(seed, n_ticks, plant=None):
    """The sensor noise sequence of frsim (sensors.Sensors, sensors.py:15-59) for one seed, as the (n_ticks // 2, 9) array the S0 model needs
    (Stage 3d). frsim draws 16 normals per 1 ms tick in a fixed order (position 3, velocity 3, gyro 3, accelerometer 3, rpm 4 with std 0)
    from np.random.default_rng(seed); Generator.normal(0, std, n) = std * standard_normal, so the stream is reproduced exactly. The controller
    runs on even ticks (episode.py:167), so only the even ticks are kept. Accelerometer and rpm noise do not enter S0 (indi 0, no NS2)."""
    plant = config.CALIBRATED_PLANT if plant is None else plant
    z = np.random.default_rng(seed).standard_normal((n_ticks, 16))[::2]
    std = np.array([config.STATE_NOISE_POSITION_M] * 3 + [config.STATE_NOISE_VELOCITY_M_S] * 3 + [plant.gyro_noise_deg_s] * 3)
    return z[:, :9] * std



def _flat_rows(ref, idx, dtype, device):
    """Stage 8: reference attitude (x, y, z, w) and body rates at the output rows, as a (rows, 7) tensor (for the paper-faithful task cost; unused by the other costs)."""
    import reference as _R
    q, w = _R.flat_ref(ref); return torch.as_tensor(np.concatenate([q, w], -1)[np.asarray(idx)], dtype=dtype, device=device)

def rollout(ref, phi, psi, dtype=torch.float64, device='cpu', keep_blocks=5, compile=False, n_blocks=None, noise=None):
    """Closed-loop S0 flight of one reference for B parameter rows.

    ref: reference.Reference; phi (B, 10), psi (B, 4): log-factors of the gains / of beta (0 = defaults / calibration).
    compile=True: the 2 ms block is compiled with torch.compile (first call about 30 s, then about 15 x faster for the
    backward pass and a fraction of the memory; eager autograd stores about 1 MB per block, 10 GB for one 23 s flight).
    noise: None, array (n_blocks, 9) (frsim_noise; the same sequence for every parameter row) or (n_blocks, B, 9) (one sequence per row) (Stage 3d).
    Returns dict: t (rows,) scenario time [s]; x, v (rows, B, 3); q (rows, B, 4); w (rows, B, 3) [rad/s]; ref_pos (rows, 3)
    (the reference at the same ticks); final state."""
    phi = torch.as_tensor(phi, dtype=dtype, device=device); psi = torch.as_tensor(psi, dtype=dtype, device=device)
    B = phi.shape[0]; nb = (ref.n_ticks // 2) if n_blocks is None else n_blocks
    t = lambda a: torch.as_tensor(np.asarray(a), dtype=dtype, device=device)
    sp = {k: t(getattr(ref, k)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    gains = gains_from(phi, dtype, device); pr = beta_from(psi, dtype, device); nets = net(dtype, device)
    jit = torch.as_tensor(R.command_jitter(nb), device=device)
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=device)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, device)).expand(B, 3).clone()
    q = T.K([0., 0., 0., 1.], dtype, device).expand(B, 4).clone()
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=device)
    st = (x, zeros3.clone(), q, zeros3.clone(), F, zeros3.clone(), zeros3.clone(), F.clone(), F.clone(), F.clone(), zeros3.clone())
    fn = _block
    if compile:
        if B > 1:
            _patch_inductor_loop_split()
        if 'fn' not in _COMPILED:
            _COMPILED['fn'] = torch.compile(_block, dynamic=False)
        fn = _COMPILED['fn']
    nzt = None
    if noise is not None:
        nzt = torch.as_tensor(np.asarray(noise)[:nb], dtype=dtype, device=device)
        nzt = nzt[:, None, :].expand(nb, B, 9) if nzt.ndim == 2 else nzt      # (nb, 9) shared by all rows, or (nb, B, 9) one sequence per row
    rows = {'x': [], 'v': [], 'q': [], 'w': []}; idx = []
    for k in range(nb):
        i = 2 * k
        if k % keep_blocks == 0:
            rows['x'].append(st[0]); rows['v'].append(st[1]); rows['q'].append(st[2]); rows['w'].append(st[3]); idx.append(i)
        st = fn(st, (sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jit[k], gains, pr, ref.has_top, nets, None if nzt is None else nzt[k])
    idx = np.array(idx)
    return dict(t=ref.time_s[idx], idx=idx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']),
                ref_pos=sp['pos'][idx], ref_vel=sp['vel'][idx], ref_q=_flat_rows(ref, idx, dtype, device), final=st)


def tilt_deg(q):
    """Tilt angle (deg) of quaternions (..., 4) (x, y, z, w)."""
    x, y, z, w = q.unbind(-1)
    return torch.rad2deg(torch.acos(torch.clamp(1 - 2 * (x * x + y * y), -1.0, 1.0)))
