#!/usr/bin/env python3
"""Differentiable S1 model (S0 + INDI mode 3, no NS2) on the frsim plant, for DiffCoTune (Stage 5, 13 Nov 2026). `ns2=True` gives S3 (INDI 3 + NS2), see s3_model.py.

Derived from s0_model.py (plant, noise option, compiled 2 ms block, beta / theta parametrisation) and from the INDI part of experiments/tuning/diff/s1_plant_torch.py (`S1.step`; that file is
untouched; it sits on an older plant with rounded inertia and an integer rpm delay). INDI mode 3 in controller_omar_indi.c (indi & 1: a_indi = a_rpm_f - a_imu_f added to the desired acceleration for
thrust and desired attitude; indi & 2: moments + tau_rpm_f - J * angular_acc_f). Four signals pass the firmware's 2nd-order Butterworth (30 Hz, 500 Hz control rate, Q 0.7071, start 0):
  a_rpm   = sum(F_rpm) / m * z_body - g e_z   (F_rpm from the rpm telemetry, force = k * rpm^2)
  a_imu   = acceleration of the previous 1 ms physics step (incl. downwash) + 9.81 R n_acc   (accelerometer reading rotated to the world frame, noise n_acc in g)
  tau_rpm = moments of the telemetry forces, angular_acc = (gyro_k - gyro_{k-1}) / 2 ms (measured gyro incl. noise).
Differences to s0_model.py: the RPM DELAY is a fifth beta (psi[:, 4], log-factor relative to the calibration plant.rpm_delay_ms = 4.0502 ms) and FRACTIONAL: the telemetry at delay d = n + f ms is the linear
interpolation (in rpm, F -> sqrt) between the motor forces n and n + 1 ms old (frsim: sensors.Sensors.rpm with the plant value rpm_delay_fractional = 1; without it frsim truncates d to whole ms).
The history holds 16 samples (delay clamped to 0 ... 14.99 ms). The noise array has 12 columns: position, velocity, gyro, accelerometer (`frsim_noise`).
Same interface as s0_model.rollout (psi (B, 5))."""
import math
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import s0_model as S0  # noqa: E402
import s2_model as S2  # noqa: E402
import reference as R  # noqa: E402
import ns2_torch as NS2  # noqa: E402
import s0_torch as T  # noqa: E402
from frsim import config  # noqa: E402

M, G = S0.M, S0.G
J_PLANT, HOVER_CMD, FMAX, ARMS = S0.J_PLANT, S0.HOVER_CMD, S0.FMAX, S0.ARMS
GAIN_NAMES, GAIN_DEFAULT, TILT_CRASH_DEG, tilt_deg, qmul, qexp = S0.GAIN_NAMES, S0.GAIN_DEFAULT, S0.TILT_CRASH_DEG, S0.tilt_deg, S0.qmul, S0.qexp
BETA_NAMES = S0.BETA_NAMES + ('rpm_delay_ms',)
RD_START = float(config.CALIBRATED_PLANT.rpm_delay_ms)
BETA_START = np.r_[S0.BETA_START, RD_START]
N_BETA = 5
HIST = 16                                              # rpm history samples (ages 0 ... 15 ms)
FC, TS, QB = 30.0, 1.0 / 500.0, 0.7071
_COMPILED = {}


def bw_coeffs(dtype, dev):
    tau = 1.0 / (2.0 * math.pi * FC); k = math.tan(TS / (2.0 * tau)); poly = k * k + k / QB + 1.0
    return torch.tensor([2.0 * (k * k - 1.0) / poly, (k * k - k / QB + 1.0) / poly, k * k / poly, 2.0 * k * k / poly], dtype=dtype, device=dev)      # a0, a1, b0, b1


def bw_step(fs, x, c):
    """fs (..., 4) = [i0, i1, o0, o1] (firmware SecondOrderLowPass), x the new input -> new state, new output."""
    i0, i1, o0, o1 = fs.unbind(-1)
    out = c[2] * x + c[3] * i0 + c[2] * i1 - c[0] * o0 - c[1] * o1
    return torch.stack([x, i0, out, o0], -1), out


def _ctrl_step(g, i_pos, i_att, pos_d, vel_d, acc_d, jerk_d, pos, vel, q, gyro_dps, F_rpm, a_imu, fs, om_prev, bw):
    """S0 `_ctrl_step` plus INDI mode 3. Returns thrust, moments, i_pos, i_att, filter state, gyro rate (rad/s) for the next angular acceleration."""
    m = M; DT = T.DT
    pos_e, vel_e = pos_d - pos, vel_d - vel
    i_pos = i_pos + DT * pos_e
    iz = torch.clamp(i_pos[..., 2], -g['Kpos_I_limit'], g['Kpos_I_limit']); i_pos = torch.cat([i_pos[..., :2], iz[..., None]], -1)
    i_term = torch.cat([torch.zeros_like(iz)[..., None], torch.zeros_like(iz)[..., None], (g['Kpos_Iz'] * iz)[..., None]], -1)
    a_d = acc_d + T.K([0, 0, T.G], pos.dtype, pos.device) + g['Kpos_D'] * vel_e + g['Kpos_P'] * pos_e + i_term
    Rm = T.quat2rot(q); zw = Rm[..., :, 2]
    om = torch.deg2rad(gyro_dps)
    t1, t2, t3, t4 = F_rpm.unbind(-1); arm, t2t = T.ARM, T.T2T
    a_rpm = (t1 + t2 + t3 + t4)[..., None] / m * zw - T.K([0, 0, 9.81], pos.dtype, pos.device)
    tau_rpm = torch.stack([-arm * t1 - arm * t2 + arm * t3 + arm * t4, -arm * t1 + arm * t2 + arm * t3 - arm * t4, -t2t * t1 + t2t * t2 - t2t * t3 + t2t * t4], -1)
    ang = (om - om_prev) / DT
    fs, o = bw_step(fs, torch.stack([a_rpm, a_imu, tau_rpm, ang], -2), bw)
    a_indi = o[..., 0, :] - o[..., 1, :]
    thrust = m * T.vdot(a_d + a_indi, zw)
    on = thrust >= 0.01
    i_pos = i_pos * on[..., None]; i_att = i_att * on[..., None]
    yc = T.K([0., 1., 0.], pos.dtype, pos.device).expand_as(a_d)
    F_d = a_d + a_indi
    xb = T.vnorm(T.vcross(yc, F_d)); yb = T.vnorm(T.vcross(F_d, xb)); zb = T.vcross(xb, yb)
    Rd = torch.stack([xb, yb, zb], -1)
    eRM = Rd.transpose(-1, -2) @ Rm - Rm.transpose(-1, -2) @ Rd
    eR = 0.5 * torch.stack([eRM[..., 2, 1], eRM[..., 0, 2], eRM[..., 1, 0]], -1)
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
    u = u + (o[..., 2, :] - Jc * o[..., 3, :])
    return thrust, u, i_pos, i_att, fs, om


def _rpm_force(Fh, d):
    """Motor forces (B, 4) as the controller sees them: delay d (B,) ms, linear interpolation in rpm between the samples n and n + 1 ms old (Fh[-1] = age 0)."""
    d = torch.clamp(d, 0.0, HIST - 1.01); n = torch.floor(d).detach().long(); f = (d - n.to(d.dtype))[..., None]
    Fr = Fh.flip(0); ar = torch.arange(Fh.shape[1], device=Fh.device)
    ra = torch.sqrt(torch.clamp(Fr[n, ar], min=1e-12)); rb = torch.sqrt(torch.clamp(Fr[n + 1, ar], min=1e-12))
    return ((1.0 - f) * ra + f * rb) ** 2


def _block(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw):
    x, v, q, w, F, i_pos, i_att, cm1, cm2, cm3, tau_pl, fs, om_prev, a_prev, Fh = st
    pos_d, vel_d, acc_d, jerk_d = spk
    tp, tv = tpk
    dtype, dev = x.dtype, x.device
    Rn = T.quat2rot(q)
    if nz is None:
        pos_m, vel_m, gyro_n, a_imu = x, v, torch.rad2deg(w), a_prev
    else:
        pos_m, vel_m, gyro_n = x + nz[..., 0:3], v + nz[..., 3:6], torch.rad2deg(w) + nz[..., 6:9]
        a_imu = a_prev + 9.81 * (Rn @ nz[..., 9:12, None])[..., 0]                # accelerometer noise [g], world frame
    a_imu = a_imu if nz is not None else a_prev
    F_rpm = _rpm_force(Fh, pr['rd'])
    th, u, i_pos, i_att, fs, om_prev = _ctrl_step(gains, i_pos, i_att, pos_d, vel_d, acc_d, jerk_d, pos_m, vel_m, q, gyro_n, F_rpm, a_imu, fs, om_prev, bw)
    cmd = T.mixer(th, u)
    a_m = 1e-3 / (pr['lag'] * 1e-3 + 1e-3)
    Jp = T.K(J_PLANT, dtype, dev); gvec = T.K([0, 0, -G], dtype, dev)
    fext_held = torch.zeros_like(x); hist = []
    for sub in range(2):
        F_c = torch.where(jit, cm3, cm2) if sub == 0 else cm2
        F_t = torch.minimum(pr['gain'][..., None] * (F_c / HOVER_CMD) ** 2, T.K(FMAX, dtype, dev))
        F = F + a_m[..., None] * (F_t - F); thrust = F.sum(-1); hist.append(F)
        tau = torch.stack([T.ARM * (-F[..., 0] - F[..., 1] + F[..., 2] + F[..., 3]), T.ARM * (-F[..., 0] + F[..., 1] + F[..., 2] - F[..., 3]), T.T2T * (-F[..., 0] + F[..., 1] - F[..., 2] + F[..., 3])], -1)
        Rm = T.quat2rot(q)
        tau = tau + tau_pl
        if sub == 0:
            if isinstance(act_top, torch.Tensor):
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
                dFk = -pr['plume_amp'] * (0.4 / torch.clamp(tp[..., 2] - pk[..., 2], min=0.05)) ** S0.PLUME_EXP * torch.exp(-rho2 / (2 * S0.PLUME_WIDTH ** 2))
                tq = tq + torch.stack([rk[1] * dFk, -rk[0] * dFk, torch.zeros_like(dFk)], -1)
            tau_pl = tq * act[..., None].to(tq.dtype)
        a_new = gvec + Rm[..., :, 2] * (thrust / M)[..., None] + fext_held / M      # acceleration of this step = what the accelerometer reads next
        v = v + a_new * 1e-3; a_prev = a_new
        x = x + v * 1e-3
        under = x[..., 2] <= 0.0
        vz = torch.where(under, torch.clamp(v[..., 2], min=0.0), v[..., 2]); hold = (under & (thrust < M * G))[..., None]
        v = torch.cat([torch.where(hold, torch.zeros_like(v[..., :2]), v[..., :2]), vz[..., None]], -1)
        x = torch.cat([x[..., :2], torch.where(under, torch.zeros_like(x[..., 2]), x[..., 2])[..., None]], -1)
        w = w + (tau - T.vcross(w, w * Jp)) / Jp * 1e-3
        q = qmul(qexp((Rm @ w[..., None])[..., 0] * 1e-3), q); q = q / torch.linalg.norm(q, dim=-1, keepdim=True)
    Fh = torch.cat([Fh[2:], hist[0][None], hist[1][None]])
    return (x, v, q, w, F, i_pos, i_att, cmd, cm1, cm2, tau_pl, fs, om_prev, a_prev, Fh)


def frsim_noise(seed, n_ticks, plant=None):
    """frsim's sensor noise for one seed (see s0_model.frsim_noise) with the accelerometer: (n_ticks // 2, 12) = position, velocity, gyro (deg/s), accelerometer (g), even ticks only."""
    plant = config.CALIBRATED_PLANT if plant is None else plant
    z = np.random.default_rng(seed).standard_normal((n_ticks, 16))[::2]
    std = np.array([config.STATE_NOISE_POSITION_M] * 3 + [config.STATE_NOISE_VELOCITY_M_S] * 3 + [plant.gyro_noise_deg_s] * 3 + [plant.accelerometer_noise_g] * 3)
    return z[:, :12] * std



def _flat_rows(ref, idx, dtype, device):
    """Stage 8: reference attitude (x, y, z, w) and body rates at the output rows, as a (rows, 7) tensor (for the paper-faithful task cost; unused by the other costs)."""
    import reference as _R
    q, w = _R.flat_ref(ref); return torch.as_tensor(np.concatenate([q, w], -1)[np.asarray(idx)], dtype=dtype, device=device)

def rollout(ref, phi, psi, dtype=torch.float64, device='cpu', keep_blocks=5, compile=False, n_blocks=None, noise=None, ns2=False):
    """Closed-loop S1 (ns2=False) or S3 (ns2=True) flight of one reference for B rows; phi (B, 10), psi (B, 5)."""
    phi = torch.as_tensor(phi, dtype=dtype, device=device); psi = torch.as_tensor(psi, dtype=dtype, device=device)
    B = phi.shape[0]; nb = (ref.n_ticks // 2) if n_blocks is None else n_blocks
    t = lambda a: torch.as_tensor(np.asarray(a), dtype=dtype, device=device)
    sp = {k: t(getattr(ref, k)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    gains = S0.gains_from(phi, dtype, device); pr = S0.beta_from(psi[:, :4], dtype, device); pr['rd'] = RD_START * torch.exp(psi[:, 4]); nets = S0.net(dtype, device)
    jit = torch.as_tensor(R.command_jitter(nb), device=device)
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=device)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, device)).expand(B, 3).clone()
    q = T.K([0., 0., 0., 1.], dtype, device).expand(B, 4).clone()
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=device)
    st = (x, zeros3.clone(), q, zeros3.clone(), F, zeros3.clone(), zeros3.clone(), F.clone(), F.clone(), F.clone(), zeros3.clone(),
          torch.zeros(B, 4, 3, 4, dtype=dtype, device=device), zeros3.clone(), zeros3.clone(), F[None].expand(HIST, B, 4).clone())
    bw = bw_coeffs(dtype, device)
    fn = _block
    if compile:
        if B > 1:
            S0._patch_inductor_loop_split()
        if 'fn' not in _COMPILED:
            _COMPILED['fn'] = torch.compile(_block, dynamic=False)
        fn = _COMPILED['fn']
    nzt = None
    if noise is not None:
        nzt = torch.as_tensor(np.asarray(noise)[:nb], dtype=dtype, device=device)
        nzt = nzt[:, None, :].expand(nb, B, nzt.shape[-1]) if nzt.ndim == 2 else nzt
    active = torch.full((B,), bool(ref.has_top), dtype=torch.bool, device=device); hold = torch.zeros(B, dtype=dtype, device=device)
    rows = {'x': [], 'v': [], 'q': [], 'w': []}; idx = []
    for k in range(nb):
        i = 2 * k
        if k % keep_blocks == 0:
            rows['x'].append(st[0]); rows['v'].append(st[1]); rows['q'].append(st[2]); rows['w'].append(st[3]); idx.append(i)
        acc = sp['acc'][i]
        if ns2:
            if k % S2.NS2_EVERY_BLOCKS == 0:
                hold = S2.ns2_hold(nets, sp, k, i, st[0], st[1], nzt, active, dtype, device)
            acc = acc - torch.cat([zeros3[..., :2], hold[..., None]], -1)
        st = fn(st, (sp['pos'][i], sp['vel'][i], acc, sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jit[k], gains, pr, ref.has_top, nets, None if nzt is None else nzt[k], bw)
    idx = np.array(idx)
    return dict(t=ref.time_s[idx], idx=idx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']),
                ref_pos=sp['pos'][idx], ref_vel=sp['vel'][idx], ref_q=_flat_rows(ref, idx, dtype, device), final=st)
