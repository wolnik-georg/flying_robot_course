#!/usr/bin/env python3
"""Differentiable rigid-body plant for S0 (solo bottom drone, no downwash), batched, mirroring experiments/tuning/plant_core.py tree mode at 1 ms:
command delay 4 ms, per-motor first-order lag, quadratic thrust law with full-scale limit fmax, tilt-to-acceleration gain kt, ground contact, takeoff / landing ramps.
Not modelled here (yet): state delay md, rpm path (not used by S0), plume / downwash (option A tunes without it)."""
import numpy as np, torch
import torch._dynamo
torch._dynamo.config.cache_size_limit = 64          # one compiled variant per (batch size, grad mode, requires_grad pattern); the default 8 was exceeded and fell back to eager
import s0_torch as T
import ns2_torch as NS2

M, G = 0.0427, 9.81
J_FBL = (2.395e-5, 2.395e-5, 3.235e-5)                   # plant body inertia of the 'fbl' plant (plant_core.plant J="fbl")
J_OMAR = (1.657e-5, 1.666e-5, 2.926e-5)                  # plant body inertia of the S0-S3 jobs (run_job: extra J='omar'); this is the plant of record for tuning
J_PLANT = J_OMAR
ARMS_T = (T.ARM, T.ARM, T.T2T)
ARMS = [(0.0354, -0.0354), (-0.0354, -0.0354), (-0.0354, 0.0354), (0.0354, 0.0354)]          # plant_core ARMS: rotor positions in the body frame [m]


def qmul(a, b):
    ax, ay, az, aw = a.unbind(-1); bx, by, bz, bw = b.unbind(-1)
    return torch.stack([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz], -1)


def qexp(r):
    th2 = (r * r).sum(-1, keepdim=True); small = th2 < 1e-8
    th = torch.sqrt(torch.where(small, torch.full_like(th2, 1e-8), th2))           # safe argument: no 0/0 or inf in the fused backward
    s = torch.where(small, 0.5 - th2 / 48.0, torch.sin(th / 2) / th)
    return torch.cat([s * r, torch.where(small, 1 - th2 / 8.0, torch.cos(th / 2))], -1)


def rollout(sp, gains, plant, dtype=torch.float64, device='cpu', x0=None, v0=None, q0=None, w0=None, F0=None, Fc0=None, ipz0=None, noise=None, use_md=None, keep=10, md_max=60):
    """sp: dict of tensors (n, B, 3) for pos / vel / acc / jerk per 1 ms step. gains: tensors (B,3)/(B,) for T.S0. plant: dict lag [ms], fmax, kt, gn [deg/s], quad (bool).
    Returns positions sampled every `keep` ms (n//keep, B, 3), final state dict."""
    n, B, _ = sp['pos'].shape; dev = device
    t = lambda a: T.K(a, dtype, dev)
    ctrl = T.S0(B, gains, mass=M, dtype=dtype, device=dev)
    lag = t(plant.get('lag', 30.0)); a_m = 1e-3 / (lag * 1e-3 + 1e-3); fmax = t(plant.get('fmax', 1e9)); kt = t(plant.get('kt', 1.0)); gn = t(plant.get('gn', 0.0))
    Jp = t(J_PLANT); gvec = t([0, 0, -G]); ktv = torch.stack([kt, kt, torch.ones_like(kt)], -1) if kt.ndim else torch.stack([kt, kt, t(1.0)])
    x = sp['pos'][0].clone() * T.K([1, 1, 0], dtype, dev) if x0 is None else x0
    v = torch.zeros(B, 3, dtype=dtype, device=dev) if v0 is None else v0
    q = T.K([0., 0., 0., 1.], dtype, dev).expand(B, 4).clone() if q0 is None else q0
    w = torch.zeros(B, 3, dtype=dtype, device=dev) if w0 is None else w0
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=dev) if F0 is None else F0
    F_c = F.clone() if Fc0 is None else Fc0; Fc_init = F_c; cmds = []; out = []; tilt_max = torch.zeros(B, dtype=dtype, device=dev)
    if ipz0 is not None: ctrl.i_pos = torch.cat([ctrl.i_pos[..., :2], ipz0[..., None]], -1)
    md = torch.as_tensor(plant.get('md', 0), device=dev).long().expand(B) if plant.get('md', 0) is not None else None
    if use_md is None: use_md = bool((torch.as_tensor(plant.get('md', 0)) > 0).any())        # graph capture: pass use_md explicitly (no host sync)
    hx = x[None].expand(md_max + 1, B, 3); hv = v[None].expand(md_max + 1, B, 3); ar = torch.arange(B, device=dev)
    for i in range(n):
        xm_, vm_ = x, v
        if noise is not None: xm_ = x + noise['pos'][i]; vm_ = v + noise['vel'][i]           # measurement noise (already scaled), (n, B, 3)
        if use_md:
            hx = torch.cat([hx[1:], xm_[None]]); hv = torch.cat([hv[1:], vm_[None]]); xm_ = hx[md_max - md, ar]; vm_ = hv[md_max - md, ar]
        if i % 2 == 0:
            gy = torch.rad2deg(w)
            if noise is not None: gy = gy + noise['gyro'][i // 2]                                 # gyro vibration (already scaled), (n/2, B, 3)
            th, u = ctrl.step(sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i], xm_, vm_, q, gy)
            cmds.append(T.mixer(th, u))
        F_c = cmds[(i - 4) // 2] if i >= 4 else Fc_init
        F_t = 0.1098 * (F_c / (0.2 * 0.618)) ** 2 if plant.get('quad', True) else F_c
        F_t = torch.minimum(F_t, fmax[..., None] if fmax.ndim else fmax)
        F = F + a_m[..., None] * (F_t - F) if a_m.ndim else F + a_m * (F_t - F)
        thrust = F.sum(-1)
        tau = torch.stack([T.ARM * (-F[..., 0] - F[..., 1] + F[..., 2] + F[..., 3]), T.ARM * (-F[..., 0] + F[..., 1] + F[..., 2] - F[..., 3]), T.T2T * (-F[..., 0] + F[..., 1] - F[..., 2] + F[..., 3])], -1)
        R = T.quat2rot(q)
        v = v + (gvec + R[..., :, 2] * ktv * (thrust / M)[..., None]) * 1e-3
        x = x + v * 1e-3
        under = x[..., 2] <= 0.0
        vz = torch.where(under, torch.clamp(v[..., 2], min=0.0), v[..., 2])
        hold = (under & (thrust < M * G))[..., None]
        v = torch.cat([torch.where(hold, torch.zeros_like(v[..., :2]), v[..., :2]), vz[..., None]], -1)
        x = torch.cat([x[..., :2], torch.where(under, torch.zeros_like(x[..., 2]), x[..., 2])[..., None]], -1)
        w = w + torch.linalg.solve(torch.diag(Jp), (tau - T.vcross(w, w * Jp)).unsqueeze(-1)).squeeze(-1) * 1e-3 if False else w + (tau - T.vcross(w, w * Jp)) / Jp * 1e-3
        q = qmul(qexp((R @ w[..., None])[..., 0] * 1e-3), q); q = q / torch.linalg.norm(q, dim=-1, keepdim=True)
        tilt_max = torch.maximum(tilt_max, torch.rad2deg(torch.acos(torch.clamp(T.quat2rot(q)[..., 2, 2], -1, 1))))
        if i % keep == 0: out.append(x)
    return torch.stack(out), dict(x=x, v=v, q=q, w=w, F=F, tilt_max=tilt_max)


def _block(st, meas, spk, ngyro, pr, gains, Jp, dwin=None, dwp=None):
    """One control period (2 ms = two 1 ms physics steps) as a pure function for torch.compile. The delayed measurement (xm_, vm_) is gathered outside (eager, inside the CUDA graph):
    the fused backward of the history gather produced NaN. Returns the new state, x after the first and after the second millisecond (for the history)."""
    x, v, q, w, F, i_pos, i_att, c1, c2, tmax, tpen, tau_ns2 = st
    xm_, vm_ = meas; pos_d, vel_d, acc_d, jerk_d = spk
    lag_a, fmax, ktv = pr
    ctrl = T.S0.__new__(T.S0); ctrl.dtype, ctrl.dev = x.dtype, x.device; ctrl.g = gains; ctrl.mass = M; ctrl.J = T.K(T.J_CTRL, x.dtype, x.device); ctrl.i_pos = i_pos; ctrl.i_att = i_att
    th, u = ctrl.step(pos_d, vel_d, acc_d, jerk_d, xm_, vm_, q, torch.rad2deg(w) + ngyro)
    cmd = T.mixer(th, u); F_c = c2                                      # command computed 2 periods (4 ms) ago acts now
    for sub in range(2):
        F_t = torch.minimum(0.1098 * (F_c / (0.2 * 0.618)) ** 2, fmax[..., None])
        F = F + lag_a[..., None] * (F_t - F); thrust = F.sum(-1)
        tau = torch.stack([T.ARM * (-F[..., 0] - F[..., 1] + F[..., 2] + F[..., 3]), T.ARM * (-F[..., 0] + F[..., 1] + F[..., 2] - F[..., 3]), T.T2T * (-F[..., 0] + F[..., 1] - F[..., 2] + F[..., 3])], -1)
        R = T.quat2rot(q); fext = 0.0
        if dwin is not None:
            tau = tau + tau_ns2                                               # plume torque computed at the previous even step (as plant_core: added before the update)
            if sub == 0:
                tp, tv = dwin; act = tp[..., 2] > x[..., 2]
                a_z = NS2.faz(dwp['net'], tp - x, tv - v, x[..., 2], v, M, act)
                fz = a_z * M * dwp['dws']; fz_ = torch.zeros_like(x); fext = torch.cat([fz_[..., :2], fz[..., None]], -1)
                A_, s_, n_ = dwp['plume']; tq = torch.zeros_like(x)
                for rk in ARMS:
                    pk = x + R[..., :, 0] * rk[0] + R[..., :, 1] * rk[1]; rho2 = (pk[..., 0] - tp[..., 0]) ** 2 + (pk[..., 1] - tp[..., 1]) ** 2
                    dFk = -A_ * (0.4 / torch.clamp(tp[..., 2] - pk[..., 2], min=0.05)) ** n_ * torch.exp(-rho2 / (2 * s_ ** 2))
                    tq = tq + torch.stack([rk[1] * dFk, -rk[0] * dFk, torch.zeros_like(dFk)], -1)
                tau_ns2 = tq * act[..., None].to(tq.dtype); fext_held = fext
            else: fext = fext_held
        v = v + (T.K([0, 0, -G], x.dtype, x.device) + R[..., :, 2] * ktv * (thrust / M)[..., None] + fext / M) * 1e-3
        x = x + v * 1e-3
        under = x[..., 2] <= 0.0
        vz = torch.where(under, torch.clamp(v[..., 2], min=0.0), v[..., 2]); hold = (under & (thrust < M * G))[..., None]
        v = torch.cat([torch.where(hold, torch.zeros_like(v[..., :2]), v[..., :2]), vz[..., None]], -1)
        x = torch.cat([x[..., :2], torch.where(under, torch.zeros_like(x[..., 2]), x[..., 2])[..., None]], -1)
        w = w + (tau - T.vcross(w, w * Jp)) / Jp * 1e-3
        q = qmul(qexp((R @ w[..., None])[..., 0] * 1e-3), q); q = q / torch.linalg.norm(q, dim=-1, keepdim=True)
        tmax = torch.maximum(tmax, torch.rad2deg(torch.acos(torch.clamp(T.quat2rot(q.detach())[..., 2, 2], -0.999999, 0.999999))))      # detached: zero grad times acos' at level flight (inf) would be NaN in the fused backward
        tpen = tpen + torch.relu(1.0 - T.quat2rot(q)[..., 2, 2] - 0.0603)          # differentiable hinge on tilt above 20 deg (1 - cos 20 deg = 0.0603), summed over ms
        if sub == 0: x_s, v_s = x, v
    return (x, v, q, w, F, ctrl.i_pos, ctrl.i_att, cmd, c1, tmax, tpen, tau_ns2), x_s, v_s


_block_c = {}


def rollout_blocks(sp, gains, plant, dtype=torch.float32, device='cuda', x0=None, v0=None, q0=None, F0=None, Fc0=None, ipz0=None, noise=None, keep_blocks=5, compile=True, md_max=60, carry=None):
    """Same model as rollout() (1 ms physics, 2 ms control), written block-wise so the per-block function can be torch.compile'd (fused kernels).
    keep_blocks=5 -> positions every 10 ms. noise must be given (zeros allowed)."""
    n, B, _ = sp['pos'].shape; dev = device; nb = n // 2
    t = lambda a: T.K(a, dtype, dev)
    lag = t(plant['lag']); lag_a = 1e-3 / (lag * 1e-3 + 1e-3); fmax = t(plant['fmax']).expand(B) if t(plant['fmax']).ndim == 0 else t(plant['fmax']); kt = t(plant['kt']); ktv = torch.stack([kt, kt, torch.ones_like(kt)], -1)
    md = plant['md'].long() if torch.is_tensor(plant['md']) else torch.full((B,), int(plant['md']), device=dev, dtype=torch.long)
    ar = torch.arange(B, device=dev); Jp = t(J_PLANT)
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=dev) if F0 is None else F0
    Fc = F.clone() if Fc0 is None else Fc0
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=dev)
    x = sp['pos'][0] * T.K([1, 1, 0], dtype, dev) if x0 is None else x0; v = zeros3 if v0 is None else v0
    q = T.K([0., 0., 0., 1.], dtype, dev).expand(B, 4).clone() if q0 is None else q0
    i_pos = torch.zeros(B, 3, dtype=dtype, device=dev) if ipz0 is None else torch.cat([zeros3[..., :2], ipz0[..., None]], -1)
    st = (x, v, q, zeros3, F, i_pos, zeros3, Fc, Fc, torch.zeros(B, dtype=dtype, device=dev), torch.zeros(B, dtype=dtype, device=dev), zeros3)
    hx = x[None].expand(md_max + 1, B, 3); hv = v[None].expand(md_max + 1, B, 3)
    if carry is not None: st, hx, hv = tuple(carry[:12]), carry[12], carry[13]          # chained windows: state, delayed-measurement history and command queue carried over
    fn = _block
    if compile:
        key = (B, dtype, str(dev))
        if key not in _block_c: _block_c[key] = torch.compile(_block, dynamic=False)
        fn = _block_c[key]
    out = []; pr = (lag_a, fmax, ktv)
    for k in range(nb):
        i = 2 * k
        hx = torch.cat([hx[1:], (st[0] + noise['pos'][i])[None]]); hv = torch.cat([hv[1:], (st[1] + noise['vel'][i])[None]])
        meas = (hx[md_max - md, ar], hv[md_max - md, ar])
        dwin = (sp['tp'][i], sp['tv'][i]) if 'tp' in sp else None
        st, xs, vs = fn(st, meas, (sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), noise['gyro'][k], pr, gains, Jp, dwin, plant.get('dw'))
        hx = torch.cat([hx[1:], (xs + noise['pos'][i + 1])[None]]); hv = torch.cat([hv[1:], (vs + noise['vel'][i + 1])[None]])
        if k % keep_blocks == 0: out.append(xs)
    x, v, q, w, F, _, _, _, _, tmax, tpen, _ = st
    return torch.stack(out), dict(x=x, v=v, q=q, w=w, F=F, tilt_max=tmax, tpen=tpen / n, carry=tuple(st) + (hx, hv))
