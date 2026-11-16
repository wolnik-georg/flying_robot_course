#!/usr/bin/env python3
"""Differentiable S2 model (S0 + the neural downwash compensation NS2 in the controller) on the frsim plant, for DiffCoTune (Stage 5, 13 Nov 2026).

S2 = frsim `Controller('S2', tree, indi 0, ns2 1)`: the S0 geometric controller plus `a_d -= ns2.hold` (controller_omar_indi.c:458-467, ns2_residual.c). Everything else is the S0 model
(`s0_model.py`: same plant, exact Omar inertia, beta = (thrust gain, motor lag, downwash scale, plume scale) as log-factors, theta = the 10 tree gains as log-factors, sensor noise option,
compiled 2 ms block); this file only adds the NS2 evaluation outside the compiled block. Copied behaviour (checked in `checks/` against frsim):
  * NS2 runs when tick % g_rnn_div == 0 with g_rnn_div = 10 (frsim sets it, controllers.py:133), the controller runs on even ticks, so every 5th block (i = 0, 10, 20, ... ms); the output is held
    in between (`hold`), exactly one evaluation per 10 ms;
  * inputs are the MEASURED position and velocity of the controller (plant state + the sensor noise of that tick: the noise array of `s0_model.frsim_noise`) and the peer feed (the true top-vehicle
    position every 10 ms, only in scenarios with a top vehicle); the peer velocity is the finite difference of consecutive peer samples (dt 10 ms); the first two feeds give zero
    (`s->prev[k].ts == 0` is the first-seen marker, and the first feed has timestamp 0, so the second feed is still "first seen": ns2_residual.c:ns2_eval);
  * the controller-side network has no 'peer above me' test (the plant side, which models the physical downwash, has); the ground term is always on; the output is clamped at 8 m/s^2 (`NS2.faz`);
  * only the z component is non-zero (ns2_eval_rel: out_accel[2]); a_d -= hold is applied before the thrust and the desired attitude, so it enters through acc_d: `a_d` is linear in `acc_d`.
Interface = s0_model.rollout (same arguments and output)."""
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import s0_model as S0  # noqa: E402
import reference as R  # noqa: E402
import ns2_torch as NS2  # noqa: E402
import s0_torch as T  # noqa: E402

M, G = S0.M, S0.G
GAIN_NAMES, GAIN_DEFAULT, BETA_NAMES, BETA_START = S0.GAIN_NAMES, S0.GAIN_DEFAULT, S0.BETA_NAMES, S0.BETA_START
TILT_CRASH_DEG, tilt_deg, frsim_noise = S0.TILT_CRASH_DEG, S0.tilt_deg, S0.frsim_noise
N_BETA = 4
NS2_EVERY_BLOCKS = 5                       # g_rnn_div 10 ticks of 1 ms = 5 control blocks of 2 ms
_COMPILED = {}


def ns2_hold(nets, sp, k, i, x, v, nzt, active, dtype, device):
    """The NS2 output (B,) [m/s^2] of the evaluation at block k (tick i = 2 k): measured state of the controller (plant state + that tick's sensor noise), true peer position and its
    10 ms finite-difference velocity (zero for the first two feeds)."""
    xm, vm = (x, v) if nzt is None else (x + nzt[k][..., 0:3], v + nzt[k][..., 3:6])
    j = k // NS2_EVERY_BLOCKS; tpk = sp['top_pos'][i]
    vpk = torch.zeros(3, dtype=dtype, device=device) if j < 2 else (tpk - sp['top_pos'][i - 2 * NS2_EVERY_BLOCKS]) / 0.01
    return NS2.faz(nets, tpk - xm, vpk - vm, xm[..., 2], vm, M, active)



def _flat_rows(ref, idx, dtype, device):
    """Stage 8: reference attitude (x, y, z, w) and body rates at the output rows, as a (rows, 7) tensor (for the paper-faithful task cost; unused by the other costs)."""
    import reference as _R
    q, w = _R.flat_ref(ref); return torch.as_tensor(np.concatenate([q, w], -1)[np.asarray(idx)], dtype=dtype, device=device)

def rollout(ref, phi, psi, dtype=torch.float64, device='cpu', keep_blocks=5, compile=False, n_blocks=None, noise=None):
    """Closed-loop S2 flight of one reference for B parameter rows (see s0_model.rollout; noise = None or the (n_blocks, 9) / (n_blocks, B, 9) array of s0_model.frsim_noise)."""
    phi = torch.as_tensor(phi, dtype=dtype, device=device); psi = torch.as_tensor(psi, dtype=dtype, device=device)
    B = phi.shape[0]; nb = (ref.n_ticks // 2) if n_blocks is None else n_blocks
    t = lambda a: torch.as_tensor(np.asarray(a), dtype=dtype, device=device)
    sp = {k: t(getattr(ref, k)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    gains = S0.gains_from(phi, dtype, device); pr = S0.beta_from(psi, dtype, device); nets = S0.net(dtype, device)
    jit = torch.as_tensor(R.command_jitter(nb), device=device)
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=device)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, device)).expand(B, 3).clone()
    q = T.K([0., 0., 0., 1.], dtype, device).expand(B, 4).clone()
    F = torch.full((B, 4), M * G / 4, dtype=dtype, device=device)
    st = (x, zeros3.clone(), q, zeros3.clone(), F, zeros3.clone(), zeros3.clone(), F.clone(), F.clone(), F.clone(), zeros3.clone())
    fn = S0._block
    if compile:
        if B > 1:
            S0._patch_inductor_loop_split()
        if 'fn' not in _COMPILED:
            _COMPILED['fn'] = torch.compile(S0._block, dynamic=False)
        fn = _COMPILED['fn']
    nzt = None
    if noise is not None:
        nzt = torch.as_tensor(np.asarray(noise)[:nb], dtype=dtype, device=device)
        nzt = nzt[:, None, :].expand(nb, B, 9) if nzt.ndim == 2 else nzt
    has_peer = bool(ref.has_top); active = torch.full((B,), has_peer, dtype=torch.bool, device=device)
    hold = torch.zeros(B, dtype=dtype, device=device)
    rows = {'x': [], 'v': [], 'q': [], 'w': []}; idx = []
    for k in range(nb):
        i = 2 * k
        if k % keep_blocks == 0:
            rows['x'].append(st[0]); rows['v'].append(st[1]); rows['q'].append(st[2]); rows['w'].append(st[3]); idx.append(i)
        if k % NS2_EVERY_BLOCKS == 0:                                   # one NS2 evaluation per 10 ms, on the measured state
            hold = ns2_hold(nets, sp, k, i, st[0], st[1], nzt, active, dtype, device)
        acc_eff = sp['acc'][i] - torch.cat([zeros3[..., :2], hold[..., None]], -1)       # a_d -= hold (z only)
        st = fn(st, (sp['pos'][i], sp['vel'][i], acc_eff, sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jit[k], gains, pr, ref.has_top, nets, None if nzt is None else nzt[k])
    idx = np.array(idx)
    return dict(t=ref.time_s[idx], idx=idx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']),
                ref_pos=sp['pos'][idx], ref_vel=sp['vel'][idx], ref_q=_flat_rows(ref, idx, dtype, device), final=st)
