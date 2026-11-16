"""Scenario batch of the differentiable S0 model (Stage 3c, C3): the tuning scenarios (and several traces per scenario) as rows of ONE rollout.

Rows = (scenario, trace) pairs. The references are stacked and padded to the longest scenario (last value held); every row has its own
top-vehicle flag (A1 / A8: downwash and plume act, CIRCLE / FIG8 / OVAL: they do not), its own scenario end and weight. Parameters phi (10) and
psi (4) are shared by all rows, so the gradient is the sum over the rows of the row costs. The block function is s0_model._block (unchanged
arithmetic). The cost of a row only uses the rows of its own time window, so the padding never enters a cost.
"""
import numpy as np
import torch

import s0_model as S
import reference as R
import costs as C
import ingest
import s0_torch as T


def stack_refs(refs, dtype, device):
    n = max(r.n_ticks for r in refs)
    pad = lambda a: np.concatenate([a, np.repeat(a[-1:], n - len(a), 0)]) if len(a) < n else a
    sp = {k: torch.as_tensor(np.stack([pad(getattr(r, k)) for r in refs], 1), dtype=dtype, device=device) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    return sp, n


def _chunk(st, spk, tpk, jit, gains, pr, has_top, nets, kb=5):
    """kb consecutive 2 ms blocks in one function (one compiled call: fewer launches and less python per block). Arithmetic = s0_model._block."""
    for j in range(kb):
        st = S._block(st, tuple(a[j] for a in spk), tuple(a[j] for a in tpk), jit[j], gains, pr, has_top, nets)
    return st


def rollout_rows(refs, phi, psi, dtype=torch.float64, device='cpu', keep_blocks=5, compile=False, chunk=False, mode=None, noise=None):
    """noise (Stage 3d): None or (B, n_blocks_row, 9) arrays per row (frsim_noise of the row's seed); padded with zeros to the longest row. refs: list of B references (one per row); phi (10,), psi (4,) tensors (shared). Returns the same dict as s0_model.rollout with rows = (time, B)."""
    B = len(refs); sp, n = stack_refs(refs, dtype, device); nb = n // 2
    phiB = torch.as_tensor(phi, dtype=dtype, device=device).expand(B, 10); psiB = torch.as_tensor(psi, dtype=dtype, device=device).expand(B, 4)
    gains = S.gains_from(phiB, dtype, device); pr = S.beta_from(psiB, dtype, device); nets = S.net(dtype, device)
    jit = torch.as_tensor(R.command_jitter(nb), device=device)
    has_top = torch.as_tensor([r.has_top for r in refs], device=device)
    zeros3 = torch.zeros(B, 3, dtype=dtype, device=device)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, device)).clone()
    q = T.K([0., 0., 0., 1.], dtype, device).expand(B, 4).clone()
    F = torch.full((B, 4), S.M * S.G / 4, dtype=dtype, device=device)
    st = (x, zeros3.clone(), q, zeros3.clone(), F, zeros3.clone(), zeros3.clone(), F.clone(), F.clone(), F.clone(), zeros3.clone())
    fn = S._block
    if compile:
        key = ('rows', chunk, mode)
        if key not in S._COMPILED:
            S._COMPILED[key] = torch.compile(_chunk if chunk else S._block, dynamic=False, mode=mode)
        fn = S._COMPILED[key]
    nzt = None
    if noise is not None:
        nzt = torch.zeros(nb, B, 9, dtype=dtype, device=device)
        for b, a in enumerate(noise):
            a = torch.as_tensor(np.asarray(a)[:nb], dtype=dtype, device=device); nzt[:a.shape[0], b] = a
    rows = {'x': [], 'v': [], 'q': [], 'w': []}; idx = []
    if chunk:                                    # keep_blocks consecutive blocks per call; the state at the start of every call is a kept row
        for k in range(0, (nb // keep_blocks) * keep_blocks, keep_blocks):
            i = 2 * k; sl = slice(i, i + 2 * keep_blocks, 2)
            for key2, j in (('x', 0), ('v', 1), ('q', 2), ('w', 3)):
                rows[key2].append(st[j])
            idx.append(i)
            st = fn(st, (sp['pos'][sl], sp['vel'][sl], sp['acc'][sl], sp['jerk'][sl]), (sp['top_pos'][sl], sp['top_vel'][sl]), jit[k:k + keep_blocks], gains, pr, has_top, nets)
        nb = 0
    for k in range(nb):
        i = 2 * k
        if k % keep_blocks == 0:
            for key, j in (('x', 0), ('v', 1), ('q', 2), ('w', 3)):
                rows[key].append(st[j])
            idx.append(i)
        st = fn(st, (sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jit[k], gains, pr, has_top, nets, None if nzt is None else nzt[k])
    idx = np.array(idx)
    t = refs[0].time_s[idx] if refs[0].n_ticks >= n else np.arange(n)[idx] * R.DT - S.config.TAKEOFF_S - S.config.HOLD_S
    return dict(t=t, idx=idx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']), ref_pos=sp['pos'][idx])


def _mask(t, ends, kind):
    lo = S.config.SCORE_AFTER_S if kind == 'task' else 0.0
    return torch.as_tensor((t[:, None] > lo) & (t[:, None] <= np.asarray(ends)[None, :]))     # (rows, B)


def j_task_rows(out, ends, weights, per_row=False):
    """Mean over the rows of the weighted per-row mean squared 3-D position error (a scalar)."""
    m = _mask(out['t'], ends, 'task').to(out['x'].device)
    e = ((out['x'] - out['ref_pos']) ** 2).sum(-1)                        # (rows, B)
    per = (e * m).sum(0) / m.sum(0)
    if per_row:
        return per
    return (per * torch.as_tensor(weights, dtype=per.dtype, device=per.device)).sum()


def j_sysid_rows(out, targets, ends, weights, states=('p', 'v', 'q', 'w'), per_row=False):
    """targets: list of B Targets on the model's output grid (ingest.on_grid per row); weights (B,) = 1 / (number of traces of the scenario) / (number of scenarios)."""
    m = _mask(out['t'], ends, 'sysid').to(out['x'].device)
    f = lambda k: torch.as_tensor(np.stack([getattr(tg, k) for tg in targets], 1), dtype=out['x'].dtype, device=out['x'].device)
    p = {'p': ((out['x'] - f('p')) ** 2).sum(-1), 'v': ((out['v'] - f('v')) ** 2).sum(-1), 'w': ((out['w'] - f('w')) ** 2).sum(-1)}
    qt = f('q'); sgn = torch.sign((out['q'] * qt).sum(-1, keepdim=True)).detach(); sgn = torch.where(sgn == 0, torch.ones_like(sgn), sgn)
    p['q'] = ((out['q'] - sgn * qt) ** 2).sum(-1)
    tot = sum(p[k] for k in states)
    per = (tot * m).sum(0) / m.sum(0)
    if per_row:
        return per
    return (per * torch.as_tensor(weights, dtype=per.dtype, device=per.device)).sum()
