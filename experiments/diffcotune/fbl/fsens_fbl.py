#!/usr/bin/env python3
"""Stage F1.4: exact forward-mode gradient of the FBL model (the analogue of fsens.py for S0-S3, which is not touched).

The state tangents d state / d z_j (z = 6 gain log-factors + 4 beta log-factors) are carried next to the state through the rollout; the cost gradient is accumulated at the cost rows
(autograd of the cost with respect to the recorded rows x, v, q, w, contracted with the recorded tangent rows). The jvp of ONE 2 ms block of fbl_model._block, vmapped over the tangents,
is traced once with make_fx into a plain aten graph and compiled with torch.compile (as fsens.py does for S0-S3). Float64, CPU, no noise, preset-1 / calibration rows given by the caller."""
import sys
from pathlib import Path
import numpy as np
import torch
from torch.func import jvp, vmap
from torch.fx.experimental.proxy_tensor import make_fx

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import fbl_model as FM  # noqa: E402
import reference as R  # noqa: E402
import s0_torch as T  # noqa: E402
import costs as C  # noqa: E402

_GRAPHS = {}
NZ = FM.N_GAINS + FM.N_BETA


def _init(sp, B, dtype):
    zeros3 = torch.zeros(B, 3, dtype=dtype)
    x = (sp['pos'][0] * T.K([1., 1., 0.], dtype, 'cpu')).expand(B, 3).clone(); q = T.K([0., 0., 0., 1.], dtype, 'cpu').expand(B, 4).clone()
    F = torch.full((B, 4), FM.M * FM.G / 4, dtype=dtype)
    return (x, zeros3.clone(), q, zeros3.clone(), F, torch.zeros(B, FM.NFS_ALL, dtype=dtype), F.clone(), F.clone(), F.clone(), zeros3.clone())


def _graph(variant, has_top, feed, T_, B, dtype, args, noisy=False):
    key = (variant, bool(has_top), bool(feed), T_, B, str(dtype), bool(noisy))
    if key in _GRAPHS:
        return _GRAPHS[key]
    flg = FM.flags(variant); nets = FM.S0.net(dtype, 'cpu')
    if B > 1:
        FM.S0._patch_inductor_loop_split()

    def f(st_, z_, c):
        gains = FM.gains_from(z_[:, :FM.N_GAINS], dtype, 'cpu'); pr = FM.beta_from(z_[:, FM.N_GAINS:], dtype, 'cpu')
        return FM._block(tuple(st_), c['spk'], c['tpk'], c['jit'], gains, pr, has_top, nets, feed, flg, c['nz'] if noisy else None)

    def one(st_, z_, tst, tz, c):
        return jvp(lambda s_, zz: f(s_, zz, c), (st_, z_), (tst, tz))

    def batched(st_, z_, tst, tz, c):
        return vmap(one, in_dims=(None, None, 0, 0, None), out_dims=(None, 0))(st_, z_, tst, tz, c)
    snap = dict(T._CACHE)
    try:
        with torch.no_grad():
            gm = make_fx(batched, tracing_mode='real')(*args)
    finally:
        T._CACHE.clear(); T._CACHE.update(snap)
    _GRAPHS[key] = torch.compile(gm, dynamic=False)
    return _GRAPHS[key]


def rollout_tangents(ref, z, idx, variant, keep_blocks=5, dtype=torch.float64, n_blocks=None, noise=None, B=1):
    """z (10,) = [6 gain log-factors, 4 beta log-factors]; idx = parameters that get a tangent. Rows like fbl_model.rollout plus tangents tx, tv, tq, tw (T, R, 1, .)."""
    nb = (ref.n_ticks // 2) if n_blocks is None else n_blocks
    t = lambda a: torch.as_tensor(np.asarray(a), dtype=dtype)
    sp = {k: t(getattr(ref, k)) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}
    jit = torch.as_tensor(R.command_jitter(nb))
    zt = torch.as_tensor(np.asarray(z, float), dtype=dtype)[None].expand(B, NZ).clone()
    nzt = None
    if noise is not None:          # (n_blocks, B, 18) from fbl_model.frsim_noise (Stage F2c: both ticks of a block)
        nzt = torch.as_tensor(np.asarray(noise)[:nb], dtype=dtype)
        nzt = nzt[:, None, :].expand(nb, B, 18) if nzt.ndim == 2 else nzt
    zeros_nz = torch.zeros(B, 18, dtype=dtype)
    idx = list(idx); T_ = len(idx); tz = torch.zeros(T_, B, NZ, dtype=dtype); tz[torch.arange(T_), :, torch.as_tensor(idx)] = 1.0
    st = _init(sp, B, dtype); tst = tuple(torch.zeros((T_,) + a.shape, dtype=dtype) for a in st)
    rows = {k: [] for k in 'xvqw'}; trows = {k: [] for k in 'xvqw'}; ridx = []
    with torch.no_grad():
        for k in range(nb):
            i = 2 * k
            if k % keep_blocks == 0:
                for name, j in zip('xvqw', range(4)):
                    rows[name].append(st[j]); trows[name].append(tst[j])
                ridx.append(i)
            c = dict(spk=(sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), tpk=(sp['top_pos'][i], sp['top_vel'][i]), jit=jit[k], nz=(nzt[k] if nzt is not None else zeros_nz))
            args = (tuple(st), zt, tst, tz, c)
            g = _graph(variant, ref.has_top, k % 5 == 0, T_, B, dtype, args, noise is not None)
            st, tst = g(*args)
    ridx = np.array(ridx)
    return dict(t=ref.time_s[ridx], idx=ridx, x=torch.stack(rows['x']), v=torch.stack(rows['v']), q=torch.stack(rows['q']), w=torch.stack(rows['w']), ref_pos=sp['pos'][ridx], ref_vel=sp['vel'][ridx],
                ref_q=FM.S0._flat_rows(ref, ridx, dtype, 'cpu'), has_top=ref.has_top, top_pos=ref.top_pos, tx=torch.stack(trows['x'], 1), tv=torch.stack(trows['v'], 1), tq=torch.stack(trows['q'], 1), tw=torch.stack(trows['w'], 1))


def _cost(out, kind, scenario_s, trace, noisy=False):
    if kind == 'task':
        return C.j_task(out, scenario_s).mean() if noisy else C.j_task(out, scenario_s).sum()
    import ingest
    return C.j_sysid(out, ingest.on_grid(ingest.from_trace(trace), out['t']), scenario_s, states=('p', 'v', 'q')).sum()


def grads_from(o, kind, scenario_s, trace=None):
    """(J, g) of one cost from recorded rows + tangents o (rollout_tangents); g has the length of the tangent set."""
    leaves = {k: o[k].clone().requires_grad_(True) for k in 'xvqw'}
    out = dict(t=o['t'], x=leaves['x'], v=leaves['v'], q=leaves['q'], w=leaves['w'], ref_pos=o['ref_pos'], ref_vel=o['ref_vel'], ref_q=o['ref_q'])
    J = _cost(out, kind, scenario_s, trace)
    gr = torch.autograd.grad(J, [leaves[k] for k in 'xvqw'], allow_unused=True)
    gr = [torch.zeros_like(leaves[k]) if g_ is None else g_ for g_, k in zip(gr, 'xvqw')]
    gz = sum(torch.einsum('rbc,trbc->t', g_, o['t' + k]) for g_, k in zip(gr, 'xvqw'))
    if not (np.isfinite(float(J)) and bool(np.isfinite(gz.numpy()).all())):          # decision D1 (F2): no silent clamp
        raise FloatingPointError('FBL gradient: non-finite cost or gradient (%s, J %s, gradient %s, r0_min %s)' % (kind, float(J), gz.numpy().tolist(), FM.r0_min(type('R_', (), dict(has_top=o['has_top'], top_pos=o['top_pos']))(), o['x']) if o.get('has_top') else None))
    return float(J), gz.numpy()


def gradient(ref, scenario_s, kind, z, idx, variant, trace=None):
    """(J, g) with g of length 10 (zeros outside idx); kind 'task' (J_task) or 'sysid' (J_sysId, p + v + q, against the real trace)."""
    o = rollout_tangents(ref, z, idx, variant)
    J, gz = grads_from(o, kind, scenario_s, trace)
    full = np.zeros(NZ); full[list(idx)] = gz
    return J, full


def value(ref, scenario_s, kind, z, variant, trace=None):
    """Forward value of the cost (eager, no gradient)."""
    z = np.asarray(z, float)
    with torch.no_grad():
        out = FM.rollout(ref, z[None, :FM.N_GAINS], z[None, FM.N_GAINS:], variant=variant)
        return float(_cost(out, kind, scenario_s, trace))


def reverse_gradient(ref, scenario_s, kind, z, variant, trace=None, compile=True):
    """Reverse-mode (autograd through the eager rollout) gradient, for information; memory heavy (about 1 MB per block), use on a few scenarios only."""
    z = torch.as_tensor(np.asarray(z, float), dtype=torch.float64).clone().requires_grad_(True)
    out = FM.rollout(ref, z[None, :FM.N_GAINS], z[None, FM.N_GAINS:], variant=variant, compile=compile)
    J = _cost(out, kind, scenario_s, trace)
    g, = torch.autograd.grad(J, z)
    return float(J), g.numpy()


def gradient_loop(ref, scenario_s, kind, z, idx, traces=None, states=('p', 'v', 'q', 'w'), noise_seeds=None, model=None):
    """The interface of fsens.gradient (used by dryrun/loop_lib.worker_rollout for FBL): (J, g) of the cost of loop_lib.worker_rollout: kind 'task' (mean over the noise rows if noise_seeds, else the single row) or
    'sysid' (mean over the traces of J_sysId, one trace per flight, `states` the parts); g has the length of z (zeros outside idx). Variant = fbl_model.VARIANT (DIFFCOTUNE_FBL_VARIANT)."""
    import ingest
    noise = None; B = 1
    if kind == 'task' and noise_seeds:
        noise = np.stack([FM.frsim_noise(sd, ref.n_ticks) for sd in noise_seeds], 1); B = len(noise_seeds)
    o = rollout_tangents(ref, z, idx, FM.VARIANT, noise=noise, B=B)
    leaves = {k: o[k].clone().requires_grad_(True) for k in 'xvqw'}
    out = dict(t=o['t'], x=leaves['x'], v=leaves['v'], q=leaves['q'], w=leaves['w'], ref_pos=o['ref_pos'], ref_vel=o['ref_vel'], ref_q=o['ref_q'])
    if kind == 'task':
        J = C.j_task(out, scenario_s).mean() if noise_seeds else C.j_task(out, scenario_s).sum()
    else:
        J = sum(C.j_sysid(out, ingest.on_grid(ingest.from_trace(tr), out['t']), scenario_s, states=states).sum() / len(traces) for tr in traces)
    gr = torch.autograd.grad(J, [leaves[k] for k in 'xvqw'], allow_unused=True)
    gr = [torch.zeros_like(leaves[k]) if g_ is None else g_ for g_, k in zip(gr, 'xvqw')]
    gz = sum(torch.einsum('rbc,trbc->t', g_, o['t' + k]) for g_, k in zip(gr, 'xvqw'))
    if not (np.isfinite(float(J)) and bool(np.isfinite(gz.numpy()).all())):          # decision D1: stop and report, no silent clamp
        raise FloatingPointError('FBL gradient: non-finite cost or gradient (%s, scenario %s, J %s, gradient %s)' % (kind, getattr(ref, 'scenario', '?'), float(J), gz.numpy().tolist()))
    full = np.zeros(len(np.asarray(z))); full[list(idx)] = gz.numpy()
    return float(J), full
