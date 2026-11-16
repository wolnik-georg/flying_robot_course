"""Stage 5b: time of one 2-ms block with 15 tangents: eager vmap(jvp) vs torch.compile(vmap(jvp)); correctness against the eager value."""
import sys, time
from pathlib import Path
import numpy as np, torch
from torch.func import jvp, vmap
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parents[0]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); mod = sys.modules[M.__name__]; s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
rec = {}; orig = mod._block; cnt = [0]
def spy(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw):
    k = cnt[0]; cnt[0] += 1
    if k == 300: rec[k] = ([t.detach().clone() for t in st], spk, tpk, jit, act_top, nets, nz, bw)
    return orig(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw)
mod._block = spy
z0 = torch.zeros(1, n, dtype=torch.float64)
with torch.no_grad(): M.rollout(ref, z0[:, :10], z0[:, 10:], n_blocks=301)
mod._block = orig
st, spk, tpk, jit, act_top, nets, nz, bw = rec[300]; dt = torch.float64
def fz(st, z):
    gains = M.S0.gains_from(z[:, :10], dt, 'cpu'); pr = M.S0.beta_from(z[:, 10:14], dt, 'cpu')
    if L.N_BETA == 5: pr['rd'] = M.RD_START * torch.exp(z[:, 14])
    return orig(tuple(st), spk, tpk, jit, gains, pr, act_top, nets, nz, bw)
def one(st, z, tst, tz): return jvp(fz, (st, z), (tst, tz))
def batched(st, z, tst, tz): return vmap(one, in_dims=(None, None, 0, 0))(st, z, tst, tz)
tst = tuple(torch.zeros((n,) + t.shape, dtype=dt) for t in st); tz = torch.eye(n, dtype=dt)[:, None, :]
def timeit(fn, k=20):
    fn(); t0 = time.time()
    for _ in range(k): fn()
    return (time.time() - t0) / k
with torch.no_grad():
    print('eager vmap(jvp) %.1f ms/block' % (1e3 * timeit(lambda: batched(tuple(st), z0, tst, tz))), flush=True)
    t0 = time.time(); cf = torch.compile(batched, dynamic=False); r = cf(tuple(st), z0, tst, tz); print('compile+first %.0f s' % (time.time() - t0), flush=True)
    print('compiled vmap(jvp) %.1f ms/block' % (1e3 * timeit(lambda: cf(tuple(st), z0, tst, tz))), flush=True)
    r0 = batched(tuple(st), z0, tst, tz)
    err = max(float((a - b).abs().max()) for a, b in zip(sum([[*r0[0]], [*r0[1]]], []), sum([[*r[0]], [*r[1]]], [])))
    print('max |compiled - eager| (primal and tangent) %.3e' % err)
