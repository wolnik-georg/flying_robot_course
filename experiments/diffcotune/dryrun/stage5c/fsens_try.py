"""Stage 5c step 3, variant (a'): exact forward sensitivity inside a COMPILED block. The jvp (torch.func.jvp, vmapped over the tangents) of one 2-ms block is traced ONCE to a plain aten graph with make_fx
(the functorch transforms are executed at trace time, so the graph contains no transform), and that graph is compiled with torch.compile (dynamo cannot compile torch.func.jvp itself, torch 2.5.1).
Test on S1 (env DIFFCOTUNE_CONTROLLER): (1) one block: compiled vs eager vmap(jvp), max difference, seconds per block; (2) the first N blocks of A1: tangent of the position at the end,
compiled vs eager jvp, seconds. usage: fsens_try.py [N blocks, default 3000]"""
import sys, time
from pathlib import Path
import numpy as np, torch
from torch.func import jvp, vmap
from torch.fx.experimental.proxy_tensor import make_fx
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); mod = sys.modules[M.__name__]; s, c = L.TUNING[0]; ref = R.build(s, c); NB = L.N_BETA; dt = torch.float64
N = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
rec = {}; orig = mod._block; cnt = [0]
def spy(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw):
    k = cnt[0]; cnt[0] += 1
    if k == 300: rec[k] = ([t.detach().clone() for t in st], spk, tpk, jit, gains, act_top, nets, bw)
    return orig(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw)
mod._block = spy
z0 = torch.zeros(1, 10 + NB, dtype=dt)
with torch.no_grad(): M.rollout(ref, z0[:, :10], z0[:, 10:], n_blocks=301)
mod._block = orig
st, spk, tpk, jit, gains, act_top, nets, bw = rec[300]
def blk(st, psi, spk, tpk, jit, gains):
    pr = M.S0.beta_from(psi[:, :4], dt, 'cpu')
    if NB == 5: pr['rd'] = M.RD_START * torch.exp(psi[:, 4])
    return orig(tuple(st), spk, tpk, jit, gains, pr, act_top, nets, None, bw)
def jv(st, psi, tst, tpsi, spk, tpk, jit, gains):
    return jvp(lambda s_, p_: blk(s_, p_, spk, tpk, jit, gains), (st, psi), (tst, tpsi))
def batched(st, psi, tst, tpsi, spk, tpk, jit, gains):
    return vmap(jv, in_dims=(None, None, 0, 0, None, None, None, None), out_dims=(None, 0))(st, psi, tst, tpsi, spk, tpk, jit, gains)
psi = torch.zeros(1, NB, dtype=dt); tpsi = torch.eye(NB, dtype=dt)[:, None, :]; tst = tuple(torch.zeros((NB,) + t.shape, dtype=dt) for t in st)
args = (tuple(st), psi, tst, tpsi, spk, tpk, jit[None][0] if jit.ndim else jit, gains)
t0 = time.time()
with torch.no_grad(): gm = make_fx(batched, tracing_mode='real')(*args)
print('make_fx %.1f s, %d nodes' % (time.time() - t0, len(gm.graph.nodes)), flush=True)
t0 = time.time(); cgm = torch.compile(gm, dynamic=False)
with torch.no_grad(): rc = cgm(*args)
print('compile + first call %.0f s' % (time.time() - t0), flush=True)
with torch.no_grad():
    re = batched(*args)
    d1 = max(float((a - b).abs().max()) for a, b in zip(list(re[0]) + list(re[1]), list(rc[0]) + list(rc[1]))); print('one block: max |compiled - eager vmap(jvp)| %.3e' % d1, flush=True)
    for name, fn in (('eager vmap(jvp)', lambda: batched(*args)), ('compiled', lambda: cgm(*args))):
        fn(); t0 = time.time()
        for _ in range(50): fn()
        print('%-18s %.2f ms/block (%d tangents)' % (name, 1e3 * (time.time() - t0) / 50, NB), flush=True)
# (2) N blocks of A1: tangent of x(t_end) wrt psi
sp = {k: torch.as_tensor(np.asarray(getattr(ref, k)), dtype=dt) for k in ('pos', 'vel', 'acc', 'jerk', 'top_pos', 'top_vel')}; jt_all = torch.as_tensor(R.command_jitter(N))
def init():
    out = {}
    def spy2(st_, *a): out['st'] = tuple(t.detach().clone() for t in st_); raise StopIteration
    mod._block = spy2
    try: M.rollout(ref, z0[:, :10], z0[:, 10:], n_blocks=1)
    except StopIteration: pass
    mod._block = orig; return out['st']
stI = init(); gains0 = M.S0.gains_from(z0[:, :10], dt, 'cpu')
def run(fn):
    s_ = stI; t_ = tuple(torch.zeros((NB,) + t.shape, dtype=dt) for t in stI); t0 = time.time()
    with torch.no_grad():
        for k in range(N):
            i = 2 * k; s_, t_ = fn(tuple(s_), psi, t_, tpsi, (sp['pos'][i], sp['vel'][i], sp['acc'][i], sp['jerk'][i]), (sp['top_pos'][i], sp['top_vel'][i]), jt_all[k], gains0)
    return s_, t_, time.time() - t0
sc, tc, sec_c = run(cgm); print('%d blocks compiled: %.1f s  (%.2f ms/block)' % (N, sec_c, 1e3 * sec_c / N), flush=True)
se, te, sec_e = run(batched); print('%d blocks eager vmap(jvp): %.1f s' % (N, sec_e), flush=True)
print('x tangent at the end: max |compiled - eager| %.3e (max |tangent| %.3e);  all state tangents max %.3e' % ((tc[0] - te[0]).abs().max(), te[0].abs().max(), max(float((a - b).abs().max()) for a, b in zip(tc, te))))
