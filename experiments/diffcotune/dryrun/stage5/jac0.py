"""Stage 5 diagnostic for S0 / S2 (block signature of s0_model._block): one-block Jacobian, autograd against central FD, and eigenvalues of the state Jacobian.
Usage: DIFFCOTUNE_CONTROLLER=S2 python3 jac0.py <first_block> <last_block> <stride>"""
import sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R, s0_model as S0  # noqa
torch.set_num_threads(2); M = L.model(); dt = torch.float64
ST = ['x', 'v', 'q', 'w', 'F', 'i_pos', 'i_att', 'cm1', 'cm2', 'cm3', 'tau_pl']
b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = 1e-6
s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
rec = {}; orig = S0._block; cnt = [0]
def spy(st, spk, tpk, jit, gains, pr, act_top, nets, nz=None):
    k = cnt[0]; cnt[0] += 1
    if b0 <= k <= b1 and (k - b0) % stride == 0: rec[k] = ([t.detach().clone() for t in st], spk, tpk, jit, act_top, nets, nz)
    return orig(st, spk, tpk, jit, gains, pr, act_top, nets, nz)
S0._block = spy
z0 = torch.zeros(1, n, dtype=dt)
with torch.no_grad(): M.rollout(ref, z0[:, :10], z0[:, 10:], n_blocks=b1 + 1)
S0._block = orig
pack = lambda st: torch.cat([t.reshape(-1) for t in st])
for k in sorted(rec):
    st, spk, tpk, jit, act_top, nets, nz = rec[k]; sh = [t.shape for t in st]; sz = [t.numel() for t in st]; ns = sum(sz)
    def f(vec):
        parts = list(torch.split(vec[:ns], sz)); stt = tuple(p.reshape(s_) for p, s_ in zip(parts, sh)); zz = vec[ns:][None]
        return pack(orig(stt, spk, tpk, jit, S0.gains_from(zz[:, :10], dt, 'cpu'), S0.beta_from(zz[:, 10:14], dt, 'cpu'), act_top, nets, nz))
    v0 = torch.cat([pack(st), z0[0]]); Jad = torch.autograd.functional.jacobian(f, v0).numpy(); Jfd = np.zeros_like(Jad)
    for j in range(v0.numel()):
        e = torch.zeros_like(v0); e[j] = h; Jfd[:, j] = ((f(v0 + e) - f(v0 - e)) / (2 * h)).detach().numpy()
    ev = np.linalg.eigvals(Jad[:, :ns]); print('block %4d (t=%.3f s)  max|JFD| %.3e  max|AD-FD| %.3e  max|eig| %.6f (n>1: %d)' % (k, -4.5 + 2e-3 * k, np.abs(Jfd).max(), np.abs(Jad - Jfd).max(), abs(ev).max(), int((abs(ev) > 1 + 1e-9).sum())), flush=True)
