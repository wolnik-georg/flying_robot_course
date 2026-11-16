"""Stage 5 diagnostic (hint of the user, 14 Nov): one-step (one 2-ms block) Jacobian of the S1 / S2 / S3 model, autograd against central finite differences,
at recorded blocks of the A1 flight (theta = 0, beta = 0, noise off). Inputs: state (156 numbers for S1) and the 15 parameters; outputs: next state.
Prints per block the largest |AD - FD| / scale and, for the worst blocks, which (output, input) pair it is.
Usage: DIFFCOTUNE_CONTROLLER=S1 python3 jac1.py <first_block> <last_block> <stride> [h]"""
import sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model()
ST = ['x', 'v', 'q', 'w', 'F', 'i_pos', 'i_att', 'cm1', 'cm2', 'cm3', 'tau_pl', 'fs', 'om_prev', 'a_prev', 'Fh']
mod = sys.modules[M.__name__]
b0, b1, stride = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); h = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-6
s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
rec = {}; orig = mod._block; cnt = [0]
def spy(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw):
    k = cnt[0]; cnt[0] += 1
    if b0 <= k <= b1 and (k - b0) % stride == 0:
        rec[k] = ([t.detach().clone() for t in st], spk, tpk, jit, act_top, nets, nz, bw)
    return orig(st, spk, tpk, jit, gains, pr, act_top, nets, nz, bw)
mod._block = spy
z0 = torch.zeros(1, n, dtype=torch.float64)
with torch.no_grad(): M.rollout(ref, z0[:, :10], z0[:, 10:], n_blocks=b1 + 1)
mod._block = orig
dt = torch.float64; shapes = None
def pack(st): return torch.cat([t.reshape(-1) for t in st])
def make_f(k):
    st, spk, tpk, jit, act_top, nets, nz, bw = rec[k]; sh = [t.shape for t in st]; sz = [t.numel() for t in st]
    def f(vec):
        sv, zz = vec[:sum(sz)], vec[sum(sz):][None]
        parts = list(torch.split(sv, sz)); stt = tuple(p.reshape(s_) for p, s_ in zip(parts, sh))
        gains = M.S0.gains_from(zz[:, :10], dt, 'cpu') if hasattr(M, 'S0') else None
        pr = M.S0.beta_from(zz[:, 10:14], dt, 'cpu')
        if L.N_BETA == 5: pr['rd'] = M.RD_START * torch.exp(zz[:, 14])
        out = orig(stt, spk, tpk, jit, gains, pr, act_top, nets, nz, bw)
        return pack(out)
    return f, sh, sz
names = []
res = []
for k in sorted(rec):
    f, sh, sz = make_f(k); v0 = torch.cat([pack(rec[k][0]), z0[0]]); ns = sum(sz)
    if not names:
        names = [(nm, i) for nm, z_ in zip(ST, sz) for i in range(z_)]
        inames = names + [('param', i) for i in range(n)]
    t0 = time.time(); Jad = torch.autograd.functional.jacobian(f, v0).numpy()
    Jfd = np.zeros_like(Jad)
    for j in range(v0.numel()):
        e = torch.zeros_like(v0); e[j] = h
        Jfd[:, j] = ((f(v0 + e) - f(v0 - e)) / (2 * h)).detach().numpy()
    err = np.abs(Jad - Jfd); scale = np.abs(Jfd).max(0)[None, :] + 1e-9
    rel = (err / np.maximum(np.abs(Jfd), scale * 1e-3 + 1e-12)); ia = np.unravel_index(np.argmax(err / (np.abs(Jfd).max() + 1e-12)), err.shape)
    o = names[ia[0]]; i_ = inames[ia[1]]
    print('block %4d (t=%.3f s)  max|JFD| %.3e  max|AD-FD| %.3e  worst: out %s[%d] in %s[%d]  AD %.4e FD %.4e  (%.0f s)' % (k, 2 * k * 1e-3, np.abs(Jfd).max(), err.max(), o[0], o[1], i_[0], i_[1], Jad[ia], Jfd[ia], time.time() - t0), flush=True)
    res.append((k, err.max()))
