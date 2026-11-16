"""Stage 5b: cost of forward mode on 300 blocks of the A1 rollout: reverse eager, reverse compiled, jvp with 1 tangent, vmap-jvp with 5 and 15 tangents."""
import sys, time
from pathlib import Path
import numpy as np, torch
from torch.func import jvp, vmap
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parents[0]))
import loop_lib as L, reference as R, costs as C  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA; NB = 300
def X(z, comp=False):
    o = M.rollout(ref, z[:, :10], z[:, 10:], compile=comp, n_blocks=NB); return o['x'][-1, 0]
z0 = torch.zeros(1, n, dtype=torch.float64)
def T(label, fn):
    t0 = time.time(); fn(); print('%-34s %.1f s for %d blocks = %.1f ms/block' % (label, time.time() - t0, NB, 1e3 * (time.time() - t0) / NB), flush=True)
def rev(comp):
    z = z0.clone().requires_grad_(True); x = X(z, comp).sum(); torch.autograd.grad(x, z)
T('primal only, eager', lambda: X(z0)); T('reverse eager', lambda: rev(False)); rev(True); T('reverse compiled (2nd call)', lambda: rev(True))
e = torch.zeros(1, n, dtype=torch.float64); e[0, 3] = 1
T('jvp, 1 tangent', lambda: jvp(lambda z: X(z), (z0,), (e,)))
for k in (5, 15):
    E = torch.zeros(k, 1, n, dtype=torch.float64); E[torch.arange(k), 0, torch.arange(k)] = 1
    T('vmap(jvp), %d tangents' % k, lambda: vmap(lambda t: jvp(lambda z: X(z), (z0,), (t,))[1])(E))
