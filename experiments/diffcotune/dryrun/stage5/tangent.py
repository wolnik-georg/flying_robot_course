"""Stage 5 diagnostic: tangent of the A1 position x(t) with respect to one parameter, forward-mode AD (torch.func.jvp) against central FD, per recorded block.
Usage: DIFFCOTUNE_CONTROLLER=S1 python3 tangent.py <param index 0..14> <n_blocks> [keep_blocks]"""
import sys, time
from pathlib import Path
import numpy as np, torch
from torch.func import jvp
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
j, nb = int(sys.argv[1]), int(sys.argv[2]); kb = int(sys.argv[3]) if len(sys.argv) > 3 else 25
def X(zz):
    o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=False, n_blocks=nb, keep_blocks=kb); return o['x'][:, 0, :], o['t']
z0 = torch.zeros(1, n, dtype=torch.float64); e = torch.zeros(1, n, dtype=torch.float64); e[0, j] = 1.0
t0 = time.time(); x0, tx = jvp(lambda zz: X(zz)[0], (z0,), (e,)); print('jvp %.0f s' % (time.time() - t0), flush=True)
h = 1e-6
with torch.no_grad(): xp, t = X(z0 + h * e); xm = X(z0 - h * e)[0]
fd = (xp - xm) / (2 * h)
for i in range(0, len(t), 1):
    if t[i] >= 1.0: print('t %.3f s  |AD| %.3e  |FD| %.3e  |AD-FD| %.3e' % (t[i], tx[i].norm(), fd[i].norm(), (tx[i] - fd[i]).norm()), flush=True)
