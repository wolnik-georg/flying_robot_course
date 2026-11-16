"""Stage 5 diagnostic: reverse-mode d x_z(t_i) / d z against forward-mode (jvp) and FD, per record, for A1. Which time does reverse mode first deviate?
Usage: DIFFCOTUNE_CONTROLLER=S1 python3 revfwd.py <param j> <n_blocks> <keep_blocks> <comma-separated record indices>"""
import sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
j, nb, kb = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]); recs = [int(a) for a in sys.argv[4].split(',')]
def X(zz):
    o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=False, n_blocks=nb, keep_blocks=kb); return o['x'][:, 0, :], o['t']
zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); x, t = X(zg)
z0 = torch.zeros(1, n, dtype=torch.float64); e = torch.zeros(1, n, dtype=torch.float64); e[0, j] = 1.0; h = 1e-6
with torch.no_grad(): xp = X(z0 + h * e)[0]; xm = X(z0 - h * e)[0]
fd = (xp - xm) / (2 * h)
for i in recs:
    for comp in (2,):
        g = torch.autograd.grad(x[i, comp], zg, retain_graph=True)[0][0, j].item()
        print('t %.3f s comp %d  reverse %.4e  FD %.4e' % (t[i], comp, g, fd[i, comp].item()), flush=True)
