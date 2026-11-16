"""Stage 5 diagnostic: is the large autograd gradient the true derivative? Central FD of J_task (window 2..T_end, A1, theta=beta=0) with shrinking step h, against eager AD."""
import sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA; T_end = float(sys.argv[1])
def J(zz):
    o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=False); mk = torch.as_tensor((o['t'] > 2.0) & (o['t'] <= T_end))
    e = o['x'][mk] - o['ref_pos'][mk][:, None, :]; return (e * e).sum(-1).mean(0)
zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); g = torch.autograd.grad(J(zg).sum(), zg)[0][0].numpy()
print('AD  max %.3e  Pz %.4e Dz %.4e thrust %.4e rpmdelay %.4e' % (abs(g).max(), g[1], g[3], g[10], g[14]), flush=True)
for h in (1e-4, 1e-6, 1e-8, 1e-10, 1e-12):
    eye = torch.eye(n, dtype=torch.float64) * h
    with torch.no_grad(): jj = J(torch.cat([eye, -eye]))
    f = ((jj[:n] - jj[n:]) / (2 * h)).numpy()
    print('FD h=%.0e max %.3e  Pz %.4e Dz %.4e thrust %.4e rpmdelay %.4e' % (h, abs(f).max(), f[1], f[3], f[10], f[14]), flush=True)
