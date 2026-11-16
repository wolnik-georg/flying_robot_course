"""Stage 5 diagnostic: A1 gradient of S2, compiled vs eager autograd vs finite difference on a truncated flight (n_blocks = 3000, 6 s): scalar = mean z error^2 over the last 1 s."""
import sys
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA; NB = 3000
def J(z, comp):
    o = M.rollout(ref, z[:, :10], z[:, 10:], compile=comp, n_blocks=NB); t = torch.as_tensor(o['t']); m = (t > t.max() - 1.0)
    e = o['x'][m] - o['ref_pos'][m][:, None, :]; return (e[..., 2] ** 2).mean(0)
res = {}
for comp in (True, False):
    z = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); j = J(z, comp); g = torch.autograd.grad(j.sum(), z)[0][0].numpy(); res[comp] = (float(j), g); print('compile', comp, 'J %.10e' % float(j), 'g Pz %.5e Dz %.5e thrust %.5e downwash %.5e' % (g[1], g[3], g[10], g[12]), flush=True)
h = 1e-6; eye = torch.eye(n, dtype=torch.float64) * h; zz = torch.cat([eye, -eye])
with torch.no_grad(): jj = J(zz, False)
f = ((jj[:n] - jj[n:]) / (2 * h)).numpy(); print('FD', 'Pz %.5e Dz %.5e thrust %.5e downwash %.5e' % (f[1], f[3], f[10], f[12]))
