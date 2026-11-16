"""Stage 5 diagnostic (controller from DIFFCOTUNE_CONTROLLER): where in the A1 flight does the autograd gradient of J_task explode?
Windows 2..T_end s, eager AD vs compiled AD vs central FD, theta=0, beta=0 (calibration)."""
import sys, time
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
print('controller', L.CTRL, 'scenario', s, 'scenario duration', float(ref.scenario_s), flush=True)
def J(zz, comp, T_end):
    o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=comp); m = torch.as_tensor((o['t'] > 2.0) & (o['t'] <= T_end))
    e = o['x'][m] - o['ref_pos'][m][:, None, :]; return (e * e).sum(-1).mean(0)
for T_end in (3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 14.0):
    t0 = time.time(); out = []
    for comp in (False, True):
        zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); j = J(zg, comp, T_end)
        g = torch.autograd.grad(j.sum(), zg)[0][0].numpy(); out.append(g)
    h = 1e-6; eye = torch.eye(n, dtype=torch.float64) * h; zz = torch.cat([eye, -eye])
    with torch.no_grad(): jj = J(zz, False, T_end)
    f = ((jj[:n] - jj[n:]) / (2 * h)).numpy()
    print('T_end %4.1f  max|g| eager %.3e compiled %.3e  FD %.3e  | Pz %.3e/%.3e/%.3e  Dz %.3e/%.3e/%.3e  (%.0f s)' % (T_end, abs(out[0]).max(), abs(out[1]).max(), abs(f).max(), out[0][1], out[1][1], f[1], out[0][3], out[1][3], f[3], time.time() - t0), flush=True)
