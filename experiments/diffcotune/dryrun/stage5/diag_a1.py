"""Stage 5: diagnostic of the A1 gradient mismatch (S2): finite-difference step scan and the minimum height of the A1 flight at the gradient point. Usage: DIFFCOTUNE_CONTROLLER=S2 python3 diag_a1.py"""
import sys, os
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R, costs as C  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
z0 = torch.zeros(1, n, dtype=torch.float64)
with torch.no_grad(): o = M.rollout(ref, z0[:, :10], z0[:, 10:]); zz = o['x'][:, 0, 2].numpy(); t = o['t']; m = (t > 0) & (t <= ref.scenario_s)
print('A1 at theta_0, beta calibration: min z %.4f m, z at scenario end %.4f, mean z task window %.3f (ref %.3f); samples with z < 0.01: %d' % (zz[m].min(), zz[m][-1], zz[(t > 2) & (t <= ref.scenario_s)].mean(), ref.pos[:, 2].mean(), int((zz < 0.01).sum())))
for h in (1e-4, 1e-5, 1e-6, 1e-7):
    eye = torch.eye(n, dtype=torch.float64) * h; zz2 = torch.cat([eye, -eye])
    with torch.no_grad(): o2 = M.rollout(ref, zz2[:, :10], zz2[:, 10:]); jt = C.j_task(o2, ref.scenario_s)
    f = ((jt[:n] - jt[n:]) / (2 * h)).numpy(); print('h %.0e FD J_task: Pz %.6e Dz %.6e thrust_gain %.6e downwash %.6e' % (h, f[1], f[3], f[10], f[12]), flush=True)
