"""Stage 5 diagnostic: A1 full flight of S2, J_task value compiled (with grad) vs eager (no grad) at theta_0 / calibration, and the gradient of J_task on horizons of 6, 10, 14, 18 s of scenario time (compiled AD vs eager FD)."""
import sys
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L, reference as R, costs as C  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
z = torch.zeros(1, n, dtype=torch.float64)
with torch.no_grad(): je = float(C.j_task(M.rollout(ref, z[:, :10], z[:, 10:]), ref.scenario_s))
zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); jc = float(C.j_task(M.rollout(ref, zg[:, :10], zg[:, 10:], compile=True), ref.scenario_s))
print('full flight J_task eager %.12e compiled %.12e' % (je, jc), flush=True)
for T_end in (6.0, 10.0, 12.0, 14.0):
    nb = int((ref.n_ticks // 2) * 0 + (T_end + 4.5 + ref.time_s[0] * 0) * 500) if False else None
    def J(zz, comp):
        o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=comp); t = torch.as_tensor(o['t']); m = torch.as_tensor((o['t'] > 2.0) & (o['t'] <= T_end))
        e = o['x'][m] - o['ref_pos'][m][:, None, :]; return (e * e).sum(-1).mean(0)
    zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); j = J(zg, True); g = torch.autograd.grad(j.sum(), zg)[0][0].numpy()
    h = 1e-6; eye = torch.eye(n, dtype=torch.float64) * h; zz = torch.cat([eye, -eye])
    with torch.no_grad(): jj = J(zz, False)
    f = ((jj[:n] - jj[n:]) / (2 * h)).numpy()
    print('window 2 .. %.0f s: AD vs FD  Pz %.4e / %.4e  Dz %.4e / %.4e  thrust %.4e / %.4e  downwash %.4e / %.4e' % (T_end, g[1], f[1], g[3], f[3], g[10], f[10], g[12], f[12]), flush=True)
