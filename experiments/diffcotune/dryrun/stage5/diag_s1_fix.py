"""Stage 5 diagnostic: A1 gradient (window 2..T_end s) with the patched copy s1_model_try (env SQRT_EPS), compiled AD vs eager FD."""
import sys, os, time, importlib.util
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[0])); sys.path.insert(0, str(HERE.parents[1]))
spec = importlib.util.spec_from_file_location('s1_model', HERE / 's1_model_try.py'); m = importlib.util.module_from_spec(spec); sys.modules['s1_model'] = m; spec.loader.exec_module(m)
import loop_lib as L, reference as R  # noqa
torch.set_num_threads(2); M = L.model(); assert M.__name__ == 's1_model' and hasattr(M, 'SQRT_EPS'), M
s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
print('SQRT_EPS', M.SQRT_EPS, flush=True)
def J(zz, comp, T_end):
    o = M.rollout(ref, zz[:, :10], zz[:, 10:], compile=comp); mk = torch.as_tensor((o['t'] > 2.0) & (o['t'] <= T_end))
    e = o['x'][mk] - o['ref_pos'][mk][:, None, :]; return (e * e).sum(-1).mean(0)
for T_end in [float(a) for a in sys.argv[1:]]:
    t0 = time.time(); zg = torch.zeros(1, n, dtype=torch.float64, requires_grad=True); j = J(zg, True, T_end)
    g = torch.autograd.grad(j.sum(), zg)[0][0].numpy()
    h = 1e-6; eye = torch.eye(n, dtype=torch.float64) * h; zz = torch.cat([eye, -eye])
    with torch.no_grad(): jj = J(zz, False, T_end)
    f = ((jj[:n] - jj[n:]) / (2 * h)).numpy()
    print('T_end %4.1f J %.6e  max|AD| %.3e max|FD| %.3e  max|AD-FD| %.3e | Pz %.4e/%.4e  Dz %.4e/%.4e  thrust %.4e/%.4e  rpmdelay %.4e/%.4e (%.0f s)' % (T_end, float(j.sum()), abs(g).max(), abs(f).max(), abs(g - f).max(), g[1], f[1], g[3], f[3], g[10], f[10], g[14], f[14], time.time() - t0), flush=True)
