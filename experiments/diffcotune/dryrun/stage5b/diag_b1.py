"""Stage 5b step 1a: reverse-mode J_task gradient of S1 on A1 (whole scenario, eager, noise off, theta = 0, beta = 0) in float32 and float64 against the central FD (float64, h = 1e-6).
If the gap to FD grows by orders of magnitude in float32, the failure is rounding driven."""
import sys, time, json
from pathlib import Path
import numpy as np, torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parents[0]))
import loop_lib as L, reference as R, costs as C  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[0]; ref = R.build(s, c); n = 10 + L.N_BETA
def J(z, dt):
    o = M.rollout(ref, z[:, :10], z[:, 10:], dtype=dt, compile=False); return C.j_task(o, ref.scenario_s).sum()
res = {}
h = 1e-6; fd = np.zeros(n)
with torch.no_grad():
    for j in range(n):
        e = torch.zeros(1, n, dtype=torch.float64); e[0, j] = h
        fd[j] = (J(e, torch.float64) - J(-e, torch.float64)).item() / (2 * h)
print('FD', fd, flush=True)
for dt in (torch.float64, torch.float32):
    t0 = time.time(); z = torch.zeros(1, n, dtype=dt, requires_grad=True)
    g = torch.autograd.grad(J(z, dt), z)[0][0].double().numpy(); print(dt, 'reverse', g, '%.0f s' % (time.time() - t0), flush=True)
    res[str(dt)] = dict(grad=g.tolist(), max_abs_err=float(np.abs(g - fd).max()), rel_to_max_fd=float(np.abs(g - fd).max() / np.abs(fd).max()))
res['fd'] = fd.tolist(); json.dump(res, open(HERE / 'results' / 'diag_b1.json', 'w'), indent=1)
for k in ('torch.float64', 'torch.float32'): print(k, 'max|g-fd| %.3e  relative to max|fd| %.3e' % (res[k]['max_abs_err'], res[k]['rel_to_max_fd']))
