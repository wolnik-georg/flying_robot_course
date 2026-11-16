"""Stage 5b step 2 first test: forward-mode (jacfwd, eager) J_task gradient of one scenario against central FD and (if it works) reverse. usage: fwd_try.py <scenario index 0..3>"""
import sys, time
from pathlib import Path
import numpy as np, torch
from torch.func import jacfwd
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parents[0]))
import loop_lib as L, reference as R, costs as C  # noqa
torch.set_num_threads(2); M = L.model(); s, c = L.TUNING[int(sys.argv[1])]; ref = R.build(s, c); n = 10 + L.N_BETA
def f(z):
    o = M.rollout(ref, z[:, :10], z[:, 10:], compile=False); J = C.j_task(o, ref.scenario_s).sum(); return J, J.detach()
z0 = torch.zeros(1, n, dtype=torch.float64); t0 = time.time()
jac, J = jacfwd(f, has_aux=True)(z0); print(s, 'jacfwd %.0f s' % (time.time() - t0), 'J', float(J), flush=True); g = jac.reshape(-1).numpy(); print('fwd', g, flush=True)
h = 1e-6; fd = np.zeros(n)
with torch.no_grad():
    for j in range(n):
        e = torch.zeros(1, n, dtype=torch.float64); e[0, j] = h
        fd[j] = (f(e)[0] - f(-e)[0]).item() / (2 * h)
print('fd ', fd); print('max|fwd-fd| %.3e  max|fd| %.3e' % (np.abs(g - fd).max(), np.abs(fd).max()))
