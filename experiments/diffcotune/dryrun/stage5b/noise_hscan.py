"""Stage 5b: with noise (seeds 1000 / 1001, theta = 0, beta = truth) the finite difference of the combined J_task depends on h (Stage 3d diagnostic). Scan h and compare with the forward-mode gradient (the exact autodiff
value; reverse mode gave the same numbers in fwd_checks.py (c)). Output: per h the largest |FD - forward| / max|forward| and the number of gates failed. Uses the stored forward gradient of results/fwd_checks_<C>.json."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5'))
import loop_lib as L  # noqa
import fwd_checks as F  # noqa
if __name__ == '__main__':
    d = json.load(open(HERE / 'results' / ('fwd_checks_%s.json' % L.CTRL))); g = np.mean(d['c_noise']['g_fwd_scen'], 0)[:10]
    pool = L.Pool(4, noise_seeds=(1000, 1001)); xn = np.r_[np.zeros(10), L.PSI_TRUE]; out = {}
    try:
        for h in (1e-4, 1e-5, 1e-6, 1e-7, 1e-8):
            fd = F.fd_of(pool, 'task', xn, list(range(10)), None, h); rows = F.judge(g, fd, F.NAMES[:10])
            out[str(h)] = dict(fd=fd.tolist(), max_err_over_max=float(np.abs(fd - g).max() / np.abs(g).max()), n_fail=sum(not r['passed'] for r in rows))
            print('h %.0e  max|FD - forward| / max|forward| = %.3e  gate failures %d of 10   FD %s' % (h, out[str(h)]['max_err_over_max'], out[str(h)]['n_fail'], np.array2string(fd, precision=4)), flush=True)
    finally:
        pool.close()
    out['forward'] = g.tolist(); json.dump(out, open(HERE / 'results' / ('noise_hscan_%s.json' % L.CTRL), 'w'), indent=1)
