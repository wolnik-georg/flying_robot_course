#!/usr/bin/env python3
"""Stage 3d diagnostic of the failed gradient check (round-1 point): (1) finite-difference step scan of the combined cost for Pz, KRxy, Kwxy;
(2) autograd vs FD per scenario (h = 1e-5)."""
import json, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L, noise_forward_check as N  # noqa
if __name__ == '__main__':
    pool = L.Pool(4, noise_seeds=(1000, 1001)); x = np.r_[N.load_phis()['round1'], L.PSI_TRUE]; out = {}
    comps = {'Pz': 1, 'KRxy': 5, 'Kwxy': 7}
    try:
        _, g = pool.rollouts('task', x, None)
        for nm, i in comps.items():
            for h in (1e-4, 1e-6):
                xp = x.copy(); xm = x.copy(); xp[i] += h; xm[i] -= h
                fd = (pool.rollouts('task_fwd', xp, None)[0] - pool.rollouts('task_fwd', xm, None)[0]) / (2 * h)
                out['combined %s h=%g' % (nm, h)] = dict(autograd=float(g[i]), fd=float(fd)); print('combined', nm, 'h', h, 'autograd %.5e fd %.5e' % (g[i], fd), flush=True)
        for sc in L.TUNING:
            _, gs = pool.rollouts('task', x, None, scen_list=[sc])
            for nm, i in comps.items():
                h = 1e-5; xp = x.copy(); xm = x.copy(); xp[i] += h; xm[i] -= h
                fd = (pool.rollouts('task_fwd', xp, None, scen_list=[sc])[0] - pool.rollouts('task_fwd', xm, None, scen_list=[sc])[0]) / (2 * h)
                out['%s %s' % (sc[0], nm)] = dict(autograd=float(gs[i]), fd=float(fd)); print(sc[0], nm, 'autograd %.5e fd %.5e rel %.4f' % (gs[i], fd, abs(gs[i] - fd) / abs(fd)), flush=True)
    finally:
        pool.close()
    json.dump(out, open(HERE / 'noise_gradient_diag.json', 'w'), indent=1)
