#!/usr/bin/env python3
"""Stage 5c open point: S2 gradient gate, noisy J_task (seeds 6000 / 6001, theta = 0, beta = truth): reverse gradient against central finite differences for several h (the gate used h = 1e-7: KRxy rel 4.0e-2).
Output results/s2_krxy_hscan.json and a print per h of the relative error of every gain (|fd - rev| / |rev|). Usage (job_runner): python3 s2_krxy_hscan.py"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5')); sys.path.insert(0, str(HERE.parent / 'stage5b'))
import loop_lib as L  # noqa
import fwd_checks as F  # noqa
if __name__ == '__main__':
    pool = L.Pool(4, noise_seeds=(6000, 6001)); out = {}
    try:
        xn = np.r_[np.zeros(10), L.PSI_TRUE]; jobs = [('task_R', s, c, xn, None, pool.states, pool.noise_seeds) for s, c in L.TUNING]
        g = np.mean([r[1] for r in pool.pool.map(L.worker_rollout, jobs, chunksize=1)], 0)[:10]; out['reverse'] = g.tolist(); print('reverse', np.array2string(g, precision=5), flush=True)
        for h in (1e-4, 1e-5, 1e-6, 1e-7, 1e-8):
            fd = F.fd_of(pool, 'task', xn, list(range(10)), None, h); rel = np.abs(fd - g) / np.maximum(np.abs(g), 1e-300)
            out[str(h)] = dict(fd=fd.tolist(), rel=rel.tolist()); print('%s h %.0e KRxy (index 5) fd %+.6e rev %+.6e rel %.2e | worst rel %.2e (%s)' % (time.strftime('%H:%M:%S'), h, fd[5], g[5], rel[5], rel.max(), F.NAMES[int(rel.argmax())]), flush=True)
    finally:
        pool.close()
    json.dump(out, open(HERE / 'results' / 's2_krxy_hscan.json', 'w'), indent=1)
