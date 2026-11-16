#!/usr/bin/env python3
"""Diagnostics of the beta fit (Stage 3, not part of the method): why does J_sysId not pull beta to the true P1 values?
 D1  at beta0, theta0: J_sysId and its parts (p, v, q, w) per scenario for noisy and noise-free real flights, the autograd gradient per scenario,
     and J_sysId along each beta coordinate (+-0.04 in log) to see how smooth the cost is.
 D2  noise-free P1 data: the beta fit (J_sysId, theta0 fixed) with Adam, 100 epochs, lr 0.03, WITHOUT the stop rule: does beta go to the truth?
Usage: python3 dryrun/diag_fit.py"""
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    pool = L.Pool(4); plant = L.P1
    try:
        gains = L.gains_of(np.zeros(10)); x0 = np.zeros(14)
        fly = {}
        for noise, fn in (('noisy', L.worker_fly), ('clean', L.worker_flyclean)):
            res = pool.pool.map(fn, [(s, c, gains, plant, 0) for s, c in L.TUNING], chunksize=1); fly[noise] = [r[0] for r in res]
        for noise in ('noisy', 'clean'):
            say('D1', noise)
            res = pool.pool.map(L.worker_parts, [(s, c, x0, fly[noise][i]) for i, (s, c) in enumerate(L.TUNING)], chunksize=1)
            for (s, _), (J, p) in zip(L.TUNING, res):
                say('  %-7s J_sysId %.4f parts %s' % (s, J, {k: round(v, 5) for k, v in p.items()}))
            tr = {i: [fly[noise][i]] for i in range(4)}
            for i, (s, c) in enumerate(L.TUNING):
                J, g = L.worker_rollout(('sysid', s, c, x0, tr[i])) if False else pool.pool.apply(L.worker_rollout, (('sysid', s, c, x0, tr[i]),))
                say('  %-7s autograd d/dpsi %s   (descent direction towards the truth: %s)' % (s, np.round(g[10:], 4), np.round(L.PSI_TRUE, 2)))
            # smoothness along each beta coordinate, A8 and A1
            for i in (0, 1):
                s, c = L.TUNING[i]
                for j in range(4):
                    ts = np.linspace(-0.04, 0.04, 9); jobs = []
                    for t in ts:
                        x = x0.copy(); x[10 + j] = t; jobs.append(('sysid_fwd', s, c, x, tr[i]))
                    vals = pool.pool.map(L.worker_rollout, jobs, chunksize=1)
                    say('  line %-3s %-15s J(t) for t = -0.04 ... +0.04: %s' % (s, L.BETA_NAMES[j], np.round([v[0] for v in vals], 4)))
        say('D2 clean P1 data, J_sysId fit, 100 epochs, lr 0.03, no stop rule')
        traces = {i: [fly['clean'][i]] for i in range(4)}
        x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, traces), x0, np.arange(10, 14), 0.03, K2=100, label='D2', stop=False)
        say('D2 (no stop rule) epochs', rec['epochs'], rec['reason'], 'psi', np.round(x[10:], 3), 'truth', np.round(L.PSI_TRUE, 3))
        L.jdump(dict(D2=rec), HERE / 'results' / 'diag_fit_D2.json')
    finally:
        pool.close()
