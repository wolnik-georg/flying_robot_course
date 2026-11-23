#!/usr/bin/env python3
"""Protocol v2.1: bounded-step test of the tuning step. The projected Adam of loop_lib.adam_inner is driven with a cost that always pushes every gain up (J = -sum x) and then always down (J = +sum x) for 50 epochs;
the gains must stop exactly at the box of the method: ours / ours2 [0.8, 1.25] x theta_l (log +-1.25, symmetric in log), ours21 [0.9, 1.1] x theta_l (linear +-10 %, log bounds [log 0.9, log 1.1]).
Run: python3 test_v21_bounds.py   (the limit is built with the same expression as round.py from loop_lib.METHOD_SETTINGS)"""
import os, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import loop_lib as L
ok = True
for method, want in (('ours', (0.8, 1.25)), ('ours2', (0.8, 1.25)), ('ours21', (0.9, 1.1))):
    S = L.METHOD_SETTINGS[method]
    limit = ((float(np.log(1 - S['gain_limit'])), float(np.log(1 + S['gain_limit']))) if S.get('linear_limit') else float(np.log(1 + S['gain_limit'])))
    x0 = np.log(np.array([1.0, 2.0, 5.0, 0.01, 7.0, 3.0, 0.5, 0.2, 1.5, 4.0]))
    res = {}
    for name, sgn in (('up', -1.0), ('down', +1.0)):
        fn = lambda x: (float(sgn * x[:10].sum()), np.where(np.arange(len(x)) < 10, sgn, 0.0))
        # best=True returns the best iterate, which for this monotone cost is the one at the limit
        x, rec = L.adam_inner(fn, np.r_[x0, np.zeros(4)], np.arange(10), 0.01, K2=50, best=True, stop=False, limit=limit)
        res[name] = np.exp(x[:10] - x0)
    g_up, g_dn = res['up'], res['down']
    good = bool(np.allclose(g_up, want[1], rtol=0, atol=1e-12) and np.allclose(g_dn, want[0], rtol=0, atol=1e-12))
    print('%-7s limit %s: factors at the limit up %s / down %s (wanted %.2f / %.2f): %s' % (method, 'log' + str(tuple(round(v, 4) for v in limit)) if isinstance(limit, tuple) else 'log +-%.4f' % limit, np.round(g_up[:3], 4).tolist(), np.round(g_dn[:3], 4).tolist(), want[1], want[0], 'OK' if good else 'FAIL'))
    ok &= good
print('ALL OK' if ok else 'FAIL'); sys.exit(0 if ok else 1)
