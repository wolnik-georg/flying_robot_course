#!/usr/bin/env python3
"""Stage 3b B1: identifiability check of beta before any loop (no theta tuning). D3 setup: P1, noisy frsim flights of theta0 on the 4 tuning
scenarios, 2 noise seeds each; beta fit with J_sysId, 50 epochs, best iterate, no early stop (Stage 3b stop rule), Adam lr in {0.01, 0.03, 0.1}.
State sets: pva = position + velocity + attitude, pv = position + velocity. Reports fitted beta (factors) vs truth, error per beta, and the
share of each state part (p, v, q, w) in the full J_sysId (all four parts, for reference) at the start and at the end of the fit.
Usage (nice -n 19): python3 dryrun/stage3b/b1_identifiability.py"""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa: E402

SETS = {'pva': ('p', 'v', 'q'), 'pv': ('p', 'v')}


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def parts_mean(pool, x, fl2):
    """Mean over scenarios and seeds of the four parts and their sum (full J_sysId of Stage 3)."""
    jobs = [(s, c, x, f['trace']) for i, (s, c) in enumerate(L.TUNING) for f in fl2[i]]
    res = pool.pool.map(L.worker_parts, jobs, chunksize=1)
    return {k: float(np.mean([r[1][k] for r in res])) for k in 'pvqw'}


if __name__ == '__main__':
    pool0 = L.Pool(4)
    try:
        fl2 = [[], [], [], []]
        for seed in (0, 1):
            fl = L.fly_set(pool0, np.zeros(10), L.P1, L.TUNING, [seed * 10 + i for i in range(4)])
            for i, f in enumerate(fl):
                fl2[i].append(f)
            say('flights seed', seed, 'real 3-D RMS %.2f cm' % L.round_cost(fl)['rms3d_cm'])
        traces = {i: [f['trace'] for f in fl2[i]] for i in range(4)}
        out = []
        for name, st in SETS.items():
            pool0.states = st
            for lr in (0.01, 0.03, 0.1):
                t0 = time.time()
                x, rec = L.adam_inner(lambda xx: pool0.rollouts('sysid', xx, traces), np.zeros(14), np.arange(10, 14), lr, K2=50, stop=False, best=True, label='B1 %s lr %g' % (name, lr))
                p0 = parts_mean(pool0, np.zeros(14), fl2); p1 = parts_mean(pool0, x, fl2)
                fac = np.exp(x[10:]); truth = np.exp(L.PSI_TRUE); err = 100 * (fac / truth - 1)
                r = dict(states=name, lr=lr, factors=fac.tolist(), truth=truth.tolist(), err_pct=err.tolist(), best_epoch=rec['best_epoch'], epochs=rec['epochs'],
                         J0=rec['J0'], J_best=rec['J_best'], parts_start=p0, parts_end=p1, seconds=time.time() - t0, log=rec['log'])
                out.append(r)
                say('%-3s lr %-5g beta %s  err %% %s  best epoch %d  J %.4g -> %.4g  (%.0f s)' % (name, lr, np.round(fac, 3), np.round(err, 1), rec['best_epoch'], rec['J0'], rec['J_best'], time.time() - t0))
                say('      parts start %s  end %s' % ({k: round(v, 5) for k, v in p0.items()}, {k: round(v, 5) for k, v in p1.items()}))
                L.jdump(out, HERE / 'results' / 'b1_identifiability.json')
    finally:
        pool0.close()
