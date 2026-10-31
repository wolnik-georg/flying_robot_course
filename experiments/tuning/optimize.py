"""Optimiser runners on a Job: CMA-ES (ask/tell, parallel population) and a scrambled-Sobol baseline with the same budget. Usage: see run_job.py."""
import time
import numpy as np
from scipy.stats import qmc
import core


def run_sobol(job, pool, budget, seed, batch, logpath):
    pts = qmc.Sobol(job.d, scramble=True, seed=seed).random(budget); best = None; t0 = time.time()
    for i in range(0, budget, batch):
        us = pts[i:i + batch]; res = core.evaluate_batch(job, pool, [job.gains(u) for u in us])
        for u, r in zip(us, res):
            r.update(method='sobol', seed=seed, index=len(open(logpath).readlines()) if False else i, u=list(map(float, u)), t=time.time() - t0); core.log_eval(logpath, r)
            if best is None or r['score'] < best['score']: best = r
    return best


def run_cmaes(job, pool, budget, seed, popsize, logpath, sigma0=0.25, x0=None):
    import cma
    x0 = job.default_u() if x0 is None else np.asarray(x0)
    es = cma.CMAEvolutionStrategy(x0, sigma0, dict(bounds=[0, 1], popsize=popsize, seed=seed + 1, verbose=-9)); best = None; n = 0; t0 = time.time()
    while n < budget:
        us = es.ask(); res = core.evaluate_batch(job, pool, [job.gains(np.asarray(u)) for u in us])
        es.tell(us, [r['score'] for r in res])
        for u, r in zip(us, res):
            r.update(method='cmaes', seed=seed, index=n, u=list(map(float, u)), t=time.time() - t0); core.log_eval(logpath, r); n += 1
            if best is None or r['score'] < best['score']: best = r
        print('gen %d  n %d  best %.3f  gen-best %.3f  sigma %.3f' % (es.countiter, n, best['score'], min(r['score'] for r in res), es.sigma), flush=True)
    return best
