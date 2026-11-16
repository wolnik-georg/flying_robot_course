#!/usr/bin/env python3
"""Stage 8 step 1b: paper-exact J_sysId (all states p, v, q, w, unweighted) on S0, the identifiability test of Stage 4e T0 repeated with the paper's form, fit only, plant P1, noisy frsim flights of theta_0 (flight seeds 0 and 1 as Stage 4e: s * 10 + i), the 5 tuning
scenarios, lr 0.01 (as Stage 4e; the learning rate of the paper-faithful variant is chosen in step 1c), compiled forward sensitivity. Variants (name: states, body-rate target, stop):
  pvq_best        position + velocity + attitude, 50 epochs with the best iterate (the frozen J_sysId of Stages 3-7)
  pvq_paperstop   the same with the paper's stop rule (raw cost, last iterate)
  pvqw_true_best  + body rates, target = the TRUE body rate of the frsim state trace, 50 epochs best iterate (no paper stop)
  pvqw_true_stop  the same with the paper stop rule
  pvqw_gyro_best  + body rates, target = measured gyro (true rate + the plant's white gyro noise, stage4e/gyro_variants.py, as real logs), 50 epochs best iterate
  pvqw_gyro_stop  the same with the paper stop rule
Reports beta factors and errors against the truth, epochs used, J_sysId parts p, v, q, w at the start and at the returned iterate. Usage (job_runner, DIFFCOTUNE_GRAD_MODE=cfwd): python3 ident8.py -> results/ident8.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage4e'))
import loop_lib as L  # noqa
import gyro_variants as G  # noqa
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
PVQ, PVQW = ('p', 'v', 'q'), ('p', 'v', 'q', 'w')
VARIANTS = [('pvq_best', PVQ, None, False), ('pvqw_true_best', PVQW, None, False), ('pvqw_true_stop', PVQW, None, True), ('pvqw_gyro_stop', PVQW, 'raw', True)]          # trimmed for time (15 Nov, urgent): pvq_paperstop, pvqw_gyro_best not run
def parts_mean(pool, x, tr_by):
    res = pool.pool.map(L.worker_parts, [(s, c, x, tr) for i, (s, c) in enumerate(L.TUNING) for tr in tr_by[i]], chunksize=1); return {k: float(np.mean([r[1][k] for r in res])) for k in 'pvqw'}
if __name__ == '__main__':
    n = len(L.TUNING); pool = L.Pool(n); out = []
    try:
        fl2 = [[] for _ in range(n)]
        for seed in (0, 1):
            fl = L.fly_set(pool, np.zeros(10), L.P1, L.TUNING, [seed * 10 + i for i in range(n)])
            for i, f in enumerate(fl): fl2[i].append(f)
            say('flights seed', seed, 'real 3-D RMS %.2f cm' % L.round_cost(fl)['rms3d_cm'])
        truth = np.exp(L.PSI_TRUE)
        for name, st, mode, paper in VARIANTS:
            fn = G.make_trace_fn(mode, L.P1['overrides'], 110, seed_base=7000) if mode else None
            tr_by = {i: [(fn(f['trace'], s, i) if fn else f['trace']) for s, f in enumerate(fl2[i])] for i in range(n)}
            pool.states = st; t0 = time.time()
            x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, tr_by), np.zeros(14), np.arange(10, 14), 0.01, K2=50, stop=False, best=not paper, paper_stop=paper, label='ident8 ' + name)
            p0 = parts_mean(pool, np.zeros(14), tr_by); p1 = parts_mean(pool, x, tr_by); fac = np.exp(x[10:]); err = 100 * (fac / truth - 1)
            r = dict(variant=name, states=list(st), gyro=mode, paper_stop=paper, factors=fac.tolist(), truth=truth.tolist(), err_pct=err.tolist(), epochs=rec['epochs'], reason=rec.get('reason'), best_epoch=rec.get('best_epoch'), J0=rec['J0'], J_best=rec.get('J_best'), J_end=rec.get('J_end'),
                     parts_start=p0, parts_end=p1, seconds=time.time() - t0)
            out.append(r); json.dump(out, open(HERE / 'results' / 'ident8.json', 'w'), indent=1)
            say('%-15s err %% %s  epochs %d (%s)  J %.4g -> %.4g  (%.0f s)' % (name, np.round(err, 1), rec['epochs'], rec.get('reason'), rec['J0'], rec['J_best'] if rec.get('J_best') is not None else rec['J_end'], time.time() - t0))
            say('                parts start %s end %s' % ({k: round(v, 5) for k, v in p0.items()}, {k: round(v, 5) for k, v in p1.items()}))
    finally:
        pool.close()
