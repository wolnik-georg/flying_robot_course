#!/usr/bin/env python3
"""Stage 4e T0: identifiability of beta with the body rates, fit only (as Stage 3b B1: P1, noisy frsim flights of theta_0 on the 4 tuning scenarios, flight seeds s*10+i for s = 0, 1,
50 epochs, best iterate, no early stop, Adam lr 0.01). Variants of the J_sysId state set / gyro target:
  pva            position + velocity + attitude (current method version)
  pvqw_raw       + body rates, unweighted, measured gyro (true rate + the plant's white gyro noise, see gyro_variants.py)
  pvqw_sg110     the same, measured gyro smoothed by a zero-phase 5th-order Savitzky-Golay filter, 110 ms (11 samples at 100 Hz)   <- main window
  pvqw_sg70, pvqw_sg210   one shorter, one longer window
Reports beta (factors) against the truth, error per value, J_sysId parts p, v, q, w at the start and at the best iterate. Usage (nice -n 19): python3 t0_identifiability.py"""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402
import gyro_variants as G  # noqa: E402

PVA, PVQW = ('p', 'v', 'q'), ('p', 'v', 'q', 'w')
VARIANTS = [('pva', PVA, None, 0), ('pvqw_raw', PVQW, 'raw', 0), ('pvqw_sg110', PVQW, 'sg', 110), ('pvqw_sg70', PVQW, 'sg', 70), ('pvqw_sg210', PVQW, 'sg', 210)]


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def parts_mean(pool, x, tr_by_scen):
    jobs = [(s, c, x, tr) for i, (s, c) in enumerate(L.TUNING) for tr in tr_by_scen[i]]
    res = pool.pool.map(L.worker_parts, jobs, chunksize=1)
    return {k: float(np.mean([r[1][k] for r in res])) for k in 'pvqw'}


if __name__ == '__main__':
    pool = L.Pool(4)
    try:
        fl2 = [[], [], [], []]
        for seed in (0, 1):
            fl = L.fly_set(pool, np.zeros(10), L.P1, L.TUNING, [seed * 10 + i for i in range(4)])
            for i, f in enumerate(fl):
                fl2[i].append(f)
            say('flights seed', seed, 'real 3-D RMS %.2f cm' % L.round_cost(fl)['rms3d_cm'])
        out = []
        truth = np.exp(L.PSI_TRUE)
        for name, st, mode, ms in VARIANTS:
            fn = G.make_trace_fn(mode, L.P1['overrides'], ms or 110, seed_base=7000)
            tr_by = {i: [fn(f['trace'], s, i) for s, f in enumerate(fl2[i])] for i in range(4)}     # key (l, i) = (flight seed index, scenario)
            pool.states = st; t0 = time.time()
            x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, tr_by), np.zeros(14), np.arange(10, 14), 0.01, K2=50, stop=False, best=True, label='T0 ' + name)
            p0 = parts_mean(pool, np.zeros(14), tr_by); p1 = parts_mean(pool, x, tr_by)
            fac = np.exp(x[10:]); err = 100 * (fac / truth - 1)
            r = dict(variant=name, states=list(st), gyro=mode, window_ms=ms, factors=fac.tolist(), truth=truth.tolist(), err_pct=err.tolist(), best_epoch=rec['best_epoch'], epochs=rec['epochs'],
                     J0=rec['J0'], J_best=rec['J_best'], parts_start=p0, parts_end=p1, seconds=time.time() - t0, log=rec['log'])
            out.append(r)
            say('%-11s beta %s  err %% %s  best epoch %d  J %.4g -> %.4g  (%.0f s)' % (name, np.round(fac, 3), np.round(err, 1), rec['best_epoch'], rec['J0'], rec['J_best'], time.time() - t0))
            say('            parts start %s  end %s' % ({k: round(v, 5) for k, v in p0.items()}, {k: round(v, 5) for k, v in p1.items()}))
            L.jdump(out, HERE / 'results' / 't0_identifiability.json')
    finally:
        pool.close()
