#!/usr/bin/env python3
"""Stage 5c rerun of Stage 5 B2: identifiability of beta before any loop, per controller (as Stage 3b B1 / Stage 4e T0 with pva): noisy frsim flights of theta_0 on the 4 tuning scenarios (flight seeds s * 10 + i for
s = 0, 1) on the true plant (S2: P1 = thrust x 0.8, lag 45 ms, downwash x 1.3, plume x 0.7; S1 / S3: P1RD = P1 + rpm delay 8 ms, the calibrated value is 4.0502 ms), fit of beta only with
J_sysId = position + velocity + attitude, 50 epochs, best iterate, Adam lr 0.01. Reports the fitted beta (factors), the truth, the error per value, and the parts p, v, q, w of the full J_sysId
at start and end. Controller from DIFFCOTUNE_CONTROLLER. Usage (nice -n 19): python3 identifiability5.py"""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L  # noqa: E402
sys.path.insert(0, str(HERE.parent / 'stage4e'))
from t0_identifiability import parts_mean  # noqa: E402

RD = L.N_BETA == 5; PLANT = L.PLANTS['P1RD' if RD else 'P1']


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        fl2 = [[], [], [], []]
        for seed in (0, 1):
            fl = L.fly_set(pool, np.zeros(10), PLANT, L.TUNING, [seed * 10 + i for i in range(4)])
            for i, f in enumerate(fl): fl2[i].append(f)
            say('flights seed', seed, 'real 3-D RMS %.2f cm, per flight %s' % (L.round_cost(fl)['rms3d_cm'], [(f['crashed'], f['floor']) for f in fl]))
        traces = {i: [f['trace'] for f in fl2[i]] for i in range(4)}; t0 = time.time()
        x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, traces), np.zeros(10 + L.N_BETA), L.PSI_FIT_IDX, 0.01, K2=50, stop=False, best=True, label='B2 ' + L.CTRL)
        tr_by = {i: [f['trace'] for f in fl2[i]] for i in range(4)}
        p0 = parts_mean(pool, np.zeros(10 + L.N_BETA), tr_by); p1 = parts_mean(pool, x, tr_by)
        fac = np.exp(x[10:]) * np.r_[np.exp(0) * np.ones(L.N_BETA)]; truth = np.exp(L.PSI_TRUE); err = 100 * (np.exp(x[10:]) / truth - 1)
        names = list(L.model().BETA_NAMES)
        out = dict(controller=L.CTRL, plant=('P1RD' if RD else 'P1'), beta_names=names, psi=x[10:].tolist(), factors_vs_calibration=np.exp(x[10:]).tolist(), truth_factors=truth.tolist(), err_pct=err.tolist(),
                   best_epoch=rec['best_epoch'], J0=rec['J0'], J_best=rec['J_best'], parts_start=p0, parts_end=p1, seconds=time.time() - t0, log=rec['log'])
        L.jdump(out, HERE / 'results' / ('identifiability_%s.json' % L.CTRL))
        say('%s beta error %% %s' % (L.CTRL, dict(zip(names, np.round(err, 1)))), 'best epoch', rec['best_epoch'], 'J %.4g -> %.4g' % (rec['J0'], rec['J_best']))
        say('parts start %s end %s' % ({k: round(v, 5) for k, v in p0.items()}, {k: round(v, 5) for k, v in p1.items()}))
        if RD:
            import config as _cfg; _rd0 = float(_cfg.CALIBRATED_PLANT.rpm_delay_ms)
            say('rpm delay fitted %.2f ms (truth 8.0, calibration %.2f ms)' % (_rd0 * np.exp(x[10 + 4]), _rd0))
    finally:
        pool.close()
