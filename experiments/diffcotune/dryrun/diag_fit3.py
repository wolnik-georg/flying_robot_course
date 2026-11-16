#!/usr/bin/env python3
"""D3 (diagnostic, not the method): beta fit on NOISY P1 flights of theta0 (one flight per tuning scenario, as in round 0), Adam lr 0.03, 100 epochs, no stop
rule, J_sysId as specified (unweighted sum of the parts). Also D3b: the same with the body-rate part left out of the cost (to show the effect of the noise floor)
-- D3b is NOT a proposal of the spec, only a diagnostic. Usage: python3 dryrun/diag_fit3.py"""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    pool = L.Pool(4)
    try:
        fl = L.fly_set(pool, np.zeros(10), L.P1, L.TUNING, [0, 1, 2, 3]); traces = {i: [f['trace']] for i, f in enumerate(fl)}
        say('D3 noisy P1 flights of theta0, real 3-D RMS %.2f cm' % L.round_cost(fl)['rms3d_cm'])
        x, rec = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, traces), np.zeros(14), np.arange(10, 14), 0.03, K2=100, label='D3', stop=False)
        say('D3 (noisy, no stop rule) epochs', rec['epochs'], 'psi', np.round(x[10:], 3), 'truth', np.round(L.PSI_TRUE, 3), 'Jn end %.3f' % rec['log'][-1]['Jn'])
        L.jdump(dict(D3=rec), HERE / 'results' / 'diag_fit_D3.json')
    finally:
        pool.close()
