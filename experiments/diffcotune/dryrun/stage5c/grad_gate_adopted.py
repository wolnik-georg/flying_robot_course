#!/usr/bin/env python3
"""Stage 5c step 5: the Stage 5 gradient gate with the ADOPTED gradient modes (combined over A1, A8, CIRCLE, FIG8, theta = 0, beta = calibration, all gains and all beta):
  J_sysId (the fit): mixed = A1 by the compiled forward sensitivity (all parameters, kind sysid_C), A8 / CIRCLE / FIG8 by compiled reverse; against central finite differences (h = 1e-6, float64, compiled primal).
  J_task noise-free (information): compiled forward sensitivity (task_C) on all four scenarios against finite differences; reverse mode listed for the record (A1 fails noise-free, known).
  J_task with noise (the tuning step as the loop runs it, 2 seeds 6000 / 6001), compiled reverse against finite differences, 10 gains, h = 1e-7 (the finite difference with noise needs a small h, Stage 5b h-scan).
Gate: same sign and (relative <= 1 % or absolute <= 0.1 % of the largest component)."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5')); sys.path.insert(0, str(HERE.parent / 'stage5b'))
import loop_lib as L  # noqa
import checks5 as C5  # noqa
import fwd_checks as F  # noqa
CTRL = L.CTRL; n = 10 + L.N_BETA; RD = L.N_BETA == 5
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
if __name__ == '__main__':
    plant = L.PLANTS['P1RD' if RD else 'P1']; out = dict(controller=CTRL); x0 = np.zeros(n); idx = list(range(n))
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        traces = {i: [C5.fr((s, c, L.gains_of(np.zeros(10)), plant['overrides'], 0, False))] for i, (s, c) in enumerate(L.TUNING)}
        # J_sysId, adopted (mixed)
        jobs = [(('sysid_C' if s == 'A1' else 'sysid_R'), s, c, x0, traces[i], pool.states, None) for i, (s, c) in enumerate(L.TUNING)]
        g = np.mean([r[1] for r in pool.pool.map(L.worker_rollout, jobs, chunksize=1)], 0); fd = F.fd_of(pool, 'sysid', x0, idx, traces); rows = F.judge(g, fd, F.NAMES); ok = all(r['passed'] for r in rows)
        out['J_sysId_mixed'] = dict(passed=ok, rows=rows); say('J_sysId (mixed): %s' % ('PASS' if ok else 'FAIL'))
        for r in rows: print('   %-15s mixed %+.6e fd %+.6e rel %-9s %s' % (r['param'], r['g'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', '' if r['passed'] else '<-- FAIL'))
        # J_task noise-free, forward and reverse
        for name, kind in (('J_task_forward_C', 'task_C'), ('J_task_reverse', 'task_R')):
            jobs = [(kind, s, c, x0, None, pool.states, None) for s, c in L.TUNING]
            g = np.mean([r[1] for r in pool.pool.map(L.worker_rollout, jobs, chunksize=1)], 0)
            if name.endswith('C'): fdt = F.fd_of(pool, 'task', x0, idx, None)
            rows = F.judge(g, fdt, F.NAMES); out[name] = dict(passed=all(r['passed'] for r in rows), rows=rows); say('%s: %s' % (name, 'PASS' if out[name]['passed'] else 'FAIL (information)'))
    finally:
        pool.close()
    pool = L.Pool(4, noise_seeds=(6000, 6001))
    try:
        xn = np.r_[np.zeros(10), L.PSI_TRUE]; jobs = [('task_R', s, c, xn, None, pool.states, pool.noise_seeds) for s, c in L.TUNING]
        g = np.mean([r[1] for r in pool.pool.map(L.worker_rollout, jobs, chunksize=1)], 0)[:10]
        fdn = F.fd_of(pool, 'task', xn, list(range(10)), None, 1e-7); rows = F.judge(g, fdn, F.NAMES[:10]); out['J_task_noise_reverse'] = dict(passed=all(r['passed'] for r in rows), rows=rows)
        say('J_task with noise (reverse, h 1e-7): %s' % ('PASS' if out['J_task_noise_reverse']['passed'] else 'FAIL'))
        for r in rows: print('   %-8s rev %+.6e fd %+.6e rel %-9s %s' % (r['param'], r['g'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', '' if r['passed'] else '<-- FAIL'))
    finally:
        pool.close()
    out['passed'] = bool(out['J_sysId_mixed']['passed'] and out['J_task_noise_reverse']['passed'] and out['J_task_forward_C']['passed'])
    json.dump(out, open(HERE / 'results' / ('grad_gate_%s.json' % CTRL), 'w'), indent=1); say('ADOPTED-MODE GRADIENT GATE %s: %s' % (CTRL, 'PASS' if out['passed'] else 'FAIL'))
