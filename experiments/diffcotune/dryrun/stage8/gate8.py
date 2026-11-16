#!/usr/bin/env python3
"""Stage 8 (copy of the Stage 7 gate without the noisy parts): the paper-faithful costs (DIFFCOTUNE_METHOD=paper: J_task = unweighted full state, J_sysId = p+v+q+w), noise-free, per scenario A1 A8 CIRCLE FIG8 A4 cell 1, adopted gradient (cfwd) against central FD at theta_0 / calibration beta. (Stage 7 text follows)\nStage 7: the gradient gate of the ADOPTED mode `cfwd` (compiled forward sensitivity for every gradient, every scenario; user decision 15 Nov), for the tuning set of the controller (A1, A8, CIRCLE, FIG8, A4 cell 1), per scenario and combined.
 1. J_sysId (the beta fit) at theta_0 / calibration beta against central finite differences (frsim default-gain flights of the dry-run plant as the real traces; the Stage 5 gate).
 2. J_task without noise, adopted gradient against finite differences (Stage 5 gate).
 3. J_task WITH noise (the tuning step: seeds 6000 / 6001) at theta_0 / true beta, adopted gradient against finite differences, h 1e-8 (h 1e-7 listed: for S2 the finite difference with noise is not converged above 1e-8, Stage 5c).
 4. GUARD at the fitted beta of every round: for each round l of a dry-run loop json (argument; the S3 rdfix run for the gate of the user decision): per scenario, J_task with the noise seeds of round l and J_sysId, the adopted gradient against a directional finite difference
    (h 1e-8 along the gradient direction, relative <= 1e-3) and, for information, compiled REVERSE mode against the adopted one (a relative difference > 1e-6 marks reverse mode as unreliable at that point).
Usage (job_runner): DIFFCOTUNE_CONTROLLER=<C> DIFFCOTUNE_METHOD=paper DIFFCOTUNE_GRAD_MODE=cfwd python3 gate8.py -> results/grad_gate7_<C>.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "stage5")); sys.path.insert(0, str(HERE.parent / 'stage5b'))
import loop_lib as L  # noqa
import checks5 as C5  # noqa
import fwd_checks as F  # noqa
CTRL = L.CTRL; n = 10 + L.N_BETA; RD = L.N_BETA == 5; NT = len(L.TUNING)
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
def per_scen(pool, kind, x, traces, noise):
    res = pool.pool.map(L.worker_rollout, [(kind, s, c, x, (traces or {}).get(i), pool.states, noise) for i, (s, c) in enumerate(L.TUNING)], chunksize=1)
    return np.array([r[0] for r in res]), (None if res[0][1] is None else np.array([r[1] for r in res]))
def dir_fd(pool, kind, x, d, traces, noise, h=1e-8):
    Jp, _ = per_scen(pool, kind + '_fwd', x + h * d, traces, noise); Jm, _ = per_scen(pool, kind + '_fwd', x - h * d, traces, noise); return (Jp - Jm) / (2 * h)
if __name__ == '__main__':
    plant = L.PLANTS['P1RD' if RD else 'P1']; out = dict(controller=CTRL, tuning=[s for s, _ in L.TUNING], grad_mode=L.GRAD_MODE); x0 = np.zeros(n); idx = list(range(n))
    pool = L.Pool(NT, states=L.METHOD_SETTINGS[L.METHOD]['states'])
    try:
        traces = {i: [C5.fr((s, c, L.gains_of(np.zeros(10)), plant['overrides'], 0, False))] for i, (s, c) in enumerate(L.TUNING)}
        g = np.mean(per_scen(pool, 'sysid', x0, traces, None)[1], 0); fd = F.fd_of(pool, 'sysid', x0, idx, traces); rows = F.judge(g, fd, F.NAMES); out['J_sysId'] = dict(passed=all(r['passed'] for r in rows), rows=rows)
        say('1 J_sysId (adopted): %s' % ('PASS' if out['J_sysId']['passed'] else 'FAIL'))
        g = np.mean(per_scen(pool, 'task', x0, None, None)[1], 0); fd = F.fd_of(pool, 'task', x0, idx, None); rows = F.judge(g, fd, F.NAMES); out['J_task_noisefree'] = dict(passed=all(r['passed'] for r in rows), rows=rows); say('2 J_task noise-free (adopted): %s' % ('PASS' if out['J_task_noisefree']['passed'] else 'FAIL'))
    finally:
        pool.close()
    out['guard_fitted_beta'] = []
    out['passed'] = bool(out['J_sysId']['passed'] and out['J_task_noisefree']['passed'])
    json.dump(out, open(HERE / 'results' / ('grad_gate8_%s_%s.json' % (CTRL, L.METHOD)), 'w'), indent=1); say('GRADIENT GATE (cfwd) %s: %s' % (CTRL, 'PASS' if out['passed'] else 'FAIL'))
