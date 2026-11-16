#!/usr/bin/env python3
"""Stage 7: the gradient gate of the ADOPTED mode `cfwd` (compiled forward sensitivity for every gradient, every scenario; user decision 15 Nov), for the tuning set of the controller (A1, A8, CIRCLE, FIG8, A4 cell 1), per scenario and combined.
 1. J_sysId (the beta fit) at theta_0 / calibration beta against central finite differences (frsim default-gain flights of the dry-run plant as the real traces; the Stage 5 gate).
 2. J_task without noise, adopted gradient against finite differences (Stage 5 gate).
 3. J_task WITH noise (the tuning step: seeds 6000 / 6001) at theta_0 / true beta, adopted gradient against finite differences, h 1e-8 (h 1e-7 listed: for S2 the finite difference with noise is not converged above 1e-8, Stage 5c).
 4. GUARD at the fitted beta of every round: for each round l of a dry-run loop json (argument; the S3 rdfix run for the gate of the user decision): per scenario, J_task with the noise seeds of round l and J_sysId, the adopted gradient against a directional finite difference
    (h 1e-8 along the gradient direction, relative <= 1e-3) and, for information, compiled REVERSE mode against the adopted one (a relative difference > 1e-6 marks reverse mode as unreliable at that point).
Usage (job_runner): DIFFCOTUNE_CONTROLLER=<C> DIFFCOTUNE_GRAD_MODE=cfwd [DIFFCOTUNE_FIX_RD=1] python3 grad_gate7.py <loop json> [<loop json> ...] -> results/grad_gate7_<C>.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5')); sys.path.insert(0, str(HERE.parent / 'stage5b'))
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
    pool = L.Pool(NT, states=('p', 'v', 'q'))
    try:
        traces = {i: [C5.fr((s, c, L.gains_of(np.zeros(10)), plant['overrides'], 0, False))] for i, (s, c) in enumerate(L.TUNING)}
        g = np.mean(per_scen(pool, 'sysid', x0, traces, None)[1], 0); fd = F.fd_of(pool, 'sysid', x0, idx, traces); rows = F.judge(g, fd, F.NAMES); out['J_sysId'] = dict(passed=all(r['passed'] for r in rows), rows=rows)
        say('1 J_sysId (adopted): %s' % ('PASS' if out['J_sysId']['passed'] else 'FAIL'))
        g = np.mean(per_scen(pool, 'task', x0, None, None)[1], 0); fd = F.fd_of(pool, 'task', x0, idx, None); rows = F.judge(g, fd, F.NAMES); out['J_task_noisefree'] = dict(passed=all(r['passed'] for r in rows), rows=rows); say('2 J_task noise-free (adopted): %s' % ('PASS' if out['J_task_noisefree']['passed'] else 'FAIL'))
    finally:
        pool.close()
    pool = L.Pool(NT, noise_seeds=(6000, 6001))
    try:
        xn = np.r_[np.zeros(10), L.PSI_TRUE]; g = np.mean(per_scen(pool, 'task', xn, None, pool.noise_seeds)[1], 0)[:10]
        for hh, key in ((1e-8, 'J_task_noise'), (1e-7, 'J_task_noise_h1e-7')):
            fdn = F.fd_of(pool, 'task', xn, list(range(10)), None, hh); rows = F.judge(g, fdn, F.NAMES[:10]); out[key] = dict(passed=all(r['passed'] for r in rows), rows=rows, h=hh)
            say('3 J_task with noise (adopted, h %g): %s' % (hh, 'PASS' if out[key]['passed'] else 'FAIL' + ('' if hh == 1e-8 else ' (information)')))
        guard = []
        for lj in sys.argv[1:]:          # one or more dry-run loop jsons (the earlier run and the new run of the controller)
            rec = json.load(open(lj))
            for l, r in enumerate(rec['rounds']):
                try: x0r = np.r_[np.array(r['phi']), np.array(r['psi_after_fit'])]
                except KeyError: continue
                if len(x0r) != n: continue
                NS = (6000 + 2 * l, 6001 + 2 * l); ent = dict(source=lj, round=l, task=[], sysid=[])
                for kind, noise in (('task', NS), ('sysid', None)):
                    Ja, Ga = per_scen(pool, kind, x0r, traces if kind == 'sysid' else None, noise); _, Gr = per_scen(pool, kind + '_R', x0r, traces if kind == 'sysid' else None, noise)
                    sl = slice(0, 10) if kind == 'task' else slice(10, n)
                    for i, (s, c) in enumerate(L.TUNING):
                        ga = Ga[i][sl]; d = np.zeros(n); d[sl] = ga / (np.linalg.norm(ga) + 1e-300); fdd = dir_fd(pool, kind, x0r, d, traces if kind == 'sysid' else None, noise)[i]
                        rel_fd = abs(float(Ga[i] @ d) - fdd) / (abs(fdd) + 1e-300); rev = float(np.linalg.norm(Gr[i][sl] - ga) / (np.linalg.norm(ga) + 1e-300))
                        e = dict(scenario=s, norm=float(np.linalg.norm(ga)), adopted_vs_fd_rel=float(rel_fd), reverse_vs_adopted_rel=rev, reverse_unreliable=bool(rev > 1e-6), passed=bool(rel_fd <= 1e-3)); ent[kind].append(e)
                        say('4 guard round %d %-6s %-6s: adopted vs FD %.1e%s ; reverse vs adopted %.1e%s' % (l, kind, s, rel_fd, '' if e['passed'] else ' FAIL', rev, ' (reverse unreliable)' if e['reverse_unreliable'] else ''))
                ent['passed'] = bool(all(e['passed'] for e in ent['task'] + ent['sysid'])); guard.append(ent)
        out['guard_fitted_beta'] = guard
    finally:
        pool.close()
    out['passed'] = bool(out['J_sysId']['passed'] and out['J_task_noisefree']['passed'] and out['J_task_noise']['passed'] and all(e['passed'] for e in out['guard_fitted_beta']))
    json.dump(out, open(HERE / 'results' / ('grad_gate7_%s.json' % CTRL), 'w'), indent=1); say('GRADIENT GATE (cfwd) %s: %s' % (CTRL, 'PASS' if out['passed'] else 'FAIL'))
