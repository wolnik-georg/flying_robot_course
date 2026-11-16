#!/usr/bin/env python3
"""Stage 5c step 2 / 3 check: the compiled forward sensitivity (fsens.py, kinds sysid_C / task_C) against the eager forward gradient of Stage 5b (stage5b/results/fwd_checks_<C>.json, theta = 0, beta = calibration, all
parameters, J_task and J_sysId per scenario) and against reverse mode on A8 / CIRCLE / FIG8; the 'mixed' gradient of the fit (beta only on A1) against the same eager forward gradient; seconds per call in the 4-worker pool.
Gate: max |difference| / max |eager forward| <= 1e-8."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5'))
import loop_lib as L  # noqa
import checks5 as C5  # noqa
CTRL = L.CTRL; n = 10 + L.N_BETA; RD = L.N_BETA == 5
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)
def per_scen(pool, kind, x, traces=None):
    t0 = time.time(); res = pool.pool.map(L.worker_rollout, [(kind, s, c, x, traces[i] if traces else None, pool.states, pool.noise_seeds) for i, (s, c) in enumerate(L.TUNING)], chunksize=1)
    return [r[0] for r in res], [r[1] for r in res], time.time() - t0
if __name__ == '__main__':
    ref = json.load(open(HERE.parent / 'stage5b' / 'results' / ('fwd_checks_%s.json' % CTRL)))['b_a']; plant = L.PLANTS['P1RD' if RD else 'P1']; out = dict(controller=CTRL)
    pool = L.Pool(4, states=('p', 'v', 'q')); x0 = np.zeros(n)
    try:
        traces = {i: [C5.fr((s, c, L.gains_of(np.zeros(10)), plant['overrides'], 0, False))] for i, (s, c) in enumerate(L.TUNING)}
        if L.GRAD_MODE == 'mixed':          # the mixed gradient of the fit as the loop calls it (kind 'sysid', GRAD_MODE mixed from the environment of the job)
            gE = np.array(ref['sysid']['g_fwd']); Jm, gm, t1 = per_scen(pool, 'sysid', x0, traces); _, _, t2 = per_scen(pool, 'sysid', x0, traces)
            rel = [float(np.abs(gm[i][10:] - gE[i][10:]).max() / np.abs(gE[i][10:]).max()) for i in range(4)]
            say('MIXED sysid, beta components vs eager forward (Stage 5b), A1 A8 CIRCLE FIG8:', ['%.2e' % r for r in rel], 'gain components of A1 are zero by construction:', float(np.abs(gm[0][:10]).max()), 'seconds first %.1f second %.1f' % (t1, t2))
            out['mixed'] = dict(rel_beta=rel, seconds_first=t1, seconds=t2, passed=bool(max(rel) <= 1e-8)); json.dump(out, open(HERE / 'results' / ('fsens_mixed_%s.json' % CTRL), 'w'), indent=1); sys.exit(0)
        for kind in ('sysid', 'task'):
            tr = traces if kind == 'sysid' else None; gE = np.array(ref[kind]['g_fwd'])
            Jc, gc, t_c = per_scen(pool, kind + '_C', x0, tr); _ = per_scen(pool, kind + '_C', x0, tr); Jc, gc, t_c2 = per_scen(pool, kind + '_C', x0, tr)
            Jr, gr, t_r = per_scen(pool, kind + '_R', x0, tr)
            rel = [float(np.abs(gc[i] - gE[i]).max() / np.abs(gE[i]).max()) for i in range(4)]; relr = [float(np.abs(gr[i] - gE[i]).max() / np.abs(gE[i]).max()) for i in range(4)]
            say(kind, 'compiled forward vs eager forward (Stage 5b), per scenario A1 A8 CIRCLE FIG8:', ['%.2e' % r for r in rel], ' reverse vs eager forward:', ['%.2e' % r for r in relr], 'seconds compiled-forward first/second call %.1f / %.1f, reverse %.1f' % (t_c, t_c2, t_r))
            out[kind] = dict(rel_compiled_vs_eager_forward=rel, rel_reverse_vs_eager_forward=relr, seconds_first=t_c, seconds=t_c2, seconds_reverse=t_r, passed=bool(max(rel) <= 1e-8), J_diff=[abs(a - b) for a, b in zip(Jc, ref[kind]['J'])])
    finally:
        pool.close()
    json.dump(out, open(HERE / 'results' / ('fsens_check_%s.json' % CTRL), 'w'), indent=1)
