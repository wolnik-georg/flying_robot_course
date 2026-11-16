#!/usr/bin/env python3
"""Stage 5c step 1: reverse (compiled) against forward (jacfwd, eager) mode for the NOISY J_task gradient (mean over 2 noise seeds, as in the tuning step), per scenario, at theta_l and beta_l (psi_after_fit) of the Stage 5 dry run
of the controller (dryrun/stage5/results/loop_<C>.json), l = 0, 1, 3, with two noise-seed pairs: the loop's pair of round l (6000 + 2 l, 6001 + 2 l) and 7000 / 7001.
A1: all 6 points (l x pair). A8, CIRCLE, FIG8: 1 point each (l = 1, the loop pair). Gradient: all 10 gains + beta (14 / 15 numbers); the number reported is max_i |g_rev - g_fwd| / max_j |g_fwd| over the 10 gain components (the tuning gradient)
and over all components. Gate: <= 1e-6 per scenario. One 4-worker pool, one task per worker at a time, A1 tasks first. Controller from DIFFCOTUNE_CONTROLLER."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
CTRL = L.CTRL


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def task(a):
    kind, si, x, seeds = a; s, c = L.TUNING[si]; t0 = time.time()
    r = L.worker_rollout((kind, s, c, x, None, L.ALL_STATES, seeds)); return si, kind, seeds, r[0], r[1], time.time() - t0


if __name__ == '__main__':
    import multiprocessing as mp
    d = json.load(open(HERE.parent / 'stage5' / 'results' / ('loop_%s.json' % CTRL))); pts = []
    for l in (0, 1, 3):
        r = d['rounds'][l]; x = np.r_[r['phi'], r['psi_after_fit']]
        for seeds in ((6000 + 2 * l, 6001 + 2 * l), (7000, 7001)):
            pts.append((0, l, seeds, x))
    for si in (1, 2, 3): pts.append((si, 1, (6002, 6003), np.r_[d['rounds'][1]['phi'], d['rounds'][1]['psi_after_fit']]))
    jobs = []
    for si, l, seeds, x in pts:
        for kind in ('task_F', 'task_R'): jobs.append((kind, si, x, seeds))
    jobs.sort(key=lambda j: (j[0] != 'task_F', j[1]))           # forward jobs (slow) first
    say('%d tasks' % len(jobs))
    with mp.get_context('spawn').Pool(4, initializer=L._w_init) as p: res = p.map(task, jobs, chunksize=1)
    by = {}
    for si, kind, seeds, J, g, sec in res: by[(si, tuple(seeds), tuple(np.round(g, 12)) if False else None, kind)] = None
    table = []; got = {}
    for (kind, si, x, seeds), (_, _, _, J, g, sec) in zip(jobs, res): got[(kind, si, tuple(seeds), tuple(np.round(x, 10)))] = (J, g, sec)
    for si, l, seeds, x in pts:
        Jf, gf, sf = got[('task_F', si, tuple(seeds), tuple(np.round(x, 10)))]; Jr, gr, sr = got[('task_R', si, tuple(seeds), tuple(np.round(x, 10)))]
        rel10 = float(np.abs(gr[:10] - gf[:10]).max() / np.abs(gf[:10]).max()); relall = float(np.abs(gr - gf).max() / np.abs(gf).max()); relJ = abs(Jr - Jf) / abs(Jf)
        comp = [float(abs(a - b) / abs(b)) if b != 0 else None for a, b in zip(gr[:10], gf[:10])]
        table.append(dict(scenario=L.TUNING[si][0], round_l=l, seeds=list(seeds), J_fwd=Jf, rel_J=relJ, rel_diff_gains=rel10, rel_diff_all=relall, max_component_rel=max([c for c in comp if c is not None] + [0]), passed=bool(rel10 <= 1e-6 and relall <= 1e-6), fwd_s=sf, rev_s=sr, g_fwd=gf.tolist(), g_rev=gr.tolist()))
        print('%-6s l=%d seeds %s  rel diff gains %.2e all %.2e  J %.2e  %s   (fwd %.0f s, rev %.0f s)' % (L.TUNING[si][0], l, seeds, rel10, relall, relJ, 'ok' if table[-1]['passed'] else 'FAIL', sf, sr), flush=True)
    ok = all(t['passed'] for t in table); out = dict(controller=CTRL, passed=ok, table=table)
    for s in ('A1', 'A8', 'CIRCLE', 'FIG8'): out['passed_' + s] = all(t['passed'] for t in table if t['scenario'] == s)
    json.dump(out, open(HERE / 'results' / ('step1_%s.json' % CTRL), 'w'), indent=1); say('STEP 1 %s: %s' % (CTRL, 'PASS' if ok else 'FAIL; per scenario %s' % {s: out['passed_' + s] for s in ('A1', 'A8', 'CIRCLE', 'FIG8')}))
