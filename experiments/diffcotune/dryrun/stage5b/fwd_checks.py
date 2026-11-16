#!/usr/bin/env python3
"""Stage 5b step 2: correctness and timing of the forward-mode gradient (loop_lib kinds 'task_F' / 'sysid_F', torch.func.jacfwd, eager) against reverse mode ('_R', compiled) and central finite differences.
 (b) the combined gradient gate of Stage 3 / checks5.py (all 10 gains and all beta, J_task and J_sysId = p+v+q, mean over A1, A8, CIRCLE, FIG8, theta = 0, beta = calibration, J_sysId against a noise-off frsim flight
     of the default gains on the dry-run plant (P1 for S0 / S2, P1RD for S1 / S3)): forward vs FD (h = 1e-6, compiled primal, 2 rows each) -> gate (same sign and relative <= 1 % or absolute <= 0.1 % of the largest component).
 (a) per scenario forward vs reverse (A8, CIRCLE, FIG8 are where reverse passes; A1 is listed too).
 (c) with noise (seeds 1000 / 1001, the mean over the two rows, as the Stage 3d noisy check): J_task, 10 gains, theta = 0, beta = plant truth; forward vs FD, and reverse.
 Timing: wall seconds of one 4-worker call (one scenario per worker) forward vs reverse (reverse after a warm-up call).
 usage (env DIFFCOTUNE_CONTROLLER): fwd_checks.py"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'stage5'))
import loop_lib as L  # noqa: E402
import checks5 as C5  # noqa: E402
CTRL = L.CTRL; NB = L.N_BETA; RD = NB == 5; n = 10 + NB
NAMES = list(L.GAIN_NAMES) + list(L.model().BETA_NAMES) + ([] if RD else [])
NAMES = list(L.GAIN_NAMES) + list(L.model().BETA_NAMES)
def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def per_scen(pool, kind, x, traces=None):
    t0 = time.time()
    res = pool.pool.map(L.worker_rollout, [(kind, s, c, x, traces[i] if traces else None, pool.states, pool.noise_seeds) for i, (s, c) in enumerate(L.TUNING)], chunksize=1)
    return [r[0] for r in res], [r[1] for r in res], time.time() - t0


def fd_of(pool, kind, x, idx, traces=None, h=1e-6):
    out = np.zeros(len(idx))
    for k, i in enumerate(idx):
        xp = x.copy(); xm = x.copy(); xp[i] += h; xm[i] -= h
        out[k] = (pool.rollouts(kind + '_fwd', xp, traces)[0] - pool.rollouts(kind + '_fwd', xm, traces)[0]) / (2 * h)
    return out


def judge(g, f, names):
    big = max(np.abs(f).max(), np.abs(g).max()); rows = []
    for i, nm in enumerate(names):
        err = abs(g[i] - f[i]); fin = np.isfinite(g[i]) and np.isfinite(f[i]); rel = float(err / abs(f[i])) if fin and f[i] != 0 else None
        ok = bool(fin and (np.sign(g[i]) == np.sign(f[i]) or max(abs(g[i]), abs(f[i])) < 1e-9 * big) and ((rel is not None and rel <= 0.01) or err <= 0.001 * big))
        rows.append(dict(param=nm, g=float(g[i]), fd=float(f[i]), rel_err=rel, abs_over_max=float(err / big), passed=ok))
    return rows


if __name__ == '__main__':
    out = dict(controller=CTRL); plant = L.PLANTS['P1RD' if RD else 'P1']
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        say('flights for J_sysId (noise off)')
        traces = {i: [C5.fr((s, c, L.gains_of(np.zeros(10)), plant['overrides'], 0, False))] for i, (s, c) in enumerate(L.TUNING)}
        x0 = np.zeros(n); idx = list(range(n)); res = {}; tim = {}
        for kind in ('task', 'sysid'):
            tr = traces if kind == 'sysid' else None
            _ = per_scen(pool, kind + '_R', x0, tr); Jr, gr, tr_t = per_scen(pool, kind + '_R', x0, tr); say(kind, 'reverse (warm) %.1f s' % tr_t)
            Jf, gf, tf_t = per_scen(pool, kind + '_F', x0, tr); say(kind, 'forward %.1f s' % tf_t)
            fd = fd_of(pool, kind, x0, idx, tr); say(kind, 'FD done')
            res[kind] = dict(J=Jf, J_rev=Jr, g_fwd=[g.tolist() for g in gf], g_rev=[g.tolist() for g in gr], fd_combined=fd.tolist()); tim[kind] = dict(forward_s=tf_t, reverse_s=tr_t)
            gate = judge(np.mean(gf, 0), fd, NAMES); res[kind]['gate_rows'] = gate; res[kind]['gate_passed'] = all(r['passed'] for r in gate)
            print('== COMBINED %s forward vs FD: %s' % (kind, 'PASS' if res[kind]['gate_passed'] else 'FAIL'))
            for r in gate: print('   %-15s fwd %+.6e fd %+.6e rel %-9s abs/max %.1e %s' % (r['param'], r['g'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', r['abs_over_max'], '' if r['passed'] else '<-- FAIL'))
            for i, (s, _c) in enumerate(L.TUNING):
                a, b = np.array(gf[i]), np.array(gr[i]); big = max(np.abs(a).max(), 1e-300)
                print('  (a) %-6s %-5s max|fwd-rev|/max|fwd| %.3e   max|rev| %.3e' % (s, kind, np.abs(a - b).max() / big, np.abs(b).max()))
        out['b_a'] = res; out['timing_4workers'] = tim; out['b_passed'] = bool(res['task']['gate_passed'] and res['sysid']['gate_passed'])
        say('(b) COMBINED GATE forward %s: %s' % (CTRL, 'PASS' if out['b_passed'] else 'FAIL'))
    finally:
        pool.close()
    json.dump(out, open(HERE / 'results' / ('fwd_checks_%s.json' % CTRL), 'w'), indent=1)
    if not out['b_passed']: sys.exit(1)
    pool = L.Pool(4, noise_seeds=(1000, 1001))
    try:
        xn = np.r_[np.zeros(10), L.PSI_TRUE]; _ = per_scen(pool, 'task_R', xn); Jr, gr, tr_t = per_scen(pool, 'task_R', xn); Jf, gf, tf_t = per_scen(pool, 'task_F', xn)
        fd = fd_of(pool, 'task', xn, list(range(10)), None, 1e-5); g = np.mean(gf, 0)[:10]; rows = judge(g, fd, NAMES[:10]); gr_m = np.mean(gr, 0)[:10]
        out['c_noise'] = dict(rows=rows, passed=all(r['passed'] for r in rows), g_rev=gr_m.tolist(), forward_s=tf_t, reverse_s=tr_t, g_fwd_scen=[x.tolist() for x in gf], g_rev_scen=[x.tolist() for x in gr])
        print('== (c) NOISE forward vs FD (h 1e-5): %s   forward %.1f s reverse %.1f s' % ('PASS' if out['c_noise']['passed'] else 'FAIL', tf_t, tr_t))
        for r, rv in zip(rows, gr_m): print('   %-8s fwd %+.6e fd %+.6e rel %-9s abs/max %.1e reverse %+.3e %s' % (r['param'], r['g'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', r['abs_over_max'], rv, '' if r['passed'] else '<-- FAIL'))
    finally:
        pool.close()
    json.dump(out, open(HERE / 'results' / ('fwd_checks_%s.json' % CTRL), 'w'), indent=1)
