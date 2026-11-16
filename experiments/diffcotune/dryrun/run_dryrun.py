#!/usr/bin/env python3
"""Stage 3 S0 dry run. Usage (system python3, nice -n 19):
  python3 dryrun/run_dryrun.py pilot                      # B1: 3 learning rates, P1, 2 rounds each
  python3 dryrun/run_dryrun.py full --lr 0.03 [--plants P1 P2]   # B2 + B3: DiffCoTune, tune only, sysId then tune, final evaluation
Results (JSON, one per stage) in dryrun/results/, logs in dryrun/logs/."""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402

RES = HERE / 'results'      # --out changes it (Stage 3b writes to dryrun/stage3b/results)


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def run_sysid_then_tune(pool, plant, lr, n=6, stop=True, best=False):
    """(c) paper baseline: fit beta once on all flights of theta_0 (6 flights per tuning scenario, noise seeds of rounds 0 ... 5), then tune theta
    in the fitted model with the same epoch budget as the loop (6 inner loops of at most 50 epochs, Adam restarted each time)."""
    x = np.zeros(14); rec = dict(plant=plant, method='sysid_then_tune', lr=lr)
    flights = [L.fly_set(pool, x[:10], plant, L.TUNING, [10 * l + i for i in range(len(L.TUNING))]) for l in range(n)]
    traces = {i: [flights[l][i]['trace'] for l in range(n)] for i in range(len(L.TUNING))}
    rec['theta0_real'] = [L.round_cost(f) for f in flights]
    fits = []; t0 = time.time()
    for k in range(n):
        x, rf = L.adam_inner(lambda xx: pool.rollouts('sysid', xx, traces), x, np.arange(10, 14), lr, label='fit beta (once, 24 flights), loop %d' % k, stop=stop, best=best)
        fits.append(rf); say('  (c) fit loop', k, rf['epochs'], rf['reason'], 'psi err', np.round(x[10:] - L.PSI_TRUE, 3).tolist())
    rec['fit'] = fits; rec['psi'] = x[10:].tolist(); rec['seconds_fit'] = time.time() - t0
    tunes = []; t0 = time.time()
    for k in range(n):
        x, rt = L.adam_inner(lambda xx: pool.rollouts('task', xx, None), x, np.arange(10), lr, label='tune theta, loop %d' % k, stop=stop, best=best)
        tunes.append(rt); say('  (c) tune loop', k, rt['epochs'], rt['reason'], 'Jn', round(rt['log'][-1]['Jn'], 3))
    rec['tune'] = tunes; rec['phi_final'] = x[:10].tolist(); rec['seconds_tune'] = time.time() - t0
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['pilot', 'full', 'eval'])
    ap.add_argument('--lr', type=float, default=0.03)
    ap.add_argument('--plants', nargs='+', default=['P1', 'P2'])
    ap.add_argument('--lrs', type=float, nargs='+', default=[0.01, 0.03, 0.1])
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--stop-rule', choices=['normalised', 'full50best'], default='normalised',
                    help='normalised = Stage 3 (early stop, last/triggering iterate); full50best = Stage 3b (no early stop, 50 epochs, best iterate)')
    ap.add_argument('--states', choices=['pvqw', 'pva', 'pv'], default='pvqw', help='J_sysId state set: pvqw = Stage 3 (all four parts), pva = p+v+attitude, pv')
    ap.add_argument('--skip-last-fit', action='store_true', help='leave out the unused beta fit of the last round (default: run it, as Stage 3 / 3b)')
    ap.add_argument('--out', default=None, help='result folder (default dryrun/results)')
    a = ap.parse_args()
    global RES
    if a.out:
        RES = Path(a.out).resolve(); RES.mkdir(parents=True, exist_ok=True)
    stop, best = (a.stop_rule == 'normalised'), (a.stop_rule == 'full50best')
    pool = L.Pool(a.workers, states=tuple(a.states.replace('a', 'q')))
    try:
        if a.mode == 'pilot':
            for lr in a.lrs:
                rec = L.run_loop(pool, L.PLANTS['P1'], 'cotune', lr, n_rounds=2, extra_final_flight=True, log=say, stop=stop, best=best)
                L.jdump(rec, RES / ('pilot_P1_lr%g.json' % lr))
        elif a.mode == 'eval':          # evaluation of saved results (final theta of each method, 5 noise seeds on every tuning and test cell)
            import json
            for pn in a.plants:
                plant = L.PLANTS[pn]
                co, to, st = [json.load(open(RES / ('full_%s_%s.json' % (pn, m)))) for m in ('cotune', 'tune_only', 'sysid_then_tune')]
                phis = {'default': np.zeros(10), 'cotune': np.array(co['best']['phi']), 'tune_only': np.array(to['best']['phi']), 'sysid_then_tune': np.array(st['phi_final'])}
                ev = {k: L.evaluate(pool, p, plant) for k, p in phis.items()}
                L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, eval=ev), RES / ('full_%s_eval.json' % pn)); say(pn, 'evaluated')
        else:
            for pn in a.plants:
                plant = L.PLANTS[pn]; out = {}
                rec = L.run_loop(pool, plant, 'cotune', a.lr, n_rounds=6, checks_rounds=(0, 2, 4), log=say, stop=stop, best=best, skip_last_fit=a.skip_last_fit)
                L.jdump(rec, RES / ('full_%s_cotune.json' % pn)); out['cotune'] = rec
                rec = L.run_loop(pool, plant, 'tune_only', a.lr, n_rounds=6, log=say, stop=stop, best=best)
                L.jdump(rec, RES / ('full_%s_tune_only.json' % pn)); out['tune_only'] = rec
                rec = run_sysid_then_tune(pool, plant, a.lr, stop=stop, best=best)
                L.jdump(rec, RES / ('full_%s_sysid_then_tune.json' % pn)); out['sysid_then_tune'] = rec
                phis = {'default': np.zeros(10), 'cotune': np.array(out['cotune']['best']['phi']), 'tune_only': np.array(out['tune_only']['best']['phi']),
                        'sysid_then_tune': np.array(out['sysid_then_tune']['phi_final'])}
                ev = {k: L.evaluate(pool, p, plant) for k, p in phis.items()}
                L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, eval=ev), RES / ('full_%s_eval.json' % pn))
                say(pn, 'done')
    finally:
        pool.close()


if __name__ == '__main__':
    main()
