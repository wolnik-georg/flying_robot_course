#!/usr/bin/env python3
"""Stage 5 B3: dry run of one controller, as Stage 3e B1 but 5 rounds (l = 0 ... 4): method version frozen on 13 Nov (50 epochs, best iterate, J_sysId = p + v + q, noise in the tuning step with 2 seeds
per round, +-25 % gain limit, lr 0.01, Adam reset per loop, each update from theta_l, last-round fit skipped), plant P1 (S2) or P1RD (S1, S3), flight seeds 100 + 10 l + i, noise seeds 6000 + 2 l (+1),
evaluation seeds 2000-2004 (theta_0 and theta_best over the flown sets). Controller from DIFFCOTUNE_CONTROLLER. Stops after round 1 if the estimate for 5 rounds exceeds 3 h.
Usage (nice -n 19, through job_runner): python3 run_5.py"""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parents[1]))
import loop_lib as L  # noqa
RES = HERE / 'results'; RES.mkdir(exist_ok=True); LIMIT_S = 3 * 3600; EVAL_SEEDS = (2000, 2001, 2002, 2003, 2004)


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    t_start = time.time(); tag = L.CTRL; plant = L.PLANTS['P1RD' if L.N_BETA == 5 else 'P1']; halt = {'why': None}
    pool = L.Pool(len(L.TUNING), states=('p', 'v', 'q'))          # Stage 7: one worker per tuning scenario (5)
    try:
        def save(r):
            L.jdump(r, RES / ('loop_%s.json' % tag))
            if len(r['rounds']) == 2:
                est = np.mean([x['seconds_round'] for x in r['rounds']]) * 5; say('estimate for 5 rounds: %.0f min' % (est / 60))
                if est > LIMIT_S: halt['why'] = 'estimate above 3 h'
            if r['rounds'] and r['rounds'][-1]['seconds_round'] > 45 * 60: halt['why'] = 'a round took more than 45 min (user rule, Stage 7)'
        rec = L.run_loop(pool, plant, 'cotune', 0.01, n_rounds=5, log=say, stop=False, best=True, skip_last_fit=True, on_round=save, stop_after=lambda r: halt['why'] is not None,
                         noise_seeds_of=lambda l: (6000 + 2 * l, 6001 + 2 * l), gain_limit=0.25, seed_base=100)
        rec['halt'] = halt['why']; save(rec); say('halt reason:', halt['why'])
        phis = {'theta_0': np.zeros(10), 'theta_best': np.array(rec['best']['phi'])}
        ev = {k: L.evaluate(pool, p, plant, seeds=EVAL_SEEDS) for k, p in phis.items()}
        L.jdump(dict(controller=tag, phis={k: p.tolist() for k, p in phis.items()}, best_round=rec['best']['round'], rounds_flown=len(rec['rounds']), halt=halt['why'], eval=ev, seconds=time.time() - t_start), RES / ('eval_%s.json' % tag))
        say(tag, 'evaluated; total %.0f min' % ((time.time() - t_start) / 60))
    finally:
        pool.close()
