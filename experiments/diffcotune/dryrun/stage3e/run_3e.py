#!/usr/bin/env python3
"""Stage 3e Part B: the chosen method version "A + B" (sensor noise in the tuning step with 2 fixed seeds per round, +-25 % gain limit per round, 50 epochs with
the best iterate, J_sysId = position + velocity + attitude, lr 0.01, --skip-last-fit), 6 rounds (l = 0 ... 5, 5 updates).
Usage: run_3e.py P1|P2      (nice -n 19, one pool of 4 workers, 2 torch threads each)
New seeds (none used in Stage 3b / 3c / 3d): flight seeds 100 + 10 l + i (i = scenario index 0-3; a tilt-crash repeat adds 1000 per try, as before);
tuning noise seeds 6000 + 2 l and 6001 + 2 l; evaluation seeds 2000 ... 2004. Evaluates theta_0 (default), theta_best (keep-best over the flown sets) and theta_5.
Stops after round 1 if the estimate for all rounds exceeds 3 h. Results in dryrun/stage3e/results/."""
import sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
RES = HERE / 'results'; RES.mkdir(exist_ok=True)
SEED_BASE = 100; EVAL_SEEDS = (2000, 2001, 2002, 2003, 2004); LIMIT_S = 3 * 3600


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    pl = sys.argv[1]; t_start = time.time()
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        def save(r):
            L.jdump(r, RES / ('loop_%s.json' % pl))
            if len(r['rounds']) == 2:
                est = np.mean([x['seconds_round'] for x in r['rounds']]) * 6
                say('estimate for 6 rounds: %.0f min' % (est / 60))
                if est > LIMIT_S:
                    raise SystemExit('ESTIMATE ABOVE 3 h: stopped after round 1 (%.0f min)' % (est / 60))
        rec = L.run_loop(pool, L.PLANTS[pl], 'cotune', 0.01, n_rounds=6, log=say, stop=False, best=True, skip_last_fit=True, on_round=save,
                         noise_seeds_of=lambda l: (6000 + 2 * l, 6001 + 2 * l), gain_limit=0.25, seed_base=SEED_BASE)
        save(rec)
        phis = {'theta_0': np.zeros(10), 'theta_best': np.array(rec['best']['phi']), 'theta_5': np.array(rec['rounds'][5]['phi'])}
        ev = {k: L.evaluate(pool, p, L.PLANTS[pl], seeds=EVAL_SEEDS) for k, p in phis.items()}
        L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, best_round=rec['best']['round'], eval=ev, seconds=time.time() - t_start), RES / ('eval_%s.json' % pl))
        say(pl, 'evaluated; total %.0f min' % ((time.time() - t_start) / 60))
    finally:
        pool.close()
