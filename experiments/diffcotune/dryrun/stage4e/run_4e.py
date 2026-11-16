#!/usr/bin/env python3
"""Stage 4e T1 / T2: the method version of Stage 3e B1 (sensor noise in the tuning step with 2 fixed seeds per round, +-25 % gain limit, lr 0.01, Adam reset per inner loop,
each update from theta_l, last-round fit skipped) over 5 rounds (l = 0 ... 4), with ONE thing changed:
  T1  : the paper's literal stop rule (loop_lib._adam_paper, raw SI cost, up to 50 epochs, last iterate), states pva
  T2v : 50 epochs + best iterate as in B1, J_sysId with the body rates: v = raw | sg110 | sg70 | sg210 (gyro_variants.py)
Seeds as Stage 3e B1: flight seeds 100 + 10 l + i, noise seeds 6000 + 2 l (+1), evaluation seeds 2000 ... 2004 (theta_0 | theta_best over the flown sets).
T1 stops after round 1 (two rounds flown, fit and tune of round 1 done) if every inner loop of round 0 stopped within 2 epochs. Stops after round 1 if the estimate for 5 rounds exceeds 3 h.
Usage (nice -n 19): run_4e.py T1 | run_4e.py T2 raw|sg110|sg70|sg210"""
import sys, time, json
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa
import gyro_variants as G  # noqa
RES = HERE / 'results'; RES.mkdir(exist_ok=True)
SEED_BASE = 100; EVAL_SEEDS = (2000, 2001, 2002, 2003, 2004); LIMIT_S = 3 * 3600


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


if __name__ == '__main__':
    which = sys.argv[1]; var = sys.argv[2] if len(sys.argv) > 2 else None; t_start = time.time()
    tag = which if var is None else '%s_%s' % (which, var)
    if which == 'T1':
        kw = dict(stop=False, best=False, paper_stop=True); states = ('p', 'v', 'q'); trace_fn = None
    else:
        kw = dict(stop=False, best=True, paper_stop=False); states = ('p', 'v', 'q', 'w')
        trace_fn = G.make_trace_fn('raw' if var == 'raw' else 'sg', L.P1['overrides'], int(var[2:]) if var != 'raw' else 110, seed_base=7000)
    pool = L.Pool(4, states=states)
    halt = {'why': None}
    try:
        def save(r):
            L.jdump(r, RES / ('loop_%s.json' % tag))
            if len(r['rounds']) == 2:
                est = np.mean([x['seconds_round'] for x in r['rounds']]) * 5
                say('estimate for 5 rounds: %.0f min' % (est / 60))
                if est > LIMIT_S:
                    halt['why'] = 'estimate above 3 h'

        def stop_after(r):
            if which == 'T1' and len(r['rounds']) == 2:
                r0 = r['rounds'][0]; ep = [r0['fit']['epochs'], r0['tune']['epochs']]
                if max(ep) <= 2:
                    halt['why'] = 'prediction holds: every inner loop of round 0 stopped within 2 epochs (%s)' % ep
            return halt['why'] is not None

        rec = L.run_loop(pool, L.PLANTS['P1'], 'cotune', 0.01, n_rounds=5, log=say, skip_last_fit=True, on_round=save, stop_after=stop_after, trace_fn=trace_fn,
                         noise_seeds_of=lambda l: (6000 + 2 * l, 6001 + 2 * l), gain_limit=0.25, seed_base=SEED_BASE, **kw)
        rec['halt'] = halt['why']; save(rec)
        say('halt reason:', halt['why'])
        phis = {'theta_0': np.zeros(10), 'theta_best': np.array(rec['best']['phi'])}
        ev = {k: L.evaluate(pool, p, L.PLANTS['P1'], seeds=EVAL_SEEDS) for k, p in phis.items()}
        L.jdump(dict(phis={k: p.tolist() for k, p in phis.items()}, best_round=rec['best']['round'], rounds_flown=len(rec['rounds']), halt=halt['why'], eval=ev, seconds=time.time() - t_start), RES / ('eval_%s.json' % tag))
        say(tag, 'evaluated; total %.0f min' % ((time.time() - t_start) / 60))
    finally:
        pool.close()
