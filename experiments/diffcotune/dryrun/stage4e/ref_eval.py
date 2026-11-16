#!/usr/bin/env python3
"""Stage 4e: reference final evaluation for B1 of Stage 3e on 5 rounds: theta_best over rounds 0-4 of loop_P1.json (round 4, 8.13 cm), evaluation seeds 2000-2004, same
evaluate() as Stage 3e. theta_0 is the Stage 3e value (eval_P1.json, same seeds, same function). Usage (nice -n 19): python3 ref_eval.py"""
import sys, json, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L  # noqa: E402

if __name__ == '__main__':
    rec = json.load(open(HERE.parent / 'stage3e' / 'results' / 'loop_P1.json'))
    costs = [r['real']['cost'] for r in rec['rounds'][:5]]
    kb = None
    for l, c in enumerate(costs):
        if kb is None or c <= costs[kb]:
            kb = l
    print('B1 rounds 0-4 real 3-D RMS cm', [round(100 * c ** .5, 2) for c in costs], 'keep-best round', kb, flush=True)
    pool = L.Pool(4)
    try:
        phi = np.array(rec['rounds'][kb]['phi']); ev = L.evaluate(pool, phi, L.P1, seeds=(2000, 2001, 2002, 2003, 2004))
        L.jdump(dict(best_round=kb, phi=phi.tolist(), eval=ev), HERE / 'results' / 'ref_eval_B1_5rounds.json')
        print('done', flush=True)
    finally:
        pool.close()
