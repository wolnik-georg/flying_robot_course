#!/usr/bin/env python3
"""Stage 8d T1 (a): unit test of the strict dry-run rule (DIFFCOTUNE_DRY_STRICT=1) with a fake pool (no frsim, no pool, no models). Run: DIFFCOTUNE_CONTROLLER=S0 DIFFCOTUNE_DRY_STRICT=1 python3 test_strict_rule.py"""
import os, sys
os.environ['DIFFCOTUNE_DRY_STRICT'] = '1'; os.environ.setdefault('DIFFCOTUNE_CONTROLLER', 'S0')
from pathlib import Path
import numpy as np
H = Path(__file__).resolve().parent; sys.path.insert(0, str(H.parent)); sys.path.insert(0, str(H.parents[1]))
import loop_lib as L, reference as R  # noqa
assert L.STRICT


class Fake:
    """fly(jobs) -> (trace, stable, tilt); the flight is a floor flight / crash when (round, scenario, attempt) is in the table; rollouts records its arguments and returns a smooth cost."""
    states = ('p', 'v', 'q')
    def __init__(self, floor=(), crash=(), fail_fallback=False): self.floor = set(floor); self.crash = set(crash); self.flown = []; self.fits = []; self.noise_seeds = None
    def fly(self, jobs):
        out = []
        for scen, cell, gains, plant, seed in jobs:
            att = seed // 1000 + 1; l = ((seed % 1000) - 100) // 10; key = (l, scen, att); self.flown.append(dict(round=l, scen=scen, attempt=att, gains=gains, seed=seed))
            sc = R.build(scen, cell).scenario_s; t = np.linspace(0, sc, 400); tr = np.zeros((400, 20)); tr[:, 0] = t; tr[:, 16] = 1.0; tr[:, 3] = 0.05 if key in self.floor else 1.0
            out.append((tr, key not in self.crash, 5.0))
        return out
    def rollouts(self, kind, x, traces, scen_list=L.TUNING):
        if kind == 'sysid': self.fits.append([s for s, _ in scen_list]); assert traces is not None and len(traces) == len(scen_list)
        return float(np.sum((np.asarray(x) - 0.1) ** 2) + 1.0), 2 * (np.asarray(x) - 0.1)
    def close(self): pass


def run(pool, n_rounds=3):
    return L.run_loop(pool, dict(variant='P1'), 'cotune', 0.01, n_rounds=n_rounds, log=lambda *a: None, stop=False, best=True, skip_last_fit=True, gain_limit=0.25, seed_base=100)
ok = True
def check(name, cond, info=''):
    global ok; ok &= bool(cond); print('PASS' if cond else 'FAIL', name, info)

# (1) all 3 attempts fail in round 1 on A1: 3 attempts, fallback with theta_best, fit gets 4 traces, set not eligible
p = Fake(floor={(1, 'A1', a) for a in (1, 2, 3)}); rec = run(p)
r1 = rec['rounds'][1]; f = [x for x in r1['flights'] if x['scenario'] == 'A1'][0]
check('1a three attempts on A1 in round 1', len(f['attempts']) == 3 and f['used_attempt'] is None and f['failed_all'], [a['tries'] for a in f['attempts']])
fb = [x for x in p.flown if x['round'] == 1 and x['scen'] == 'A1' and x['attempt'] == 4]
th_best = L.gains_of(np.array(rec['rounds'][0]['phi']))          # round 0 was eligible and is the only keep-best set before round 1
check('1b fallback flown once with theta_best', len(fb) == 1 and all(abs(fb[0]['gains'][k] - th_best[k]) < 1e-12 for k in th_best) and f['fallback'] is not None and f['fallback']['used'] is False)
check('1c fit of round 1 gets 4 scenarios without A1', any(len(x) == 4 and 'A1' not in x for x in p.fits), p.fits[:3])
check('1d round 1 not eligible, r3 scenario listed', r1['eligible'] is False and r1['r3_scenarios'] == ['A1'] and r1['real_used']['n_used'] == 4)
check('1e keep-best skips round 1 (round 2 or 0)', rec['best']['round'] in (0, 2) and rec['best']['round'] != 1, rec['best']['round'])
check('1f as-flown cost differs from used cost (A1 floor flight included in as-flown)', r1['real']['cost'] != r1['real_used']['cost'])
# (2) floor on attempt 1 only: attempt 2 used
p = Fake(floor={(1, 'A8', 1)}); rec = run(p); f = [x for x in rec['rounds'][1]['flights'] if x['scenario'] == 'A8'][0]
check('2 floor on attempt 1 only: attempt 2 used, set eligible, seed + 1000', f['used_attempt'] == 2 and rec['rounds'][1]['eligible'] and f['attempts'][1]['seed'] == f['attempts'][0]['seed'] + 1000 and rec['rounds'][1]['real_used']['n_used'] == 5)
check('2b a crash on attempt 1 is repeated the same way', run(Fake(crash={(0, 'FIG8', 1)}))['rounds'][0]['flights'][3]['used_attempt'] == 2)
# (3) round 0 with 3 fails: no fallback
p = Fake(floor={(0, 'A1', a) for a in (1, 2, 3)}); rec = run(p)
check('3 round 0, 3 failed attempts: no fallback flight, not eligible', not [x for x in p.flown if x['round'] == 0 and x['attempt'] == 4] and rec['rounds'][0]['eligible'] is False and [x for x in rec['rounds'][0]['flights'] if x['scenario'] == 'A1'][0]['fallback'] is None)
# (4) no eligible set at all (Stage 8d2): 2 extension rounds are flown, then "ask the user", never theta_0
p = Fake(floor={(l, 'A1', a) for l in range(8) for a in (1, 2, 3)}); rec = run(p)
check('4 no eligible set after 3 + 2 rounds: best has no phi (no theta_0), note "ask the user", 5 rounds, extension 2', rec['best']['eligible'] is False and rec['best']['round'] is None and rec['best']['phi'] is None and 'ask the user' in rec['best']['note'] and 'after extension' in rec['best']['note'] and len(rec['rounds']) == 5 and rec['extension_rounds_used'] == 2, (len(rec['rounds']), rec['best']))
# (6) extension: A1 fails in rounds 0..2 (all planned rounds), round 3 (extension 1) is eligible
p = Fake(floor={(l, 'A1', a) for l in range(3) for a in (1, 2, 3)}); rec = run(p)
check('6a one extension round flown, eligible, becomes theta_best', len(rec['rounds']) == 4 and rec['extension_rounds_used'] == 1 and rec['best']['round'] == 3 and rec['best']['eligible'] and rec['best']['phi'] == rec['rounds'][3]['phi'], (len(rec['rounds']), rec['best']['round']))
check('6b the update after the last planned round was computed (fit + tune before the extension round)', 'tune' in rec['rounds'][2] and 'fit' in rec['rounds'][2] and rec['rounds'][2].get('extension_round_added') == 1)
# (7) eligible set exists: no extension, record as before
p = Fake(); rec = run(p)
check('7 eligible sets: no extension round', len(rec['rounds']) == 3 and rec['extension_rounds_used'] == 0 and not any('extension_round_added' in r for r in rec['rounds']))
# (8) both extension rounds needed: round 4 is the first eligible
p = Fake(floor={(l, 'A8', a) for l in range(4) for a in (1, 2, 3)}); rec = run(p)
check('8 two extension rounds, round 4 eligible = theta_best', len(rec['rounds']) == 5 and rec['extension_rounds_used'] == 2 and rec['best']['round'] == 4)
# (9) round 0 eligible but later rounds fail: no extension even if the last planned round is not eligible
p = Fake(floor={(l, 'A1', a) for l in (1, 2) for a in (1, 2, 3)}); rec = run(p)
check('9 eligible round 0 exists: last planned round not eligible, no extension, theta_best = round 0', len(rec['rounds']) == 3 and rec['extension_rounds_used'] == 0 and rec['best']['round'] == 0)
# (5) after a set without eligibility the fallback uses the best ELIGIBLE set (round 0 here), never the non eligible round 1
p = Fake(floor={(1, 'A1', a) for a in (1, 2, 3)} | {(2, 'A8', a) for a in (1, 2, 3)}); rec = run(p)
fb2 = [x for x in p.flown if x['round'] == 2 and x['attempt'] == 4]; keep_before = rec['rounds'][0]['phi']
check('5 fallback of round 2 uses theta_best of the eligible sets so far (round 0 or 0-eligible best)', len(fb2) == 1 and rec['rounds'][2]['flights'][1]['fallback']['theta_round'] == 0, rec['rounds'][2]['flights'][1]['fallback'] and rec['rounds'][2]['flights'][1]['fallback']['theta_round'])
print('ALL PASS' if ok else 'SOME FAILED'); sys.exit(0 if ok else 1)
