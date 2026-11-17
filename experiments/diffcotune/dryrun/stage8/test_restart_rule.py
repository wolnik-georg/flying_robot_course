#!/usr/bin/env python3
"""Stage 8e T4 (a): unit test of rule RS (restart from theta_best, stop after two R3 rounds in a row) on top of the strict rule (DIFFCOTUNE_DRY_STRICT=1) with a fake pool (no frsim, no pool, no models). Run: DIFFCOTUNE_CONTROLLER=S0 DIFFCOTUNE_DRY_STRICT=1 python3 test_strict_rule.py"""
import os, sys
os.environ['DIFFCOTUNE_DRY_STRICT'] = '1'; os.environ['DIFFCOTUNE_RESTART_BEST'] = '1'; os.environ.setdefault('DIFFCOTUNE_CONTROLLER', 'S0')
from pathlib import Path
import numpy as np
H = Path(__file__).resolve().parent; sys.path.insert(0, str(H.parent)); sys.path.insert(0, str(H.parents[1]))
import loop_lib as L, reference as R  # noqa
assert L.STRICT and L.RESTART_BEST


class Fake:
    """fly(jobs) -> (trace, stable, tilt); the flight is a floor flight / crash when (round, scenario, attempt) is in the table; rollouts records its arguments and returns a smooth cost."""
    states = ('p', 'v', 'q')
    def __init__(self, floor=(), crash=(), fail_fallback=False): self.floor = set(floor); self.crash = set(crash); self.flown = []; self.fits = []; self.noise_seeds = None; self.starts = []; self.new = False; self.rounds_fit = 0
    def fly(self, jobs):
        out = []
        for scen, cell, gains, plant, seed in jobs:
            att = seed // 1000 + 1; l = ((seed % 1000) - 100) // 10; key = (l, scen, att); self.flown.append(dict(round=l, scen=scen, attempt=att, gains=gains, seed=seed))
            sc = R.build(scen, cell).scenario_s; t = np.linspace(0, sc, 400); tr = np.zeros((400, 20)); tr[:, 0] = t; tr[:, 16] = 1.0; tr[:, 3] = 0.05 if key in self.floor else 1.0
            out.append((tr, key not in self.crash, 5.0))
        return out
    def rollouts(self, kind, x, traces, scen_list=L.TUNING):
        if kind == 'sysid': self.new = True
        if kind == 'task' and self.new: self.starts.append(np.array(x[:10]).copy()); self.new = False
        if kind == 'sysid': self.fits.append([s for s, _ in scen_list]); assert traces is not None and len(traces) == len(scen_list)
        return float(np.sum((np.asarray(x) - 0.1) ** 2) + 1.0), 2 * (np.asarray(x) - 0.1)
    def close(self): pass


def run(pool, n_rounds=3):
    return L.run_loop(pool, dict(variant='P1'), 'cotune', 0.01, n_rounds=n_rounds, log=lambda *a: None, stop=False, best=True, skip_last_fit=True, gain_limit=0.25, seed_base=100)
ok = True
def check(name, cond, info=''):
    global ok; ok &= bool(cond); print('PASS' if cond else 'FAIL', name, info)
def run5(pool, n=5): return L.run_loop(pool, dict(variant='P1'), 'cotune', 0.01, n_rounds=n, log=lambda *a: None, stop=False, best=True, skip_last_fit=True, gain_limit=0.25, seed_base=100)
F3 = lambda l: {(l, 'A1', a) for a in (1, 2, 3)}
Z = np.zeros(10)
# (1) R3 in round 1, theta_best = round 0: the tuning step of round 1 starts from theta_best (zeros), not from theta_1; round 2 (no R3) from theta_2
p = Fake(floor=F3(1)); rec = run5(p); r = rec['rounds']
check('1a RS: R3 round 1 tunes from theta_best (round 0 = zeros), not from theta_1', np.allclose(p.starts[1], Z) and not np.allclose(r[1]['phi'], Z) and r[1]['restart_from_best'] is True and r[1]['restart_theta_best_round'] == 0, (p.starts[1][:2], r[1]['phi'][:2]))
check('1b round 0 and the round after the R3 round start from their own theta', np.allclose(p.starts[0], r[0]['phi']) and np.allclose(p.starts[2], r[2]['phi']) and r[2]['restart_from_best'] is False)
check('1c the beta fit of the R3 round is unchanged (4 usable scenarios, fit flights without A1), loop not stopped', any(len(f) == 4 and 'A1' not in f for f in p.fits) and len(r) == 5 and rec.get('rs_stop') is None)
# (2) R3 in round 0 (no theta_best): start from theta_0 = the flown theta, no restart
p = Fake(floor=F3(0)); rec = run5(p); r = rec['rounds']
check('2 R3 in round 0: no theta_best, tuning from theta_0, restart False, no stop', np.allclose(p.starts[0], Z) and r[0]['restart_from_best'] is False and len(r) == 5)
# (3) two R3 rounds in a row (rounds 1 and 2): stop after round 2, result theta_best, halt reason, no tune / fit in round 2, no further round
p = Fake(floor=F3(1) | F3(2)); rec = run5(p); r = rec['rounds']
check('3a two R3 rounds in a row: loop ends after round 2, halt reason, theta_best = round 0', len(r) == 3 and rec['halt'] == 'two R3 rounds in a row' and rec['rs_stop'] == 'two R3 rounds in a row' and rec['best']['round'] == 0 and rec['best']['eligible'], (len(r), rec['best']))
check('3b the stopping round has no fit and no tuning step, and no later flight is made', 'tune' not in r[2] and 'fit' not in r[2] and r[2]['rs_stop'] and not [x for x in p.flown if x['round'] >= 3] and len(p.starts) == 2)
check('3c the first R3 round (round 1) restarted from theta_best and still had its tuning step', r[1]['restart_from_best'] is True and 'tune' in r[1])
# (4) R3 rounds that are not consecutive: no stop
p = Fake(floor=F3(1) | F3(3)); rec = run5(p); r = rec['rounds']
check('4 R3 in rounds 1 and 3 (not consecutive): no stop, 5 rounds, both restarted from theta_best', len(r) == 5 and rec.get('rs_stop') is None and r[1]['restart_from_best'] and r[3]['restart_from_best'])
# (5) R3 in round 0 and 1 with no theta_best in round 1: no stop (no theta_best), round 1 starts from its own theta
p = Fake(floor=F3(0) | F3(1)); rec = run5(p); r = rec['rounds']
check('5 R3 rounds 0 and 1 without theta_best: no stop, no restart', len(r) == 5 and rec.get('rs_stop') is None and not r[1]['restart_from_best'] and np.allclose(p.starts[1], r[1]['phi']))
# (6) RS switched off (Stage 8d behaviour): same floors as case 1 and case 3
L.RESTART_BEST = False
p = Fake(floor=F3(1)); rec = run5(p); r = rec['rounds']
check('6a RS = 0, R3 round 1: tuning from theta_1 (Stage 8d behaviour), no RS fields', np.allclose(p.starts[1], r[1]['phi']) and 'restart_from_best' not in r[1] and len(r) == 5)
p = Fake(floor=F3(1) | F3(2)); rec = run5(p); r = rec['rounds']
check('6b RS = 0, two R3 rounds in a row: loop goes on to round 4, no stop fields', len(r) == 5 and 'rs_stop' not in rec and rec.get('halt') is None and 'rs_stop' not in r[2])
print('ALL PASS' if ok else 'SOME FAILED'); sys.exit(0 if ok else 1)
