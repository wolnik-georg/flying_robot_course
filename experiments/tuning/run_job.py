#!/usr/bin/env python3
"""  CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build python3 experiments/tuning/run_job.py fbl --method cmaes --seed 0 --budget 120 --pop 12
Jobs: fbl (6 parameters, 5 tuning cells, 4 search variants). Logs: experiments/tuning/out/<job>_<method>_s<seed>.jsonl"""
import argparse, json, sys, time
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import core, optimize
import plant_core as P

def _s0_map(g):      # S0 parameters -> Omar C struct fields (xy tied)
    m = {'Kpos_P.x': g['Pxy'], 'Kpos_P.y': g['Pxy'], 'Kpos_P.z': g['Pz'], 'Kpos_D.x': g['Dxy'], 'Kpos_D.y': g['Dxy'], 'Kpos_D.z': g['Dz'], 'Kpos_I.z': g['Iz'],
         'KR.x': g['KRxy'], 'KR.y': g['KRxy'], 'KR.z': g['KRz'], 'Komega.x': g['Kwxy'], 'Komega.y': g['Kwxy'], 'Komega.z': g['Kwz'], 'KI.x': g['KIxy'], 'KI.y': g['KIxy']}
    return m
S0_PARAMS = {'Pxy': (7.0, .25, 4.0), 'Pz': (7.0, .25, 4.0), 'Dxy': (4.0, .25, 4.0), 'Dz': (4.0, .25, 4.0), 'Iz': (1.5, .1, 4.0), 'KRxy': (0.007, .25, 3.0), 'KRz': (0.008, .25, 3.0),
             'Kwxy': (0.00115, .25, 3.0), 'Kwz': (0.002, .25, 3.0), 'KIxy': (0.03, .1, 4.0)}      # factors around the baseline (baseline_before_autotune/), wide log bounds
S0_CELLS = {      # tuning settings from the NS2 training grid (dz 0.25 / 0.5 / 0.75, offset 0.05 / 0.1 / 0.15, speed 0.1-0.5), none is a study cell
    'A1': P.TUNING_CELLS[:2], 'A4': P.TUNING_CELLS[2:],
    'A8': [("A8", dict(dz=0.25, span=1.0, duration=8.0, settle=2.0, passes=2)), ("A8", dict(dz=0.75, span=1.0, duration=5.0, settle=2.0, passes=2))]}
JOBS = {
    # FBL: factors around preset 1; xy tied x = y; wide log bounds (fairness rule 6)
    'fbl': lambda: core.Job('fbl', {'kp_xy': (14.0, 0.25, 6.0), 'kv_xy': (9.5, 0.4, 3.0), 'ki_xy': (12.0, 0.25, 8.0), 'kp_z': (11.25, 0.25, 4.0), 'kv_z': (5.4, 0.4, 3.0), 'ki_z': (12.0, 0.25, 4.0)}, P.TUNING_CELLS),
}
for _sc in ('A1', 'A4', 'A8'): JOBS['s0_' + _sc] = (lambda sc=_sc: core.Job('s0_' + sc, S0_PARAMS, S0_CELLS[sc], extra=dict(J='omar', tree=True, indi=0, ns2=0), tree_map=_s0_map))
for _sc in ('A1', 'A4', 'A8'): JOBS['s0r_' + _sc] = (lambda sc=_sc: core.Job('s0r_' + sc, S0_PARAMS, S0_CELLS[sc], variants=core.SEARCH_VARIANTS + [dict(lag=60)], extra=dict(J='omar', tree=True, indi=0, ns2=0), tree_map=_s0_map))      # robust: motor lag 60 ms in the search (the first S0 run crashed there)
for _sc in ('A1', 'A4', 'A8'): JOBS['s0n_' + _sc] = (lambda sc=_sc: core.Job('s0n_' + sc, S0_PARAMS, S0_CELLS[sc], variants=core.SEARCH_VARIANTS + [dict(lag=45)], extra=dict(J='omar', tree=True, indi=0, ns2=0), tree_map=_s0_map))      # lag-30 plant, robust to motor lag 45 ms
for _sc in ('A1', 'A4', 'A8'): JOBS['s0p_' + _sc] = JOBS['s0n_' + _sc]      # same job as s0n; the runs differ only by the crash threshold of the plant (70 deg instead of 45)
JOBS['fbln'] = lambda: core.Job('fbln', JOBS['fbl']().params, P.TUNING_CELLS, variants=core.SEARCH_VARIANTS + [dict(lag=45)])      # FBL on the lag-30 plant, robust to motor lag 45 ms
for _sc in ('A1', 'A4', 'A8'): JOBS['s2p_' + _sc] = (lambda sc=_sc: core.Job('s2p_' + sc, S0_PARAMS, S0_CELLS[sc], variants=core.SEARCH_VARIANTS + [dict(lag=45)], extra=dict(J='omar', tree=True, indi=0, ns2=1), tree_map=_s0_map))      # S2 = geometric + NS2, own tuning
S0_CELLS_V2 = {'A1': P.TUNING_CELLS_V2[:2], 'A4': P.TUNING_CELLS_V2[2:], 'A8': [("A8", dict(dz=0.375, span=1.0, duration=8.0, settle=2.0, passes=2)), ("A8", dict(dz=0.75, span=1.0, duration=5.0, settle=2.0, passes=2))]}
for _sc in ('A1', 'A4', 'A8'):      # option 1 jobs: same parameters and variants, cells without dz 0.25
    JOBS['s0q_' + _sc] = (lambda sc=_sc: core.Job('s0q_' + sc, S0_PARAMS, S0_CELLS_V2[sc], variants=core.SEARCH_VARIANTS + [dict(lag=45)], extra=dict(J='omar', tree=True, indi=0, ns2=0), tree_map=_s0_map))
    JOBS['s2q_' + _sc] = (lambda sc=_sc: core.Job('s2q_' + sc, S0_PARAMS, S0_CELLS_V2[sc], variants=core.SEARCH_VARIANTS + [dict(lag=45)], extra=dict(J='omar', tree=True, indi=0, ns2=1), tree_map=_s0_map))
JOBS['fblq'] = lambda: core.Job('fblq', JOBS['fbl']().params, P.TUNING_CELLS_V2, variants=core.SEARCH_VARIANTS + [dict(lag=45)])
if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument('job'); ap.add_argument('--method', default='cmaes'); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--budget', type=int, default=120); ap.add_argument('--pop', type=int, default=12); ap.add_argument('--procs', type=int, default=20); a = ap.parse_args()
    job = JOBS[a.job](); out = Path(__file__).parent / 'out'; out.mkdir(exist_ok=True); lp = out / f'{a.job}_{a.method}_s{a.seed}.jsonl'
    if lp.exists(): lp.unlink()
    (P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
    with Pool(a.procs) as pool:
        t0 = time.time()
        best = optimize.run_cmaes(job, pool, a.budget, a.seed, a.pop, lp) if a.method == 'cmaes' else optimize.run_sobol(job, pool, a.budget, a.seed, a.pop, lp)
        print('best', round(best['score'], 3), {k: round(v, 2) for k, v in best['gains'].items()}, '%.0f s' % (time.time() - t0))
