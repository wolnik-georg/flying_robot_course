#!/usr/bin/env python3
"""Rank the downwash-trained gradient candidates (out/diff_s0D_<scen>_seed*.json) in the REAL firmware plant, with downwash, per scenario type.
Selection stage: the scenario type's tuning cells (S0_CELLS_V2) x the 9 full variants, by (crashes, score). Final stage: study cells of the type x nominal / ffs 0.4 / md 40 / md 60,
next to the baseline and the black-box (CMA-ES s0q) pick. Usage: SCEN=A8 CFFIRMWARE_DIR=... python3 diff/rank_diff_scen.py"""
import glob, json, os, sys
from multiprocessing import Pool
from pathlib import Path
here = Path(__file__).parent.parent; sys.path.insert(0, str(here))
import core, plant_core as P
from run_job import JOBS, S0_PARAMS
(P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
SC = os.environ.get('SCEN', 'A8'); STUDY = {'A1': P.STUDY_CELLS[:1], 'A4': P.STUDY_CELLS[1:3], 'A8': P.STUDY_CELLS[3:]}[SC]
job = JOBS['s0q_' + SC](); cand = {'baseline': {k: v[0] for k, v in S0_PARAMS.items()}}
for f in sorted(glob.glob(str(here / f'out/diff_s0D_{SC}_seed*.json'))): d = json.load(open(f)); cand[f"grad s{d['seed']}"] = d['gains']
rr = json.load(open(here / 'out/s0q_rerank.json'))[SC]; pick = {'A1': 'cmaes s1', 'A4': 'cmaes s1', 'A8': 'cmaes s1'}[SC]; cand[f'cma-es {pick}'] = rr[pick]['gains']
variants = [('nominal', dict()), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40)), ('md60', dict(md=60))]; out = {}
with Pool(20) as pool:
    names = list(cand); gl = [cand[n] for n in names]
    res = core.evaluate_batch(job, pool, gl, variants=core.FULL_VARIANTS)
    for n, g, r in zip(names, gl, res): out[n] = dict(gains=g, sel_score=r['score'], sel_crashes=sum(x >= 100 for x in r['per_episode']), sel_tilt=r['tilt_max']); print('%s %-18s select: full %.2f (crash %d) tilt %.0f' % (SC, n, r['score'], out[n]['sel_crashes'], r['tilt_max']), flush=True)
    for tag, v in variants:
        for n, r in zip(names, core.evaluate_batch(job, pool, gl, variants=[v], cells=STUDY)): out[n]['study_' + tag] = r['score']
    for n in names: print('%s %-18s study: %s' % (SC, n, ' '.join('%.2f' % out[n]['study_' + t] for t, _ in variants)), flush=True)
json.dump(out, open(here / f'out/diff_s0D_{SC}_rank.json', 'w'), indent=1)
