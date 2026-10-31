#!/usr/bin/env python3
"""S2 with S0's tuned gains (variant b of the C tree): score the chosen S0 sets and the baseline on the S2 job (NS2 on): tuning cells x 9 full variants, and the study cells (nominal, ffs 0.4, md 40, md 60)."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import core, plant_core as P
from run_job import JOBS
here = Path(__file__).parent; (P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
PICK = {'A1': 'cmaes s1', 'A4': 'cmaes s1', 'A8': 'cmaes s0'}; STUDY = {'A1': P.STUDY_CELLS[:1], 'A4': P.STUDY_CELLS[1:3], 'A8': P.STUDY_CELLS[3:]}
d = json.load(open(here / 'out/s0p_rerank.json')); out = {}
with Pool(20) as pool:
    for sc, n in PICK.items():
        job = JOBS['s2p_' + sc](); cand = {'S2 baseline': d[sc]['baseline']['gains'], 'S0 tuned (' + n + ')': d[sc][n]['gains']}; o = out[sc] = {}
        full = core.evaluate_batch(job, pool, list(cand.values()), variants=core.FULL_VARIANTS)
        for (k, g), r in zip(cand.items(), full): o[k] = dict(gains=g, full=r['score'], crashes=sum(x >= 100 for x in r['per_episode']))
        for tag, v in (('nominal', {}), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40)), ('md60', dict(md=60))):
            for (k, g), r in zip(cand.items(), core.evaluate_batch(job, pool, list(cand.values()), variants=[v], cells=STUDY[sc])): o[k]['study_' + tag] = r['score']
        for k in cand: print(sc, '%-24s full %.2f (crash %d) | study %s' % (k, o[k]['full'], o[k]['crashes'], ' '.join('%.2f' % o[k]['study_' + t] for t in ('nominal', 'ffs0.4', 'md40', 'md60'))), flush=True)
json.dump(out, open(here / 'out/s2_reuse.json', 'w'), indent=1)
