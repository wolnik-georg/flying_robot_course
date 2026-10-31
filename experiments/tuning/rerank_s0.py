#!/usr/bin/env python3
"""Re-rank the best S0 gain set of every run (and the baseline) on the 9 full variants (tuning cells) and on the held-out study cells (nominal, ffs 0.4, md 40, md 60)."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import core, plant_core as P
from run_job import JOBS
here = Path(__file__).parent; (P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
STUDY = {'A1': P.STUDY_CELLS[:1], 'A4': P.STUDY_CELLS[1:3], 'A8': P.STUDY_CELLS[3:]}
variants = [('nominal', dict()), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40)), ('md60', dict(md=60))]
PRE = sys.argv[1] if len(sys.argv) > 1 else 's0'; out = {}
with Pool(20) as pool:
    for sc in ('A1', 'A4', 'A8'):
        job = JOBS[PRE + '_' + sc](); cand = {'baseline': job.gains(job.default_u())}
        for m in [m for m in ('cmaes', 'sobol') if (here / f'out/{PRE}_{sc}_{m}_s0.jsonl').exists()]:
            for s in range(3):
                rows = [json.loads(l) for l in open(here / f'out/{PRE}_{sc}_{m}_s{s}.jsonl') if l.strip()]; b = min(rows, key=lambda r: r['score']); cand[f'{m} s{s}'] = b['gains']; cand[f'{m} s{s}_search'] = b['score']
        names = [n for n in cand if not n.endswith('_search')]; gl = [cand[n] for n in names]
        full = core.evaluate_batch(job, pool, gl, variants=core.FULL_VARIANTS)
        o = out[sc] = {n: dict(gains=g, search=cand.get(n + '_search'), full=r['score'], full_tilt=r['tilt_max'], crashes=sum(x >= 100 for x in r['per_episode'])) for n, g, r in zip(names, gl, full)}
        for tag, v in variants:
            res = core.evaluate_batch(job, pool, gl, variants=[v], cells=STUDY[sc])
            for n, r in zip(names, res): o[n]['study_' + tag] = r['score']; o[n]['study_tilt_' + tag] = r['tilt_max']
        for n in names: print(sc, '%-10s search %s full %.2f (crash %d) tilt %.0f | study ' % (n, o[n]['search'] and round(o[n]['search'], 2), o[n]['full'], o[n]['crashes'], o[n]['full_tilt']) + ' '.join('%.2f' % o[n]['study_' + t] for t, _ in variants), flush=True)
json.dump(out, open(here / f'out/{PRE}_rerank.json', 'w'), indent=1)
