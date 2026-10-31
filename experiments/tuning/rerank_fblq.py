#!/usr/bin/env python3
"""Re-rank the best gain set of each FBL run (and references) on the 9 full variants, and on the held-out study cells (nominal + ffs 0.4)."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import core, plant_core as P
from run_job import JOBS
here = Path(__file__).parent; job = JOBS['fblq'](); (P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
base = dict(kp_xy=14.0, kv_xy=9.5, ki_xy=12.0, kp_z=11.25, kv_z=5.4, ki_z=12.0)
cand = {'preset 1': base, 'Step B A': dict(base, kp_xy=28.0, kv_xy=9.5, ki_xy=48.0)}
for m in ('cmaes',):
    for s in range(3):
        rows = [json.loads(l) for l in open(here / f'out/fblq_{m}_s{s}.jsonl') if l.strip()]; cand[f'{m} s{s}'] = min(rows, key=lambda r: r['score'])['gains']
study = [('nominal', dict()), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40))]
out = {}
with Pool(20) as pool:
    full = core.evaluate_batch(job, pool, list(cand.values()), variants=core.FULL_VARIANTS)
    for (n, g), r in zip(cand.items(), full): out[n] = dict(gains=g, full=r['score'], full_tilt=r['tilt_max'], full_per=r['per_episode']); print('%-10s full %.2f tilt %.0f' % (n, r['score'], r['tilt_max']), flush=True)
    if hasattr(P, 'STUDY_CELLS'):
        for tag, v in study:
            res = core.evaluate_batch(job, pool, list(cand.values()), variants=[v], cells=P.STUDY_CELLS)
            for (n, g), r in zip(cand.items(), res): out[n]['study_' + tag] = r['score']; print('%-10s study %-7s %.2f' % (n, tag, r['score']), flush=True)
json.dump(out, open(here / 'out/fblq_rerank.json', 'w'), indent=1)
