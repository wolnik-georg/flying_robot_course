#!/usr/bin/env python3
"""Option 1: rank gradient-tuned S0 candidates (all out/diff_<TAG>_seed*.json) in the REAL firmware plant, solo (no downwash, option A).
Selection stage: tuning cells (NS2-grid cells, none a study cell; TUNING_CELLS_V2 A1 + A4) x the 9 full variants; ranked by (crashes, full score).
Final stage for the top 3 (+ baseline + CMA-ES A8 pick): study cells, 4 variants, solo and with downwash. System python3 + CFFIRMWARE_DIR."""
import glob, json, os, sys
from multiprocessing import Pool
from pathlib import Path
here = Path(__file__).parent.parent; sys.path.insert(0, str(here))
import core, plant_core as P
from run_job import S0_PARAMS, _s0_map
(P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
TAG = os.environ.get('TAG', 's0x')
cand = {'baseline': {k: v[0] for k, v in S0_PARAMS.items()}}
for f in sorted(glob.glob(str(here / f'out/diff_{TAG}_seed*.json')), key=lambda p: int(p.split('seed')[-1].split('.')[0])):
    d = json.load(open(f)); cand[f"{TAG} s{d['seed']}"] = d['gains']
solo = dict(J='omar', tree=True, indi=0, ns2=0, plume=None, dws=0.0); down = dict(J='omar', tree=True, indi=0, ns2=0)
sel_job = core.Job('sel', S0_PARAMS, P.TUNING_CELLS_V2, extra=solo, tree_map=_s0_map)
out = {}
with Pool(20) as pool:
    names = list(cand); gl = [cand[n] for n in names]
    res = core.evaluate_batch(sel_job, pool, gl, variants=core.FULL_VARIANTS)
    for n, g, r in zip(names, gl, res): out[n] = dict(gains=g, sel_score=r['score'], sel_crashes=sum(x >= 100 for x in r['per_episode']), sel_tilt=r['tilt_max']); print('%-12s select: full %.2f (crash %d) tilt %.0f' % (n, r['score'], out[n]['sel_crashes'], r['tilt_max']), flush=True)
    top = sorted([n for n in names if n != 'baseline'], key=lambda n: (out[n]['sel_crashes'], out[n]['sel_score']))[:3]
    rr = json.load(open(here / 'out/s0q_rerank.json')); final = {'baseline': cand['baseline'], **{n: cand[n] for n in top}, 'cma-es s1 (A8)': rr['A8']['cmaes s1']['gains']}
    variants = [('nominal', dict()), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40)), ('lag45', dict(lag=45))]
    fn = list(final); fg = [final[n] for n in fn]
    for tag, ex in (('solo', solo), ('downwash', down)):
        job = core.Job('fin', S0_PARAMS, P.STUDY_CELLS, extra=ex, tree_map=_s0_map)
        for vt, v in variants:
            for n, r in zip(fn, core.evaluate_batch(job, pool, fg, variants=[v])): out.setdefault(n, dict(gains=final[n]))[f'{tag}_{vt}'] = r['score']
        for n, r in zip(fn, core.evaluate_batch(job, pool, fg, variants=core.FULL_VARIANTS)): out[n][f'{tag}_full'] = r['score']; out[n][f'{tag}_full_crashes'] = sum(x >= 100 for x in r['per_episode'])
    for n in fn: print('%-16s study solo: %s | full %.2f (crash %d) | downwash: %s | full %.2f (crash %d)' % (n, ' '.join('%.2f' % out[n][f'solo_{t}'] for t, _ in variants), out[n]['solo_full'], out[n]['solo_full_crashes'], ' '.join('%.2f' % out[n][f'downwash_{t}'] for t, _ in variants), out[n]['downwash_full'], out[n]['downwash_full_crashes']), flush=True)
out['_top3'] = top
json.dump(out, open(here / f'out/diff_{TAG}_rank.json', 'w'), indent=1)
