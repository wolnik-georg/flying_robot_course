#!/usr/bin/env python3
"""Check the gradient-tuned S0 gains in the REAL firmware plant (not the differentiable copy): baseline vs diff seeds vs the black-box (CMA-ES s0q) picks.
Study cells (A1 dz .45, A4 offsets .075/.125, A8) solo (no downwash) and with downwash; variants nominal / ffs 0.4 / md 40 / lag 45 (+ full variants for crash counts).
System python3 + CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build."""
import glob, json, os, sys
from multiprocessing import Pool
from pathlib import Path
here = Path(__file__).parent.parent; sys.path.insert(0, str(here))
import core, plant_core as P
from run_job import S0_PARAMS, _s0_map
(P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
cand = {'baseline': {k: v[0] for k, v in S0_PARAMS.items()}}
for f in sorted(glob.glob(str(here / f"out/diff_{os.environ.get('TAG', 's0')}_seed*.json"))): cand[os.environ.get('TAG', 's0') + ' s%s' % json.load(open(f))['seed']] = json.load(open(f))['gains']
rr = json.load(open(here / 'out/s0q_rerank.json'))
for sc, pick in (('A1', 'cmaes s1'), ('A4', 'cmaes s1'), ('A8', 'cmaes s1')): cand['cma-es %s (%s)' % (pick, sc)] = rr[sc][pick]['gains']
variants = [('nominal', dict()), ('ffs0.4', dict(ffs=0.4)), ('md40', dict(md=40)), ('lag45', dict(lag=45))]
out = {}
with Pool(20) as pool:
    for tag, extra in (('solo', dict(plume=None, dws=0.0)), ('downwash', dict())):
        job = core.Job('eval', S0_PARAMS, P.STUDY_CELLS, extra=dict(J='omar', tree=True, indi=0, ns2=0, **extra), tree_map=_s0_map)
        names = list(cand); gl = [cand[n] for n in names]
        for vt, v in variants:
            res = core.evaluate_batch(job, pool, gl, variants=[v])
            for n, r in zip(names, res): out.setdefault(n, {})[f'{tag}_{vt}'] = dict(score=r['score'], per_cell=r['per_episode'], tilt=r['tilt_max'])
        full = core.evaluate_batch(job, pool, gl, variants=core.FULL_VARIANTS)
        for n, r in zip(names, full): out[n][f'{tag}_full'] = dict(score=r['score'], crashes=sum(x >= 100 for x in r['per_episode']), tilt=r['tilt_max'])
for n in out:
    print('%-22s' % n + ' | '.join('%s: ' % t + ' '.join('%.2f' % out[n][f'{t}_{v}']['score'] for v, _ in variants) + ' full %.2f (crash %d)' % (out[n][f'{t}_full']['score'], out[n][f'{t}_full']['crashes']) for t in ('solo', 'downwash')), flush=True)
json.dump(out, open(here / f"out/diff_{os.environ.get('TAG', 's0')}_eval.json", 'w'), indent=1)
