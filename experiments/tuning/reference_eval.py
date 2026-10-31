#!/usr/bin/env python3
"""Score reference gain sets with the job's objective (search variants and the full 9 variants): preset 1, Step B hand candidates, the authors' preset 0."""
import json, sys
from multiprocessing import Pool
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))
import core
from run_job import JOBS
job = JOBS['fbl'](); (core.P.OUT / "stock_inner_traces").mkdir(parents=True, exist_ok=True)
base = dict(kp_xy=14.0, kv_xy=9.5, ki_xy=12.0, kp_z=11.25, kv_z=5.4, ki_z=12.0)
refs = {'preset 1': base, 'Step B candidate A (28/9.5/48 xy)': dict(base, kp_xy=28.0, kv_xy=9.5, ki_xy=48.0), 'authors preset 0': dict(kp_xy=56.0, kv_xy=38.0, ki_xy=48.0, kp_z=45.0, kv_z=21.6, ki_z=3.0)}
out = {}
with Pool(20) as pool:
    for variants, tag in ((core.SEARCH_VARIANTS, 'search'), (core.FULL_VARIANTS, 'full')):
        res = core.evaluate_batch(job, pool, list(refs.values()), variants=variants)
        for (n, g), r in zip(refs.items(), res): out.setdefault(n, {})[tag] = r['score']; print('%-36s %-6s score %.2f  tilt max %.0f' % (n, tag, r['score'], r['tilt_max']), flush=True)
json.dump(out, open(Path(__file__).parent / 'out/reference_scores_calibrated.json', 'w'), indent=1)
