#!/usr/bin/env python3
"""Stage 8: the share of p, v, q, w in the paper task cost J_task (costs.j_task_paper, noise-free model) at the (theta_l, beta after the fit of round l) of every round of a dry run, per scenario and as the mean over the scenarios.
Usage (job_runner): DIFFCOTUNE_CONTROLLER=<C> DIFFCOTUNE_METHOD=paper DIFFCOTUNE_GRAD_MODE=cfwd python3 taskshare.py <tag> -> results/taskshare_<C>_<tag>.json"""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa
def worker(args):
    scen, cell, x = args
    import torch, costs as C
    S = L.model(); ref = L.R.build(scen, cell); z = torch.tensor(np.asarray(x)[None], dtype=torch.float64)
    with torch.no_grad():
        out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True); J, parts = C.j_task_paper(out, ref.scenario_s, parts=True)
    return float(J), {k: float(v) for k, v in parts.items()}
if __name__ == '__main__':
    tag = sys.argv[1]; C = L.CTRL; rec = json.load(open(HERE / 'results' / ('loop_%s_%s.json' % (C, tag)))); pool = L.Pool(len(L.TUNING)); out = []
    try:
        for l, r in enumerate(rec['rounds']):
            x = np.r_[np.array(r['phi']), np.array(r.get('psi_after_fit', r['psi']))]
            res = pool.pool.map(worker, [(s, c, x) for s, c in L.TUNING], chunksize=1)
            tot = {k: float(np.mean([q[1][k] for q in res])) for k in 'pvqw'}; J = sum(tot.values())
            out.append(dict(round=l, J=J, parts=tot, share={k: tot[k] / J for k in tot}, per_scenario={s: q[1] for (s, _), q in zip(L.TUNING, res)}))
            print(time.strftime('%H:%M:%S'), 'round', l, 'J %.4g' % J, {k: '%.1f%%' % (100 * v / J) for k, v in tot.items()}, flush=True)
        json.dump(out, open(HERE / 'results' / ('taskshare_%s_%s.json' % (C, tag)), 'w'), indent=1)
    finally:
        pool.close()
