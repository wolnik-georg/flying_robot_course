#!/usr/bin/env python3
"""Protocol v2.1: J_sysId parts (p v q w) of the round 0 flights before and after the beta fit of a v2 desk step (mean over the 5 tuning flights, the quantity the fit minimises), from v2/<C>/round_0/report.json (psi_after_fit).
Run with DIFFCOTUNE_METHOD=ours2 DIFFCOTUNE_CONTROLLER=<C>. Output lab/results/v2_fit_parts_<C>.json."""
import json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('DIFFCOTUNE_METHOD', 'ours21'); os.environ.setdefault('DIFFCOTUNE_FIX_RD', '0'); os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun')); sys.path.insert(0, str(HERE))
import torch; torch.set_num_threads(2)
import loop_lib as L, usd_ingest as U, round as RD, costs as C, ingest  # noqa
ctrl = L.CTRL; folder = HERE.parent.parent / 'logs' / 'usd_merged' / 'rounds' / ctrl / 'round_0'
rep = json.load(open(HERE / 'v21' / ctrl / 'round_0' / 'report.json')); psi_after = np.array(rep['psi_after_fit'])
USED = {f['scenario']: f['file'] for f in rep['flights'] if f.get('used')}
rot = dict(A1=None, A8=None, A4=-90.0, CIRCLE=-90.0, FIG8=-90.0); S = L.model(); res = {}
for tag, psi in (('before', np.zeros(L.N_BETA)), ('after', psi_after)):
    acc = {k: [] for k in 'pvqw'}; per = {}
    for s, _ in L.TUNING:
        g = U.ingest_flight(str(folder / USED[s]), s, RD.cell_of(s, 'tuning', 0), top_from_log=s in ('A1', 'A8'), rotate_deg=rot[s]); ref = g['ref']
        with torch.no_grad():
            out = S.rollout(ref, torch.zeros(1, 10, dtype=torch.float64), torch.tensor(psi[None], dtype=torch.float64), compile=False)
            J, parts = C.j_sysid(out, ingest.on_grid(ingest.from_trace(g['trace']), out['t']), ref.scenario_s, parts=True)
        per[s] = {k: float(v[0]) for k, v in parts.items()}
        for k in 'pvqw': acc[k].append(per[s][k])
    res[tag] = dict(mean={k: float(np.mean(v)) for k, v in acc.items()}, per_scenario=per)
    res[tag]['total'] = sum(res[tag]['mean'].values())
res['psi_after_fit'] = psi_after.tolist(); res['beta_names'] = L.BETA_NAMES + (['rpm_delay'] if L.N_BETA == 5 else []); res['beta_after'] = dict(zip(res['beta_names'], np.exp(psi_after).tolist()))
json.dump(res, open(HERE / 'results' / ('v21_fit_parts_%s.json' % ctrl), 'w'), indent=1)
for tag in ('before', 'after'): print(ctrl, tag, {k: '%.4g' % v for k, v in res[tag]['mean'].items()}, 'total %.4g' % res[tag]['total'])
print(ctrl, 'psi after fit (log factors)', np.round(psi_after, 3).tolist(), 'factors', {k: round(v, 3) for k, v in res['beta_after'].items()})
