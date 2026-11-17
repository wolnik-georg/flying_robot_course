#!/usr/bin/env python3
"""Stage F2.4 (copy of dryrun/stage8/compare_rs.py with the F2 file names): Stage 8e T5: regression of rule RS on the controllers whose only R3 round is round 0 (S0, S2): the strict run with DIFFCOTUNE_RESTART_BEST=1 (tag ours_rs_regress) against the Stage 8d strict run (tag ours_strict).
Compared per round: phi, psi, psi_after_fit, real cost as flown, real cost over used flights, eligibility, R3 scenarios, per flight attempts (cost) and the keep-best set. Writes results/regress9r_<C>.json, exit 0 only if every difference is 0.0 and the structure is equal.
Usage: python3 compare_rs.py S0"""
import json, sys
from pathlib import Path
import numpy as np
R = Path(__file__).resolve().parents[1] / 'stage8' / 'results'; C = sys.argv[1]
old = json.load(open(R / ('loop_%s_ours_strict.json' % C))); new = json.load(open(R / ('loop_%s_ours_rs_regressF2.json' % C)))
d = {}; same = len(old['rounds']) == len(new['rounds']); struct = []
def mx(a, b): return float(np.abs(np.asarray(a, float) - np.asarray(b, float)).max()) if len(np.atleast_1d(a)) else 0.0
for k in ('phi', 'psi', 'psi_after_fit'): d[k] = max(mx(a[k], b[k]) for a, b in zip(old['rounds'], new['rounds']))
d['real_cost'] = max(abs(a['real']['cost'] - b['real']['cost']) for a, b in zip(old['rounds'], new['rounds']))
d['real_used_cost'] = max(abs((a['real_used'] or {'cost': 0})['cost'] - (b['real_used'] or {'cost': 0})['cost']) for a, b in zip(old['rounds'], new['rounds']))
d['attempt_costs'] = max(abs(x['cost'] - y['cost']) for a, b in zip(old['rounds'], new['rounds']) for fa, fb in zip(a['flights'], b['flights']) for x, y in zip(fa['attempts'], fb['attempts']))
d['tune_J_best'] = max(abs(a['tune']['J_best'] - b['tune']['J_best']) for a, b in zip(old['rounds'], new['rounds']) if 'tune' in a and 'tune' in b)
struct.append(('eligibility', [r['eligible'] for r in old['rounds']] == [r['eligible'] for r in new['rounds']]))
struct.append(('r3 scenarios', [r['r3_scenarios'] for r in old['rounds']] == [r['r3_scenarios'] for r in new['rounds']]))
struct.append(('attempts per flight', [[len(f['attempts']) for f in r['flights']] for r in old['rounds']] == [[len(f['attempts']) for f in r['flights']] for r in new['rounds']]))
struct.append(('best set', old['best']['round'] == new['best']['round'] and old['best']['phi'] == new['best']['phi']))
struct.append(('no RS action', not any(r.get('restart_from_best') for r in new['rounds']) and new.get('rs_stop') is None))
struct.append(('rounds', same))
out = dict(controller=C, max_abs_diff=d, structure={k: bool(v) for k, v in struct}, rounds_old=len(old['rounds']), rounds_new=len(new['rounds']), rs_on=bool(new.get('restart_best')),
           passed=bool(same and all(v == 0.0 for v in d.values()) and all(v for _, v in struct)), reference='results/loop_%s_ours_strict.json' % C, new='results/loop_%s_ours_rs_regressF2.json' % C)
json.dump(out, open(R / ('regressF2_%s.json' % C), 'w'), indent=1); print(json.dumps(out)); sys.exit(0 if out['passed'] else 1)
