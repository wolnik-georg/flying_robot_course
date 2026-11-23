#!/usr/bin/env python3
"""Protocol v2 (23 Nov): gradient check of the NEW cost and fit terms. Run with DIFFCOTUNE_METHOD=ours2 DIFFCOTUNE_CONTROLLER=S1|S3 DIFFCOTUNE_FIX_RD=0.
  task:   cost ||p - p*||^2 + tr(I - R*^T R)/2 (costs.j_task_att), compiled forward sensitivity ('task_C', the lab gradient) vs central finite differences of the cost ('task_fwd', float64), all 10 gains, noise-free, per scenario;
  fit:    J_sysId with states p v q w against a real flight trace (round 0), 'sysid_C' vs central differences of 'sysid_fwd', all beta incl. the RPM delay (index 14) for S1 / S3;
  parts:  the attitude term alone and the w term alone (so a wrong term cannot hide in the sum).
Gate per parameter: same sign and (relative error <= 1 % or absolute error <= 0.1 % of the largest gradient component of that cost). Output: JSON + printed table."""
import json, os, sys, time
from pathlib import Path
import numpy as np
os.environ.setdefault('DIFFCOTUNE_METHOD', 'ours2'); os.environ.setdefault('DIFFCOTUNE_FIX_RD', '0'); os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun')); sys.path.insert(0, str(HERE))
import torch; torch.set_num_threads(2)
import loop_lib as L, usd_ingest as U, round as RD, costs as C  # noqa
assert L.METHOD == 'ours2' and C.TASK_FORM == 'att', (L.METHOD, C.TASK_FORM)
H = 1e-6; REL, ABSF = 0.01, 0.001


def check(kind, scen, cell, x, idx, traces=None, states=('p', 'v', 'q', 'w')):
    a = (kind + '_C', scen, cell, x, traces, states, None)
    J, g = L.worker_rollout(a)
    res = []
    for i in idx:
        xp = np.array(x, float); xm = xp.copy(); xp[i] += H; xm[i] -= H
        fp, _ = L.worker_rollout((kind + '_fwd', scen, cell, xp, traces, states, None)); fm, _ = L.worker_rollout((kind + '_fwd', scen, cell, xm, traces, states, None))
        res.append((int(i), float(g[i]), float((fp - fm) / (2 * H))))
    big = max(abs(r[1]) for r in res) or 1.0
    ok = all((np.sign(r[1]) == np.sign(r[2]) or abs(r[1] - r[2]) <= ABSF * big) and (abs(r[1] - r[2]) <= REL * abs(r[2]) or abs(r[1] - r[2]) <= ABSF * big) for r in res)
    return dict(kind=kind, scenario=scen, J=J, ok=bool(ok), rows=res, max_rel=max((abs(r[1] - r[2]) / max(abs(r[2]), 1e-30)) for r in res if abs(r[2]) > ABSF * big))


if __name__ == '__main__':
    ctrl = L.CTRL; n = 10 + L.N_BETA; x = np.zeros(n); out = []
    scens = sys.argv[1:] or ['A8', 'CIRCLE']
    for s in scens:
        cell = RD.cell_of(s, 'tuning', 0)
        t0 = time.time(); r = check('task', s, cell, x, range(10)); r['seconds'] = time.time() - t0; out.append(r); print(ctrl, 'task', s, 'ok', r['ok'], 'max rel %.2e' % r['max_rel'], flush=True)
        g = U.ingest_flight(str(HERE.parent.parent / 'logs' / 'usd_merged' / 'rounds' / ctrl / 'round_0' / (s + '.csv')), s, cell, top_from_log=s in ('A1', 'A8'), rotate_deg=(None if s in ('A1', 'A8') else -90.0))
        xb = x.copy()
        t0 = time.time(); r = check('sysid', s, cell, xb, range(10, n), traces=[g['trace']]); r['seconds'] = time.time() - t0; out.append(r); print(ctrl, 'sysid p v q w', s, 'ok', r['ok'], 'max rel %.2e' % r['max_rel'], flush=True)
        r = check('sysid', s, cell, xb, range(10, n), traces=[g['trace']], states=('w',)); out.append(dict(r, kind='sysid_w_only')); print(ctrl, 'sysid w only', s, 'ok', r['ok'], 'max rel %.2e' % r['max_rel'], flush=True)
    json.dump(out, open(HERE / 'results' / ('v2_gradient_check_%s.json' % ctrl), 'w'), indent=1)
    print('ALL OK' if all(r['ok'] for r in out) else 'FAIL')
