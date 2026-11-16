#!/usr/bin/env python3
"""Gradient check (Stage 3 version, gate on the cost the optimiser uses): autograd vs central finite differences (float64) for all
14 parameters (10 gains theta, 4 beta, as log-factors) of the COMBINED costs J_task and J_sysId = mean over the 4 tuning scenarios
A1, A8, CIRCLE, FIG8 (first study cell of each). Gate (decision of 11 Nov evening) per parameter and per combined cost: same sign and
(relative error <= 1 % or absolute error <= 0.1 % of the largest gradient component of that cost). The single-scenario tables are
reported, not gated. (The Stage 2 single-cell version is kept as gradient_check_stage2_singlecell.py.)

J_sysId is evaluated against a frsim trace (noise off) of S0 with default gains on a SHIFTED plant (thrust-law gain x 0.8,
motor lag 45 ms, downwash scale x 1.3, plume scale x 0.7), at theta = defaults and beta = calibration.
Pass: relative error <= 1 % for every parameter (our own choice, stated as such). A parameter whose gradient is zero (|g| <
1e-9 of the largest |g| of that cost and cell) or not finite is flagged.

  python3 checks/gradient_check.py [--h 1e-6] [--scen A1 A8 CIRCLE FIG8] [--out FILE.json]
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import costs as C  # noqa: E402
import ingest  # noqa: E402
import reference as R  # noqa: E402
import s0_model as S  # noqa: E402
from forward_check import _frsim  # noqa: E402
from frsim import config  # noqa: E402
from frsim.scenarios import study_cells  # noqa: E402

NAMES = list(S.GAIN_NAMES) + list(S.BETA_NAMES)
REL_TOL, ABS_FRAC = 0.01, 0.001
SHIFT = {'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0, 'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'}


def costs_of(ref, target, z, grad):
    phi, psi = z[:, :10], z[:, 10:]
    if grad:
        out = S.rollout(ref, phi, psi, compile=True)
    else:
        with torch.no_grad():                     # eager: the compiled B = 28 inference graph hits an inductor C++ codegen bug (torch 2.5.1)
            out = S.rollout(ref, phi, psi)
    return C.j_task(out, ref.scenario_s), C.j_sysid(out, ingest.on_grid(target, out['t']), ref.scenario_s)


def judge(g, f):
    """Per-parameter rows (autograd g, finite difference f) with the gate of this stage."""
    big = max(np.abs(f).max(), np.abs(g).max()); rows = []
    for i, n in enumerate(NAMES):
        flag = 'NOT FINITE' if not (np.isfinite(g[i]) and np.isfinite(f[i])) else ('ZERO' if max(abs(g[i]), abs(f[i])) < 1e-9 * big else '')
        err = abs(g[i] - f[i]); rel = float(err / abs(f[i])) if flag == '' and abs(f[i]) > 0 else None
        same_sign = bool(np.sign(g[i]) == np.sign(f[i])) or flag == 'ZERO'
        ok = flag != 'NOT FINITE' and same_sign and (flag == 'ZERO' or (rel is not None and rel <= REL_TOL) or err <= ABS_FRAC * big)
        rows.append(dict(param=n, autograd=float(g[i]), fd=float(f[i]), rel_err=rel, abs_err_over_max=float(err / big), same_sign=same_sign, flag=flag, passed=bool(ok)))
    return rows


def show(title, rows, ok_text):
    print('== %s: %s' % (title, ok_text))
    for r in rows:
        print('   %-15s autograd %+.6e  fd %+.6e  rel %-9s abs/max %.1e %s %s' % (r['param'], r['autograd'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', r['abs_err_over_max'], r['flag'], '' if r['passed'] else 'FAIL'), flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--h', type=float, default=1e-6)
    ap.add_argument('--scen', nargs='+', default=['A1', 'A8', 'CIRCLE', 'FIG8'])
    ap.add_argument('--out', default=str(HERE / 'gradient_check.json'))
    a = ap.parse_args()
    shifted = config.plant_with(SHIFT)
    per = {}
    for s in a.scen:
        cell = study_cells(s)[0]; ref = R.build(s, cell)
        tr, _, _ = _frsim((s, cell, None, shifted))
        tgt = ingest.from_trace(tr)
        t0 = time.time()
        z = torch.zeros(1, 14, dtype=torch.float64, requires_grad=True)
        jt, js = costs_of(ref, tgt, z, True)
        gt = torch.autograd.grad(jt.sum(), z, retain_graph=True)[0][0].numpy(); gs = torch.autograd.grad(js.sum(), z)[0][0].numpy()
        eye = torch.eye(14, dtype=torch.float64) * a.h
        jtp, jsp = costs_of(ref, tgt, torch.cat([eye, -eye]), False)
        ft = ((jtp[:14] - jtp[14:]) / (2 * a.h)).numpy(); fs = ((jsp[:14] - jsp[14:]) / (2 * a.h)).numpy()
        per[s] = dict(J_task=float(jt), J_sysId=float(js), g_task=gt, f_task=ft, g_sysid=gs, f_sysid=fs)
        print('%s done in %.0f s' % (s, time.time() - t0), flush=True)
    result = dict(h=a.h, rel_tol=REL_TOL, abs_frac=ABS_FRAC, scenarios=a.scen, combined={}, single={})
    all_ok = True
    for cname, gk, fk, jk in (('J_task', 'g_task', 'f_task', 'J_task'), ('J_sysId', 'g_sysid', 'f_sysid', 'J_sysId')):
        g = np.mean([per[s][gk] for s in a.scen], axis=0); f = np.mean([per[s][fk] for s in a.scen], axis=0)
        rows = judge(g, f); ok = all(r['passed'] for r in rows); all_ok &= ok
        J = float(np.mean([per[s][jk] for s in a.scen]))
        result['combined'][cname] = dict(value=J, passed=ok, rows=rows)
        show('COMBINED %s over %s (mean), J = %.6e' % (cname, '/'.join(a.scen), J), rows, 'PASS' if ok else 'FAIL')
    for s in a.scen:                                  # single-scenario tables (reported, not gated)
        result['single'][s] = {}
        for cname, gk, fk, jk in (('J_task', 'g_task', 'f_task', 'J_task'), ('J_sysId', 'g_sysid', 'f_sysid', 'J_sysId')):
            rows = judge(per[s][gk], per[s][fk]); result['single'][s][cname] = dict(value=per[s][jk], rows=rows)
            show('single %s %s, J = %.6e (not gated; rows failing the same test: %d)' % (s, cname, per[s][jk], sum(not r['passed'] for r in rows)), rows, '')
    print('COMBINED GRADIENT GATE:', 'PASS' if all_ok else 'FAIL', '(same sign and (rel <= %.0f %% or abs <= %.1f %% of the largest component); h = %g in log-parameters)' % (100 * REL_TOL, 100 * ABS_FRAC, a.h))
    json.dump(result, open(a.out, 'w'), indent=1)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
