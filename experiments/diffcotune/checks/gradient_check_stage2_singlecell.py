#!/usr/bin/env python3
"""Gradient check: autograd vs central finite differences (float64) for all 14 parameters (10 gains theta, 4 beta, as
log-factors) of J_task and J_sysId, on 3 cells (A1, A8, CIRCLE).

J_sysId is evaluated against a frsim trace (noise off) of S0 with default gains on a SHIFTED plant (thrust-law gain x 0.8,
motor lag 45 ms, downwash scale x 1.3, plume scale x 0.7), at theta = defaults and beta = calibration.
Pass: relative error <= 1 % for every parameter (our own choice, stated as such). A parameter whose gradient is zero (|g| <
1e-9 of the largest |g| of that cost and cell) or not finite is flagged.

  python3 checks/gradient_check.py [--h 1e-6] [--scen A1 A8 CIRCLE] [--out FILE.json]
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
TOL = 0.01
SHIFT = {'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0, 'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'}


def costs_of(ref, target, z, grad):
    phi, psi = z[:, :10], z[:, 10:]
    if grad:
        out = S.rollout(ref, phi, psi, compile=True)
    else:
        with torch.no_grad():                     # eager: the compiled B = 28 inference graph hits an inductor C++ codegen bug (torch 2.5.1)
            out = S.rollout(ref, phi, psi)
    return C.j_task(out, ref.scenario_s), C.j_sysid(out, ingest.on_grid(target, out['t']), ref.scenario_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--h', type=float, default=1e-6)
    ap.add_argument('--scen', nargs='+', default=['A1', 'A8', 'CIRCLE'])
    ap.add_argument('--out', default=str(HERE / 'gradient_check.json'))
    a = ap.parse_args()
    shifted = config.plant_with(SHIFT)
    result, all_ok = [], True
    for s in a.scen:
        cell = study_cells(s)[0]; ref = R.build(s, cell)
        tr, _, _ = _frsim((s, cell, None, shifted))
        tgt = ingest.from_trace(tr)
        t0 = time.time()
        z = torch.zeros(1, 14, dtype=torch.float64, requires_grad=True)
        jt, js = costs_of(ref, tgt, z, True)
        gt = torch.autograd.grad(jt.sum(), z, retain_graph=True)[0][0].numpy()
        gs = torch.autograd.grad(js.sum(), z)[0][0].numpy()
        t_ad = time.time() - t0
        eye = torch.eye(14, dtype=torch.float64) * a.h
        zz = torch.cat([eye, -eye])
        t0 = time.time()
        jtp, jsp = costs_of(ref, tgt, zz, False)
        t_fd = time.time() - t0
        ft = ((jtp[:14] - jtp[14:]) / (2 * a.h)).numpy(); fs = ((jsp[:14] - jsp[14:]) / (2 * a.h)).numpy()
        for cname, g, f, J in (('J_task', gt, ft, float(jt)), ('J_sysId', gs, fs, float(js))):
            big = max(np.abs(f).max(), np.abs(g).max())
            rows = []
            for i, n in enumerate(NAMES):
                flag = ''
                if not (np.isfinite(g[i]) and np.isfinite(f[i])):
                    flag = 'NOT FINITE'
                elif max(abs(g[i]), abs(f[i])) < 1e-9 * big:
                    flag = 'ZERO'
                rel = float(abs(g[i] - f[i]) / max(abs(f[i]), 1e-300)) if flag == '' else None
                rows.append(dict(param=n, autograd=float(g[i]), fd=float(f[i]), rel_err=rel, flag=flag))
            worst = max((r['rel_err'] for r in rows if r['rel_err'] is not None), default=0.0)
            ok = worst <= TOL and not any(r['flag'] == 'NOT FINITE' for r in rows)
            all_ok &= ok
            result.append(dict(scenario=s, cost=cname, value=J, worst_rel_err=worst, passed=ok, rows=rows, seconds_autograd=t_ad, seconds_fd_batch=t_fd))
            print('== %s %s  J = %.6e   worst relative error %.3e  %s   (autograd %.0f s incl. one rollout with graph, 28-row FD batch %.0f s)' % (s, cname, J, worst, 'PASS' if ok else 'FAIL', t_ad, t_fd), flush=True)
            for r in rows:
                print('   %-15s autograd %+.6e  fd %+.6e  rel %s %s' % (r['param'], r['autograd'], r['fd'], ('%.2e' % r['rel_err']) if r['rel_err'] is not None else '-', r['flag']))
    print('GRADIENT CHECK:', 'PASS' if all_ok else 'FAIL', '(tolerance %.0f %% per parameter, h = %g in log-parameters)' % (100 * TOL, a.h))
    json.dump(dict(h=a.h, tolerance=TOL, passed=all_ok, results=result), open(a.out, 'w'), indent=1)
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
