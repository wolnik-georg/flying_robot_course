#!/usr/bin/env python3
"""Stage F1.4: gradient check of fbl_model.py per variant (as dryrun/stage8/gate9.py): exact forward-mode gradient (fsens_fbl.py) against a central finite difference ALONG THE GRADIENT
(h 1e-8, relative <= 1e-3, the gate of gate9.py), at preset 1 (phi = 0) and the calibration beta (psi = 0), the 5 tuning scenarios, no noise:
  J_task    mean squared 3-D position error, gradient with respect to the 6 gains (direction d = g / |g| in the gain subspace)
  J_sysId   p + v + q state error against a real trace (frsim FBL flight of the dry-run plant P1 with preset-1 gains, noise off), gradient with respect to the 4 beta (direction in the beta subspace)
Reverse mode (compiled autograd through the rollout) is evaluated for information only (--reverse), against the forward-mode gradient (a relative difference > 1e-6 marks reverse mode as unreliable).
Variant 'net' has no frsim flight; its J_sysId trace is the frsim 'both' flight (the model is only checked against itself by finite differences, which is what the gate tests).
  python3 fbl/gradient_check_fbl.py [--variants base rom both net] [--scen A1 ...] [--reverse A1] [--out FILE.json]"""
import argparse, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '2')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent))
import numpy as np
import reference as R
import fbl_model as FM
import fsens_fbl as FS
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells, tuning_cells

TUNING = [('A1', tuning_cells('A1')[0]), ('A8', tuning_cells('A8')[0]), ('CIRCLE', tuning_cells('CIRCLE')[0]), ('FIG8', tuning_cells('FIG8')[0]), ('A4', study_cells('A4')[0])]
P1 = {'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0, 'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'}
H = 1e-8; TOL = 1e-3


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def _trace(args):
    var, scen, cell = args
    os.environ['FRSIM_FBL_VARIANT'] = var
    return run_episode(EpisodeConfig('FBL', scen, cell, {}, 'nominal', 0, plant=config.plant_with(P1), noise=False, record_state=True)).state_trace


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--variants', nargs='+', default=['base', 'rom', 'both', 'net']); ap.add_argument('--scen', nargs='+', default=[s for s, _ in TUNING])
    ap.add_argument('--reverse', nargs='*', default=[], help='scenarios for the reverse-mode information check'); ap.add_argument('--out', default=str(HERE / 'gradient_check_fbl.json'))
    a = ap.parse_args()
    cells = dict(TUNING); z0 = np.zeros(FM.N_GAINS + FM.N_BETA); gi, bi = list(range(FM.N_GAINS)), list(range(FM.N_GAINS, FM.N_GAINS + FM.N_BETA)); out = dict(h=H, tol=TOL, rows=[])
    for var in a.variants:
        tv = 'both' if var == 'net' else var
        with Pool(2) as pool: traces = dict(zip(a.scen, pool.map(_trace, [(tv, s, cells[s]) for s in a.scen], chunksize=1)))
        for s in a.scen:
            ref = R.build(s, cells[s]); t0 = time.time()
            o = FS.rollout_tangents(ref, z0, gi + bi, var)
            for kind, idx, tr in (('task', gi, None), ('sysid', bi, traces[s])):
                J, gz = FS.grads_from(o, kind, ref.scenario_s, tr); g = np.zeros(len(z0)); g[idx] = gz[idx]
                d = np.zeros(len(z0)); d[idx] = g[idx] / (np.linalg.norm(g[idx]) + 1e-300)
                Jp = FS.value(ref, ref.scenario_s, kind, z0 + H * d, var, tr); Jm = FS.value(ref, ref.scenario_s, kind, z0 - H * d, var, tr)
                fd = (Jp - Jm) / (2 * H); an = float(g @ d); rel = abs(an - fd) / (abs(fd) + 1e-300)
                row = dict(variant=var, scenario=s, cost=kind, J=J, grad_norm=float(np.linalg.norm(g[idx])), directional_forward_mode=an, directional_fd=fd, rel=rel, passed=bool(rel <= TOL), grad=g[idx].tolist())
                if s in a.reverse:
                    try:
                        Jr, gr = FS.reverse_gradient(ref, ref.scenario_s, kind, z0, var, tr); row['reverse_vs_forward_rel'] = float(np.linalg.norm(gr[idx] - g[idx]) / (np.linalg.norm(g[idx]) + 1e-300)); row['reverse_unreliable'] = bool(row['reverse_vs_forward_rel'] > 1e-6)
                    except Exception as e:                      # memory or compile failure of the information check
                        row['reverse_error'] = repr(e)[:200]
                out['rows'].append(row); say('%-5s %-6s %-5s J %.4e |g| %.3e forward %.6e fd %.6e rel %.1e %s%s' % (var, s, kind, J, row['grad_norm'], an, fd, rel, 'PASS' if row['passed'] else 'FAIL', (' reverse-vs-forward %.1e' % row['reverse_vs_forward_rel']) if 'reverse_vs_forward_rel' in row else (' reverse ' + row['reverse_error'][:60] if 'reverse_error' in row else '')))
            say('  %s %s done in %.0f s' % (var, s, time.time() - t0))
            json.dump(out, open(a.out, 'w'), indent=1)
    out['passed'] = bool(all(r['passed'] for r in out['rows'])); json.dump(out, open(a.out, 'w'), indent=1); say('GRADIENT CHECK FBL:', 'PASS' if out['passed'] else 'FAIL', '(%d rows, relative <= %g along the gradient, h %g)' % (len(out['rows']), TOL, H))
    return 0 if out['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
