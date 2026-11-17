#!/usr/bin/env python3
"""Stage F1.3: forward check of fbl_model.py against frsim's FBL (noise off in both), per variant.

Same procedure and gate as checks/forward_check.py (S0, Stage 2/3): per cell the 3-D RMS of each over the frsim scoring window (scenario time > SCORE_AFTER_S up to the end of the
scenario), pass = relative 3-D RMS difference <= 0.5 % (TOL_PCT of checks/forward_check.py) and the same crash flag (here the FBL limit, tilt >= 45 deg inside the window); max position /
velocity / attitude differences are reported, not gated.
Cells: the 5 tuning scenarios (A1, A8, CIRCLE, FIG8 = tuning_cells()[0], A4 = study cell 0) and the test scenarios (A4 study cell 1, OVAL), preset-1 gains, rows
  calibration  the calibrated plant (beta = calibration, psi = 0)
  P1           the dry-run plant P1 of dryrun/loop_lib.py: thrust-law gain x0.8, motor lag 45 ms, downwash scale x1.3, plume scale x0.7 (the model gets the same beta through psi)
Variants base / rom / both are compared with frsim (FRSIM_FBL_VARIANT, simulator patch 0001); variant net (ROM off, network on) has no firmware mode, so it is only run in the model
(finite, no crash) and reported as 'no reference'.
  python3 fbl/forward_check_fbl.py [--procs 2] [--out FILE.json] [--variants base rom both net] [--scen A1 ...]"""
import argparse, json, os, sys, time
from multiprocessing import Pool
from pathlib import Path
os.environ.setdefault('OMP_NUM_THREADS', '2')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import numpy as np, torch
import reference as R
import fbl_model as F
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells, tuning_cells

TOL_PCT = 0.5
CELLS = {'A1': ('tuning', tuning_cells('A1')[0]), 'A8': ('tuning', tuning_cells('A8')[0]), 'CIRCLE': ('tuning', tuning_cells('CIRCLE')[0]), 'FIG8': ('tuning', tuning_cells('FIG8')[0]),
         'A4': ('tuning', study_cells('A4')[0]), 'A4c2': ('test', study_cells('A4')[1]), 'OVAL': ('test', study_cells('OVAL')[0])}
SCEN_OF = {'A4c2': 'A4'}
ROWS = {'calibration': {}, 'P1': {'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0, 'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'}}


def psi_of(plant):
    vals = np.array([plant.thrust_law_gain_n, plant.motor_lag_ms, plant.downwash_scale, plant.plume_amplitude_n / config.PLUME_FIT_AMPLITUDE_N])
    return np.log(vals / F.BETA_START)


def _frsim(args):
    variant, scen, cell, plant = args
    os.environ['FRSIM_FBL_VARIANT'] = variant
    r = run_episode(EpisodeConfig('FBL', scen, cell, {}, 'nominal', 0, plant=plant, noise=False, record_state=True))
    return r.state_trace


def win_rms(t, p, rp, scen_s):
    m = (t > config.SCORE_AFTER_S) & (t <= scen_s)
    return 100 * float(np.sqrt(np.mean(np.sum((p[m] - rp[m]) ** 2, -1))))


def qang(a, b):
    return np.degrees(2 * np.arccos(np.abs(np.sum(a * b, -1)).clip(0, 1)))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--procs', type=int, default=2); ap.add_argument('--out', default=str(HERE / 'forward_check_fbl.json'))
    ap.add_argument('--variants', nargs='+', default=['base', 'rom', 'both', 'net']); ap.add_argument('--scen', nargs='+', default=list(CELLS))
    a = ap.parse_args()
    names = list(ROWS); plants = {n: config.plant_with(ROWS[n]) for n in names}
    psi = torch.as_tensor(np.stack([psi_of(plants[n]) for n in names]), dtype=torch.float64); phi = torch.zeros(len(names), 6, dtype=torch.float64)
    table = []
    for var in a.variants:
        ref_ok = var != 'net'; fr = {}
        if ref_ok:
            jobs = [(var, SCEN_OF.get(s, s), CELLS[s][1], plants[n]) for s in a.scen for n in names]; t0 = time.time()
            with Pool(a.procs) as pool: res = pool.map(_frsim, jobs, chunksize=1)
            fr = {(s, n): res[i * len(names) + j] for i, s in enumerate(a.scen) for j, n in enumerate(names)}
            print('variant %s: frsim %d episodes %.0f s' % (var, len(jobs), time.time() - t0), flush=True)
        for s in a.scen:
            sc = SCEN_OF.get(s, s); ref = R.build(sc, CELLS[s][1]); t1 = time.time()
            with torch.no_grad(): out = F.rollout(ref, phi, psi, variant=var)
            print('  %s %s model %.0f s' % (var, s, time.time() - t1), flush=True)
            for j, n in enumerate(names):
                pt, vt, qt = out['x'][:, j].numpy(), out['v'][:, j].numpy(), out['q'][:, j].numpy(); tt = out['t']
                win = (tt > 0) & (tt <= ref.scenario_s); tilt_t = float(F.tilt_deg(out['q'][:, j])[win].max())
                row = dict(variant=var, scenario=s, set=CELLS[s][0], row=n, rms3d_model_cm=win_rms(tt, pt, out['ref_pos'].numpy(), ref.scenario_s), tilt_max_model_deg=tilt_t, finite=bool(np.isfinite(pt).all()))
                if ref_ok:
                    tr = fr[(s, n)]; k = min(len(tr), len(tt)); pf, vf, qf = tr[:k, 1:4], tr[:k, 4:7], tr[:k, 7:11]; rp = tr[:k, 14:17]
                    row['rms3d_frsim_cm'] = win_rms(tr[:k, 0], pf, rp, ref.scenario_s); row['rms3d_model_cm'] = win_rms(tr[:k, 0], pt[:k], rp, ref.scenario_s)
                    row['rel_diff_pct'] = 100 * abs(row['rms3d_model_cm'] - row['rms3d_frsim_cm']) / row['rms3d_frsim_cm']
                    row['max_pos_diff_mm'] = 1000 * float(np.max(np.linalg.norm(pf - pt[:k], axis=-1))); row['max_vel_diff_mm_s'] = 1000 * float(np.max(np.linalg.norm(vf - vt[:k], axis=-1)))
                    row['max_att_diff_deg'] = float(np.max(qang(qf, qt[:k])))
                    tw = tr[:k, 0]; wk = (tw > 0) & (tw <= ref.scenario_s); tilt_f = float(F.tilt_deg(torch.as_tensor(qf))[wk].max())
                    row['tilt_max_frsim_deg'] = tilt_f; row['crash_frsim'] = bool(tilt_f >= F.TILT_CRASH_DEG); row['crash_model'] = bool(tilt_t >= F.TILT_CRASH_DEG)
                    row['pass'] = bool(row['rel_diff_pct'] <= TOL_PCT and row['crash_frsim'] == row['crash_model'] and row['finite'])
                else:
                    row['pass'] = None; row['note'] = 'no frsim reference (no firmware mode for network without ROM); finite %s, crash flag %s' % (row['finite'], tilt_t >= F.TILT_CRASH_DEG)
                table.append(row)
    print('%-5s %-7s %-12s %10s %10s %8s %9s %10s %10s %6s %6s %s' % ('var', 'scen', 'row', 'frsim cm', 'model cm', 'rel %', 'dx mm', 'dv mm/s', 'dq deg', 'crF', 'crM', 'pass'))
    for r in table:
        if r['pass'] is None: print('%-5s %-7s %-12s %10s %10.4f %8s %9s %10s %10s %6s %6s %s' % (r['variant'], r['scenario'], r['row'], '-', r['rms3d_model_cm'], '-', '-', '-', '-', '-', r['tilt_max_model_deg'] >= F.TILT_CRASH_DEG, 'no reference'))
        else: print('%-5s %-7s %-12s %10.4f %10.4f %8.4f %9.4f %10.4f %10.5f %6s %6s %s' % (r['variant'], r['scenario'], r['row'], r['rms3d_frsim_cm'], r['rms3d_model_cm'], r['rel_diff_pct'], r['max_pos_diff_mm'], r['max_vel_diff_mm_s'], r['max_att_diff_deg'], r['crash_frsim'], r['crash_model'], r['pass']))
    ref_rows = [r for r in table if r['pass'] is not None]; ok = all(r['pass'] for r in ref_rows)
    print('FORWARD CHECK FBL:', 'PASS' if ok else 'FAIL', '(%d cells with a frsim reference, tolerance %.1f %% relative 3-D RMS)' % (len(ref_rows), TOL_PCT))
    json.dump(dict(tolerance_pct=TOL_PCT, passed=ok, table=table), open(a.out, 'w'), indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
