#!/usr/bin/env python3
"""Forward check: the differentiable S0 model vs frsim (noise off in both), the gate for Stage 3.

Cells: S0 x {A1, A4, A8, CIRCLE, FIG8, OVAL} (first study cell of each) x 5 parameter rows:
  default      default gains, calibrated plant
  gains1/2     random gain sets within +-30 % of the defaults (uniform, seeds 1 and 2), calibrated plant
  shift1       default gains, thrust-law gain x 0.8 and motor lag 45 ms
  shift2       default gains, downwash scale x 1.3 and plume scale x 0.7
Per cell: 3-D RMS of each model over the frsim scoring window (scenario time > 2 s up to the end of the scenario),
relative difference, max position difference over the whole flight (mm), max attitude difference (deg), crash flag
(tilt >= 70 deg inside the scoring window). Pass: relative 3-D RMS difference <= 0.5 % in every cell (max position
difference is reported, not gated).

  python3 checks/forward_check.py [--procs 4] [--out FILE.json] [--scen A1 ...]   (system python3; needs the firmware bindings of frsim)
"""
import argparse
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import reference as R  # noqa: E402
import s0_model as S  # noqa: E402
from frsim import config  # noqa: E402
from frsim.episode import EpisodeConfig, run_episode  # noqa: E402
from frsim.scenarios import study_cells  # noqa: E402

SCENARIOS = ['A1', 'A4', 'A8', 'CIRCLE', 'FIG8', 'OVAL']
TOL_PCT = 0.5


def rows_spec():
    """name -> (gain factors (10,), plant overrides dict)."""
    out = {'default': (np.ones(10), {})}
    for k, seed in ((1, 1), (2, 2)):
        out['gains%d' % k] = (np.random.default_rng(seed).uniform(0.7, 1.3, 10), {})
    out['shift1'] = (np.ones(10), {'thrust_gain': 'x0.8', 'motor_lag_ms': 45.0})
    out['shift2'] = (np.ones(10), {'downwash_scale': 'x1.3', 'plume_scale': 'x0.7'})
    return out


def psi_of(plant):
    """log-factors of beta of a Plant relative to the calibration (the model's parametrisation)."""
    vals = np.array([plant.thrust_law_gain_n, plant.motor_lag_ms, plant.downwash_scale, plant.plume_amplitude_n / config.PLUME_FIT_AMPLITUDE_N])
    return np.log(vals / S.BETA_START)


def _frsim(args):
    scen, cell, gains, plant = args
    r = run_episode(EpisodeConfig('S0', scen, cell, gains, 'nominal', 0, plant=plant, noise=False, record_state=True))
    return r.state_trace, r.stable, r.tilt_max_deg


def window_rms3d(t, pos, ref, scenario_s):
    """3-D RMS (cm) of |pos - ref| over the scoring window of frsim (scenario time > 2 s ... end of scenario)."""
    m = (t > config.SCORE_AFTER_S) & (t <= scenario_s)
    return 100 * float(np.sqrt(np.mean(np.sum((pos[m] - ref[m]) ** 2, axis=-1))))


def quat_angle_deg(a, b):
    d = np.abs(np.sum(a * b, axis=-1)).clip(0, 1)
    return np.degrees(2 * np.arccos(d))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--out', default=str(HERE / 'forward_check.json'))
    ap.add_argument('--scen', nargs='+', default=SCENARIOS)
    a = ap.parse_args()
    spec = rows_spec(); names = list(spec)
    plants = {n: config.plant_with(spec[n][1]) for n in names}
    gains = {n: {k: float(v * f) for k, v, f in zip(S.GAIN_NAMES, S.GAIN_DEFAULT, spec[n][0])} for n in names}
    cells = {s: study_cells(s)[0] for s in a.scen}
    t0 = time.time()
    jobs = [(s, cells[s], gains[n], plants[n]) for s in a.scen for n in names]
    with Pool(a.procs) as pool:
        fr = pool.map(_frsim, jobs, chunksize=1)
    print('frsim: %d episodes in %.0f s' % (len(jobs), time.time() - t0), flush=True)
    phi = torch.as_tensor(np.log(np.stack([spec[n][0] for n in names])), dtype=torch.float64)
    psi = torch.as_tensor(np.stack([psi_of(plants[n]) for n in names]), dtype=torch.float64)
    table = []
    for si, s in enumerate(a.scen):
        ref = R.build(s, cells[s]); t1 = time.time()
        with torch.no_grad():
            out = S.rollout(ref, phi, psi)
        dt_torch = time.time() - t1
        for ni, n in enumerate(names):
            tr, stable_fr, tilt_fr = fr[si * len(names) + ni]
            k = min(len(tr), len(out['t']))
            pf, qf = tr[:k, 1:4], tr[:k, 7:11]; pr_ = tr[:k, 14:17]
            pt, qt = out['x'][:k, ni].numpy(), out['q'][:k, ni].numpy()
            tt = tr[:k, 0]
            rms_f, rms_t = window_rms3d(tt, pf, pr_, ref.scenario_s), window_rms3d(tt, pt, pr_, ref.scenario_s)
            win = (tt > 0) & (tt <= ref.scenario_s)
            tilt_t = float(S.tilt_deg(out['q'][:k, ni])[win].max())
            tilt_f = float(S.tilt_deg(torch.as_tensor(qf))[win].max())
            row = dict(scenario=s, row=n, rms3d_frsim_cm=rms_f, rms3d_model_cm=rms_t, rel_diff_pct=100 * abs(rms_t - rms_f) / rms_f,
                       max_pos_diff_mm=1000 * float(np.max(np.linalg.norm(pf - pt, axis=-1))),
                       max_att_diff_deg=float(np.max(quat_angle_deg(qf, qt))), crash_frsim=bool(tilt_f >= S.TILT_CRASH_DEG), crash_model=bool(tilt_t >= S.TILT_CRASH_DEG),
                       tilt_max_frsim_deg=tilt_f, tilt_max_model_deg=tilt_t, rows_compared=k, model_seconds_batch5=dt_torch)
            row['pass'] = bool(row['rel_diff_pct'] <= TOL_PCT and row['crash_frsim'] == row['crash_model'])
            table.append(row)
        print(s, 'model rollout (5 rows batched, float64 CPU) %.1f s' % dt_torch, flush=True)
    print('%-7s %-8s %9s %9s %8s %10s %10s %6s %6s %s' % ('scen', 'row', 'frsim cm', 'model cm', 'rel %', 'max dx mm', 'max dq deg', 'crF', 'crM', 'pass'))
    for r in table:
        print('%-7s %-8s %9.4f %9.4f %8.4f %10.3f %10.4f %6s %6s %s' % (r['scenario'], r['row'], r['rms3d_frsim_cm'], r['rms3d_model_cm'], r['rel_diff_pct'], r['max_pos_diff_mm'], r['max_att_diff_deg'], r['crash_frsim'], r['crash_model'], 'yes' if r['pass'] else 'NO'))
    ok = all(r['pass'] for r in table)
    print('FORWARD CHECK:', 'PASS' if ok else 'FAIL', '(%d cells, tolerance %.1f %% relative 3-D RMS)' % (len(table), TOL_PCT))
    json.dump(dict(tolerance_pct=TOL_PCT, passed=ok, cells=cells, table=table), open(a.out, 'w'), indent=1)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
