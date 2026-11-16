#!/usr/bin/env python3
"""Stage 3d forward check WITH sensor noise: differentiable S0 model (frsim's own noise sequence, same seed) vs frsim with noise on.

Cells: gains {default, round 1, round 3 of Stage 3b loop (d)} x P1 beta (true) x the 4 tuning scenarios x seeds {1000, 1001}.
Case "identical noise sequence" applies (s0_model.frsim_noise reproduces sensors.Sensors draw for draw). Gate (our choice, stated):
relative 3-D RMS difference <= 0.5 % per cell, and the floor failure of the round-3 gains is reproduced (z mean of the scored part
< 50 % of the reference z mean in frsim <=> in the model). Eager float64 CPU, the three gain sets as rows of one rollout.
"""
import json, sys, time
from multiprocessing import get_context
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import reference as R  # noqa
import s0_model as S   # noqa
import loop_lib as L   # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa

SEEDS = (1000, 1001)
TOL = 0.5


def load_phis():
    d = json.load(open(HERE.parent / 'stage3b/results/full_P1_cotune.json'))
    return {'default': np.zeros(10), 'round1': np.array(d['rounds'][1]['phi']), 'round3': np.array(d['rounds'][3]['phi'])}


def stats(t, pos, ref, scen_s):
    m = (t > config.SCORE_AFTER_S) & (t <= scen_s)
    e = pos[m] - ref[m]
    return 100 * float(np.sqrt((e * e).sum(-1).mean())), float(pos[m][:, 2].mean()), float(ref[m][:, 2].mean())


def job(args):
    scen, cell, seed = args
    import torch
    torch.set_num_threads(2)
    phis = load_phis(); names = list(phis)
    plant = config.plant_with(L.P1['overrides'])
    ref = R.build(scen, cell)
    fr = {}
    for n in names:
        g = L.gains_of(phis[n])
        r = run_episode(EpisodeConfig('S0', scen, cell, g, 'nominal', seed, plant=plant, noise=True, record_state=True))
        fr[n] = r.state_trace
    phi = torch.tensor(np.stack([phis[n] for n in names])); psi = torch.tensor(np.tile(L.PSI_TRUE, (len(names), 1)))
    nz = S.frsim_noise(seed, ref.n_ticks)
    with torch.no_grad():
        out = S.rollout(ref, phi, psi, noise=nz)
    rows = []
    for i, n in enumerate(names):
        tr = fr[n]; k = min(len(tr), len(out['t']))
        t = tr[:k, 0]; refp = tr[:k, 14:17]
        rf, zf, zr = stats(t, tr[:k, 1:4], refp, ref.scenario_s)
        rm, zm, _ = stats(t, out['x'][:k, i].numpy(), refp, ref.scenario_s)
        rows.append(dict(scenario=scen, seed=seed, gains=n, rms_frsim_cm=rf, rms_model_cm=rm, rel_pct=100 * abs(rm - rf) / rf,
                         zmean_frsim=zf, zmean_model=zm, zmean_ref=zr, floor_frsim=bool(zf < 0.5 * zr), floor_model=bool(zm < 0.5 * zr)))
    return rows


if __name__ == '__main__':
    t0 = time.time()
    jobs = [(s, c, sd) for s, c in L.TUNING for sd in SEEDS]
    with get_context('spawn').Pool(8) as p:
        res = p.map(job, jobs, chunksize=1)
    table = [r for rr in res for r in rr]
    print('%-7s %5s %-8s %9s %9s %8s %7s %7s %7s %s %s' % ('scen', 'seed', 'gains', 'frsim cm', 'model cm', 'rel %', 'zF m', 'zM m', 'zRef', 'flF', 'flM'))
    for r in table:
        print('%-7s %5d %-8s %9.3f %9.3f %8.3f %7.3f %7.3f %7.3f %s %s' % (r['scenario'], r['seed'], r['gains'], r['rms_frsim_cm'], r['rms_model_cm'], r['rel_pct'], r['zmean_frsim'], r['zmean_model'], r['zmean_ref'], r['floor_frsim'], r['floor_model']))
    ok_rel = all(r['rel_pct'] <= TOL for r in table); ok_floor = all(r['floor_frsim'] == r['floor_model'] for r in table)
    print('rel gate (<= %.1f %%): %s ; floor failure reproduced in every cell: %s ; max rel %.3f %% ; %.0f s' % (TOL, ok_rel, ok_floor, max(r['rel_pct'] for r in table), time.time() - t0))
    json.dump(dict(case='identical noise sequence', tol_pct=TOL, pass_rel=ok_rel, pass_floor=ok_floor, table=table), open(HERE / 'noise_forward_check.json', 'w'), indent=1)
