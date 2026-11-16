#!/usr/bin/env python3
"""Stage 3d forward check with noise, NEW GATE (user decision 13 Nov, option 1 with addition; our choice, written into the spec):
 * a cell is "stable" when frsim itself moves < 0.5 % in 3-D RMS under a 1e-6 relative change of the thrust-law gain (same gains, same seed);
   there the model must agree with frsim within 0.5 %;
 * in the other cells (frsim is chaotic) the model must show the failure too: floor rule failed, or 3-D RMS > 3 x the round-1 value of the same
   scenario and seed (frsim round-1 value).
Inputs: noise_forward_check.json (model vs frsim, 24 cells) and a new frsim sensitivity run over all 24 cells."""
import json, sys
from multiprocessing import get_context
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import loop_lib as L, reference as R  # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa
import noise_forward_check as N  # noqa


def job(a):
    scen, seed, name, eps = a
    phi = N.load_phis()[name]; cell = dict(L.TUNING)[scen]
    plant = config.plant_with(dict(L.P1['overrides'], thrust_gain='x%.12f' % (0.8 * (1 + eps))))
    r = run_episode(EpisodeConfig('S0', scen, cell, L.gains_of(phi), 'nominal', seed, plant=plant, noise=True, record_state=True))
    tr = r.state_trace; return N.stats(tr[:, 0], tr[:, 1:4], tr[:, 14:17], R.build(scen, cell).scenario_s)[0]


if __name__ == '__main__':
    fc = json.load(open(HERE / 'noise_forward_check.json'))['table']
    keys = [(r['scenario'], r['seed'], r['gains']) for r in fc]
    jobs = [(s, sd, g, e) for s, sd, g in keys for e in (0.0, 1e-6)]
    with get_context('spawn').Pool(8) as p: res = p.map(job, jobs, chunksize=1)
    sens = {k: (res[2 * i], res[2 * i + 1]) for i, k in enumerate(keys)}
    rows = []; ok = True
    for r in fc:
        k = (r['scenario'], r['seed'], r['gains']); r0, r6 = sens[k]; ch = 100 * abs(r6 - r0) / r0
        r1 = [x for x in fc if (x['scenario'], x['seed'], x['gains']) == (k[0], k[1], 'round1')][0]['rms_frsim_cm']
        stable = ch < 0.5
        if stable:
            passed = r['rel_pct'] <= 0.5; rule = 'model within 0.5 %'
        else:
            fail_model = r['floor_model'] or r['rms_model_cm'] > 3 * r1
            passed = bool(fail_model); rule = 'model shows failure (floor %s, RMS %.1f cm vs 3 x round 1 = %.1f cm)' % (r['floor_model'], r['rms_model_cm'], 3 * r1)
        ok &= passed
        rows.append(dict(r, sens_change_pct=ch, stable=stable, round1_frsim_cm=r1, passed=bool(passed), rule=rule))
    print('%-7s %5s %-7s %9s %9s %8s %9s %-6s %s' % ('scen', 'seed', 'gains', 'frsim cm', 'model cm', 'rel %', 'sens %', 'stable', 'pass'))
    for r in rows:
        print('%-7s %5d %-7s %9.3f %9.3f %8.3f %9.3f %-6s %s' % (r['scenario'], r['seed'], r['gains'], r['rms_frsim_cm'], r['rms_model_cm'], r['rel_pct'], r['sens_change_pct'], r['stable'], 'yes' if r['passed'] else 'NO'))
    print('NEW GATE:', 'PASS' if ok else 'FAIL', '; stable cells %d of %d' % (sum(r['stable'] for r in rows), len(rows)))
    json.dump(dict(pass_=bool(ok), table=rows), open(HERE / 'noise_gate.json', 'w'), indent=1)
