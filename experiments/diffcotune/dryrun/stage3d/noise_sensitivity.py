#!/usr/bin/env python3
"""Stage 3d diagnostic (after the noise forward check failed in 3 round-3 cells): how much does frsim ITSELF change when the plant is changed by 1e-6
(relative, thrust-law gain)? Same noise seed, same gains. If frsim moves by more than the 0.5 % gate in the failing cells, the gate cannot be met
there by any model, whatever the noise model."""
import sys
from multiprocessing import get_context
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent))
import loop_lib as L, reference as R  # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa
sys.path.insert(0, str(HERE)); import noise_forward_check as N  # noqa

CELLS = [('A8', 1001), ('CIRCLE', 1001), ('FIG8', 1001), ('A1', 1000), ('FIG8', 1000)]
def job(a):
    scen, seed, eps = a
    phi = N.load_phis()['round3']; cell = dict(L.TUNING)[scen]
    plant = config.plant_with(dict(L.P1['overrides'], thrust_gain='x%.12f' % (0.8 * (1 + eps))))
    r = run_episode(EpisodeConfig('S0', scen, cell, L.gains_of(phi), 'nominal', seed, plant=plant, noise=True, record_state=True))
    tr = r.state_trace; return N.stats(tr[:, 0], tr[:, 1:4], tr[:, 14:17], R.build(scen, cell).scenario_s)[0]
if __name__ == '__main__':
    jobs = [(s, sd, e) for s, sd in CELLS for e in (0.0, 1e-6, 1e-4)]
    with get_context('spawn').Pool(4) as p: res = p.map(job, jobs, chunksize=1)
    for i, (s, sd) in enumerate(CELLS):
        r0, r6, r4 = res[3 * i:3 * i + 3]
        print('%-7s seed %d round3 gains: frsim RMS %.3f cm; thrust-gain x(1+1e-6): %.3f cm (%.3f %%); x(1+1e-4): %.3f cm (%.3f %%)' % (s, sd, r0, r6, 100 * abs(r6 - r0) / r0, r4, 100 * abs(r4 - r0) / r0))
