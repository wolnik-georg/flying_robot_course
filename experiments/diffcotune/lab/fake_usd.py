"""Stage 4a C2 dry test helper: frsim flights written as fake merged-uSD files (500 Hz, `# meta:t_zero=scenario_start`, columns `<drone>.<channel>` as in the real merged files,
bottom drone `cf5_dry`, top drone `cf_second_dry`). Channels written: the ones usd_ingest reads. Values with 17 significant digits, so the ingest returns the frsim trace exactly
(gyro stored in deg/s, as in a uSD log; quaternion as est_qw/qx/qy/qz). The state trace is the true plant state (record_every = 2 ticks = 500 Hz)."""
import os, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import reference as R  # noqa
import loop_lib as L  # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa


def fly_500hz(scenario, cell, gains, plant, seed):
    p = config.plant_with(plant['overrides'])
    r = run_episode(EpisodeConfig(os.environ.get('DIFFCOTUNE_CONTROLLER', 'S0'), scenario, cell, gains, plant['variant'], seed, plant=p, noise=True, record_state=True, record_every=2))
    return r.state_trace, bool(r.stable)


def write_merged(path, trace, scenario, cell):
    ref = R.build(scenario, cell); t = trace[:, 0]; n = len(t)
    cols = {}
    b = 'cf5_dry.'; tp = 'cf_second_dry.'
    for i, k in enumerate(('x', 'y', 'z')):
        cols[b + k] = trace[:, 1 + i]; cols[b + 'v' + k] = trace[:, 4 + i]
    cols[b + 'est_qx'], cols[b + 'est_qy'], cols[b + 'est_qz'], cols[b + 'est_qw'] = trace[:, 7], trace[:, 8], trace[:, 9], trace[:, 10]
    for i, k in enumerate(('x', 'y', 'z')):
        cols[b + 'gyro_' + k] = np.degrees(trace[:, 11 + i])
        cols[b + 'ctrltarget_' + k] = trace[:, 14 + i]; cols[b + 'ctrltarget_v' + k] = trace[:, 17 + i]; cols[b + 'ctrltarget_a' + k] = trace[:, 20 + i]
    if ref.has_top:
        for i, k in enumerate(('x', 'y', 'z')):
            cols[tp + k] = np.interp(t, ref.time_s, ref.top_pos[:, i]); cols[tp + 'v' + k] = np.interp(t, ref.time_s, ref.top_vel[:, i])
    names = sorted(cols)
    with open(path, 'w') as f:
        f.write('# meta:t_zero=scenario_start\n# fake merged uSD file written from an frsim flight (Stage 4a C2 dry test)\n')
        f.write('t,' + ','.join(names) + '\n')
        for r_ in range(n):
            f.write(repr(float(t[r_])) + ',' + ','.join(repr(float(cols[k][r_])) for k in names) + '\n')
