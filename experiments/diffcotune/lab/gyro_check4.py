#!/usr/bin/env python3
"""Stage 4b item 2 (e), model side: S0 model body rate against frsim for the 4 tuning cells (default gains, P1 plant = beta PSI_TRUE in the model, noise off, planned top path): per axis the correlation
and the max difference; and the sign / axis test on the model itself (body rate against 2 Im(conj(q) dq/dt) of the model quaternion). Axes with zero motion are reported as 'no motion'.
Output lab/results/gyro_check4.json."""
import json, os, sys
from pathlib import Path
import numpy as np
os.environ.setdefault('TORCHINDUCTOR_COMPILE_THREADS', '1')
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import torch
torch.set_num_threads(2)
import s0_model as S, reference as R, loop_lib as L, gyro_check as G  # noqa
from frsim import config  # noqa
from frsim.episode import EpisodeConfig, run_episode  # noqa

if __name__ == '__main__':
    out = []
    for sc, cell in L.TUNING:
        ref = R.build(sc, cell)
        r_ = run_episode(EpisodeConfig('S0', sc, cell, L.gains_of(np.zeros(10)), 'nominal', 0, plant=config.plant_with(L.P1['overrides']), noise=False, record_state=True, record_every=10))
        trf = r_.state_trace
        with torch.no_grad():
            o = S.rollout(ref, np.zeros((1, 10)), L.PSI_TRUE[None, :], compile=False)
        tm = np.asarray(o['t']); wm = o['w'][:, 0, :].numpy(); qm = o['q'][:, 0, :].numpy(); m = (tm > 0) & (tm <= ref.scenario_s); dt = float(np.median(np.diff(tm)))
        wf = np.stack([np.interp(tm, trf[:, 0], trf[:, 11 + j]) for j in range(3)], 1); d = G.omega_from_q(qm, dt, k=2)
        def cc(a, b):
            return [float(np.corrcoef(a[m, j], b[m, j])[0, 1]) if a[m, j].std() > 1e-9 and b[m, j].std() > 1e-9 else None for j in range(3)]
        res = dict(scenario=sc, std_model_w=wm[m].std(0).tolist(), max_abs_diff_model_vs_frsim=float(np.abs(wm - wf)[m].max()), corr_model_vs_frsim=cc(wm, wf), corr_model_w_vs_model_q_derivative=cc(wm, d),
                   slope_model_w_on_q_derivative=[float((wm[m, j] * d[m, j]).sum() / (d[m, j] ** 2).sum()) if d[m, j].std() > 1e-9 else None for j in range(3)])
        out.append(res); print(json.dumps(res), flush=True)
    json.dump(out, open(HERE / 'results' / 'gyro_check4.json', 'w'), indent=1)
