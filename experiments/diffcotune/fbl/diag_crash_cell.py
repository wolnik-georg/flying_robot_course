import os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1])); sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np, torch
import forward_check_fbl as FC, fbl_model as F, reference as R
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
var, key = sys.argv[1], sys.argv[2]; sc = FC.SCEN_OF.get(key, key); cell = FC.CELLS[key][1]; os.environ['FRSIM_FBL_VARIANT'] = var
tr = run_episode(EpisodeConfig('FBL', sc, cell, {}, 'nominal', 0, plant=config.CALIBRATED_PLANT, noise=False, record_state=True)).state_trace
ref = R.build(sc, cell)
with torch.no_grad(): o = F.rollout(ref, np.zeros((1, 6)), np.zeros((1, 4)), variant=var)
k = min(len(tr), len(o['t'])); pf = tr[:k, 1:4]; pt = o['x'][:k, 0].numpy(); d = 1000 * np.linalg.norm(pf - pt, axis=-1); t = tr[:k, 0]
tilt_f = F.tilt_deg(torch.as_tensor(tr[:k, 7:11])).numpy(); tilt_m = F.tilt_deg(o['q'][:k, 0]).numpy()
print('frsim rows', len(tr), 'model rows', len(o['t']), 'scenario_s', ref.scenario_s)
for lo, hi in ((-5, 0), (0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 15), (15, 30)):
    m = (t >= lo) & (t < hi)
    if m.any(): print('t[%g,%g): max dx %.2f mm, tilt max frsim %.1f model %.1f deg, z frsim %.3f model %.3f' % (lo, hi, d[m].max(), tilt_f[m].max(), tilt_m[m].max(), pf[m, 2].min(), pt[m, 2].min()))
