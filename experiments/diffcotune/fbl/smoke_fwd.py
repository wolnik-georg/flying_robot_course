import os, sys, time
os.environ.setdefault('OMP_NUM_THREADS', '2')
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, torch
import reference as R
import fbl_model as F
from frsim import config
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells
scen = sys.argv[1] if len(sys.argv) > 1 else 'A1'; var = sys.argv[2] if len(sys.argv) > 2 else 'both'
os.environ['FRSIM_FBL_VARIANT'] = var
cell = study_cells(scen)[0]
t0 = time.time(); r = run_episode(EpisodeConfig('FBL', scen, cell, {}, 'nominal', 0, plant=config.CALIBRATED_PLANT, noise=False, record_state=True)); tr = r.state_trace
print('frsim %.0f s' % (time.time() - t0), tr.shape, r.stable, r.tilt_max_deg, flush=True)
ref = R.build(scen, cell); t0 = time.time()
with torch.no_grad(): out = F.rollout(ref, np.zeros((1, 6)), np.zeros((1, 4)), variant=var)
print('model %.0f s' % (time.time() - t0), out['x'].shape, flush=True)
k = min(len(tr), len(out['t'])); pf = tr[:k, 1:4]; pt = out['x'][:k, 0].numpy(); vf = tr[:k, 4:7]; vt = out['v'][:k, 0].numpy()
print('max pos diff mm', 1000 * np.abs(pf - pt).max(), 'max vel diff mm/s', 1000 * np.abs(vf - vt).max())
m = (tr[:k, 0] > 2) & (tr[:k, 0] <= ref.scenario_s)
rms = lambda p: 100 * np.sqrt(np.mean(np.sum((p[m] - tr[:k, 14:17][m]) ** 2, -1)))
print('rms3d frsim %.4f model %.4f cm' % (rms(pf), rms(pt)))
d = np.linalg.norm(pf - pt, axis=-1) * 1000; tt = tr[:k, 0]
for a_, b_ in ((-10, 0), (0, 1), (1, 2), (2, 4), (4, 8), (8, 100)):
    mm = (tt >= a_) & (tt < b_)
    if mm.any(): print('t [%g,%g) max |dx| %.3f mm  mean %.3f  frsim err %.3f mm model err %.3f mm' % (a_, b_, d[mm].max(), d[mm].mean(), 1000*np.linalg.norm(pf[mm]-tr[:k,14:17][mm],axis=-1).mean(), 1000*np.linalg.norm(pt[mm]-tr[:k,14:17][mm],axis=-1).mean()))
print('t0', tt[:3], 'ts end', tt[-1])
