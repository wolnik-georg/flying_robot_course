"""Stage F1 diagnostic: the model's FBL law (fbl_step) fed with the frsim state of every 1 ms tick vs the mixer forces frsim's firmware produced (even ticks)."""
import os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, torch
import reference as R
import fbl_model as F
from frsim import config, vehicle
from frsim.episode import EpisodeConfig, run_episode
from frsim.scenarios import study_cells
scen = sys.argv[1] if len(sys.argv) > 1 else 'A1'; var = sys.argv[2] if len(sys.argv) > 2 else 'both'; N = int(sys.argv[3]) if len(sys.argv) > 3 else 6000
os.environ['FRSIM_FBL_VARIANT'] = var
rec = []; orig = vehicle.mixer_forces
def mf(fw, control):
    f = orig(fw, control); rec.append(f.copy()); return f
vehicle.mixer_forces = mf
cell = study_cells(scen)[0]
r = run_episode(EpisodeConfig('FBL', scen, cell, {}, 'nominal', 0, plant=config.CALIBRATED_PLANT, noise=False, record_state=True, record_every=1)); tr = r.state_trace
ref = R.build(scen, cell); dt = torch.float64
t = lambda a: torch.as_tensor(np.asarray(a), dtype=dt)
flg = F.flags(var); fst = torch.zeros(1, F.NFS, dtype=dt); gains = F.gains_from(torch.zeros(1, 6, dtype=dt), dt, 'cpu')
worst = []; 
for i in range(min(N, len(tr) - 1)):
    x = t(tr[i, 1:4])[None]; v = t(tr[i, 4:7])[None]; q = t(tr[i, 7:11])[None]; w = t(tr[i, 11:14])[None]
    sp = (t(ref.pos[i])[None], t(ref.vel[i])[None], t(ref.acc[i])[None], t(ref.jerk[i])[None])
    if i % 2: continue          # the controller state advances on even ticks only (see fbl_model.py docstring)
    with torch.no_grad():
        fst, cmd = F.fbl_step(fst, gains, sp, x, v, q, torch.rad2deg(w), t(ref.top_pos[i])[None], i % 10 == 0, ref.has_top, *flg)
    if i % 2 == 0:
        d = np.abs(cmd[0].numpy() - rec[i // 2]); worst.append((d.max(), i))
a = np.array([w_[0] for w_ in worst]); idx = np.array([w_[1] for w_ in worst])
print('ticks', len(idx) * 2, 'max |cmd diff| N %.3e at tick %d; median %.3e; frac > 1e-6 N: %.3f' % (a.max(), idx[a.argmax()], np.median(a), (a > 1e-6).mean()))
bad = np.where(a > 1e-6)[0]
print('first bad ticks', idx[bad][:10], 'time_s', tr[idx[bad][:5], 0])
print('thrust cmd range of frsim [N]', min(f.min() for f in rec), max(f.max() for f in rec))
