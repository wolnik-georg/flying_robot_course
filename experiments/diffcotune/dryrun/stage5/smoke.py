import sys, time
sys.path.insert(0, '.'); sys.path.insert(0, 'dryrun')
import numpy as np, torch
torch.set_num_threads(2)
import reference as R
from frsim.scenarios import tuning_cells
import s2_model, s1_model, s3_model
ref = R.build('A8', tuning_cells('A8')[0])
for name, M in (('S2', s2_model), ('S1', s1_model), ('S3', s3_model)):
    nb = M.N_BETA; t0 = time.time()
    with torch.no_grad():
        o = M.rollout(ref, np.zeros((1, 10)), np.zeros((1, nb)), n_blocks=1500, noise=M.frsim_noise(1000, ref.n_ticks))
    x = o['x'][:, 0].numpy(); print(name, 'ok', x.shape, np.isfinite(x).all(), 'z end %.3f' % x[-1, 2], '%.1f s' % (time.time() - t0), flush=True)
