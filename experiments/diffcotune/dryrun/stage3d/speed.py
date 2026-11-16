#!/usr/bin/env python3
"""Stage 3d speed of variant A: one tuning epoch (cost + gradient) = 4 scenarios x 2 noise seeds = 8 rollouts.
  cpu : pool of 4 workers, 2 rows (seeds) per worker as one compiled batch, 2 torch threads each (8 workers x 1.5 GB do not fit next to other work: not run)
  gpu : python3 speed.py gpu  -> float64, 8 rows in ONE batch (s0_batch.rollout_rows), uncompiled-vs-compiled as in Stage 3c (compiled)."""
import json, sys, time
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1])); sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
def main():
    if sys.argv[1] == 'cpu':
        import loop_lib as L
        pool = L.Pool(4, noise_seeds=(5000, 5001)); x = np.zeros(14); x[10:] = L.PSI_TRUE; ts = []
        try:
            for i in range(4):
                t0 = time.time(); J, g = pool.rollouts('task', x, None); ts.append(time.time() - t0); print('cpu epoch', i, '%.1f s' % ts[-1], 'J %.6e' % J, flush=True)
        finally:
            pool.close()
        json.dump(dict(epoch_seconds=ts, rows=8, workers=4), open(HERE / 'speed_cpu.json', 'w'))
    else:
        import torch, reference as R, s0_batch as SB, loop_lib as L
        from frsim.scenarios import tuning_cells
        names = ('A1', 'A8', 'CIRCLE', 'FIG8'); seeds = (5000, 5001)
        refs = [R.build(s, tuning_cells(s)[0]) for s in names for _ in seeds]; ends = [r.scenario_s for r in refs]
        import s0_model as S
        noise = [S.frsim_noise(sd, r.n_ticks) for r, sd in zip(refs, seeds * 4)]
        phi = torch.zeros(10, dtype=torch.float64, device='cuda', requires_grad=True); psi = torch.tensor(L.PSI_TRUE, dtype=torch.float64, device='cuda', requires_grad=True); ts = []
        for it in range(2):
            t0 = time.time(); out = SB.rollout_rows(refs, phi, psi, dtype=torch.float64, device='cuda', compile=True, noise=noise)
            J = SB.j_task_rows(out, ends, np.full(8, 1 / 8)); J.backward(); torch.cuda.synchronize(); ts.append(time.time() - t0)
            print('gpu f64 8 rows iter', it, '%.1f s' % ts[-1], 'J %.6e' % float(J), flush=True)
        json.dump(dict(epoch_seconds=ts, rows=8, J=float(J), grad=phi.grad.cpu().numpy().tolist()), open(HERE / 'speed_gpu.json', 'w'))


if __name__ == '__main__':
    main()
