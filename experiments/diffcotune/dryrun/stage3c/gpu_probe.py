#!/usr/bin/env python3
"""C3 probe: time the scenario-batched rollout (s0_batch) on the GPU / CPU. Usage: python3 dryrun/stage3c/gpu_probe.py <device> <dtype> <compile 0|1> <grad 0|1> [chunk [mode]]"""
import os, sys, time
from pathlib import Path
import numpy as np, torch
sys.path.insert(0, str(Path(__file__).resolve().parents[2])); sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import reference as R
import s0_batch as SB
import costs as C
from frsim.scenarios import tuning_cells
dev, dt, comp, grad = sys.argv[1], {'f32': torch.float32, 'f64': torch.float64}[sys.argv[2]], bool(int(sys.argv[3])), bool(int(sys.argv[4]))
chunk = len(sys.argv) > 5 and sys.argv[5] == 'chunk'; mode = sys.argv[6] if len(sys.argv) > 6 else None
names = ('A1', 'A8', 'CIRCLE', 'FIG8')
K = int(os.environ.get('ROWS_PER_SCENARIO', '1'))
refs = [R.build(s, tuning_cells(s)[0]) for s in names] * K; ends = [r.scenario_s for r in refs]
PSI = 0.5 * np.log([0.8, 1.5, 1.3, 0.7])                  # the point of the C1 CPU reference (c1_memory_*.json)
phi = torch.zeros(10, dtype=dt, device=dev, requires_grad=grad); psi = torch.tensor(PSI, dtype=dt, device=dev, requires_grad=grad)
for it in range(2):
    t0 = time.time()
    with torch.set_grad_enabled(grad):
        out = SB.rollout_rows(refs, phi, psi, dtype=dt, device=dev, compile=comp, chunk=chunk, mode=mode)
        J = SB.j_task_rows(out, ends, np.full(4 * K, 1.0 / (4 * K)))
    if dev == 'cuda': torch.cuda.synchronize()
    t1 = time.time()
    if grad:
        J.backward()
        if dev == 'cuda': torch.cuda.synchronize()
    print('rows', 4 * K, 'chunk', chunk, 'mode', mode, '%s %s compile %s grad %s iter %d: J %.6e forward %.1f s total %.1f s' % (dev, sys.argv[2], comp, grad, it, float(J), t1 - t0, time.time() - t0), flush=True)
if grad:
    g = np.concatenate([phi.grad.cpu().numpy(), psi.grad.cpu().numpy()]); print('J', repr(float(J)), 'grad', repr(g.tolist()))
    import json; json.dump(dict(J=float(J), grad=g.tolist(), sec_per_epoch=time.time() - t0), open(Path(__file__).resolve().parent / 'results' / ('gpu_%s_%s_rows%d.json' % (sys.argv[2], 'chunk' if chunk else 'plain', 4 * K)), 'w'))
if dev == 'cuda': print('GPU peak MB', torch.cuda.max_memory_allocated() / 1e6)
