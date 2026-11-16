#!/usr/bin/env python3
"""Runtime of the compiled A8 rollout (forward and forward + backward of J_task) against the number of parameter rows B (rows of
one batch, e.g. several gain sets), CPU float64 and GPU float32; sizes the Stage 3 loop. Peak RSS is of the whole process.
  python3 checks/runtime_batch.py [--rows 1 4 16] [--scen A8]"""
import argparse, resource, sys, time
from pathlib import Path
import torch
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import costs as C, reference as R, s0_model as S  # noqa: E402
from frsim.scenarios import study_cells  # noqa: E402

ap = argparse.ArgumentParser(); ap.add_argument('--rows', type=int, nargs='+', default=[1, 4, 16]); ap.add_argument('--scen', default='A8'); ap.add_argument('--threads', type=int, default=4)
a = ap.parse_args(); torch.set_num_threads(a.threads)
ref = R.build(a.scen, study_cells(a.scen)[0])
for dev, dt in (('cpu', torch.float64), ('cuda', torch.float32)):
    for B in a.rows:
        def run(grad):
            z = torch.zeros(B, 14, dtype=dt, device=dev, requires_grad=grad)
            if dev == 'cuda': torch.cuda.synchronize()
            t = time.time()
            out = S.rollout(ref, z[:, :10], z[:, 10:], dt, dev, compile=True)
            if grad: C.j_task(out, ref.scenario_s).sum().backward()
            if dev == 'cuda': torch.cuda.synchronize()
            return time.time() - t
        try:
            with torch.no_grad(): run(False)          # warm-up: compile
            run(True)
            tf = None
            with torch.no_grad(): tf = run(False)
            tb = run(True)
            print('%s %s B=%2d rollout %s: forward %.1f s, forward+backward %.1f s, peak RSS so far %.1f GB' % (a.scen, dev, B, ref.n_ticks, tf, tb, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1e6), flush=True)
        except Exception as e:  # noqa: BLE001
            print(a.scen, dev, 'B=%d' % B, 'ERROR', repr(e)[:160], flush=True)
