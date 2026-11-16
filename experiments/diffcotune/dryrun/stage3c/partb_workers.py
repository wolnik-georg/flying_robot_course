"""Worker functions for dryrun/stage3c/partb.py (imported by the spawned pool processes)."""
import sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent.parent))
import reference as R  # noqa: E402


def worker_partgrad(args):
    """(scenario, cell, x (14,), trace) -> (parts dict p/v/q/w of J_sysId, gradient of each part with respect to the 4 beta (4 x 4))."""
    scen, cell, x, tr = args
    import torch
    import costs as C
    import ingest
    import s0_model as S
    ref = R.build(scen, cell)
    z = torch.tensor(np.asarray(x)[None], dtype=torch.float64, requires_grad=True)
    out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True)
    _, parts = C.j_sysid(out, ingest.on_grid(ingest.from_trace(tr), out['t']), ref.scenario_s, parts=True)
    vals, grads = {}, {}
    for k in 'pvqw':
        vals[k] = float(parts[k].sum())
        grads[k] = torch.autograd.grad(parts[k].sum(), z, retain_graph=True)[0][0, 10:].numpy().copy()
    return vals, grads
