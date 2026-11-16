"""Stage 3e add-on 3: Pool of loop_lib with a per-scenario top-vehicle path (the logged one) as the peer input of the S0 model. loop_lib itself is not changed (the Stage 3e runs use it).
`tops[i]` = None (planned top path of frsim) or (top_pos (n, 3), top_vel (n, 3)) on the reference's 1 ms ticks, for tuning scenario i of loop_lib.TUNING. Same maths as loop_lib.worker_rollout."""
import dataclasses, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / 'dryrun'))
import loop_lib as L  # noqa
import reference as R  # noqa


def worker_rollout_top(args):
    kind, scen, cell, x, traces, states, noise_seeds, top = args
    import torch
    import costs as C, ingest
    S = L.model()
    ref = R.build(scen, cell)
    if top is not None:
        ref = dataclasses.replace(ref, top_pos=top[0], top_vel=top[1])
    fwd = kind.endswith('_fwd'); grad = (not fwd) or bool(noise_seeds)
    mixed_task_a1 = L.GRAD_MODE == 'mixed2' and kind == 'task' and scen == 'A1' and not fwd          # Stage 7: noisy task gradient of the tuning step on A1 by the compiled forward sensitivity
    cfwd_all = L.GRAD_MODE == 'cfwd' and not fwd          # Stage 7: compiled forward sensitivity for every gradient
    if (L.GRAD_MODE in ('mixed', 'mixed2') and kind == 'sysid' and scen == 'A1') or mixed_task_a1 or cfwd_all:          # Stage 5c: the noise-free beta fit on A1 by the exact compiled forward sensitivity (fsens.py), as loop_lib.worker_rollout
        import fsens
        return fsens.gradient(L.CTRL, ref, ref.scenario_s, kind, x, range(10) if (mixed_task_a1 or (cfwd_all and kind == 'task')) else range(10, np.asarray(x).shape[0]), traces=traces, states=states, noise_seeds=noise_seeds, model=S)
    z = torch.tensor(np.asarray(x)[None], dtype=torch.float64, requires_grad=grad)
    with torch.set_grad_enabled(grad):
        if kind.startswith('task') and noise_seeds:
            nz = np.stack([S.frsim_noise(sd, ref.n_ticks) for sd in noise_seeds], 1)
            out = S.rollout(ref, z[:, :10].expand(len(noise_seeds), 10), z[:, 10:].expand(len(noise_seeds), z.shape[1] - 10), compile=True, noise=nz)
            Js = [C.j_task(out, ref.scenario_s).mean()]
        elif kind.startswith('task'):
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True); Js = [C.j_task(out, ref.scenario_s).sum()]
        else:
            out = S.rollout(ref, z[:, :10], z[:, 10:], compile=True)
            Js = [C.j_sysid(out, ingest.on_grid(ingest.from_trace(tr), out['t']), ref.scenario_s, states=states).sum() / len(traces) for tr in traces]
    J = sum(Js)
    if fwd:
        return float(J), None
    for i, Ji in enumerate(Js):
        Ji.backward(retain_graph=i < len(Js) - 1)
    return float(J), z.grad[0].numpy().copy()


class PoolTop(L.Pool):
    def __init__(self, n=None, states=L.ALL_STATES, noise_seeds=None, tops=None):
        n = len(L.TUNING) if n is None else n          # Stage 7: one worker per tuning scenario
        super().__init__(n, states, noise_seeds); self.tops = tops or {}

    def rollouts(self, kind, x, traces_by_scen, scen_list=L.TUNING):
        res = self.pool.map(worker_rollout_top, [(kind, s, c, x, traces_by_scen.get(i) if traces_by_scen else None, self.states, self.noise_seeds, self.tops.get(i)) for i, (s, c) in enumerate(scen_list)], chunksize=1)
        J = float(np.mean([r[0] for r in res])); g = None if res[0][1] is None else np.mean([r[1] for r in res], axis=0)
        return J, g
