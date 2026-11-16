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
    if L.GRAD_MODE == 'mixed' and kind == 'sysid' and scen == 'A1':          # Stage 5c: the noise-free beta fit on A1 by the exact compiled forward sensitivity (fsens.py), as loop_lib.worker_rollout
        import fsens
        return fsens.gradient(L.CTRL, ref, ref.scenario_s, kind, x, range(10, np.asarray(x).shape[0]), traces=traces, states=states, noise_seeds=noise_seeds, model=S)
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


def _w_init_warm(threads, n_seeds, warm):
    """Pool initializer: threads per worker, then (warm) compile every graph this worker can be asked for, so the first epoch of the round is not slower (Stage 6 lever c)."""
    import os
    os.environ['DIFFCOTUNE_THREADS'] = str(threads)
    L._w_init()
    if not warm:
        return
    import torch, time
    S = L.model(); n = 10 + L.N_BETA; t0 = time.time()
    for s_, c_ in L.TUNING:
        ref = R.build(s_, c_)
        for rows, noisy in ((1, False), (1, True), (n_seeds, True)):
            if rows == 1 and noisy and n_seeds == 1: continue
            z = torch.zeros(1, n, dtype=torch.float64, requires_grad=True)
            nz = None if not noisy else np.stack([S.frsim_noise(100 + j, ref.n_ticks) for j in range(rows)], 1)
            with torch.set_grad_enabled(True):
                out = S.rollout(ref, z[:, :10].expand(rows, 10), z[:, 10:].expand(rows, n - 10), compile=True, noise=nz, n_blocks=6)
                out['x'].sum().backward()
        if L.GRAD_MODE == 'mixed' and s_ == 'A1':
            import fsens
            fsens.rollout_tangents(L.CTRL, ref, np.zeros(n), range(10, n), n_blocks=6)
    print('worker warm-up %.0f s' % (time.time() - t0), flush=True)


def _noop(i):
    return i


class PoolTop(L.Pool):
    """Stage 6 options (defaults = the Stage 5c behaviour): threads per worker; split_seeds: the noisy tuning rollout of one scenario is split into one task per noise seed (n workers = scenarios x seeds);
    warm: the workers compile all graphs at start (`wait_ready()` blocks until they are done)."""
    def __init__(self, n=4, states=L.ALL_STATES, noise_seeds=None, tops=None, threads=2, split_seeds=False, warm=False, n_seeds=2):
        import multiprocessing as mp
        self.states = tuple(states); self.noise_seeds = tuple(noise_seeds) if noise_seeds else None; self.n = n; self.split_seeds = split_seeds
        self.pool = mp.get_context('spawn').Pool(n, initializer=_w_init_warm, initargs=(threads, n_seeds, warm)); self.tops = tops or {}

    def wait_ready(self):
        self.pool.map(_noop, range(self.n), chunksize=1)

    def rollouts(self, kind, x, traces_by_scen, scen_list=L.TUNING):
        if self.split_seeds and kind == 'task' and self.noise_seeds and len(self.noise_seeds) > 1:
            jobs = [(kind, s, c, x, None, self.states, (sd,), self.tops.get(i)) for i, (s, c) in enumerate(scen_list) for sd in self.noise_seeds]
            res = self.pool.map(worker_rollout_top, jobs, chunksize=1); k = len(self.noise_seeds)
            Js = [float(np.mean([r[0] for r in res[i * k:(i + 1) * k]])) for i in range(len(scen_list))]; gs = [np.mean([r[1] for r in res[i * k:(i + 1) * k]], axis=0) for i in range(len(scen_list))]
            return float(np.mean(Js)), np.mean(gs, axis=0)
        res = self.pool.map(worker_rollout_top, [(kind, s, c, x, traces_by_scen.get(i) if traces_by_scen else None, self.states, self.noise_seeds, self.tops.get(i)) for i, (s, c) in enumerate(scen_list)], chunksize=1)
        J = float(np.mean([r[0] for r in res])); g = None if res[0][1] is None else np.mean([r[1] for r in res], axis=0)
        return J, g
