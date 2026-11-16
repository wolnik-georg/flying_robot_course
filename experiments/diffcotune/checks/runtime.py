#!/usr/bin/env python3
"""Runtime of one rollout of each scenario: frsim episode (noise off), differentiable S0 model forward and forward + backward
(B = 1, J_task backward) on CPU float64, CPU float32 and GPU float32 (eager), optionally torch.compile. Sizes the Stage 3 dry run.
  python3 checks/runtime.py [--gpu] [--compile] [--threads 4] [--out FILE.json]"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE))
import costs as C  # noqa: E402
import reference as R  # noqa: E402
import s0_model as S  # noqa: E402
from forward_check import SCENARIOS  # noqa: E402
from frsim.episode import EpisodeConfig, run_episode  # noqa: E402
from frsim.scenarios import study_cells  # noqa: E402


def timed(fn, device):
    if device == 'cuda':
        torch.cuda.synchronize()
    t = time.time(); r = fn()
    if device == 'cuda':
        torch.cuda.synchronize()
    return time.time() - t, r


def run_model(ref, dtype, device, compile):
    z = torch.zeros(1, 14, dtype=dtype, device=device, requires_grad=True)
    with torch.no_grad():
        tf, _ = timed(lambda: S.rollout(ref, z[:, :10], z[:, 10:], dtype, device, compile=compile), device)
    def fb():
        out = S.rollout(ref, z[:, :10], z[:, 10:], dtype, device, compile=compile)
        j = C.j_task(out, ref.scenario_s).sum()
        j.backward()
    tb, _ = timed(fb, device)
    return tf, tb


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--gpu', action='store_true'); ap.add_argument('--compile', action='store_true'); ap.add_argument('--eager-cpu', action='store_true'); ap.add_argument('--gpu-eager', action='store_true')
    ap.add_argument('--threads', type=int, default=4); ap.add_argument('--out', default=str(HERE / 'runtime.json'))
    ap.add_argument('--scen', nargs='+', default=SCENARIOS)
    a = ap.parse_args()
    torch.set_num_threads(a.threads)
    res = []
    configs = ([('cpu', torch.float64, True), ('cpu', torch.float32, True)] if a.compile else []) + ([('cuda', torch.float32, True)] if a.gpu and a.compile else []) + ([('cuda', torch.float32, False)] if a.gpu and a.gpu_eager else []) + ([('cpu', torch.float64, False)] if a.eager_cpu else [])
    for s in a.scen:
        cell = study_cells(s)[0]; ref = R.build(s, cell)
        t0 = time.time(); r = run_episode(EpisodeConfig('S0', s, cell, noise=False)); tf_fr = time.time() - t0
        row = dict(scenario=s, ticks=ref.n_ticks, seconds_of_flight=ref.n_ticks * 1e-3, frsim_episode_s=tf_fr)
        for dev, dt, comp in configs:
            key = '%s_%s%s' % (dev, 'f64' if dt == torch.float64 else 'f32', '_compile' if comp else '_eager')
            try:
                if dev == 'cuda' or comp:
                    run_model(ref, dt, dev, comp)           # warm-up (kernel build / torch.compile; its time is not reported)
                tf, tb = run_model(ref, dt, dev, comp)
                row[key + '_forward_s'], row[key + '_forward_backward_s'] = tf, tb
                print('  ', s, key, 'forward %.1f s, forward+backward %.1f s' % (tf, tb), flush=True)
            except Exception as e:  # noqa: BLE001
                row[key + '_error'] = repr(e)[:200]
        print(row, flush=True); res.append(row)
    json.dump(dict(threads=a.threads, rows=res), open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
