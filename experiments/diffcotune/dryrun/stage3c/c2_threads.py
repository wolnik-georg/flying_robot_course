#!/usr/bin/env python3
"""Stage 3c C2: seconds per epoch (4 scenarios in 4 worker processes) at 2 / 4 / 8 torch threads per worker. One task epoch and one fit epoch
(1 trace per scenario), 3 timed epochs after 2 warm-up epochs; the load average (1 min) at the end of the run is recorded.
Usage: python3 dryrun/stage3c/c2_threads.py <threads>"""
import json, os, sys, time
from pathlib import Path
T = sys.argv[1]
os.environ['DIFFCOTUNE_THREADS'] = T
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import loop_lib as L  # noqa: E402
if __name__ == '__main__':
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        fl = L.fly_set(pool, np.zeros(10), L.P1, L.TUNING, [0, 1, 2, 3]); traces = {i: [f['trace']] for i, f in enumerate(fl)}
        x = np.zeros(14); x[10:] = L.PSI_TRUE * 0.5; rec = dict(threads=int(T), task=[], sysid=[])
        for kind, key in (('task', 'task'), ('sysid', 'sysid')):
            for k in range(5):
                t0 = time.time(); J, g = pool.rollouts(kind, x, traces if kind == 'sysid' else None); dt = time.time() - t0
                if k >= 2: rec[key].append(dt)
        rec['load1_end'] = os.getloadavg()[0]; rec['task_mean_s'] = float(np.mean(rec['task'])); rec['sysid_mean_s'] = float(np.mean(rec['sysid']))
        print(json.dumps(rec)); L.jdump(rec, HERE / 'results' / ('c2_threads_%s.json' % T))
    finally:
        pool.close()
