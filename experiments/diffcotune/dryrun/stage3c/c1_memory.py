#!/usr/bin/env python3
"""Stage 3c C1: memory per process (PSS from /proc/<pid>/smaps_rollup, workers and their children) and compile time of the first call, with and
without TORCHINDUCTOR_COMPILE_THREADS=1; and the numbers of one task epoch and one fit epoch with both settings (they must be identical).
Usage: python3 dryrun/stage3c/c1_memory.py <default|one>   (run once per setting; the setting is applied before the pool starts)"""
import json, os, sys, time
from pathlib import Path
mode = sys.argv[1]
if mode == 'default':
    os.environ.pop('TORCHINDUCTOR_COMPILE_THREADS', None)
else:
    os.environ['TORCHINDUCTOR_COMPILE_THREADS'] = '1'
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
os.environ.pop('TORCHINDUCTOR_COMPILE_THREADS', None) if mode == 'default' else None   # loop_lib sets the default to 1 on import; undo for "default"
import loop_lib as L  # noqa: E402
if mode == 'default':
    os.environ.pop('TORCHINDUCTOR_COMPILE_THREADS', None)


def pss_mb(pid):
    try:
        for line in open('/proc/%d/smaps_rollup' % pid):
            if line.startswith('Pss:'):
                return int(line.split()[1]) / 1024
    except OSError:
        return 0.0
    return 0.0


def tree(pid):
    kids = []
    for p in os.listdir('/proc'):
        if p.isdigit():
            try:
                ppid = int(open('/proc/%s/stat' % p).read().rsplit(')', 1)[1].split()[1])
            except OSError:
                continue
            if ppid == pid:
                kids.append(int(p))
    out = [pid]
    for k in kids:
        out += tree(k)
    return out


if __name__ == '__main__':
    pool = L.Pool(4, states=('p', 'v', 'q'))
    try:
        fl = L.fly_set(pool, __import__('numpy').zeros(10), L.P1, L.TUNING, [0, 1, 2, 3]); traces = {i: [f['trace']] for i, f in enumerate(fl)}
        x = np.zeros(14); x[10:] = L.PSI_TRUE * 0.5
        t0 = time.time(); Jt, gt = pool.rollouts('task', x, None); t_first_task = time.time() - t0
        t0 = time.time(); Jt2, gt2 = pool.rollouts('task', x, None); t_second_task = time.time() - t0
        t0 = time.time(); Js, gs = pool.rollouts('sysid', x, traces); t_first_sysid = time.time() - t0
        t0 = time.time(); Js2, gs2 = pool.rollouts('sysid', x, traces); t_second_sysid = time.time() - t0
        procs = tree(os.getpid()); workers = [p for p in procs if p != os.getpid()]
        pss = {p: pss_mb(p) for p in procs}
        # workers = direct children of this process
        direct = [int(d) for d in os.listdir('/proc') if d.isdigit() and os.path.exists('/proc/%s/stat' % d) and int(open('/proc/%s/stat' % d).read().rsplit(')', 1)[1].split()[1]) == os.getpid()]
        per_worker = {}
        for w in direct:
            per_worker[w] = sum(pss_mb(p) for p in tree(w))
        rec = dict(mode=mode, env=os.environ.get('TORCHINDUCTOR_COMPILE_THREADS'), processes_total=len(procs), pss_total_mb=sum(pss.values()), pss_main_mb=pss[os.getpid()],
                   pss_per_direct_child_mb=per_worker, n_children_of_each=[len(tree(w)) for w in direct],
                   first_task_s=t_first_task, second_task_s=t_second_task, first_sysid_s=t_first_sysid, second_sysid_s=t_second_sysid,
                   J_task=repr(Jt), J_task_again=repr(Jt2), J_sysid=repr(Js), grad_task=[repr(v) for v in gt], grad_sysid=[repr(v) for v in gs])
        print(json.dumps(rec, indent=1))
        L.jdump(rec, HERE / 'results' / ('c1_memory_%s.json' % mode))
    finally:
        pool.close()
