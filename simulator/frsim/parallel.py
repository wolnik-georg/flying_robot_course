"""Run many episodes in parallel. Results come back in the order of the configs; every episode has its own explicit seed (in its config), so the
numbers do not depend on the number of processes or on which process runs which episode."""
from multiprocessing import Pool

from .episode import run_episode


def run_many(configs, procs=4):
    """List of EpisodeResult, same order as `configs`. procs=1 runs in this process."""
    configs = list(configs)
    if procs <= 1 or len(configs) <= 1:
        return [run_episode(c) for c in configs]
    with Pool(min(procs, len(configs))) as pool:
        return pool.map(run_episode, configs, chunksize=1)


def wait_for_resources(procs, max_load=22.0, min_mem_gb=3.0, poll_s=60):
    """Block until the machine can take `procs` more processes (1-minute load + procs <= max_load, MemAvailable >= min_mem_gb)."""
    import time
    while True:
        load = float(open('/proc/loadavg').read().split()[0])
        mem_gb = [int(line.split()[1]) for line in open('/proc/meminfo') if line.startswith('MemAvailable')][0] / 1048576
        if load + procs <= max_load and mem_gb >= min_mem_gb:
            return
        print('waiting: load %.1f + %d processes > %.0f, or MemAvailable %.1f GB < %.1f GB' % (load, procs, max_load, mem_gb, min_mem_gb), flush=True)
        time.sleep(poll_s)
