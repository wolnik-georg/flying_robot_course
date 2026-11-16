#!/usr/bin/env python3
"""Stage 3c C2: small job runner for independent jobs under the standing resource limits.

A job is a command (argv) with the number of CPU processes it will use (`procs`, a pool of 4 workers + main = 5; one frsim flight = 1) and a memory
estimate (`mem_gb`, the PSS of the job at full size). A queued job starts only when ALL of these hold; otherwise it waits (checked every `poll` s):
  * sum of `procs` of the running jobs + procs of the new job <= max_procs (16);
  * the 1-minute load average <= max_load (22);
  * MemAvailable - mem_gb >= min_mem_gb (3 GB): the memory a running job has not yet taken (its estimate minus what it holds now) is reserved,
    so two big jobs cannot start within the same minute;
  * jobs start in the order of the queue (no overtaking), every job under `nice -n 19`.
Independent jobs only: the fit and the tune of one round, and the rounds of one loop, depend on each other and are NOT run by this.
Usage: python3 job_runner.py jobs.json   (list of {"name", "argv", "procs", "mem_gb", "log"})   or   python3 job_runner.py --selftest
"""
import json, os, subprocess, sys, time

HOLD = {}                                   # pid -> (estimate gb)


def mem_available_gb():
    for line in open('/proc/meminfo'):
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) / 1048576.0


def pss_gb(pid):
    tot = 0.0
    try:
        for p in os.listdir('/proc'):
            if p.isdigit():
                try:
                    st = open('/proc/%s/stat' % p).read().rsplit(')', 1)[1].split()
                    if int(p) == pid or int(st[1]) == pid:
                        for line in open('/proc/%s/smaps_rollup' % p):
                            if line.startswith('Pss:'):
                                tot += int(line.split()[1]) / 1048576.0
                except OSError:
                    pass
    except OSError:
        pass
    return tot


class JobRunner:
    def __init__(self, max_procs=16, max_load=22.0, min_mem_gb=3.0, poll=5.0, log=print):
        self.max_procs, self.max_load, self.min_mem_gb, self.poll, self.log = max_procs, max_load, min_mem_gb, poll, log
        self.queue, self.running, self.done = [], {}, []

    def submit(self, name, argv, procs, mem_gb, logfile=None):
        self.queue.append(dict(name=name, argv=list(argv), procs=procs, mem_gb=mem_gb, log=logfile))

    def _reserved_gb(self):
        return sum(max(0.0, j['mem_gb'] - pss_gb(p)) for p, j in self.running.items())

    def _blocked_by(self, job):
        used = sum(j['procs'] for j in self.running.values())
        if used + job['procs'] > self.max_procs:
            return 'processes %d + %d > %d' % (used, job['procs'], self.max_procs)
        if os.getloadavg()[0] > self.max_load:
            return 'load %.1f > %.1f' % (os.getloadavg()[0], self.max_load)
        free = mem_available_gb() - self._reserved_gb() - job['mem_gb']
        if free < self.min_mem_gb:
            return 'memory: available %.1f - reserved %.1f - job %.1f = %.1f < %.1f GB' % (mem_available_gb(), self._reserved_gb(), job['mem_gb'], free, self.min_mem_gb)
        return None

    def run(self):
        last_msg = None
        while self.queue or self.running:
            for p, j in self.running.items():          # Stage 3e: log the load one minute after each start (standing rule)
                if not j.get('load_logged') and time.time() - j['t_start'] >= 60:
                    j['load_logged'] = True; self.log('%s load after 1 min: %s MemAvailable %.1f GB%s' % (j['name'], open('/proc/loadavg').read().split()[:3], mem_available_gb(), '  ** ABOVE %.0f **' % self.max_load if os.getloadavg()[0] > self.max_load else ''))
            for p in [p for p, j in self.running.items() if j['proc'].poll() is not None]:
                j = self.running.pop(p); j['rc'] = j['proc'].returncode; self.done.append(j); self.log('%s finished, exit code %s' % (j['name'], j['rc']))
            while self.queue:
                why = self._blocked_by(self.queue[0])
                if why:
                    if why != last_msg:
                        self.log('waiting for %s: %s' % (self.queue[0]['name'], why)); last_msg = why
                    break
                job = self.queue.pop(0); out = open(job['log'], 'w') if job['log'] else subprocess.DEVNULL
                job['proc'] = subprocess.Popen(['nice', '-n', '19'] + job['argv'], stdout=out, stderr=subprocess.STDOUT)
                job['t_start'] = time.time(); self.running[job['proc'].pid] = job; last_msg = None
                self.log('started %s (pid %d, procs %d, mem estimate %.1f GB)' % (job['name'], job['proc'].pid, job['procs'], job['mem_gb']))
                time.sleep(min(self.poll, 3))          # let the new job take its memory before the next check
            time.sleep(self.poll)
        return self.done


def selftest():
    """Dummy jobs (sleep and a memory allocation) show: the process limit, the load limit and the memory limit each hold a job back, and the order stays."""
    say = lambda *a: print(time.strftime('%H:%M:%S'), *a, flush=True)
    py = [sys.executable, '-c']
    busy = py + ['import time; b = bytearray(200*1024*1024); time.sleep(12)']
    say('--- test 1: process limit (3 jobs of 6 processes, limit 16: two run, the third waits)')
    r = JobRunner(poll=1, log=say)
    for i in range(3):
        r.submit('proc-test-%d' % i, busy, 6, 0.3)
    r.run()
    say('--- test 2: load limit (limit 0.01, a 6 s CPU burner keeps the load above it; with the limit this low nothing may start until the load average falls; limit raised after 15 s to finish)')
    r = JobRunner(poll=1, max_load=0.01, log=say)
    burner = subprocess.Popen(py + ['import time; t=time.time()\nwhile time.time()-t<6: pass'])
    r.submit('load-test', py + ['pass'], 1, 0.1)
    t0 = time.time()
    import threading
    threading.Timer(15, lambda: setattr(r, 'max_load', 22.0)).start()
    r.run(); burner.wait()
    say('--- test 3: memory limit (job estimate = MemAvailable - 2 GB with the 3 GB rule: must wait; the estimate is then set to 0.1 GB after 8 s)')
    r = JobRunner(poll=1, log=say)
    r.submit('mem-test', py + ['pass'], 1, max(mem_available_gb() - 2.0, 0.2))
    threading.Timer(8, lambda: r.queue[0].__setitem__('mem_gb', 0.1) if r.queue else None).start()
    r.run()
    say('self-test done')


if __name__ == '__main__':
    if sys.argv[1:] == ['--selftest']:
        selftest()
    else:
        r = JobRunner(); [r.submit(j['name'], j['argv'], j['procs'], j['mem_gb'], j.get('log')) for j in json.load(open(sys.argv[1]))]
        r.run()
