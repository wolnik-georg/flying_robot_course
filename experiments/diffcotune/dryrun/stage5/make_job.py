#!/usr/bin/env python3
"""Stage 5: write a job file for dryrun/stage3c/job_runner.py. usage: make_job.py <name> <controller> <procs> <mem_gb> <command...>  -> jobs_<name>.json
The job runs `DIFFCOTUNE_CONTROLLER=<c> OMP_NUM_THREADS=2 TORCHINDUCTOR_COMPILE_THREADS=1 <command>` in dryrun/stage5, log logs/<name>.log, exit code logs/<name>.rc, start / end times logs/t_*_<name>.txt."""
import json, shlex, sys
from pathlib import Path
name, ctrl, procs, mem = sys.argv[1:5]; cmd = ' '.join(shlex.quote(a) for a in sys.argv[5:])
here = Path(__file__).resolve().parent
sh = 'export DIFFCOTUNE_CONTROLLER=%s OMP_NUM_THREADS=2 TORCHINDUCTOR_COMPILE_THREADS=1; cd %s && date +%%T > logs/t_start_%s.txt && %s > logs/%s.log 2>&1; echo $? > logs/%s.rc; date +%%T > logs/t_end_%s.txt' % (ctrl, here, name, cmd, name, name, name)
(here / ('jobs_%s.json' % name)).write_text(json.dumps([dict(name=name, argv=['bash', '-c', sh], procs=int(procs), mem_gb=float(mem), log='logs/job_%s.log' % name)], indent=1)); print('jobs_%s.json' % name)
