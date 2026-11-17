#!/usr/bin/env python3
"""Stage F2.6: write the job files of the FBL queue (dryrun/fbl/jobs_<name>.json, for dryrun/stage3c/job_runner.py) in the order of f2.sh. Every job runs in dryrun/stage8 (results there, with FBL tags),
logs in dryrun/fbl/logs/<name>.log / .rc / t_start_ / t_end_."""
import json, shlex
from pathlib import Path
HERE = Path(__file__).resolve().parent; S8 = HERE.parent / 'stage8'; TOP = HERE.parents[1]
PY = '/usr/bin/python3'
ENV = 'DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd OMP_NUM_THREADS=2 TORCHINDUCTOR_COMPILE_THREADS=1'
jobs = []
def job(name, ctrl, variant, procs, mem, cmd):
    env = 'export DIFFCOTUNE_CONTROLLER=%s %s%s; ' % (ctrl, ENV, (' DIFFCOTUNE_FBL_VARIANT=%s' % variant) if variant else '')
    sh = env + 'cd %s && date +%%T > %s/logs/t_start_%s.txt && %s > %s/logs/%s.log 2>&1; echo $? > %s/logs/%s.rc; date +%%T > %s/logs/t_end_%s.txt' % (S8, HERE, name, cmd, HERE, name, HERE, name, HERE, name)
    (HERE / ('jobs_%s.json' % name)).write_text(json.dumps([dict(name=name, argv=['bash', '-c', sh], procs=procs, mem_gb=mem, log='logs/job_%s.log' % name)], indent=1)); jobs.append(name)
job('regressF2_S0', 'S0', None, 6, 10.0, '%s/regressF2.sh S0' % HERE)
job('regressF2_S2', 'S2', None, 6, 10.0, '%s/regressF2.sh S2' % HERE)
for v in ('base', 'rom', 'both'):
    tag = 'FBL' + v
    job('gateF_' + v, 'FBL', v, 2, 5.0, '%s %s/fbl/gate9F.py --tag %s --preset' % (PY, TOP, tag))
    job('dryF_' + v, 'FBL', v, 6, 10.0, 'env DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1 %s run_8.py --tag %s_ours_strict --no-eval' % (PY, tag))
    job('rtestF_' + v, 'FBL', v, 6, 10.0, '%s %s/lab/dry_test5_fbl.py --controller FBL --variant %s --strict-dry' % (PY, TOP, v))
    job('gate9F_' + v, 'FBL', v, 2, 5.0, '%s %s/fbl/gate9F.py --tag %s --loop results/loop_FBL_%s_ours_strict.json' % (PY, TOP, tag, tag))
(HERE / 'f2_jobs.txt').write_text('\n'.join(jobs) + '\n'); print(jobs)
