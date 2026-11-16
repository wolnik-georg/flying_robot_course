#!/usr/bin/env python3
"""Stage 5: runs the controller jobs strictly one after another through dryrun/stage3c/job_runner.py (one pool at a time, load / memory limits before every start, 1-minute log by the runner).
A controller whose forward check or noise gate fails is stopped (its later jobs are skipped) and the next controller goes on, as the task says. Gradient failures are reported, not blocking
(the combined gradient gate of S2 failed on A1 only; reported). Usage: python3 driver5.py   (nohup, log logs/driver.log)"""
import json, subprocess, sys, time
from pathlib import Path
HERE = Path(__file__).resolve().parent; RUNNER = HERE.parent / 'stage3c' / 'job_runner.py'
PY = '/usr/bin/python3'


def say(*a): print(time.strftime('%H:%M:%S'), *a, flush=True)


def job(name, ctrl, cmd, procs=5, mem=6):
    subprocess.run([PY, str(HERE / 'make_job.py'), name, ctrl, str(procs), str(mem)] + cmd, cwd=HERE, check=True, capture_output=True)
    say('start', name); t = time.time()
    subprocess.run([PY, str(RUNNER), str(HERE / ('jobs_%s.json' % name))], cwd=HERE, stdout=open(HERE / 'logs' / ('runner_%s.log' % name), 'w'), stderr=subprocess.STDOUT)
    rc = (HERE / 'logs' / ('%s.rc' % name)).read_text().strip() if (HERE / 'logs' / ('%s.rc' % name)).exists() else 'none'
    say('done', name, 'rc', rc, '%.0f min' % ((time.time() - t) / 60)); return rc


def passed(f):
    try: return bool(json.load(open(HERE / 'results' / f))['passed'])
    except Exception: return False


def chain_after_checks(c):
    job('%s_ident_real' % c, c, ['bash', '-c', '%s identifiability5.py > logs/%s_ident.log 2>&1; %s realdata5.py > logs/%s_realdata.log 2>&1' % (PY, c, PY, c)])
    job('%s_dryrun' % c, c, [PY, 'run_5.py'])
    job('%s_roundtest' % c, c, [PY, '../../lab/dry_test5.py', '--controller', c], 5, 6)


if __name__ == '__main__':
    while not (HERE / 'logs' / 'S2_ident_real.rc').exists(): time.sleep(20)       # S2 identifiability + real-data job was started by hand
    say('S2 ident_real finished')
    job('S2_dryrun', 'S2', [PY, 'run_5.py'])
    job('S2_roundtest', 'S2', [PY, '../../lab/dry_test5.py', '--controller', 'S2'])
    job('S0_roundtest', 'S0', [PY, '../../lab/dry_test5.py', '--controller', 'S0'])
    for c in ('S1', 'S3'):
        job('%s_forward' % c, c, [PY, 'checks5.py', 'forward'], 5, 4)
        if not passed('%s_forward_check.json' % c): say(c, 'FORWARD GATE FAILED: controller stopped'); continue
        job('%s_noise_grad' % c, c, ['bash', '-c', '%s checks5.py noise > logs/%s_noise.log 2>&1; %s checks5.py gradient > logs/%s_gradient.log 2>&1' % (PY, c, PY, c)], 5, 5)
        if not passed('%s_noise_check.json' % c): say(c, 'NOISE GATE FAILED: controller stopped'); continue
        chain_after_checks(c)
    say('ALL DONE')
