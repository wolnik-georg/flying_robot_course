#!/bin/bash
cd "$(dirname "$0")"
for c in S1 S3; do
 python3 make_job.py dry_$c $c 5 8 env DIFFCOTUNE_GRAD_MODE=mixed DIFFCOTUNE_FIX_RD=1 /usr/bin/python3 run_5.py
 python3 make_job.py rtest_$c $c 5 8 env DIFFCOTUNE_DRY_TAG=_5cfix /usr/bin/python3 ../../lab/dry_test5.py --controller $c --fix-rd
 python3 make_job.py gate_$c $c 5 8 env DIFFCOTUNE_GRAD_MODE=mixed DIFFCOTUNE_FIX_RD=1 /usr/bin/python3 grad_gate_adopted.py
done
