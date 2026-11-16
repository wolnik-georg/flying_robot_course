#!/bin/bash
# Stage 5c step 5: reruns S1, S3, S2 with GRAD_MODE mixed (only started if the step 2/3 checks passed; round.py table must say 'mixed' for S1-S3 before the round tests).
cd "$(dirname "$0")"
until grep -q "ALL DONE" logs/chain3.log; do sleep 20; done
for c in S1 S3 S2; do
  python3 - <<PY || { echo "$(date +%T) checks of $c did not pass: stop"; exit 1; }
import json,sys
ok = json.load(open('results/fsens_check_$c.json'))['sysid']['passed'] and json.load(open('results/fsens_check_$c.json'))['task']['passed'] and json.load(open('results/fsens_mixed_$c.json'))['mixed']['passed']
sys.exit(0 if ok else 1)
PY
done
for c in S1 S3 S2; do
  for j in ident_$c dry_$c rtest_$c gate_$c; do
    echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
    if [ "$j" = "dry_$c" ] && ! [ -f results/eval_$c.json ]; then echo "$(date +%T) dry run of $c has no eval file (halted after round 1?)"; fi
  done
done
echo "$(date +%T) ALL DONE"
