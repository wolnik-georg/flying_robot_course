#!/bin/bash
cd "$(dirname "$0")"
for c in S1 S3 S2 S0; do
  echo "$(date +%T) start $c"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_fwdchk_$c.json > logs/runner_fwdchk_$c.log 2>&1
  echo "$(date +%T) done $c rc $(cat logs/fwdchk_$c.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
