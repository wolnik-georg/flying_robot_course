#!/bin/bash
# Stage 5c, RPM delay fixed: S1 then S3 (dry run 5 rounds, round test, gradient gate). Starts after chain4 and chain5 printed ALL DONE.
cd "$(dirname "$0")"
until grep -q "ALL DONE" ../stage5c/logs/chain4.log && grep -q "ALL DONE" ../stage5c/logs/chain5.log; do sleep 30; done
for c in S1 S3; do
  for j in dry_$c rtest_$c gate_$c; do
    echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
  done
done
echo "$(date +%T) ALL DONE"
