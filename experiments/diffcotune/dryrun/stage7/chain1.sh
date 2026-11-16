#!/bin/bash
# Stage 7 step 1 a+b: diag7 for S3 then S1 (one pool at a time).
cd "$(dirname "$0")"
for j in diag7_S3 diag7_S1; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
