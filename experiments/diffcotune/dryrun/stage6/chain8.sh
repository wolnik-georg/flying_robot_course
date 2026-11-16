#!/bin/bash
# Stage 6 step 1 (cont.): S2 reference, S2 with 4 threads, ingest timing; one job after another.
cd "$(dirname "$0")"
for j in ingest s2_ref s2_b; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
