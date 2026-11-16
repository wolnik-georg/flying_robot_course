#!/bin/bash
# Stage 7: after diag7_S3 (started by chain1) ends: fix_S3 (reverse vs forward noisy gradient per round), diag7_cd (NS2 path checks, eager, single process), fix_S1, diag7_S1 (same descent/landscape tests for S1). One job at a time.
cd "$(dirname "$0")"
until [ -f logs/diag7_S3.rc ]; do sleep 20; done
for j in fix_S3 diag7_cd fix_S1 diag7_S1; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
