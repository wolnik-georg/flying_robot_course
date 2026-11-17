#!/bin/bash
# Stage 8d queue (user decision C, 16 Nov): strict dry runs (lab flight rule + R3). One job at a time, same pattern as c7.sh. Waits until logs/dry8o_S2.rc exists (the old-mode S2 run, the "before" run).
# Dropped (superseded by the strict runs): rtest8o_S2, gate9o_S2, gate9o_S0, dry8o_S1, rtest8o_S1, gate9o_S1, dry8o_S3, rtest8o_S3, gate9o_S3, gate9_S1 (paper).
cd "$(dirname "$0")"
echo "$(date +%T) c8.sh waits for logs/dry8o_S2.rc"
until [ -e logs/dry8o_S2.rc ]; do sleep 30; done
echo "$(date +%T) dry8o_S2 rc $(cat logs/dry8o_S2.rc)"
for j in regress8s_S0 dry8s_S0 rtest8s_S0 dry8s_S2 rtest8s_S2 gate9s_S0 gate9s_S2 dry8s_S1 rtest8s_S1 gate9s_S1 dry8s_S3 rtest8s_S3 gate9s_S3; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
