#!/bin/bash
# Stage 8b queue resume after the desktop-session crash (16 Nov 03:17 killed c6.sh during gate9o_S0 and slot D5 during gate9_S1).
# Done before the crash: rtest8o_S0, dry8o_S0 (6 rounds). Order now: S2 first (session 1, nothing done yet), then the S0 gate, then S1, S3, then the paper S1 gate rerun. One job at a time (memory).
cd "$(dirname "$0")"
for j in dry8o_S2 rtest8o_S2 gate9o_S2 gate9o_S0 dry8o_S1 rtest8o_S1 gate9o_S1 dry8o_S3 rtest8o_S3 gate9o_S3 gate9_S1; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
