#!/bin/bash
cd "$(dirname "$0")"
for j in fchk_S3 fchk_S2 fmix_S1 fmix_S3 fmix_S2; do echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"; done
echo "$(date +%T) ALL DONE"
