#!/bin/bash
# Stage 8 slot runner (two slots may run at the same time, user decision 15 Nov): usage: slot.sh <name> <wait-file-or-'none'> <job> [<job> ...]
cd "$(dirname "$0")"; name=$1; waitf=$2; shift 2
[ "$waitf" != none ] && until grep -q "ALL DONE" $waitf; do sleep 20; done
for j in "$@"; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
