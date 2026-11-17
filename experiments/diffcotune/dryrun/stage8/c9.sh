#!/bin/bash
# Stage 8e queue (user decisions 17 Nov, scope change: lab sessions only): only the two RS regressions, one job at a time, pattern of c8.sh.
# regress9r_S0 / S2: full strict run with rule RS on (DIFFCOTUNE_RESTART_BEST=1), must be bit-identical to the Stage 8d strict run (their only R3 round is round 0, which has no theta_best).
# Dropped by the user: S1 / S3 reruns (P1, P1RD), the plant switch, dry9r_* / rtest9r_* / gate9r_*.
cd "$(dirname "$0")"
echo "$(date +%T) c9.sh start"
for j in regress9r_S0 regress9r_S2; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
