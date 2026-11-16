#!/bin/bash
# Stage 6 step 1: S0 desk-speed benchmarks, one job after another (ref, a: 8 tuning workers, b: threads 4, c: warm pool, f: FX graph cache run twice).
cd "$(dirname "$0")"
for j in s0_ref s0_a s0_b s0_c s0_f s0_f2; do
  [ "$j" = s0_f2 ] && cp jobs_s0_f.json jobs_s0_f2.json && sed -i 's/--tag f/--tag f2/; s/s0_f/s0_f2/g' jobs_s0_f2.json
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
