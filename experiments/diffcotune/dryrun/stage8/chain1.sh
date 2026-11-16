#!/bin/bash
# Stage 8: after the Stage 7 diagnosis jobs (chain2 ALL DONE): repro check (METHOD ours reproduces Stage 5c S1 rdfix theta_1), error floors, paper-cost gradient gate, paper-exact identifiability, learning-rate probes (paper variant, S0). One job at a time.
cd "$(dirname "$0")"
until grep -q "ALL DONE" ../stage7/logs/chain2.log; do sleep 30; done
for j in c1redo repro_S1 gate8_S0 floors_S0 ident8_S0 lr_0.001 lr_0.003 lr_0.01 lr_0.03 lr_0.1; do
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
