#!/bin/bash
# waits for the Stage 5b h-scan chain, then the S0 identity proof (Stage 5b), then step 1 for S1, S3, S2 (strictly one job at a time, through job_runner)
cd "$(dirname "$0")"
until grep -q "ALL DONE" ../stage5b/logs/chain_hscan.log; do sleep 20; done
echo "$(date +%T) start s0id"; (cd ../stage5b && nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_s0id.json > logs/runner_s0id.log 2>&1); echo "$(date +%T) done s0id rc $(cat ../stage5b/logs/s0id.rc)"
for c in S1 S3 S2; do echo "$(date +%T) start step1 $c"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_step1_$c.json > logs/runner_step1_$c.log 2>&1; echo "$(date +%T) done step1 $c rc $(cat logs/step1_$c.rc 2>/dev/null)"; done
echo "$(date +%T) ALL DONE"
