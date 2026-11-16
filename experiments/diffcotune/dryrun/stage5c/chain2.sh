#!/bin/bash
# after chain1 (step 1): S0 identity proof (Stage 5b, repeated after a loader fix), then the step-3 prototype test (variant a')
cd "$(dirname "$0")"
until grep -q "ALL DONE" logs/chain1.log; do sleep 20; done
echo "$(date +%T) start s0id"; (cd ../stage5b && nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_s0id.json > logs/runner_s0id.log 2>&1); echo "$(date +%T) done s0id rc $(cat ../stage5b/logs/s0id.rc)"
echo "$(date +%T) start fsens_try"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_fsens_try.json > logs/runner_fsens_try.log 2>&1; echo "$(date +%T) done fsens_try rc $(cat logs/fsens_try.rc 2>/dev/null)"
echo "$(date +%T) ALL DONE"
