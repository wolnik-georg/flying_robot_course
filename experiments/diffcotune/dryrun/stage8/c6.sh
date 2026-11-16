#!/bin/bash
# Stage 8b queue (ours, one job at a time because of memory): after the S0 ours run (halted after round 3 by the 45 min rule, caused by the 21:03 overload) ends: keep its files as *_overloaded, round test of round 0, rerun of the S0 ours dry run (6 rounds, deterministic: rounds 0-3 must reproduce), gate, then S2, S1, S3 (dry, rtest, gate each).
cd "$(dirname "$0")"
until [ -f logs/dry8o_S0.rc ]; do sleep 20; done
cp results/loop_S0_ours.json results/loop_S0_ours_overloaded.json; cp results/eval_S0_ours.json results/eval_S0_ours_overloaded.json
for j in rtest8o_S0 dry8o_S0 gate9o_S0 dry8o_S2 rtest8o_S2 gate9o_S2 dry8o_S1 rtest8o_S1 gate9o_S1 dry8o_S3 rtest8o_S3 gate9o_S3; do
  [ "$j" = dry8o_S0 ] && mv logs/dry8o_S0.log logs/dry8o_S0_overloaded.log && rm -f logs/dry8o_S0.rc
  echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
done
echo "$(date +%T) ALL DONE"
