#!/bin/bash
# Stage 7 reruns (user decision 15 Nov: cfwd gradient everywhere, tuning set A1, A8, CIRCLE, FIG8, A4 cell 1, 5 workers): S3 (rdfix), S0, S2, S1 (rdfix); each: dry run, round test, gradient gate. Starts after chain2 (diagnosis jobs) printed ALL DONE. One job at a time.
cd "$(dirname "$0")"
until grep -q "ALL DONE" logs/chain2.log; do sleep 30; done
for c in S3 S0 S2 S1; do
  for j in dry7_$c rtest7_$c gate7_$c; do
    echo "$(date +%T) start $j"; nice -n 19 /usr/bin/python3 ../stage3c/job_runner.py jobs_$j.json > logs/runner_$j.log 2>&1; echo "$(date +%T) done $j rc $(cat logs/$j.rc 2>/dev/null)"
    if [ "$j" = "dry7_$c" ] && ! [ -f results/eval_$c.json ]; then echo "$(date +%T) dry run of $c has no eval file (halted?): $c skipped"; break; fi
  done
done
echo "$(date +%T) ALL DONE"
