#!/bin/bash
# NOT USED (Stage 4b): this script waited for dry_test.py and started a second pool at the same time; that caused the oomd kill at 14:52:53. Run lab/dry_round1_strict.py after dry_test.py has finished, via dryrun/stage3c/job_runner.py.
exit 1
# waits for the round-1 flights of the dry test, then runs the strict=True round 1 (logs the load before and 1 min after the start)
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune/dryrun/stage4b
until [ -f ../../lab/dry/round1_logs/flights.json ]; do sleep 10; done
echo "$(date +%T) start dry_round1_strict load $(cut -d' ' -f1-3 /proc/loadavg) $(grep MemAvail /proc/meminfo)" >> logs/chain.log
OMP_NUM_THREADS=2 TORCHINDUCTOR_COMPILE_THREADS=1 nice -n 19 /usr/bin/python3 ../../lab/dry_round1_strict.py > logs/dry_round1_strict.log 2>&1 &
PID=$!; sleep 60
echo "$(date +%T) dry_round1_strict pid $PID load after 1 min $(cut -d' ' -f1-3 /proc/loadavg) $(grep MemAvail /proc/meminfo)" >> logs/chain.log
wait $PID; echo "$(date +%T) end dry_round1_strict exit $?" >> logs/chain.log
