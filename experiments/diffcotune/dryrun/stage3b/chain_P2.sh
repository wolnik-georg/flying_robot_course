#!/bin/bash
# waits for the P1 full run (exact PID 2084108) to end, then starts the P2 full run
while ps -p 2084108 >/dev/null; do sleep 30; done
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune
exec nice -n 19 python3 dryrun/run_dryrun.py full --lr 0.01 --plants P2 --stop-rule full50best --states pva --out dryrun/stage3b/results > dryrun/stage3b/logs/full_P2.log 2>&1
