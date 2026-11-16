#!/bin/bash
# after chain2 (exact PID $1): re-fly the Stage 3b loop (d) gain sets theta_0..3 for the per-flight floor flags
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune
while ps -p "$1" >/dev/null; do sleep 30; done
echo "$(date +%T) start floorref" >> dryrun/stage3d/logs/chain.log
nice -n 19 python3 dryrun/stage3d/run_3d.py floorref > dryrun/stage3d/logs/floorref.log 2>&1
echo "$(date +%T) end floorref exit $?" >> dryrun/stage3d/logs/chain.log
