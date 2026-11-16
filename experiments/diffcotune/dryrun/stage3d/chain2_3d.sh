#!/bin/bash
# after the main chain (exact PID $1): gradient check with noise again at h = 1e-6 (same gate)
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune
while ps -p "$1" >/dev/null; do sleep 30; done
echo "$(date +%T) start gradient h1e-6" >> dryrun/stage3d/logs/chain.log
nice -n 19 python3 dryrun/stage3d/noise_gradient_check.py 1e-6 > dryrun/stage3d/logs/noise_gradient_check_h1e-6.log 2>&1
echo "$(date +%T) end gradient h1e-6 exit $?" >> dryrun/stage3d/logs/chain.log
