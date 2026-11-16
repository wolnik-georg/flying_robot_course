#!/bin/bash
cd /home/georg/Desktop/flying_robot_course/experiments/diffcotune
for st in predict fit_truth oracle; do
  nice -n 19 python3 dryrun/stage3c/partb.py $st > dryrun/stage3c/logs/partb_$st.log 2>&1
done
