#!/bin/bash
# Task A.3: CS2 SIL predict-only with full-bank trained weights (rnn.en=0).
set -eo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
export FLYING_ROBOT_COURSE_ROOT="${FLYING_ROBOT_COURSE_ROOT:-$REPO}"
LOG=$OUT/c2_fullbank_predict.log
CSV=$OUT/c2_fullbank_predict.csv
rm -f "$CSV" "$LOG"
source /opt/ros/humble/setup.bash
cd "$CS2"
source install/setup.bash
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}
setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
  crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim.yaml \
  server_yaml_file:=$OUT/server_c2_fullbank_predict.yaml \
  gui:=false rviz:=false > "$LOG" 2>&1 &
GPID=$!
sleep 15
timeout 380 ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 \
  --auto-center --yes --ros-args -p use_sim_time:=true \
  > "$OUT/c2_fullbank_client.log" 2>&1 || true
sleep 2
kill -- -$GPID 2>/dev/null || true
sleep 3
N=$( [ -f "$CSV" ] && wc -l < "$CSV" || echo 0 )
echo "rows=$N log=$LOG"
