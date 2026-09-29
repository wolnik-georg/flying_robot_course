#!/bin/bash
# Single-drone SIL hover smoke: oot4 vs oot5 (same plant, same algorithm family).
set -eo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
CTRL=${1:-oot5}
case "$CTRL" in
  oot4) SERVER=$CS2/crazyflie/config/server_sim_omar_indi.yaml ;;
  oot5) SERVER=$CS2/crazyflie/config/server_sim_omar_indi_rust.yaml ;;
  *) echo "usage: $0 oot4|oot5"; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
cd "$CS2" && source install/setup.bash
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}
pkill -9 -f crazyflie_server 2>/dev/null || true
sleep 2
LOG=$OUT/sil_smoke_${CTRL}.log
rm -f "$LOG"
timeout 90 ros2 launch crazyflie launch.py backend:=sim \
  crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim1.yaml \
  server_yaml_file:=$SERVER \
  gui:=false rviz:=false mocap:=false teleop:=false > "$LOG" 2>&1 &
LPID=$!
sleep 15
timeout 60 ros2 run crazyflie_examples cmd_full_state -- 0 0 1 0 0 0 0 0 0 \
  --ros-args -p use_sim_time:=true >> "$LOG" 2>&1 || true
sleep 5
kill -9 $LPID 2>/dev/null || true
pkill -9 -f crazyflie_server 2>/dev/null || true
grep -E 'ERROR|Traceback|Unknown controller|oot_omar|simulated airframe' "$LOG" | tail -15
echo "log=$LOG"
