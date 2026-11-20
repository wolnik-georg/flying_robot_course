#!/bin/bash
# ROS-level probe (2026-11-20): is the crazyflie_server actually subscribed to /parameter_events (QoS compatible), which callbacks/services exist?
# Read-only, no hardware commands. CS2 must be RUNNING (launch it with the tee command first). Writes debug/lab_logs/ros_probe_<time>.log (NOT committed).
#   Usage: cd ~/flying_robot_course && experiments/diffcotune/lab/ros_events_probe.sh
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"; cd "$REPO" || exit 1
OUT="$REPO/debug/lab_logs/ros_probe_$(date +%F_%H%M%S).log"; mkdir -p "$REPO/debug/lab_logs"
pgrep -f crazyflie_server >/dev/null || { echo "STOP: CS2 is not running."; exit 1; }
r() { echo "" | tee -a "$OUT"; echo "\$ $*" | tee -a "$OUT"; timeout 30 "$@" 2>&1 | cut -c1-220 | tee -a "$OUT"; }
echo "=== ROS probe $(date)  ROS_DISTRO=${ROS_DISTRO:-unset} RMW=${RMW_IMPLEMENTATION:-unset}" | tee -a "$OUT"
r ros2 topic info -v /parameter_events
r ros2 node list
r ros2 node info /crazyflie_server
r ros2 param describe /crazyflie_server cf5.params.usd.runTag
r ros2 param get /crazyflie_server cf5.params.usd.runTag
r ros2 param get /crazyflie_server cf5.params.led.bitmask
echo "" | tee -a "$OUT"; echo "=== one set with the parameter_events stream shown (5 s)" | tee -a "$OUT"
( timeout 6 ros2 topic echo /parameter_events 2>&1 | cut -c1-200 >> "$OUT" ) &
sleep 1
r ros2 param set /crazyflie_server cf5.params.led.bitmask 1
sleep 1
r ros2 param get /crazyflie_server cf5.params.led.bitmask
r ros2 param set /crazyflie_server cf5.params.led.bitmask 0
wait
r ros2 doctor --report
echo "" ; echo ">>> Log written: $OUT  (commit/push it yourself: cd $REPO && git add debug/lab_logs && git commit -m logs && git push)"
