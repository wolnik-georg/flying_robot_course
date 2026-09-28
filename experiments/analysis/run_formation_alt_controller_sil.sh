#!/bin/bash
# SIL smoke: run_formation A1 with installed crazyflies.yaml (cf5 + cf_second).
# One server controller profile per invocation: oot4 (c=9), oot2 (c=7), oot3 (c=8).
set -eo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
LOGDIR=$REPO/experiments/logs
CTRL=$1
if [ -z "$CTRL" ]; then
  echo "usage: $0 oot4|oot2|oot3"
  exit 1
fi
case "$CTRL" in
  oot4) SERVER=$CS2/crazyflie/config/server_sim_omar_indi.yaml; STATE=state_oot4_formation ;;
  oot2) SERVER=$CS2/crazyflie/config/server_sim_naindi.yaml; STATE=state_oot2_formation ;;
  oot3) SERVER=$CS2/crazyflie/config/server_sim_naindi_hybrid.yaml; STATE=state_oot3_formation ;;
  *) echo "unknown controller profile $CTRL"; exit 1 ;;
esac
source /opt/ros/humble/setup.bash
cd "$CS2"
source install/setup.bash
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}
pkill -9 -f "crazyflie_sim/lib/crazyflie_sim/crazyflie_server" 2>/dev/null || true
sleep 2
MARKER=$(mktemp)
LOG=$OUT/run_formation_sil_${CTRL}.log
CLIENT=$OUT/run_formation_sil_${CTRL}_client.log
rm -f "$LOG" "$CLIENT"
setsid timeout 300 ros2 launch crazyflie launch.py backend:=sim \
  crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies.yaml \
  server_yaml_file:=$SERVER \
  gui:=false rviz:=false mocap:=false teleop:=false > "$LOG" 2>&1 &
GPID=$!
sleep 12
set +e
timeout 240 ros2 run crazyflie_examples run_formation -- \
  --scenario A1 --dz 0.30 --hold 5 --auto-center --yes \
  --ros-args -p use_sim_time:=true > "$CLIENT" 2>&1
RC=$?
set -e
sleep 2
kill -9 -- -$GPID 2>/dev/null || true
pkill -9 -f "crazyflie_sim/lib/crazyflie_sim/crazyflie_server" 2>/dev/null || true
sleep 2
META=$(find "$LOGDIR" -name 'A1_*.meta.json' -newer "$MARKER" 2>/dev/null | sort | tail -1)
echo "controller_profile=$CTRL client_rc=$RC meta=${META:-none}"
if [ -n "$META" ]; then
  python3 "$REPO/experiments/analysis/verify_formation_sim.py" "$META" "$LOGDIR" --append /dev/null 2>&1 | tail -5 || true
fi
grep -E 'ABORT|ERROR|refusing|done, landing|realised relative' "$CLIENT" | tail -8 || true
exit "$RC"
