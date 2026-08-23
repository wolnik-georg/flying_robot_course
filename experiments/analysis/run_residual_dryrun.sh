#!/bin/bash
# End-to-end residual-learning dry run, in simulation.
#
#   run_residual_dryrun.sh [scenario args...]      # default: A3 at dz 0.30, strong downwash
#
# Rehearses the whole of thesis C.1 -> C.2 without a drone:
#
#   1 COLLECT     plain geometric, no network      -> residual_collect.csv
#   2 TRAIN       tools/residual/train.py          -> residual_weights.npz
#   3 PREDICT     weights loaded, rnn.en=0         -> residual_predict.csv
#   4 COMPENSATE  same weights, rnn.en=1           -> residual_compensate.csv
#   5 COMPARE     analyse_residual_dryrun.py
#
# Phase 3 exists because phase 4 alone cannot be interpreted. With the network in the loop the
# residual it sees is the one that survives its own compensation, so "small a_res" would be
# consistent with both a good model and a feedback loop that happens to be stable. Phase 3
# measures prediction against measurement with the model touching nothing.
#
# The point of the exercise is the plumbing, not the physics: every step here has to work in the
# lab, and each one can fail. The numbers it produces are simulation numbers and no thesis result
# depends on them.

# NOT `set -u`: ROS's setup.bash references unbound variables and aborts under it.
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
RES=$REPO/flying_drone_stack/tools/residual
SCEN=("$@")
[ ${#SCEN[@]} -eq 0 ] && SCEN=(--scenario A3 --dz 0.30)

mkdir -p "$OUT"

fly() {                       # fly <phase-config-suffix> <label>
  local CFG=$1 LABEL=$2
  echo "=== $LABEL ==="
  source /opt/ros/humble/setup.bash
  cd "$CS2" || exit 1
  source install/setup.bash
  export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

  setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$CS2/crazyflie/config/server_sim_res_$CFG.yaml \
    gui:=false rviz:=false > "$OUT/dryrun_$CFG.log" 2>&1 &
  local GPID=$!
  sleep 15

  timeout 380 ros2 run crazyflie_examples run_formation "${SCEN[@]}" \
    --auto-center --yes --ros-args -p use_sim_time:=true \
    > "$OUT/dryrun_client_$CFG.log" 2>&1
  local RC=$?
  sleep 2
  kill -- -$GPID 2>/dev/null
  sleep 5                     # the residual CSV is closed on server shutdown

  # A server that never loaded the controller still produces a CSV, just an empty one, so the
  # row count is checked rather than the file's existence.
  local CSV=$OUT/residual_$CFG.csv
  local N=$( [ -f "$CSV" ] && wc -l < "$CSV" || echo 0 )
  echo "  client rc=$RC, $N rows in $(basename "$CSV")"
  if [ "$N" -lt 100 ]; then
    echo "  FAILED: too few samples. See $OUT/dryrun_$CFG.log"
    return 1
  fi
  grep -i "residual network\|rnn.en" "$OUT/dryrun_$CFG.log" | sed 's/^/  /'
  return 0
}

fly collect "PHASE 1/4  COLLECT -- geometric, no network" || exit 1

echo "=== PHASE 2/4  TRAIN ==="
python3 "$RES/train.py" "$OUT/residual_collect.csv" \
  -o "$OUT/residual_weights.npz" --epochs 400 || exit 1

fly predict    "PHASE 3/4  PREDICT -- weights loaded, rnn.en=0" || exit 1
fly compensate "PHASE 4/4  COMPENSATE -- rnn.en=1" || exit 1

echo "=== COMPARE ==="
python3 "$REPO/experiments/analysis/analyse_residual_dryrun.py" \
  --collect "$OUT/residual_collect.csv" \
  --predict "$OUT/residual_predict.csv" \
  --compensate "$OUT/residual_compensate.csv" \
  --out "$OUT/RESIDUAL_DRYRUN.md"
