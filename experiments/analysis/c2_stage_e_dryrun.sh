#!/bin/bash
# Stage E (docs/40): CS2 SIL A3 with real C.1 LOO weights (A3-only train), rnn.en=0 vs 1.
# Go/no-fly gate for hardware — not a quality claim.

REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
export FLYING_ROBOT_COURSE_ROOT="${FLYING_ROBOT_COURSE_ROOT:-$REPO}"
OUT=$REPO/experiments/sim_validation
WEIGHTS=$REPO/experiments/analysis/out/c2_e2e_2026-09-21/loo_weights/train_without_A1_12-51-16.npz

if [ ! -f "$WEIGHTS" ]; then
  echo "missing weights: $WEIGHTS (run c2_e2e_validation.py Stage B first)"
  exit 1
fi

mkdir -p "$OUT"

fly() {
  local CFG=$1 LABEL=$2
  echo "=== $LABEL ==="
  source /opt/ros/humble/setup.bash
  cd "$CS2" || exit 1
  source install/setup.bash
  export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

  setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$OUT/server_c2_e2e_$CFG.yaml \
    gui:=false rviz:=false > "$OUT/c2_e2e_$CFG.log" 2>&1 &
  local GPID=$!
  sleep 15

  timeout 380 ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 \
    --auto-center --yes --ros-args -p use_sim_time:=true \
    > "$OUT/c2_e2e_client_$CFG.log" 2>&1
  local RC=$?
  sleep 2
  kill -- -$GPID 2>/dev/null
  sleep 5

  local CSV=$OUT/c2_e2e_$CFG.csv
  local N=$( [ -f "$CSV" ] && wc -l < "$CSV" || echo 0 )
  echo "  client rc=$RC, $N rows in $(basename "$CSV")"
  if [ "$N" -lt 100 ]; then
    echo "  FAILED: too few samples. See $OUT/c2_e2e_$CFG.log"
    return 1
  fi
  if grep -q "Permission denied: '/home/flyingrobots'" "$OUT/c2_e2e_client_$CFG.log" 2>/dev/null; then
    echo "  WARN: formation client could not write logs — set FLYING_ROBOT_COURSE_ROOT and rebuild crazyflie_examples if needed"
  fi
  grep -i "residual network\|rnn.en" "$OUT/c2_e2e_$CFG.log" | sed 's/^/  /'
  return 0
}

fly predict "C2 E2E predict (rnn.en=0)" || exit 1
fly compensate "C2 E2E compensate (rnn.en=1)" || exit 1

python3 "$REPO/experiments/analysis/c2_stage_e_summary.py" \
  --predict "$OUT/c2_e2e_predict.csv" \
  --compensate "$OUT/c2_e2e_compensate.csv" \
  -o "$OUT/c2_e2e_stage_e.json" || exit 1

cat "$OUT/c2_e2e_stage_e.json"
echo "Stage E summary written to $OUT/c2_e2e_stage_e.json"
