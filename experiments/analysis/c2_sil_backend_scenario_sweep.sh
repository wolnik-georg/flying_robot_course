#!/bin/bash
# Multi-scenario SIL predict-only: backend=np vs neuralswarm (rnn.en=0, full bank weights).
set -eo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation/sil_backend_sweep
WEIGHTS=$REPO/experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz
PY=${PYTHON:-/home/georg/.pyenv/versions/flying_robots/bin/python3}
export FLYING_ROBOT_COURSE_ROOT="$REPO"
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

mkdir -p "$OUT"

write_server() {
  local backend=$1 csv=$2 dest=$3
  cat > "$dest" <<EOF
/crazyflie_server:
  ros__parameters:
    warnings:
      frequency: 1.0
    firmware_params:
      query_all_values_on_connect: False
    sim:
      max_dt: 0
      rnn_weights: "$WEIGHTS"
      rnn_enable: false
      residual_log: "$csv"
      residual_log_hz: 100.0
      backend: $backend
      visualizations:
        rviz: {enabled: false}
        pdf: {enabled: false}
        record_states: {enabled: false}
        blender: {enabled: false}
      controller: oot
      oot_ctrl_mode: 0
EOF
}

fly_one() {
  local id=$1 backend=$2 scenario=$3 extra_args=$4
  local csv="$OUT/${id}_${backend}.csv"
  local yaml="$OUT/server_${id}_${backend}.yaml"
  write_server "$backend" "$csv" "$yaml"
  echo "=== $id backend=$backend scenario=$scenario ==="
  if [ "${REUSE_VALID_CSV:-0}" = 1 ] && [ -f "$csv" ]; then
    local existing
    existing=$(wc -l < "$csv")
    if [ "$existing" -ge 1000 ]; then
      echo "  reuse existing csv rows=$existing"
      return 0
    fi
  fi
  rm -f "$csv"
  source /opt/ros/humble/setup.bash
  cd "$CS2" && source install/setup.bash
  setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$yaml gui:=false rviz:=false > "$OUT/${id}_${backend}.launch.log" 2>&1 &
  GPID=$!
  sleep 15
  # Formation CLI args must follow `--` (otherwise rclpy treats them as ROS args).
  # shellcheck disable=SC2086
  timeout 380 ros2 run crazyflie_examples run_formation -- \
    --scenario "$scenario" --auto-center --yes $extra_args \
    --ros-args -p use_sim_time:=true \
    > "$OUT/${id}_${backend}.client.log" 2>&1 || true
  sleep 2
  kill -- -$GPID 2>/dev/null || true
  sleep 3
  local n
  n=$( [ -f "$csv" ] && wc -l < "$csv" || echo 0 )
  echo "  rows=$n csv=$csv"
  [ "$n" -ge 100 ] || return 1
}

# id | scenario | extra CLI (dz, A7 flags, ...)
run_case() {
  local id=$1 scen=$2
  shift 2
  fly_one "$id" neuralswarm "$scen" "$*" || exit 1
  fly_one "$id" np "$scen" "$*" || exit 1
}

[ -f "$WEIGHTS" ] || { echo "missing $WEIGHTS"; exit 1; }

run_case a1_dz030 A1 --dz 0.30
run_case a1_dz050 A1 --dz 0.50
run_case a2_dz030 A2 --dz 0.30
run_case a7_merge A7 --allow-extreme
A3NS="$REPO/experiments/sim_validation/c2_fullbank_predict.csv"
A3NP="$REPO/experiments/sim_validation/c2_fullbank_predict_np.csv"
EXTRA=()
if [ -f "$A3NS" ] && [ -f "$A3NP" ]; then
  EXTRA=(--extra "a3_dz030,$A3NS,$A3NP")
else
  fly_one a3_dz030 np A3 --dz 0.30 || exit 1
  EXTRA=(--extra "a3_dz030,$A3NS,$OUT/a3_dz030_np.csv")
fi

echo "=== aggregate ==="
"$PY" "$REPO/experiments/analysis/c2_sil_backend_scenario_sweep.py" \
  --sweep-dir "$OUT" "${EXTRA[@]}" \
  -o "$REPO/experiments/analysis/out/c2_e2e_2026-09-27/sil_backend_scenario_sweep.json"
