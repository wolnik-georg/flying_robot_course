#!/bin/bash
# Stage E (docs/40): full-bank weights, A3 dz=0.30, predict vs compensate (rnn.en 0/1).
set -eo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
WEIGHTS=$REPO/experiments/analysis/out/c2_e2e_2026-09-26/full_bank_40.npz
export FLYING_ROBOT_COURSE_ROOT="${FLYING_ROBOT_COURSE_ROOT:-$REPO}"
export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

fly() {
  local CFG=$1 LABEL=$2 OUTCSV=$3 SERVER=$4
  echo "=== $LABEL ==="
  source /opt/ros/humble/setup.bash
  cd "$CS2" && source install/setup.bash
  rm -f "$OUTCSV"
  setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/crazyflies_sim.yaml \
    server_yaml_file:=$SERVER \
    gui:=false rviz:=false > "$OUT/${CFG}.log" 2>&1 &
  GPID=$!
  sleep 15
  timeout 380 ros2 run crazyflie_examples run_formation --scenario A3 --dz 0.30 \
    --auto-center --yes --ros-args -p use_sim_time:=true \
    > "$OUT/c2_fullbank_client_${CFG}.log" 2>&1 || true
  sleep 2
  kill -- -$GPID 2>/dev/null || true
  sleep 3
  N=$( [ -f "$OUTCSV" ] && wc -l < "$OUTCSV" || echo 0 )
  echo "  rows=$N csv=$OUTCSV"
  [ "$N" -ge 100 ] || return 1
  return 0
}

mkdir -p "$OUT"
[ -f "$WEIGHTS" ] || { echo "missing $WEIGHTS"; exit 1; }

PRED_CSV="$OUT/c2_fullbank_predict.csv"
if [ ! -f "$PRED_CSV" ] || [ "$(wc -l < "$PRED_CSV")" -lt 100 ]; then
  fly fullbank_predict "predict rnn.en=0" "$PRED_CSV" \
    "$OUT/server_c2_fullbank_predict.yaml" || exit 1
else
  echo "=== predict rnn.en=0 (reuse existing $PRED_CSV) ==="
fi

# compensate arm — new yaml
cat > "$OUT/server_c2_fullbank_compensate.yaml" <<EOF
/crazyflie_server:
  ros__parameters:
    warnings:
      frequency: 1.0
    firmware_params:
      query_all_values_on_connect: False
    sim:
      max_dt: 0
      rnn_weights: "$WEIGHTS"
      rnn_enable: true
      residual_log: "$OUT/c2_fullbank_compensate.csv"
      residual_log_hz: 100.0
      backend: neuralswarm
      visualizations:
        rviz: {enabled: false}
        pdf: {enabled: false}
        record_states: {enabled: false}
        blender: {enabled: false}
      controller: oot
      oot_ctrl_mode: 0
EOF

fly fullbank_compensate "compensate rnn.en=1" "$OUT/c2_fullbank_compensate.csv" \
  "$OUT/server_c2_fullbank_compensate.yaml" || exit 1

python3 "$REPO/experiments/analysis/c2_stage_e_summary.py" \
  --predict "$PRED_CSV" \
  --compensate "$OUT/c2_fullbank_compensate.csv" \
  --weights "full_bank_40.npz (17-file bank, 181489 samples)" \
  -o "$OUT/c2_fullbank_stage_e.json"
echo "Wrote $OUT/c2_fullbank_stage_e.json"
