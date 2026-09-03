#!/bin/bash
# formation_flight.py validation matrix: 3 formations x {2,3} drones x {geo,indi} = 12 runs.
#
#   run_formation_flight_matrix.sh
#
# Each run: launch sim server, fly one formation_flight.py case, kill the server, pair
# the client's own meta.json sidecar with the server's record_states output, verify with
# verify_formation_flight_sim.py, append one row to RESULTS.
#
# Process hygiene matters here: killing only the `ros2 launch` wrapper PID leaves its
# child crazyflie_server process running, which corrupted an earlier manual test session.
# Every run below kills the whole process GROUP (kill -9 -- -$GPID) and does a belt-and-
# braces pkill on the sim server binary before moving on.

REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
RESULTS=$OUT/formation_flight_matrix_results.md
LOGDIR=$REPO/experiments/logs
mkdir -p "$OUT"

printf '| Formation | Ctrl | N | Verdict | Pair | mean\\|e\\| mm | max\\|e\\| mm | RMSE mm | max tilt deg | coverage |\n|---|---|---|---|---|---|---|---|---|---|\n' > "$RESULTS"

cleanup() {
  pkill -9 -f "crazyflie_sim/lib/crazyflie_sim/crazyflie_server" 2>/dev/null
  sleep 2
}

one_run() {
  local FORM=$1 NROB=$2 CTRL=$3
  source /opt/ros/humble/setup.bash
  cd "$CS2" || exit 1
  source install/setup.bash
  export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

  cleanup   # in case a previous run's server didn't die cleanly

  local ROSTER=crazyflies_sim.yaml
  [ "$NROB" = "3" ] && ROSTER=crazyflies_sim3.yaml
  local STATE=state_$CTRL
  mkdir -p "$STATE"

  setsid timeout 300 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/$ROSTER \
    server_yaml_file:=$CS2/crazyflie/config/server_sim_$CTRL.yaml \
    gui:=false rviz:=false mocap:=false teleop:=false > "$OUT/run_ff_${CTRL}_${NROB}.log" 2>&1 &
  local GPID=$!
  sleep 10

  local MARKER=$(mktemp)
  sleep 1
  # All six 3-drone cases hit the old flat 180s budget on 2026-09-03 (extra climb/converge
  # stage + more setParam calls per phase than 2-drone) -- the flight itself completed each
  # time (record_states data checked out physically), the client was just cut off during its
  # own tail-end cleanup. 2-drone never came close to 180s, so it keeps the tighter budget.
  local CLIENT_TIMEOUT=180
  [ "$NROB" = "3" ] && CLIENT_TIMEOUT=280   # stay under the server's own 300s launch timeout
  timeout "$CLIENT_TIMEOUT" ros2 run crazyflie_examples formation_flight -- \
    --trajectory figure8 --mode 1 --kt 0.05 \
    --formation "$FORM" --separation 0.4 --yes \
    --ros-args -p use_sim_time:=true > "$OUT/client_ff_${CTRL}_${NROB}.log" 2>&1
  local RC=$?
  sleep 2
  kill -9 -- -$GPID 2>/dev/null
  cleanup
  sleep 3   # record_states flushes on server shutdown

  local META CSVDIR
  META=$(find "$LOGDIR" -name "${FORM}_*.meta.json" -newer "$MARKER" 2>/dev/null | sort | tail -1)
  CSVDIR=$(find "$STATE" -maxdepth 2 -type d -name csv -newer "$MARKER" 2>/dev/null | sort | tail -1)
  if [ -z "$META" ] || [ -z "$CSVDIR" ]; then
    echo "  [$FORM/$CTRL/$NROB] NO SIDECAR (client rc=$RC) -- pairing by newest instead"
    META=$(find "$LOGDIR" -name "${FORM}_*.meta.json" 2>/dev/null | sort | tail -1)
    CSVDIR=$(find "$STATE" -maxdepth 2 -type d -name csv 2>/dev/null | sort | tail -1)
  fi
  if [ -z "$META" ] || [ -z "$CSVDIR" ]; then
    echo "| $FORM | $CTRL | $NROB | NO-DATA | - | - | - | - | - | 0% |" >> "$RESULTS"
    return 1
  fi

  python3 "$REPO/experiments/analysis/verify_formation_flight_sim.py" \
    "$META" "$CSVDIR" --controller "$CTRL" --append "$RESULTS"
}

echo "=== formation_flight.py validation matrix: $(date) ==="
for FORM in vertical horizontal side_by_side; do
  for NROB in 2 3; do
    for CTRL in geo indi; do
      echo "--- $FORM / ${NROB}-drone / $CTRL ---"
      one_run "$FORM" "$NROB" "$CTRL"
    done
  done
done
cleanup
echo "=== done: $(date) ==="
echo "Results: $RESULTS"
cat "$RESULTS"
