#!/bin/bash
# Sim formation validation matrix.
#
#   run_sim_matrix.sh all                       # the whole matrix (~1 h, 20 runs)
#   run_sim_matrix.sh one geo 2 --scenario A3 --dz 0.30
#
# Runs each scenario end to end in the ROS simulator and verifies the realised geometry
# against what the scenario commanded, appending one row per run to RESULTS.
#
# Two yaml files select two different things:
#   crazyflies_sim.yaml / crazyflies_sim3.yaml -> HOW MANY ROBOTS
#   server_sim_geo.yaml / server_sim_indi.yaml -> WHICH CONTROLLER
# The controller cannot be chosen from the runner: run_formation is a client and the
# controller lives in the server process.
#
# Each run takes roughly 3 minutes -- the neuralswarm backend is about 4x slower than
# real time, and every run pays a fresh server start.

# NOT `set -u`: ROS's setup.bash references unbound variables and aborts under it.
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
RESULTS=$OUT/matrix_results.md
LOGDIR=$REPO/experiments/logs
mkdir -p "$OUT"

header() {
  printf '| Scenario | Ctrl | N | Verdict | mean\\|ez\\| mm | RMSE mm | max tilt | diverged | covered | params | states |\n|---|---|---|---|---|---|---|---|---|---|---|\n' > "$RESULTS"
}

one_run() {
  local CTRL=$1 NROB=$2; shift 2
  source /opt/ros/humble/setup.bash
  cd "$CS2" || exit 1
  source install/setup.bash
  export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

  local ROSTER=crazyflies_sim.yaml
  [ "$NROB" = "3" ] && ROSTER=crazyflies_sim3.yaml
  local STATE=state_$CTRL
  rm -rf "$STATE"

  setsid timeout 420 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/$ROSTER \
    server_yaml_file:=$CS2/crazyflie/config/server_sim_$CTRL.yaml \
    gui:=false rviz:=false > "$OUT/run_$CTRL.log" 2>&1 &
  local GPID=$!
  sleep 15

  local MARKER=$(mktemp)
  sleep 1                       # so "newer than MARKER" cannot catch a same-second file
  timeout 380 ros2 run crazyflie_examples run_formation "$@" --yes \
    --ros-args -p use_sim_time:=true > "$OUT/client_$CTRL.log" 2>&1
  local RC=$?
  sleep 2
  kill -- -$GPID 2>/dev/null
  sleep 5          # record_states flushes on server shutdown

  # Pair with THIS run's sidecar only. Taking the newest sidecar unconditionally is
  # wrong: if a run dies before it writes one, the previous run's file is still the
  # newest, and the flight silently gets verified against a different scenario's
  # commanded geometry. That produced a plausible-looking but meaningless number once
  # (A1 at dz=0.25 scored against dz=0.50, giving exactly the 0.21 m difference).
  local META CSVDIR
  META=$(find "$LOGDIR" -name '*.meta.json' -newer "$MARKER" 2>/dev/null | sort | tail -1)
  CSVDIR=$(ls -dt "$STATE"/*/csv 2>/dev/null | head -1)
  if [ -z "$META" ] || [ -z "$CSVDIR" ]; then
    echo "  [$CTRL/$NROB $*] NO SIDECAR from this run (client rc=$RC)"
    return 1
  fi
  ~/.pyenv/versions/flying_robots/bin/python \
    "$REPO/experiments/analysis/verify_formation_sim.py" \
    "$META" "$CSVDIR" --controller "$CTRL" --append "$RESULTS" --min-coverage 0.9
  return $?
}

# One retry. The sim clock free-runs before the client connects, so a slow start can
# leave the trajectory beginning after the recording ends -- a transient, not a real
# failure of the scenario. Retrying once absorbs it; a second failure is recorded.
run_with_retry() {
  local CTRL=$1 NROB=$2; shift 2
  if one_run "$CTRL" "$NROB" "$@"; then
    echo "RUNDONE $CTRL/$NROB/$*"; return 0
  fi
  echo "  retrying $CTRL/$NROB/$*"
  if one_run "$CTRL" "$NROB" "$@"; then
    echo "RUNDONE $CTRL/$NROB/$* (2nd attempt)"; return 0
  fi
  echo "| ${*} | $CTRL | $NROB | NO-DATA | - | - | - | - | 0% | - | failed twice |" >> "$RESULTS"
  echo "RUNDONE $CTRL/$NROB/$* (FAILED TWICE)"
  return 1
}

# Priority A, two robots. Lap/pass counts trimmed where the default is long; the
# geometry under test is unchanged.
A_SET=(
  "--scenario A1 --dz 0.50 --hold 10"
  "--scenario A1 --dz 0.25 --hold 10"
  "--scenario A2 --dz 0.30 --path line"
  "--scenario A3 --dz 0.30 --passes 2"
  "--scenario A4 --dz 0.60 --offset 0.10 --laps 1"
  "--scenario A5 --dz 0.50 --laps 1"
  "--scenario A8 --dz 0.25"
)
# Priority B, three robots
B_SET=(
  "--scenario B1 --dz2 0.30 --path line"
  "--scenario B2 --dz2 0.30 --r 0.10 --path line"
  "--scenario B3 --dz 0.22"
)

case "${1:-all}" in
  one)
    shift
    one_run "$@"
    ;;
  all)
    header
    for CTRL in geo indi; do
      for ARGS in "${A_SET[@]}"; do
        echo "### $CTRL 2 $ARGS"; run_with_retry $CTRL 2 $ARGS
      done
      for ARGS in "${B_SET[@]}"; do
        echo "### $CTRL 3 $ARGS"; run_with_retry $CTRL 3 $ARGS
      done
    done
    echo MATRIXDONE
    ;;
  *)
    echo "usage: $0 all | one <geo|indi> <2|3> --scenario ..." ; exit 2 ;;
esac
