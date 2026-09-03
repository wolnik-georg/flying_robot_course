#!/bin/bash
# simple_flight.py validation matrix: {figure8,circle,oval,hover} x {1,2} drones = 8 runs.
#
#   run_simple_flight_matrix.sh
#
# simple_flight.py has no formation/offset/safety-gate logic and no verification sidecar
# by design (it's the deliberately-simple script) -- this checks it runs crash-free and, for
# the 2-drone cases, whether the two drones' recorded paths stay physically separated
# (there is no safety net stopping them from converging; this is exactly the property in
# question, not something to assume).

REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
RESULTS=$OUT/simple_flight_matrix_results.md
mkdir -p "$OUT"

printf '| Trajectory | N | Verdict | Exit | Notes |\n|---|---|---|---|---|\n' > "$RESULTS"

cleanup() {
  pkill -9 -f "crazyflie_sim/lib/crazyflie_sim/crazyflie_server" 2>/dev/null
  sleep 2
}

one_run() {
  local TRAJ=$1 NROB=$2
  source /opt/ros/humble/setup.bash
  cd "$CS2" || exit 1
  source install/setup.bash
  export PYTHONPATH=/home/georg/Desktop/crazyflie-firmware/build:${PYTHONPATH:-}

  cleanup

  local ROSTER=crazyflies_sim1.yaml
  [ "$NROB" = "2" ] && ROSTER=crazyflies_sim.yaml
  local STATE=state_geo
  mkdir -p "$STATE"

  setsid timeout 240 ros2 launch crazyflie launch.py backend:=sim \
    crazyflies_yaml_file:=$CS2/crazyflie/config/$ROSTER \
    server_yaml_file:=$CS2/crazyflie/config/server_sim_geo.yaml \
    gui:=false rviz:=false mocap:=false teleop:=false > "$OUT/run_sf_${TRAJ}_${NROB}.log" 2>&1 &
  local GPID=$!
  sleep 10

  local MARKER=$(mktemp)
  sleep 1
  local ARGS="--trajectory $TRAJ"
  [ "$TRAJ" = "hover" ] && ARGS="$ARGS --duration 8"
  timeout 150 ros2 run crazyflie_examples simple_flight -- $ARGS \
    --ros-args -p use_sim_time:=true > "$OUT/client_sf_${TRAJ}_${NROB}.log" 2>&1
  local RC=$?
  sleep 2
  kill -9 -- -$GPID 2>/dev/null
  cleanup
  sleep 3

  local CSVDIR
  CSVDIR=$(find "$STATE" -maxdepth 2 -type d -name csv -newer "$MARKER" 2>/dev/null | sort | tail -1)

  local HAS_TB="no"
  grep -q "Traceback" "$OUT/client_sf_${TRAJ}_${NROB}.log" && HAS_TB="yes"

  if [ "$RC" != "0" ] || [ "$HAS_TB" = "yes" ]; then
    echo "  [$TRAJ/$NROB] FAIL rc=$RC traceback=$HAS_TB"
    echo "| $TRAJ | $NROB | FAIL | $RC | traceback=$HAS_TB |" >> "$RESULTS"
    return 1
  fi

  local NOTE="clean, no verification sidecar (script has none by design)"
  if [ "$NROB" = "2" ] && [ -n "$CSVDIR" ]; then
    # No formation offsets in this script -- check the two drones' recorded paths didn't
    # converge to (near) the same point at any point in the flight, since nothing else does.
    local MINSEP
    MINSEP=$(python3 - "$CSVDIR" <<'PYEOF'
import sys, glob
import numpy as np
files = sorted(glob.glob(sys.argv[1] + "/*.csv"))
if len(files) < 2:
    print("n/a")
    sys.exit(0)
data = [np.loadtxt(f, delimiter=",", skiprows=1, ndmin=2) for f in files[:2]]
t0 = max(d[0, 0] for d in data)
t1 = min(d[-1, 0] for d in data)
grid = np.linspace(t0, t1, 200)
pos = [np.stack([np.interp(grid, d[:, 0], d[:, c]) for c in (1, 2, 3)], axis=1) for d in data]
d = np.linalg.norm(pos[0] - pos[1], axis=1)
print(f"{d.min():.3f}")
PYEOF
)
    NOTE="min inter-drone separation over the flight: ${MINSEP} m"
  fi
  echo "  [$TRAJ/$NROB] PASS -- $NOTE"
  echo "| $TRAJ | $NROB | PASS | 0 | $NOTE |" >> "$RESULTS"
}

echo "=== simple_flight.py validation matrix: $(date) ==="
for TRAJ in figure8 circle oval hover; do
  for NROB in 1 2; do
    echo "--- $TRAJ / ${NROB}-drone ---"
    one_run "$TRAJ" "$NROB"
  done
done
cleanup
echo "=== done: $(date) ==="
echo "Results: $RESULTS"
cat "$RESULTS"
