#!/bin/bash
# A3 speed sweep: "one drone flies from one point to another through the downwash" at
# varying peak translation speed -- 0.1, 0.2, 0.3, 0.4, 0.5 m/s x {geo, indi} = 10 runs.
#
#   run_speed_sweep.sh
#
# A3 ("static-top: fixed wash source, lower vehicle sweeps through -- captures entry and
# exit") is the scenario that matches this directly: one vehicle hovers, the other
# translates underneath it through the wash. A coarse 2-point check (0.30/0.50 m/s)
# already exists in SPEED_MATRIX.md; this is the dense version requested 2026-09-03.
#
# Reuses run_sim_matrix.sh's one_run()/run_with_retry() via --source-only (built for
# exactly this: "Load the helpers without running anything, for a companion matrix
# script.") rather than duplicating the launch/verify logic.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_A3_results.md"
source ./run_sim_matrix.sh --source-only

header

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    ARGS="--scenario A3 --dz 0.30 --speed $SPEED --passes 2"
    echo "### $CTRL 2 $ARGS"
    run_with_retry "$CTRL" 2 $ARGS
  done
done

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
