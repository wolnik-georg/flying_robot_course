#!/bin/bash
# Re-run of exactly the 19 not-yet-clean cases identified in
# docs/21_Formation_Speed_Sweep_Report.md sec.5, after bumping run_sim_matrix.sh's client
# timeout 380->700 (server 420->760) to fix categories 1/2. Category 3 (A8/geo/speed=0.2,
# a speed-tolerance-only flag unrelated to truncation) is re-checked here too, to see if it
# was a one-off -- the timeout bump should have no effect on it either way.
#
#   run_speed_sweep_requeue.sh
#
# Reuses run_sim_matrix.sh's one_run()/run_with_retry() via --source-only, same pattern as
# every other sweep in this series.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_requeue_results.md"
source ./run_sim_matrix.sh --source-only

header

# Category 1: trajectory duration alone exceeds the old timeout
echo "### geo 2 --scenario A2 --speed 0.1 (category 1)"
run_with_retry geo 2 --scenario A2 --dz 0.30 --speed 0.1
echo "### indi 2 --scenario A2 --speed 0.1 (category 1)"
run_with_retry indi 2 --scenario A2 --dz 0.30 --speed 0.1
echo "### geo 2 --scenario A5 --period 47.123 (category 1, speed 0.1)"
run_with_retry geo 2 --scenario A5 --dz 0.50 --radius 0.75 --period 47.123 --laps 2.0
echo "### indi 2 --scenario A5 --period 47.123 (category 1, speed 0.1)"
run_with_retry indi 2 --scenario A5 --dz 0.50 --radius 0.75 --period 47.123 --laps 2.0

# Category 2: fixed 3-drone overhead truncates low speed
for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3; do
    echo "### $CTRL 3 --scenario B1 --speed $SPEED (category 2)"
    run_with_retry "$CTRL" 3 --scenario B1 --dz2 0.30 --speed "$SPEED"
  done
done
for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3; do
    echo "### $CTRL 3 --scenario B2 --speed $SPEED (category 2)"
    run_with_retry "$CTRL" 3 --scenario B2 --dz2 0.30 --r 0.10 --speed "$SPEED"
  done
done
echo "### geo 3 --scenario C2 --speed 0.1 (category 2)"
run_with_retry geo 3 --scenario C2 --speed 0.1 --rotate 90
echo "### indi 3 --scenario C2 --speed 0.1 (category 2)"
run_with_retry indi 3 --scenario C2 --speed 0.1 --rotate 90

# Category 3: speed-tolerance-only, timeout bump should have no effect
echo "### geo 2 --scenario A8 --timescale 1.8229 (category 3, speed 0.2 recheck)"
run_with_retry geo 2 --scenario A8 --dz 0.25 --span 1.0 --timescale 1.8229

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
