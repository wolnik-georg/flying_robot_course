#!/bin/bash
# Extended speed sweep: A2 (2-robot tracking), B1 (I-stack, 3-robot), B2 (V-stack, 3-robot)
# x 0.1-0.5 m/s in 0.1 steps x {geo, indi} = 30 runs.
#
#   run_speed_sweep_extended.sh
#
# Companion to run_speed_sweep.sh (which did the same for A3). These three are the other
# speed-parameterized scenarios in the library that matter for the comparative study --
# A2/B1/B2 all default to path='circle', so --speed goes through the period conversion
# fixed 2026-08-24 (SPEED_MATRIX.md), same as A3.
#
# Reuses run_sim_matrix.sh's one_run()/run_with_retry() via --source-only, same as
# run_speed_sweep.sh.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_A2_B1_B2_results.md"
source ./run_sim_matrix.sh --source-only

header

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 2 --scenario A2 --speed $SPEED"
    run_with_retry "$CTRL" 2 --scenario A2 --dz 0.30 --speed "$SPEED"
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 3 --scenario B1 --speed $SPEED"
    run_with_retry "$CTRL" 3 --scenario B1 --dz2 0.30 --speed "$SPEED"
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 3 --scenario B2 --speed $SPEED"
    run_with_retry "$CTRL" 3 --scenario B2 --dz2 0.30 --r 0.10 --speed "$SPEED"
  done
done

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
