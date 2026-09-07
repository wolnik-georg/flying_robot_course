#!/bin/bash
# Second extended speed sweep: A4 (line variant), A6 (extreme), A7 (extreme), C2, C5
# x 0.1-0.5 m/s in 0.1 steps x {geo, indi} = 50 runs.
#
#   run_speed_sweep_extended2.sh
#
# Completes speed characterisation of every scenario in the library that has a working
# --speed parameter, beyond A2/A3/B1/B2 (done in run_speed_sweep.sh /
# run_speed_sweep_extended.sh). The rest of the library (A1,A5,A8,B3,C1,C3,C4) is
# hover/duration-fixed or period-only-without-a-speed-param and has no speed axis to sweep.
#
# A4: only the --motion line variant responds to --speed at all (its default motion is
# lemniscate, paced by period, and A4 does not expose period<-speed conversion the way
# A2/B1/B2 do since it builds the lemniscate curve directly rather than through _path()).
# A6/A7: both scenario-gated extreme (tight 0.10 m separation) -- --allow-extreme required;
# sim-only, per A6's own docstring ("fly last, and only in sim first"). A7 additionally
# needs a lower base --height (0.5 m) so its dz_start=1.10 m top vehicle doesn't leave the
# z geofence (default height=1.0 puts it at 2.10 m, over the 1.70 m ceiling).
# C2: needs --rotate 90 so its along-track length (3 robots x 0.5 m gap + 1.2 m travel)
# fits the x-geofence instead of the y one.
# C5 (single robot): reuses run_sim_matrix.sh's existing NROB=1 -> crazyflies_sim1.yaml path.
#
# Reuses run_sim_matrix.sh's one_run()/run_with_retry() via --source-only, same pattern as
# both prior sweeps. This is a freshly-sourced process, so it picks up the
# committed pairing-bug fix (c4b4bc4) from the start -- no manual recovery expected this
# time, unlike the previous sweep which was already running when that fix landed.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_A4_A6_A7_C2_C5_results.md"
source ./run_sim_matrix.sh --source-only

header

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 2 --scenario A4 --motion line --speed $SPEED"
    run_with_retry "$CTRL" 2 --scenario A4 --motion line --offset 0.10 --dz 0.60 --speed "$SPEED"
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 2 --scenario A6 --speed $SPEED"
    run_with_retry "$CTRL" 2 --scenario A6 --dz 0.10 --speed "$SPEED" --allow-extreme
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 2 --scenario A7 --speed $SPEED"
    run_with_retry "$CTRL" 2 --scenario A7 --speed "$SPEED" --allow-extreme --height 0.5
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 3 --scenario C2 --speed $SPEED"
    run_with_retry "$CTRL" 3 --scenario C2 --speed "$SPEED" --rotate 90
  done
done

for CTRL in geo indi; do
  for SPEED in 0.1 0.2 0.3 0.4 0.5; do
    echo "### $CTRL 1 --scenario C5 --speed $SPEED"
    run_with_retry "$CTRL" 1 --scenario C5 --speed "$SPEED"
  done
done

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
