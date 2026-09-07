#!/bin/bash
# Third and final speed sweep: A5, A8, B3, C4 -- the scenarios paced by --period or
# --duration instead of --speed, converted to the same 0.1-0.5 m/s target grid used
# everywhere else. 5 speeds x 2 controllers x 4 scenarios = 40 runs.
#
#   run_speed_sweep_extended3.sh
#
# Closes the last gap in speed-characterising the whole formation library (A1, C1, C3
# remain untouched -- they are pure hover, nothing moves, so there is no speed-like axis
# to vary at all).
#
# A5/A8/B3/C4 have no --speed CLI argument; their pace is set directly by --period (A5)
# or --duration (A8, B3, C4). Peak speed scales as 1/period (or 1/duration) for a fixed
# geometry -- the same scaling law scenarios.py's own build() uses internally to convert
# --speed to --period for circle/lemniscate paths (see its "Speed handling" section) --
# so the period/duration values below were derived by building each scenario once at a
# reference pace, measuring its realised peak speed, and scaling linearly to hit each
# 0.1-0.5 m/s target. Confirmed by direct measurement, not assumed:
#   A5 (radius=0.75, ref period=7.5s -> peak 0.628 m/s):
#     0.1->47.123s  0.2->23.562s  0.3->15.708s  0.4->11.781s  0.5->9.425s
#   A8 (span=1.0, ref duration=6.0s -> peak 0.365 m/s):
#     0.1->21.875s  0.2->10.937s  0.3->7.292s  0.4->5.469s  0.5->4.375s
#   B3 (span=0.55, ref duration=8.0s -> peak 0.301 m/s):
#     0.1->24.062s  0.2->12.031s  0.3->8.021s  0.4->6.016s  0.5->4.812s
#   C4 (lateral=1.0, ref duration=8.0s -> peak 0.295 m/s):
#     0.1->23.560s  0.2->11.780s  0.3->7.853s  0.4->5.890s  0.5->4.712s
# C4 ends at dz_end=0.10m by construction (< 0.15m), so it is tagged extreme and needs
# --allow-extreme regardless of speed -- confirmed via --dry-run --check.
#
# Reuses run_sim_matrix.sh's one_run()/run_with_retry() via --source-only, same pattern
# as every prior sweep in this series.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_A5_A8_B3_C4_results.md"
source ./run_sim_matrix.sh --source-only

header

for CTRL in geo indi; do
  for PAIR in "0.1 47.123" "0.2 23.562" "0.3 15.708" "0.4 11.781" "0.5 9.425"; do
    set -- $PAIR
    echo "### $CTRL 2 --scenario A5 --period $2 (speed $1)"
    run_with_retry "$CTRL" 2 --scenario A5 --dz 0.50 --radius 0.75 --period "$2" --laps 2.0
  done
done

for CTRL in geo indi; do
  for PAIR in "0.1 21.875" "0.2 10.937" "0.3 7.292" "0.4 5.469" "0.5 4.375"; do
    set -- $PAIR
    echo "### $CTRL 2 --scenario A8 --duration $2 (speed $1)"
    run_with_retry "$CTRL" 2 --scenario A8 --dz 0.25 --span 1.0 --duration "$2" --settle 2.0
  done
done

for CTRL in geo indi; do
  for PAIR in "0.1 24.062" "0.2 12.031" "0.3 8.021" "0.4 6.016" "0.5 4.812"; do
    set -- $PAIR
    echo "### $CTRL 3 --scenario B3 --duration $2 (speed $1)"
    run_with_retry "$CTRL" 3 --scenario B3 --dz 0.22 --span 0.55 --duration "$2" --settle 2.0
  done
done

for CTRL in geo indi; do
  for PAIR in "0.1 23.560" "0.2 11.780" "0.3 7.853" "0.4 5.890" "0.5 4.712"; do
    set -- $PAIR
    echo "### $CTRL 2 --scenario C4 --duration $2 (speed $1)"
    run_with_retry "$CTRL" 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
      --duration "$2" --settle 2.0 --allow-extreme
  done
done

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
