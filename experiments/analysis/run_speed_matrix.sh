#!/bin/bash
# Speed/separation grid validation (Phase 2 of the speed-parameterisation fix).
#
#   run_speed_matrix.sh
#
# Confirms that --speed is honoured end to end in the ROS simulator, on both the
# period-paced paths that used to ignore it (A2 circle, A4 lemniscate) and on a hover
# scenario where speed is not a factor at all (A1).
#
# This is a REGIME-COVERAGE check, not a search: 16 cells, chosen to exercise each
# fix once on each controller. It is deliberately not the cross product of the grids.
#
# Reuses one_run() from run_sim_matrix.sh so the launch, pairing and verification
# logic stays in one place.

REPO=/home/georg/Desktop/flying_robot_course
OUT=$REPO/experiments/sim_validation
export RESULTS=$OUT/speed_matrix_results.md
mkdir -p "$OUT"

# shellcheck source=/dev/null
source "$REPO/experiments/analysis/run_sim_matrix.sh" --source-only

printf '| Scenario | Ctrl | N | Verdict | mean\\|ez\\| mm | RMSE mm | max tilt | diverged | covered | params | states |\n|---|---|---|---|---|---|---|---|---|---|---|\n' > "$RESULTS"

# 1) Circle tracking -- the path that silently ignored --speed before the fix.
# 2) Offset stack -- lateral separation crossed with a converted lemniscate speed.
# 3) Hover -- speed is not a parameter; separation only.
CASES=(
  "2|--scenario A2 --dz 0.40 --speed 0.30 --laps 1"
  "2|--scenario A2 --dz 0.40 --speed 0.50 --laps 1"
  "2|--scenario A2 --dz 0.30 --speed 0.30 --laps 1"
  "2|--scenario A2 --dz 0.30 --speed 0.50 --laps 1"
  "2|--scenario A4 --dz 0.60 --offset 0.10 --speed 0.30 --laps 1"
  "2|--scenario A4 --dz 0.60 --offset 0.20 --speed 0.30 --laps 1"
  "2|--scenario A1 --dz 0.50 --hold 10"
  "2|--scenario A1 --dz 0.30 --hold 10"
)

for CTRL in geo indi; do
  for C in "${CASES[@]}"; do
    NROB=${C%%|*}
    ARGS=${C#*|}
    echo "=== $CTRL  $ARGS"
    # shellcheck disable=SC2086
    run_with_retry "$CTRL" "$NROB" $ARGS
  done
done

echo
echo "Results -> $RESULTS"
