#!/bin/bash
# Correction to run_speed_sweep_extended3.sh: A8/B3/C4 were swept using a --duration flag
# that run_formation.py does not have (it uses parse_known_args, so the flag was silently
# dropped and every "speed" actually flew at the scenario's default duration). The real
# lever is --timescale, which run_formation.py applies at upload time
# (allcfs.startTrajectory(0, timescale=...)) and which verify_formation_sim.py already
# divides out when computing commanded speed. A5's part of the original sweep is fine (it
# used the real --period flag) and is not re-run here.
#
#   run_speed_sweep_extended3b.sh
#
# timescale values were derived the same way as before: build each scenario once at its
# default pace, measure peak speed, then timescale = v_ref / target (since real velocity
# scales as 1/timescale for a fixed geometry). Confirmed by direct measurement:
#   A8 (span=1.0, ref duration=6.0s/settle=2.0s -> peak 0.365 m/s):
#     0.1->3.6458  0.2->1.8229  0.3->1.2153  0.4->0.9115  0.5->0.7292
#   B3 (span=0.55, ref duration=8.0s/settle=2.0s -> peak 0.301 m/s):
#     0.1->3.0078  0.2->1.5039  0.3->1.0026  0.4->0.7520  0.5->0.6016
#   C4 (lateral=1.0, ref duration=8.0s/settle=2.0s -> peak 0.295 m/s):
#     0.1->2.9450  0.2->1.4725  0.3->0.9817  0.4->0.7363  0.5->0.5890
# All three confirmed safe via --dry-run --check at their fastest (timescale<1) setting
# before running for real -- timescale<1 means the physically flown speed is FASTER than
# the nominal curve the safety gate itself evaluates (the gate does not know about
# --timescale), so this was checked explicitly rather than assumed safe. Sim-only, so the
# consequence of a miss would be a bad number, not a crash.

REPO=/home/georg/Desktop/flying_robot_course
cd "$REPO/experiments/analysis" || exit 1

RESULTS="$REPO/experiments/sim_validation/speed_sweep_A5_A8_B3_C4_results.md"
source ./run_sim_matrix.sh --source-only

for CTRL in geo indi; do
  for PAIR in "0.1 3.6458" "0.2 1.8229" "0.3 1.2153" "0.4 0.9115" "0.5 0.7292"; do
    set -- $PAIR
    echo "### $CTRL 2 --scenario A8 --timescale $2 (speed $1)"
    run_with_retry "$CTRL" 2 --scenario A8 --dz 0.25 --span 1.0 --timescale "$2"
  done
done

for CTRL in geo indi; do
  for PAIR in "0.1 3.0078" "0.2 1.5039" "0.3 1.0026" "0.4 0.7520" "0.5 0.6016"; do
    set -- $PAIR
    echo "### $CTRL 3 --scenario B3 --timescale $2 (speed $1)"
    run_with_retry "$CTRL" 3 --scenario B3 --dz 0.22 --span 0.55 --timescale "$2"
  done
done

for CTRL in geo indi; do
  for PAIR in "0.1 2.9450" "0.2 1.4725" "0.3 0.9817" "0.4 0.7363" "0.5 0.5890"; do
    set -- $PAIR
    echo "### $CTRL 2 --scenario C4 --timescale $2 (speed $1)"
    run_with_retry "$CTRL" 2 --scenario C4 --lateral 1.0 --dz-start 0.50 --dz-end 0.10 \
      --timescale "$2" --allow-extreme
  done
done

echo SWEEPDONE
echo "Results: $RESULTS"
cat "$RESULTS"
