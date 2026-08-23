#!/bin/bash
# Re-run the geometric cases that predate the a_res sign fix, so the whole validation
# record comes from one build. Geometric behaviour is provably unchanged by that fix --
# a_indi is zero under geometric -- and the probe measured the same 10.2 mm sag before
# and after. This is belt and braces, not a correction.
R=/home/georg/Desktop/flying_robot_course/experiments/analysis/run_sim_matrix.sh
for a in "--scenario A1 --dz 0.50 --hold 10" "--scenario A1 --dz 0.25 --hold 10" \
         "--scenario A2 --dz 0.30 --path line" "--scenario A3 --dz 0.30 --passes 2" \
         "--scenario A5 --dz 0.50 --laps 1" "--scenario A8 --dz 0.25"; do
  echo "### geo 2 $a"; bash $R one geo 2 $a
done
for a in "--scenario B2 --dz2 0.30 --r 0.10 --path line" "--scenario B3 --dz 0.22"; do
  echo "### geo 3 $a"; bash $R one geo 3 $a
done
echo "### geo 3 B1"; bash $R one geo 3 --scenario B1 --dz2 0.30 --path line
echo GEODONE
