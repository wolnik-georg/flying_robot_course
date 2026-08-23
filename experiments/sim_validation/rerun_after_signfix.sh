#!/bin/bash
# Re-run every case invalidated by the a_res sign fix, plus the two geometric failures.
# The eight passing geometric runs are NOT re-run: a_indi is zero under geometric, so the
# fix cannot have changed them.
R=/home/georg/Desktop/flying_robot_course/experiments/analysis/run_sim_matrix.sh
for a in "--scenario A1 --dz 0.50 --hold 10" "--scenario A1 --dz 0.25 --hold 10" \
         "--scenario A2 --dz 0.30 --path line" "--scenario A3 --dz 0.30 --passes 2" \
         "--scenario A4 --dz 0.60 --offset 0.10 --laps 1" "--scenario A5 --dz 0.50 --laps 1" \
         "--scenario A8 --dz 0.25"; do
  echo "### indi 2 $a"; bash $R one indi 2 $a
done
for a in "--scenario B1 --dz2 0.30 --path line" "--scenario B2 --dz2 0.30 --r 0.10 --path line" \
         "--scenario B3 --dz 0.22"; do
  echo "### indi 3 $a"; bash $R one indi 3 $a
done
echo "### geo 2 A4"; bash $R one geo 2 --scenario A4 --dz 0.60 --offset 0.10 --laps 1
echo "### geo 3 B1"; bash $R one geo 3 --scenario B1 --dz2 0.30 --path line
echo RERUN12DONE
