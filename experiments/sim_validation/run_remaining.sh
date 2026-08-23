#!/bin/bash
# The library scenarios not yet in the matrix: A6, A7, C1-C5, both controllers.
R=/home/georg/Desktop/flying_robot_course/experiments/analysis/run_sim_matrix.sh
for C in geo indi; do
  bash $R one $C 2 --scenario A6 --dz 0.10 --allow-extreme
  bash $R one $C 2 --scenario A7 --allow-extreme
  bash $R one $C 2 --scenario C4 --allow-extreme
  bash $R one $C 3 --scenario C1
  bash $R one $C 3 --scenario C2 --gap 0.40 --length 0.90
  bash $R one $C 3 --scenario C3
  bash $R one $C 1 --scenario C5
done
echo REMAINDONE
