#!/bin/bash
R=/home/georg/Desktop/flying_robot_course/experiments/analysis/run_sim_matrix.sh
echo "### geo 2 A4"; bash $R one geo 2 --scenario A4 --dz 0.60 --offset 0.10 --laps 1
echo "### indi 2 A4"; bash $R one indi 2 --scenario A4 --dz 0.60 --offset 0.10 --laps 1
echo A4DONE
