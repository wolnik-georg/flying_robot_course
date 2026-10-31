#!/bin/bash
# Option 1 (no dz 0.25 cells), lag-30 plant, CMA-ES 3 seeds: S0 (s0q), FBL (fblq, budget 120 pop 12), S2 own tuning (s2q). Starts when the previous chain is done.
cd "$(dirname "$0")/../.."; export CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build; O=experiments/tuning/out
while [ ! -f $O/fbl_s2_chain.done ]; do sleep 60; done
for sc in A1 A4 A8; do for s in 0 1 2; do python3 experiments/tuning/run_job.py s0q_$sc --method cmaes --seed $s --budget 240 --pop 16 > $O/s0q_${sc}_cmaes_$s.log 2>&1; done; done
for s in 0 1 2; do python3 experiments/tuning/run_job.py fblq --method cmaes --seed $s --budget 120 --pop 12 > $O/fblq_cmaes_$s.log 2>&1; done
for sc in A1 A4 A8; do for s in 0 1 2; do python3 experiments/tuning/run_job.py s2q_$sc --method cmaes --seed $s --budget 240 --pop 16 > $O/s2q_${sc}_cmaes_$s.log 2>&1; done; done
touch $O/q_chain.done
