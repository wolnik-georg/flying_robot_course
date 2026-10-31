#!/bin/bash
# S0 on the lag-30 plant: search variants + motor lag 45 ms, CMA-ES, 3 seeds, budget 240, pop 16
cd "$(dirname "$0")/../.."; export CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build
for sc in A1 A4 A8; do for m in cmaes; do for s in 0 1 2; do
  python3 experiments/tuning/run_job.py s0n_$sc --method $m --seed $s --budget 240 --pop 16 > experiments/tuning/out/s0n_${sc}_${m}_$s.log 2>&1
done; done; done
