#!/bin/sh
# FBL end-to-end on the calibrated plant: references, then CMA-ES and Sobol (same budget) for seeds 0-2. Logs next to the jsonl files.
cd "$(dirname "$0")/../.." || exit 1
export CFFIRMWARE_DIR=${CFFIRMWARE_DIR:-$HOME/Desktop/crazyflie-firmware/build}
python3 experiments/tuning/reference_eval.py > experiments/tuning/out/reference_eval.log 2>&1
for s in 0 1 2; do
  python3 experiments/tuning/run_job.py fbl --method cmaes --seed $s --budget 120 --pop 12 > experiments/tuning/out/fbl_cmaes_$s.log 2>&1
  python3 experiments/tuning/run_job.py fbl --method sobol --seed $s --budget 120 --pop 12 > experiments/tuning/out/fbl_sobol_$s.log 2>&1
done
