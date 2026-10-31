#!/bin/bash
# FBL retune on the lag-30 plant (CMA-ES 3 seeds, budget 120, pop 12), then S2: reuse of S0's gains, then own tuning (CMA-ES 3 seeds per scenario, budget 240, pop 16)
cd "$(dirname "$0")/../.."; export CFFIRMWARE_DIR=~/Desktop/crazyflie-firmware/build; O=experiments/tuning/out
python3 experiments/tuning/reference_eval.py > $O/reference_eval_lag30.log 2>&1
for s in 0 1 2; do python3 experiments/tuning/run_job.py fbln --method cmaes --seed $s --budget 120 --pop 12 > $O/fbln_cmaes_$s.log 2>&1; done
python3 experiments/tuning/s2_reuse.py > $O/s2_reuse.log 2>&1
for sc in A1 A4 A8; do for s in 0 1 2; do python3 experiments/tuning/run_job.py s2p_$sc --method cmaes --seed $s --budget 240 --pop 16 > $O/s2p_${sc}_cmaes_$s.log 2>&1; done; done
touch $O/fbl_s2_chain.done
