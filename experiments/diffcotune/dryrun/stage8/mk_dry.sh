#!/bin/bash
# usage: mk_dry.sh <controller> <lr>  -> jobs_dry8_<C>.json, jobs_rtest8_<C>.json, jobs_gate9_<C>.json (paper variant, forward gradient, 5 scenarios, 6 rounds; RPM delay fixed for S1 / S3)
cd "$(dirname "$0")"; C=$1; LR=$2; P=/usr/bin/python3; L=../../lab; FX=""; RF=""
if [ "$C" = S1 ] || [ "$C" = S3 ]; then FX="DIFFCOTUNE_FIX_RD=1"; RF="--fix-rd"; fi
python3 make_job.py dry8_$C $C 6 10 env DIFFCOTUNE_METHOD=paper DIFFCOTUNE_LR=$LR DIFFCOTUNE_GRAD_MODE=cfwd $FX $P run_8.py --tag paper
python3 make_job.py rtest8_$C $C 6 10 env DIFFCOTUNE_DRY_TAG=_8 DIFFCOTUNE_METHOD=paper DIFFCOTUNE_LR=$LR DIFFCOTUNE_GRAD_MODE=cfwd $P $L/dry_test5.py --controller $C $RF
python3 make_job.py gate9_$C $C 6 10 env DIFFCOTUNE_METHOD=paper DIFFCOTUNE_LR=$LR DIFFCOTUNE_GRAD_MODE=cfwd $FX $P gate9.py results/loop_${C}_paper.json
