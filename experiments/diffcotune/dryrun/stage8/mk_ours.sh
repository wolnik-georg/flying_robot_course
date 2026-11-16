#!/bin/bash
# usage: mk_ours.sh <controller>  -> jobs_dry8o_<C>.json, jobs_rtest8o_<C>.json, jobs_gate9o_<C>.json  (fallback: DIFFCOTUNE_METHOD=ours, 5 scenarios, 6 rounds, cfwd, lr 0.01, +-25 % limit, noise 2 seeds, 50 epochs best iterate, J_sysId pvq; RPM delay fixed for S1 / S3)
cd "$(dirname "$0")"; C=$1; P=/usr/bin/python3; L=../../lab; FX=""; RF=""
if [ "$C" = S1 ] || [ "$C" = S3 ]; then FX="DIFFCOTUNE_FIX_RD=1"; RF="--fix-rd"; fi
python3 make_job.py dry8o_$C $C 6 10 env DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $FX $P run_8.py --tag ours
python3 make_job.py rtest8o_$C $C 6 10 env DIFFCOTUNE_DRY_TAG=_8o DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $P $L/dry_test5.py --controller $C $RF --method ours
python3 make_job.py gate9o_$C $C 6 10 env DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $FX $P gate9.py results/loop_${C}_ours.json
