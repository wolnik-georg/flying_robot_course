#!/bin/bash
# Stage 8d: usage mk_strict.sh <controller> -> jobs_dry8s_<C>.json, jobs_rtest8s_<C>.json, jobs_gate9s_<C>.json (strict dry run: DIFFCOTUNE_DRY_STRICT=1, tag ours_strict; same method settings as mk_ours.sh); S0 also jobs_regress8s_S0.json
cd "$(dirname "$0")"; C=$1; P=/usr/bin/python3; L=../../lab; FX=""; RF=""
if [ "$C" = S1 ] || [ "$C" = S3 ]; then FX="DIFFCOTUNE_FIX_RD=1"; RF="--fix-rd"; fi
python3 make_job.py dry8s_$C $C 6 10 env DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $FX $P run_8.py --tag ours_strict
python3 make_job.py rtest8s_$C $C 6 10 env DIFFCOTUNE_DRY_TAG=_8s DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $P $L/dry_test5.py --controller $C $RF --method ours --strict-dry
python3 make_job.py gate9s_$C $C 6 10 env DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd $FX $P gate9s.py results/loop_${C}_ours_strict.json
if [ "$C" = S0 ]; then python3 make_job.py regress8s_S0 S0 6 10 ./regress8s.sh; fi
