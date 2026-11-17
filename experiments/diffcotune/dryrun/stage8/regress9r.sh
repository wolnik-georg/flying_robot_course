#!/bin/bash
# Stage 8e T5: full strict run of controller $1 (S0 or S2) with rule RS on; compare with the Stage 8d strict run (must be bit-identical: their only R3 round is round 0, which has no theta_best)
cd "$(dirname "$0")"; C=$1
env DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1 DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd /usr/bin/python3 run_8.py --tag ours_rs_regress --no-eval && /usr/bin/python3 compare_rs.py $C
