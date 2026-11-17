#!/bin/bash
# Stage F2.4 regression: full strict run of S0 or S2 with rule RS on after the FBL edits of loop_lib.py / lab_pool.py; compare with the Stage 8d strict run (must be bit-identical: max abs diff 0.0)
cd "$(dirname "$0")/../stage8"; C=$1
env DIFFCOTUNE_DRY_STRICT=1 DIFFCOTUNE_RESTART_BEST=1 DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd /usr/bin/python3 run_8.py --tag ours_rs_regressF2 --no-eval && /usr/bin/python3 ../fbl/compare_rsF2.py $C
