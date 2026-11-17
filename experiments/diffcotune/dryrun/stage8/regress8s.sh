#!/bin/bash
# Stage 8d T1 (b): strict = 0 regression, round 0 of S0 "ours" only
cd "$(dirname "$0")"
env DIFFCOTUNE_DRY_STRICT=0 DIFFCOTUNE_METHOD=ours DIFFCOTUNE_GRAD_MODE=cfwd /usr/bin/python3 run_8.py --tag regress8s --stop-after 1 --no-eval && /usr/bin/python3 compare_regress8s.py
