#!/usr/bin/env bash
# Pre-generate Poly4D CSVs for all likely flight combinations.
# Run once before a lab session (or after pulling new Rust changes).
#
# Output: crazyswarm2/crazyflie_examples/crazyflie_examples/data/
#   {label}.csv          — HLC Poly4D (Mode E, default flight.py)
#   {label}_onboard.csv  — OOT traj params (Mode D, flight.py --onboard)
#
# Usage:  bash Controls/pregenerate_poly4d.sh
#         (run from any directory inside the repo)
#
# kt → approximate lap time reference (figure-8, ~5 m path):
#   kt=0.008 → 9.1 s   (gentle start)
#   kt=0.02  → 8.1 s
#   kt=0.05  → 6.9 s
#   kt=0.1   → 5.9 s
#   kt=0.2   → 4.9 s
#   kt=0.5   → 3.7 s   (fast — likely exceeds CF2.1 thrust/ω limits in sim)
#   kt=0.75  → 3.2 s
#   kt=1.0   → 2.9 s   (v_avg ~2.0 m/s)
#   kt=1.2   → 2.7 s
#   kt=1.4   → 2.5 s
#   kt=1.49  → 2.5 s   (Richter QP ceiling for this figure-8; kt≥1.5 diverges)
#   kt=2.0   → N/A     (QP infeasible — use Mode 0 --speed 2.0 for ~3.6 s/lap instead)
#
# kt → approximate lap time reference (circle, r=0.25m):
#   kt=0.01  → 2.4 s   (0.66 m/s)
#   kt=0.02  → 2.2 s   (0.72 m/s)
#   kt=0.05  → 1.8 s   (0.86 m/s)  ← upper safe limit; kt≥0.1 causes QP divergence
#   (circle mode 0 speed variants cover the practical slow-to-fast range instead)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STACK_DIR="$(cd "$SCRIPT_DIR/../flying_drone_stack" && pwd)"

echo "=== Pre-generating Poly4D CSVs ==="
echo "    Stack : $STACK_DIR"
echo ""

run() {
    echo -n "  --trajectory $1 --mode $2$([ $# -ge 3 ] && echo " $3 $4" || true) ... "
    if output=$(cargo run --release --bin export_poly4d -- \
                    --trajectory "$1" --mode "$2" "${@:3}" 2>&1); then
        echo "$output" | grep -oE "total=[0-9.]+s" || echo "ok"
    else
        echo "FAILED"
        echo "$output" | tail -3 >&2
    fi
}

run_onboard() {
    echo -n "  --trajectory $1 --mode $2$([ $# -ge 3 ] && echo " $3 $4" || true) --onboard ... "
    if output=$(cargo run --release --bin export_poly4d -- \
                    --trajectory "$1" --mode "$2" "${@:3}" --onboard 2>&1); then
        echo "$output" | grep -oE "total=[0-9.]+s" || echo "ok"
    else
        echo "FAILED"
        echo "$output" | tail -3 >&2
    fi
}

cd "$STACK_DIR"

# ── figure8 ────────────────────────────────────────────────────────────────
echo "── figure8  Mode 0 (fixed-duration spline, speed variants) ──"
run figure8 0                          # 7.28 s (default)
run figure8 0 --speed 0.5             # 14.56 s  (half speed)
run figure8 0 --speed 0.75            # 9.71 s
run figure8 0 --speed 1.5             # 4.85 s
run figure8 0 --speed 2.0             # 3.64 s   (double speed)
echo ""

echo "── figure8  Mode 1 (Richter auto-timing) ──"
run figure8 1 --kt 0.008              # 9.1 s
run figure8 1 --kt 0.02               # 8.1 s
run figure8 1 --kt 0.05               # 6.9 s
run figure8 1 --kt 0.1                # 5.9 s
run figure8 1 --kt 0.2                # 4.9 s
run figure8 1 --kt 0.3                # 4.4 s
run figure8 1 --kt 0.5                # 3.7 s
run figure8 1 --kt 0.75               # 3.2 s
run figure8 1 --kt 1.0                # 2.9 s  (v_avg ~2.0 m/s)
run figure8 1 --kt 1.2                # 2.7 s
run figure8 1 --kt 1.4                # 2.5 s
run figure8 1 --kt 1.49               # 2.5 s  (Richter QP ceiling)
echo ""

echo "── figure8  Mode 1 onboard (OOT traj params, flight.py --onboard) ──"
run_onboard figure8 1 --kt 0.008
run_onboard figure8 1 --kt 0.05
run_onboard figure8 1 --kt 0.1
run_onboard figure8 1 --kt 0.2
run_onboard figure8 1 --kt 0.3
run_onboard figure8 1 --kt 0.5
run_onboard figure8 1 --kt 0.75
run_onboard figure8 1 --kt 1.0
run_onboard figure8 1 --kt 1.2
run_onboard figure8 1 --kt 1.4
run_onboard figure8 1 --kt 1.49
echo ""

echo "── figure8  Mode 2 (SE3) ──"
run figure8 2 --kt 0.008
run figure8 2 --kt 0.05
run figure8 2 --kt 0.1
echo ""

echo "── figure8  Mode 3 (Paper / Richter + redistribution) ──"
run figure8 3 --kt 0.008
run figure8 3 --kt 0.05
run figure8 3 --kt 0.1
echo ""

# ── circle ─────────────────────────────────────────────────────────────────
# Note: circle mode 0 + speed variants is the primary way to vary speed.
# Mode 1/2/3 work only up to kt≈0.05 — above that the Richter QP diverges
# because the short arc segments (~0.196 m each) push T below T_MIN=0.15 s.
echo "── circle  Mode 0 (fixed-duration spline, speed variants) ──"
run circle 0                           # 10.47 s (ω=0.6 rad/s baseline)
run circle 0 --speed 0.5              # 20.94 s  (half speed)
run circle 0 --speed 0.75             # 13.96 s
run circle 0 --speed 1.5              # 6.98 s
run circle 0 --speed 2.0              # 5.24 s
echo ""

echo "── circle  Mode 1/2/3 (kt≤0.05 only — higher values cause QP divergence) ──"
run circle 1 --kt 0.01
run circle 1 --kt 0.02
run circle 1 --kt 0.05
run circle 2 --kt 0.01
run circle 2 --kt 0.02
run circle 2 --kt 0.05
run circle 3 --kt 0.01
run circle 3 --kt 0.02   # kt=0.05 causes QP divergence for mode 3 circle
echo ""

DATA_DIR="/home/georg/Desktop/crazyswarm2/crazyflie_examples/crazyflie_examples/data"
N=$(ls "$DATA_DIR"/*.csv 2>/dev/null | wc -l)
echo "=== All done. $N CSVs in $DATA_DIR"
