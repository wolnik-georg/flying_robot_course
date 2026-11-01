#!/bin/bash
# Longer-horizon gradient tuning of S0 (chained 3 s chunks over 18 s flights), seeds given as $1 (default 0-5). Marker out/diff_s0L.done when finished.
cd "$(dirname "$0")"
for s in ${1:-0 1 2 3 4 5}; do
  python3 train_long.py --seed $s --iters 300 --B 256 --val_every 25 --out ../out/diff_s0L_seed$s.json > ../out/diff_s0L_seed$s.log 2>&1
done
touch ../out/diff_s0L.done
