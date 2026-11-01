#!/bin/bash
# Gradient tuning of S0 with downwash, per scenario type, 3 seeds each, GPU. Markers out/diff_s0D_<scen>.done per scenario, out/diff_s0D.done at the end.
cd "$(dirname "$0")"
for sc in ${1:-A8 A1 A4}; do
  for s in 0 1 2; do
    python3 train_scen.py --scen $sc --seed $s --iters 250 --B 160 --val_every 25 --out ../out/diff_s0D_${sc}_seed$s.json > ../out/diff_s0D_${sc}_seed$s.log 2>&1
  done
  touch ../out/diff_s0D_$sc.done
done
touch ../out/diff_s0D.done
