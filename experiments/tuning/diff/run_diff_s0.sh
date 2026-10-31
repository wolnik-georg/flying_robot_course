#!/bin/bash
# Gradient tuning of S0 (option A: solo, no downwash), 3 seeds, GPU. TAG=s0 (narrow randomisation) or s0w (wide, --wide). Marker out/diff_$TAG.done when finished.
TAG=${1:-s0}; EXTRA=""; case "$TAG" in s0w|s0v) EXTRA="--wide";; s0x) EXTRA="--wide --robust";; esac
cd "$(dirname "$0")"
SEEDS=${2:-0 1 2}
for s in $SEEDS; do
  python3 train_s0.py --seed $s --iters 800 --Bs 512 512 256 --win 1.0 2.0 4.0 --patience 15 --val_every 20 --out ../out/diff_${TAG}_seed$s.json $EXTRA > ../out/diff_${TAG}_seed$s.log 2>&1
done
touch ../out/diff_${TAG}${3}.done
