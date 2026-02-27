#!/usr/bin/env bash
set -euo pipefail

# Interactive sweep: runs assignment5 across a grid and pauses after each run
# Prompts the user to continue/stop. Useful when you want to inspect logs/plots
# between runs.

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$ROOT/target/debug/assignment5"

# Ensure binary exists
if [ ! -x "$BIN" ]; then
    echo "Building assignment5 binary..."
    (cd "$ROOT" && cargo build --bin assignment5)
fi

mkdir -p "$ROOT/results/data"

# Parameter grids (edit as desired)
taus=(0.01 0.02 0.03 0.05)
Rs=(5 10 20 40)

for tau in "${taus[@]}"; do
  for R in "${Rs[@]}"; do
    echo "\n=== Running tau=$tau, R=$R ==="
    (cd "$ROOT" && "$BIN" figure8 1.0 1.0 1e6 "$tau" "$R")

    # archive outputs
    mv -f "$ROOT/results/data/assignment5_planned_figure8.csv" "$ROOT/results/data/assignment5_planned_figure8_tau${tau}_R${R}.csv"
    mv -f "$ROOT/results/data/assignment5_closedloop_figure8.csv" "$ROOT/results/data/assignment5_closedloop_figure8_tau${tau}_R${R}.csv"

    echo "Run finished. Files written to results/data/assignment5_planned_figure8_tau${tau}_R${R}.csv and assignment5_closedloop_figure8_tau${tau}_R${R}.csv"

    # Prompt user whether to continue
    while true; do
      read -p "Continue to next run? [Y/n/s(ave & stop)]: " yn
      case "$yn" in
        [Yy]*|"")
          echo "Continuing..."
          break
          ;;
        [Nn]*)
          echo "Stopping interactive sweep as requested."
          exit 0
          ;;
        [Ss]*)
          echo "Saving progress and exiting. You can resume manually by running this script again.";
          exit 0
          ;;
        *)
          echo "Please answer yes (Y), no (n) or s to stop and save." ;;
      esac
    done
  done
done

echo "Interactive sweep complete."
