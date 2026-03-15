#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

# Use the project's pyenv environment which has numpy/matplotlib/pandas
export PYENV_VERSION=flying_robots
PY="python3"

echo "=== cargo test ==="
cargo test 2>&1 | grep "test result"

echo ""
echo "=== assignment1 ==="
cargo run --release --bin assignment1 2>&1 | grep -E "Done!|Saved to|error\[E"

echo ""
echo "=== assignment2 ==="
cargo run --release --bin assignment2 2>&1 | grep -E "Done!|Saved to|error\[E"

echo ""
echo "=== assignment3 ==="
cargo run --release --bin assignment3 2>&1 | grep -E "→|RMS|error\[E"

echo ""
echo "=== assignment4 ==="
cargo run --release --bin assignment4 2>&1 | grep -E "Written:|RMS|error\[E"

echo ""
echo "=== assignment5 hover ==="
cargo run --release --bin assignment5 -- hover 2>&1 | grep -E "Wrote|finished|error\[E"

echo ""
echo "=== assignment5 circle ==="
cargo run --release --bin assignment5 -- circle 2>&1 | grep -E "Wrote|finished|error\[E"

echo ""
echo "=== assignment5 figure8 ==="
cargo run --release --bin assignment5 -- figure8 2>&1 | grep -E "Wrote|finished|error\[E"

echo ""
echo "=== plot_assignment1.py ==="
$PY scripts/plot_assignment1.py 2>&1 | tail -3

echo ""
echo "=== plot_assignment2.py ==="
$PY scripts/plot_assignment2.py 2>&1 | tail -3

echo ""
echo "=== plot_assignment3.py ==="
$PY scripts/plot_assignment3.py 2>&1 | tail -5

echo ""
echo "=== plot_assignment4.py ==="
$PY scripts/plot_assignment4.py 2>&1 | tail -5

echo ""
echo "=== plot_assignment5.py ==="
$PY scripts/plot_assignment5.py 2>&1 | tail -5

echo ""
echo "=== ALL DONE ==="
