#!/usr/bin/env bash
# Diagnose + fix a stale (non-symlinked) crazyflie_examples ROS2 install.
#
# Symptom this targets: flight.py source edits (e.g. the z_mode conditional
# fix) don't take effect at flight time — `ros2 run` keeps executing an old
# compiled snapshot from a previous `colcon build` that wasn't run with
# --symlink-install. Confirmed as the root cause of a near-identical issue
# earlier in this session (see conversation / project_trajectory_port_2026-07
# memory) where the installed copy was 17 days stale.
#
# Usage: bash scripts/fix_stale_install.sh
set -euo pipefail

CS2_DIR="$HOME/Desktop/crazyswarm2"
SRC="$CS2_DIR/crazyflie_examples/crazyflie_examples/flight.py"

echo "=== 1. Locate the currently-installed flight.py ==="
INSTALLED=$(find "$CS2_DIR/install" -name "flight.py" 2>/dev/null | head -1)
if [ -z "$INSTALLED" ]; then
  echo "No installed flight.py found under $CS2_DIR/install — package was never built, or install dir is elsewhere."
else
  echo "Installed copy: $INSTALLED"
  if [ -L "$INSTALLED" ]; then
    echo "  -> symlink, points to: $(readlink -f "$INSTALLED")"
    echo "  -> GOOD: symlinked installs always reflect the latest source edits."
  else
    echo "  -> REGULAR FILE (not a symlink) — this is a static snapshot from the last colcon build."
    echo "  -> Installed mtime: $(stat -c '%y' "$INSTALLED" 2>/dev/null || stat -f '%Sm' "$INSTALLED")"
    echo "  -> Source mtime:    $(stat -c '%y' "$SRC" 2>/dev/null || stat -f '%Sm' "$SRC")"
  fi
fi

echo
echo "=== 2. Check whether the installed copy has the z_mode fix ==="
if [ -n "$INSTALLED" ]; then
  COUNT=$(grep -c "needs_z" "$INSTALLED" 2>/dev/null || echo 0)
  echo "Occurrences of 'needs_z' in installed copy: $COUNT"
  if [ "$COUNT" -eq 0 ]; then
    echo "  -> STALE. The installed copy predates the z_mode conditional fix. This is almost"
    echo "     certainly why flight still misbehaves even after firmware revert + git pull."
  else
    echo "  -> Up to date. The z_mode fix is present in the installed copy."
  fi
fi

echo
echo "=== 3. Rebuilding with --symlink-install (fixes this permanently) ==="
cd "$CS2_DIR"
colcon build --symlink-install --packages-select crazyflie_examples
echo
echo "=== 4. Re-checking installed copy after rebuild ==="
INSTALLED2=$(find "$CS2_DIR/install" -name "flight.py" 2>/dev/null | head -1)
if [ -L "$INSTALLED2" ]; then
  echo "OK: $INSTALLED2 is now a symlink -> $(readlink -f "$INSTALLED2")"
  echo "Future source edits will take effect immediately, no rebuild needed."
else
  echo "WARNING: still not a symlink after --symlink-install. Check for a stale build/ dir:"
  echo "  rm -rf $CS2_DIR/build/crazyflie_examples $CS2_DIR/install/crazyflie_examples"
  echo "  then re-run this script."
fi

echo
echo "=== 5. Don't forget to source the workspace in THIS shell before flying ==="
echo "  source $CS2_DIR/install/setup.bash"
