#!/usr/bin/env bash
# Lab PC, CS2 STOPPED. Pulls crazyswarm2 71219f3 (runtime params via post-set callback), rebuilds the
# server, checks the marker. Keeps the local yaml edits (the fix only changes crazyflie_server.cpp).
# Log: debug/lab_logs/apply_fix_<time>.log
set -u
FRC=$HOME/georg/flying_robot_course
WS=$HOME/georg/ros2_ws
CS2=$WS/src/crazyswarm2
YAML=$CS2/crazyflie/config/crazyflies.yaml
OUT=$FRC/debug/lab_logs/apply_fix_$(date +%Y-%m-%d_%H%M%S).log
mkdir -p "$(dirname "$OUT")"
exec > >(tee -a "$OUT") 2>&1
fail() { echo; echo "STOP: $*"; echo "Log: $OUT"; exit 1; }

if pgrep -f "crazyflie_server" >/dev/null; then fail "CS2 is running. Stop it (Ctrl+C in its terminal), then run this again."; fi
set +u; source /opt/ros/jazzy/setup.bash; set -u   # ROS setup scripts use unset variables

echo "== 1. yaml backup + sha before"
cp "$YAML" "$HOME/crazyflies.yaml.before_fix_$(date +%H%M%S)"
echo "yaml sha before: $(sha256sum "$YAML" | cut -c1-16)"

echo "== 2. pull crazyswarm2"
cd "$CS2" || fail "no $CS2"
git pull --ff-only || fail "git pull failed (see above)"
HEAD=$(git rev-parse --short HEAD); echo "crazyswarm2 HEAD: $HEAD ($(git branch --show-current))"
git merge-base --is-ancestor 71219f3 HEAD || fail "71219f3 is not in this branch"
echo "yaml sha after:  $(sha256sum "$YAML" | cut -c1-16)   (must equal 'before')"

echo "== 3. build the server"
cd "$WS" && { set +u; colcon build --symlink-install --packages-select crazyflie; RC=$?; set -u; } && [ $RC -eq 0 ] || fail "build failed"
BIN=$WS/install/crazyflie/lib/crazyflie/crazyflie_server
N=$(grep -c "CS2_RUNTIME_PARAMS_POST_SET_V1" "$(readlink -f "$BIN")")
echo "fix marker in binary: $N   (must be 1 or more)"
[ "$N" -ge 1 ] || fail "new binary does not contain the fix"

echo "== 4. pull flying_robot_course"
cd "$FRC" && git pull --ff-only || fail "flying_robot_course pull failed"

echo
echo "DONE: fix built. Next: step 2 (launch CS2). Log: $OUT"
