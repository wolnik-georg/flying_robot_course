#!/bin/bash
# Planning-agent step 3(a), 2026-11-20: rebuild ONLY the C++ crazyflie package (crazyflie_server) from the current crazyswarm2 main source.
# Also records the ROS package history (step 1) in the log. No hardware is touched. CS2 must be STOPPED first.
#   Usage (lab PC, venv shell as for the earlier builds):  cd ~/flying_robot_course && experiments/diffcotune/lab/rebuild_cs2_server.sh
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"; cd "$REPO" || exit 1
WS="${WS:-$HOME/georg/ros2_ws}"; CS2SRC="$WS/src/crazyswarm2"
OUT="$REPO/debug/lab_logs/rebuild_server_$(date +%F_%H%M%S).log"; mkdir -p "$REPO/debug/lab_logs"
log() { printf '%s\n' "$*" | tee -a "$OUT"; }
pgrep -f crazyflie_server >/dev/null && { echo "STOP: CS2 is running. Stop it first (Ctrl-C in the launch terminal)."; exit 1; }
log "=== ROS package history (what changed in ROS, with dates)"
for f in /var/log/apt/history.log /var/log/apt/history.log.*.gz; do
  [ -f "$f" ] || continue
  (zcat -f "$f" 2>/dev/null) | awk '/^Start-Date/{d=$0} /^Commandline/{c=$0} /^(Upgrade|Install|Remove):/{ if ($0 ~ /ros-jazzy|rclcpp|rmw|fastrtps|cyclonedds/) {print d; print c; print substr($0,1,300)} }'
done 2>&1 | tee -a "$OUT"
log "--- installed versions now:"
dpkg -l 2>/dev/null | awk '$2 ~ /^ros-jazzy-(rclcpp|rcl|rmw-fastrtps-cpp|rmw-implementation|rclcpp-action|rcl-interfaces)$/ {print $2, $3}' | tee -a "$OUT"
log "--- dpkg upgrade lines for rclcpp/rmw/fastrtps/rcl:"
(zcat -f /var/log/dpkg.log* 2>/dev/null) | grep " upgrade " | grep -E "ros-jazzy-(rclcpp |rcl |rmw|fastrtps|rcl-interfaces)" | cut -c1-200 | tail -20 | tee -a "$OUT"
log ""
log "=== BEFORE: $(ls -l --time-style=full-iso "$WS/build/crazyflie/crazyflie_server" 2>&1 | cut -c1-200)"
cp -a "$WS/build/crazyflie/crazyflie_server" "$HOME/crazyflie_server.before_rebuild_$(date +%F_%H%M)" 2>&1 | tee -a "$OUT"
log "crazyswarm2 src: $(git -C "$CS2SRC" log --oneline -1 | cut -c1-80)  branch $(git -C "$CS2SRC" branch --show-current)"
log "=== submodules of crazyswarm2 (the build needs crazyflie/deps/*):"
git -C "$CS2SRC" submodule status 2>&1 | tee -a "$OUT"
if [ ! -f "$CS2SRC/crazyflie/deps/crazyflie_tools/CMakeLists.txt" ]; then
  log "STOP: crazyflie/deps/crazyflie_tools is empty (git submodules not checked out). Run:"
  log "  cd $CS2SRC && git submodule update --init --recursive      # needs network; checks out the submodule commits recorded by main"
  log "then rerun this script. The server binary has NOT been changed."
  exit 1
fi
cd "$WS" || exit 1
log "=== colcon build crazyflie (several minutes)"
colcon build --symlink-install --packages-select crazyflie 2>&1 | tee -a "$OUT" | tail -15
cd "$REPO"
BIN="$WS/install/crazyflie/lib/crazyflie/crazyflie_server"
log "=== AFTER: $(ls -lL --time-style=full-iso "$BIN" 2>&1 | cut -c1-200)"
log "pacing marker count in the NEW binary: $(strings "$(readlink -f "$BIN")" 2>/dev/null | grep -c CS2_CONNECT_PARAM_PACE)   (must be >= 1)"
log "LOG FILE: $OUT"
echo ">>> Log written (not committed). To share it:  cd $REPO && git add debug/lab_logs && git commit -m logs && git push"
