#!/usr/bin/env bash
# uSD / CS2 runtime-param diagnosis (2026-11-20). NO motors, NO takeoff. Writes everything to debug/lab_logs/usd_diag_<time>.log.
# Question it answers: when a usd.* param is set at runtime (ros2 CLI, broadcast, Python gate test), does
#   (1) the ROS parameter event get published, (2) the crazyflie_server print "Update parameter" (= forwards it to the drone),
#   (3) the drone's usd.runTag (read back through the usd_status topic) change?
#
# Usage (lab PC, shell where `ros2` works, drones on the pad, motors idle):
#   Terminal 1:  cd ~/flying_robot_course && mkdir -p debug/lab_logs && ros2 launch crazyflie launch.py 2>&1 | tee debug/lab_logs/cs2_launch_$(date +%H%M%S).log
#                (wait until both drones are connected + 10 s)
#   Terminal 2:  cd ~/flying_robot_course && experiments/diffcotune/lab/usd_cs2_diag.sh
# The script never commits or pushes; it only writes the log files (debug/lab_logs/usd_diag_*). You commit and push yourself.
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO" || exit 1
mkdir -p debug/lab_logs
TS=$(date +%H%M%S)
OUT="$REPO/debug/lab_logs/usd_diag_$(date +%F)_$TS.log"
LAUNCHLOG=$(ls -t "$REPO"/debug/lab_logs/cs2_launch_*.log 2>/dev/null | head -1)
WS="${WS:-$HOME/georg/ros2_ws}"
CS2SRC="${CS2SRC:-$WS/src/crazyswarm2}"

log() { printf '%s\n' "$*" | tee -a "$OUT"; }
sec() { log ""; log "=== $* ($(date +%H:%M:%S))"; }
run() { log "\$ $*"; "$@" 2>&1 | tee -a "$OUT"; }

sec "1. ENVIRONMENT"
log "host $(hostname)  user $USER  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}  RMW=${RMW_IMPLEMENTATION:-default}"
log "flying_robot_course: $(git -C "$REPO" log --oneline -1)"
log "crazyswarm2 src $CS2SRC: $(git -C "$CS2SRC" log --oneline -1 2>&1 | cut -c1-90)  branch $(git -C "$CS2SRC" branch --show-current 2>&1)"
BIN="$WS/install/crazyflie/lib/crazyflie/crazyflie_server"
run ls -l --time-style=full-iso "$BIN"
log "pacing marker count in server binary: $(strings "$BIN" 2>/dev/null | grep -c CS2_CONNECT_PARAM_PACE)   (src of main has it; 0 = binary older than 2026-09-29)"
log "server source mtime: $(stat -c %y "$CS2SRC/crazyflie/src/crazyflie_server.cpp" 2>&1)"
YSRC="$CS2SRC/crazyflie/config/crazyflies.yaml"
YINS="$WS/install/crazyflie/share/crazyflie/config/crazyflies.yaml"
log "yaml src sha $(sha256sum "$YSRC" 2>&1 | cut -c1-16)   install sha $(sha256sum "$YINS" 2>&1 | cut -c1-16)   (install is a symlink? $(readlink "$YINS" 2>&1 || true))"
log "query_all_values_on_connect in yaml: $(grep -n query_all_values "$YSRC" 2>&1 | head -2)"
log "ROS: ROS_DISTRO=${ROS_DISTRO:-unset} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
log "--- apt/dpkg ROS-related install/upgrade lines (last 40; look for dates after 2026-10-24):"
{ grep -hE "ros-(jazzy|humble|rolling)|rclcpp|rmw|fastrtps|cyclonedds" /var/log/apt/history.log* /var/log/dpkg.log* 2>/dev/null | grep -E "upgrade|install|Upgrade|Install" | tail -40; } 2>&1 | cut -c1-220 | tee -a "$OUT"
log "--- ldd of the server (missing libs / rclcpp):"
ldd "$BIN" 2>&1 | grep -E "not found|rclcpp" | tee -a "$OUT"
log "crazyflie_server processes: $(pgrep -af crazyflie_server | cut -c1-120 | tr '\n' ';')"
log "launch log used for server lines: ${LAUNCHLOG:-NONE (start CS2 with the tee command from the header)}"
pgrep -f crazyflie_server >/dev/null || { log "STOP: CS2 is not running. Start it (Terminal 1 command in the header), then rerun."; exit 1; }

val() {  # val <cf> -> "runTag fatWrBps" from the usd_status topic (what the drone itself reports)
  local raw v
  raw=$(timeout 10 ros2 topic echo --once "/$1/usd_status" 2>&1)
  v=$(printf '%s\n' "$raw" | sed -n '/^values:/,$p' | grep -E '^- ' | sed 's/^- //' | tr '\n' ' ')
  if [ -n "$v" ]; then printf '%s' "$v"; else printf 'NO_DATA(raw: %s)' "$(printf '%s' "$raw" | head -c 120 | tr '\n' ' ')"; fi
}
loglines() { [ -n "$LAUNCHLOG" ] && wc -l < "$LAUNCHLOG" || echo 0; }

RES=()   # verdict lines
probe() {  # probe <label> <ros param name> <value> <cf to read back> [grep pattern for the server Update line, default usd\.]
  local label="$1" pname="$2" value="$3" cf="$4" pat="${5:-usd\.}"
  local EV="/tmp/usd_diag_ev_${TS}_${label}.txt"
  sec "TEST $label: ros2 param set /crazyflie_server $pname $value"
  local before after L0 L1 ev_seen srv_seen
  before=$(val "$cf"); log "drone $cf before (runTag fatWrBps): $before"
  L0=$(loglines)
  ( timeout 6 ros2 topic echo /parameter_events > "$EV" 2>&1 & )
  sleep 1
  log "\$ ros2 param set /crazyflie_server $pname $value"
  ros2 param set /crazyflie_server "$pname" "$value" 2>&1 | tee -a "$OUT"
  sleep 3
  after=$(val "$cf"); log "drone $cf after  (runTag fatWrBps): $after"
  L1=$(loglines)
  if [ -n "$LAUNCHLOG" ]; then
    log "--- server log lines produced during the test:"
    tail -n +$((L0+1)) "$LAUNCHLOG" | head -$((L1-L0+5)) | cut -c1-220 | tee -a "$OUT"
    srv_seen=$(tail -n +$((L0+1)) "$LAUNCHLOG" | grep -c "Update parameter.*${pat}")
  else srv_seen="n/a"; fi
  ev_seen=$(grep -c "${pat%\\.}" "$EV" 2>/dev/null)
  log "--- parameter_events lines mentioning usd: $ev_seen ; server 'Update parameter usd.*' lines: $srv_seen"
  RES+=("$label: event_published=$([ "${ev_seen:-0}" -gt 0 ] && echo yes || echo NO)  server_forwarded=$([ "$srv_seen" = n/a ] && echo n/a || { [ "${srv_seen:-0}" -gt 0 ] && echo yes || echo NO; })  drone_before=[$before] drone_after=[$after]")
}

sec "2. BASELINE (what the drones report now)"
log "cf5: $(val cf5)   cf_second: $(val cf_second)   (runTag fatWrBps; a value like 777 = leftover from an earlier cflib test, power-cycle clears it)"

probe cf5_cli        cf5.params.usd.runTag        999 cf5
probe cf2_cli        cf_second.params.usd.runTag  997 cf_second
probe all_broadcast  all.params.usd.runTag        995 cf5
log "cf_second after broadcast: $(val cf_second)"
# generic harmless param that exists in the stock TOC (LED bitmask, no effect on flight): does ANY runtime set produce a server Update line?
probe generic_led_on  cf5.params.led.bitmask 1 cf5 'led\.bitmask'
probe generic_led_off cf5.params.led.bitmask 0 cf5 'led\.bitmask'

sec "3. PYTHON GATE TEST (CS2 path used by the flight scripts; writes one test file per card if it works, no motors)"
L0=$(loglines)
python3 flying_drone_stack/tools/usd_gate_ground_test.py --mode pass 2>&1 | tee -a "$OUT"
if [ -n "$LAUNCHLOG" ]; then
  sec "server log lines during the gate test (look for 'Update parameter' usd.runTag / usd.logging)"
  tail -n +$((L0+1)) "$LAUNCHLOG" | grep -E "Update parameter|usd|uSD|Logging|error|ERROR|WARN" | cut -c1-220 | tee -a "$OUT"
fi

sec "4. VERDICT LINES"
for r in "${RES[@]}"; do log "$r"; done
log ""
log "How to read: event_published=NO -> ROS side of the server never sees the set. event yes + server_forwarded NO -> the server callback drops it."
log "             server_forwarded yes + drone unchanged -> the drone/firmware ignores it. All yes + drone changed -> runtime path works (then the gate/flight script is the problem)."
log "LOG FILE: $OUT"

log "Log written (not committed). To share it:  cd $REPO && git add debug/lab_logs && git commit -m logs && git push"
