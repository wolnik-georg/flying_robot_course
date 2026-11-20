#!/usr/bin/env bash
# One-command lab runner for the lab PC (2026-11-18). Wraps ONLY the commands of S0_LAB_CARD.md /
# INTERLEAVED_LAB_CARD.md; no new rules. You do: mocap, launch CS2, stop CS2 when asked, fly (Enter).
#
#   lab_run.sh prep S0 0      # yaml switch + gains merge, checks, readback (everything before the flights)
#   lab_run.sh flights        # prints the exact flight commands to copy (you fly them yourself)
#   lab_run.sh solo S0 0      # after A1, A8, A4: cf_second off, check, readback (before CIRCLE / FIG8)
# (prep S0 0 is also the shakedown prep: then fly only A1.)
# Paths: experiments/diffcotune/lab/lab_run.sh
#
# Run from ~/flying_robot_course in the shell where `ros2` works. Override if needed:
#   YAML=/path/to/crazyflies.yaml  CF5_URI=radio://0/80/2M/E7E7E7BB02  CF2_URI=radio://0/80/2M/E7E7E7E7E7
# DRY=1 prints hardware commands instead of running them (yaml edits still happen: use a scratch YAML).
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LAB="$REPO/experiments/diffcotune/lab"
if [ -z "${YAML:-}" ]; then
  for c in "$HOME/georg/ros2_ws/src/crazyswarm2" "$HOME/ros2_ws/src/crazyswarm2" "$HOME/Desktop/crazyswarm2"; do
    [ -f "$c/crazyflie/config/crazyflies.yaml" ] && { YAML="$c/crazyflie/config/crazyflies.yaml"; break; }
  done
fi
YAML="${YAML:-/NOT_FOUND/set_YAML}"
CF5_URI="${CF5_URI:-radio://0/80/2M/E7E7E7BB02}"
CF2_URI="${CF2_URI:-radio://0/80/2M/E7E7E7E7E7}"
LOG="${LOG:-$HOME/lab_run_log_$(date +%F).txt}"
DRY="${DRY:-0}"
cd "$REPO" || exit 1

say()  { printf '\n=== %s\n' "$*"; }
note() { printf '%s  %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
die()  { echo "STOP: $*" >&2; note "STOP: $*"; exit 1; }
ask()  { [ "$DRY" = 1 ] && return 0; read -r -p ">>> $* [Enter = go, Ctrl-C = stop] " _; }
hw()   { if [ "$DRY" = 1 ]; then echo "[dry] $*"; else "$@"; fi; }   # hardware command
need_cs2_stopped() {
  [ "$DRY" = 1 ] && return 0
  while pgrep -f crazyflie_server >/dev/null 2>&1; do
    echo "CS2 (crazyflie_server) still running: stop it (the radio is shared)."; read -r -p ">>> stopped? [Enter] " _
  done
}

stack_guard() {  # 2026-11-20 incident: an unnoticed ROS reinstall on the lab PC silently killed every runtime param set
  local CS2DIR WS BIN FP FPFILE="$HOME/.ros_jazzy_fingerprint_validated"
  CS2DIR="$(dirname "$(dirname "$(dirname "$YAML")")")"; WS="$(dirname "$(dirname "$CS2DIR")")"
  BIN="$WS/install/crazyflie/lib/crazyflie/crazyflie_server"
  [ "$DRY" = 1 ] && return 0
  grep -q "CS2_RUNTIME_PARAMS_POST_SET_V1" "$(readlink -f "$BIN")" 2>/dev/null \
    || die "server binary $BIN lacks the runtime-param fix (crazyswarm2 71219f3): run experiments/diffcotune/lab/apply_runtime_param_fix.sh"
  FP="$(dpkg-query -W -f='${Package} ${Version}\n' 'ros-jazzy-*' 2>/dev/null | sort | sha256sum | cut -c1-16)"
  if [ -f "$FPFILE" ] && [ "$(cat "$FPFILE")" != "$FP" ]; then
    echo "!!! ROS PACKAGES CHANGED since the last validated session ($(cat "$FPFILE") -> $FP)."
    echo "!!! Someone updated/reinstalled ROS. The uSD gate test below must pass before ANY flight; tell the planning agent."
    note "ROS fingerprint changed $(cat "$FPFILE") -> $FP"
    ask "Understood (continue to the checks)"
  fi
  ROS_FP_NOW="$FP"
}
prep() {  # prep <S0|S2|S1|S3> <round>
  local C="$1" L="$2" G="$LAB/$1/round_$2/gains.yaml"
  [ -f "$G" ] || die "no $G (round.py --init / desk step not done for $C round $L)"
  [ -f "$YAML" ] || die "yaml not found: $YAML (set YAML=...)"
  say "PREP $C round $L   yaml=$YAML"
  need_cs2_stopped
  stack_guard
  cp -n "$YAML" "$YAML.before_lab_run_2026-11-18" 2>/dev/null
  python3 flying_drone_stack/tools/set_tree_strategy.py "$C" --yaml "$YAML" || die "set_tree_strategy failed"
  python3 "$LAB/apply_gains.py" merge --fragment "$G" --yaml "$YAML" --out "$YAML.new" || die "merge failed"
  mv "$YAML.new" "$YAML" || die "could not put the merged yaml in place"
  note "$C round $L yaml sha256 $(sha256sum "$YAML" | cut -c1-16)"
  python3 flying_drone_stack/tools/lab_check_strategy.py "$(dirname "$(dirname "$(dirname "$YAML")")")" || die "lab_check_strategy failed"
  ask "Launch CS2 now (it applies the yaml to the drones), wait until both drones are connected, then STOP CS2 again"
  need_cs2_stopped
  # cflib checks need the radio: CS2 must be stopped for all of them
  hw python3 "$LAB/apply_gains.py" readback --uri "$CF5_URI" --fragment "$G" --strategy "$C" || die "READBACK MISMATCH: do not fly"
  hw python3 flying_drone_stack/tools/check_active_controller.py "$CF5_URI" 9 || die "cf5 is not running controller 9"
  hw python3 flying_drone_stack/tools/check_active_controller.py "$CF2_URI" 9 || die "cf_second is not running controller 9"
  hw python3 flying_drone_stack/tools/check_usd_deck.py "$CF5_URI" || die "usd deck check cf5"
  hw python3 flying_drone_stack/tools/check_usd_deck.py "$CF2_URI" || die "usd deck check cf_second"
  ask "Relaunch CS2 (drones on the pad, motors idle) for the uSD gate ground test"
  # uSD gate (2026-11-18): the CS2 broadcast path must make BOTH decks write; needs the new crazyswarm2 build + yaml topic usd_status
  hw python3 flying_drone_stack/tools/usd_gate_ground_test.py --mode pass || die "uSD gate failed: do not fly"
  echo ">>> REMINDER: the gate test left one test file (usd.runTag 888) on each card. Archive it before the flights"
  echo ">>>   (power off, cards out, move thesis* to _archive_<date>_gate_tests, cards back, then usd_card_count.py --expect 0)."
  note "PREP OK $C round $L (readback, controller 9 x2, usd deck x2)"
  [ "$DRY" = 1 ] || { echo "${ROS_FP_NOW:-}" > "$HOME/.ros_jazzy_fingerprint_validated"; note "ROS fingerprint validated: ${ROS_FP_NOW:-}"; }
}

flights() {
  cat <<EOT

Flights (run them yourself, one after the other, hand on abort: |roll| or |pitch| > 25 deg for 0.5 s, or z < 0.25 m):
 1 A1  : ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.375 --hold 15.0 --height 0.5 --auto-center --yes
 2 A8  : ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.375 --duration 8.0 --passes 2 --height 0.5 --auto-center --yes
 3 A4  : ros2 run crazyflie_examples run_formation --scenario A4 --dz 0.4 --speed 0.35 --offset 0.125 --motion line --rotate 90 --height 0.5 --auto-center --yes
 then  : stop CS2, run   experiments/diffcotune/lab/lab_run.sh solo <S0|S2> <round>   (cf_second off)
 4 CIRCLE: python3 experiments/diffcotune/lab/fly_file.py --cell CIRCLE --yaml $YAML
 5 FIG8  : python3 experiments/diffcotune/lab/fly_file.py --cell FIG8 --yaml $YAML
(shakedown: only flight 1)
EOT
}

solo() {  # solo <C> <round>
  local C="$1" L="$2" G="$LAB/$1/round_$2/gains.yaml"
  [ -f "$G" ] || die "no $G"; [ -f "$YAML" ] || die "yaml not found: $YAML"
  say "SOLO SWITCH $C round $L (cf_second disabled)"
  need_cs2_stopped
  python3 flying_drone_stack/tools/set_tree_strategy.py "$C" --solo --yaml "$YAML" || die "solo switch failed"
  python3 flying_drone_stack/tools/lab_check_strategy.py "$(dirname "$(dirname "$(dirname "$YAML")")")" || die "lab_check_strategy failed"
  ask "Launch CS2, wait for cf5, then STOP CS2 again (for the readback)"; need_cs2_stopped
  hw python3 "$LAB/apply_gains.py" readback --uri "$CF5_URI" --fragment "$G" --strategy "$C" || die "READBACK MISMATCH: do not fly"
  note "SOLO OK $C round $L"
  echo; echo ">>> Relaunch CS2, then fly CIRCLE and FIG8 (commands: lab_run.sh flights)"
}

case "${1:-}" in
  prep)    [ $# -eq 3 ] || die "usage: prep <S0|S2> <round>"; prep "$2" "$3"; flights;;
  solo)    [ $# -eq 3 ] || die "usage: solo <S0|S2> <round>"; solo "$2" "$3";;
  flights) flights;;
  *) sed -n 2,10p "$0"; exit 1;;
esac
