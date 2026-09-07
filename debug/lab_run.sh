#!/bin/bash
# Prefix any lab command with this to also capture its full terminal output to a timestamped
# file under debug/lab_logs/, so it can be committed and pulled into a debugging chat on a
# different machine instead of copy-pasting terminal output by hand. Touches nothing else --
# simple_flight.py and run_formation.py run exactly as they would without this wrapper.
#
#   debug/lab_run.sh <command...>
#
# Examples:
#   debug/lab_run.sh ros2 run crazyflie_examples run_formation --scenario A1 --dz 0.75 \
#     --brushless --height 0.85 --auto-center --yes
#   debug/lab_run.sh ros2 run crazyflie_examples simple_flight -- --trajectory hover --duration 15
#   debug/lab_run.sh python3 debug/connect_probe.py --attempts 10 --pause 3

set -u
if [ $# -lt 1 ]; then
  echo "usage: $0 <command...>" >&2
  exit 2
fi

REPO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LOGDIR="$REPO_ROOT/debug/lab_logs"
mkdir -p "$LOGDIR"

# Auto-label from the command itself -- nothing to type or remember beyond the command you were
# already going to run. Sanitised to a filename-safe fragment, capped so it stays readable.
LABEL=$(echo "$*" | tr -c 'a-zA-Z0-9' '_' | sed 's/_\+/_/g; s/^_//; s/_$//' | cut -c1-50)
LOGFILE="$LOGDIR/$(date +%Y%m%d_%H%M%S)_${LABEL}.log"

{
  echo "### host: $(hostname)  time: $(date -Is)"
  echo "### cmd:  $*"
  echo "###"
} > "$LOGFILE"

stdbuf -oL -eL "$@" 2>&1 | tee -a "$LOGFILE"

echo
echo "### wrote $LOGFILE"
