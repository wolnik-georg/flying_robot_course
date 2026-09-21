#!/bin/bash
# Desk smoke: re-verify B1 + B2 in sim (3 robots) and append a short report.
#
#   ./run_3drone_smoke.sh           # run B1 + B2 geo ( ~6–12 min each )
#   ./run_3drone_smoke.sh verify    # only verify latest paired meta + states on disk
#
# Full matrix history: docs/12_Sim_Formation_Validation_Report.md
# Thesis note: docs/33_Three_Drone_Sim_Smoke.md

set -euo pipefail
REPO=/home/georg/Desktop/flying_robot_course
CS2=/home/georg/Desktop/crazyswarm2
OUT=$REPO/experiments/sim_validation
REPORT=$OUT/three_drone_smoke_2026-09-21.md
MATRIX=$REPO/experiments/analysis/run_sim_matrix.sh
PY=${PY:-python3}

write_header() {
  cat > "$REPORT" <<EOF
# Three-drone sim smoke — $(date -Iseconds)

Commands use \`run_sim_matrix.sh one <geo|indi> 3\` with \`crazyflies_sim3.yaml\`.

| Scenario | Controller | Verdict | Notes |
|---|---|---|---|
EOF
}

append_row() {
  local sc=$1 ctrl=$2 verdict=$3 notes=$4
  printf '| %s | %s | %s | %s |\n' "$sc" "$ctrl" "$verdict" "$notes" >> "$REPORT"
}

verify_newest() {
  local SC=$1 CTRL=$2
  local LOGDIR=$REPO/experiments/logs
  local STATE=$CS2/state_$CTRL
  local META CSVDIR
  META=$(find "$LOGDIR" -name "${SC}_*.meta.json" -exec stat -c '%Y %n' {} \; 2>/dev/null \
    | sort -n | tail -1 | cut -d' ' -f2-)
  CSVDIR=$(find "$STATE" -maxdepth 2 -type d -name csv -exec stat -c '%Y %n' {} \; 2>/dev/null \
    | sort -n | tail -1 | cut -d' ' -f2-)
  if [ -z "$META" ] || [ -z "$CSVDIR" ]; then
    append_row "$SC" "$CTRL" "NO-DATA" "missing meta or record_states csv"
    return 1
  fi
  if $PY "$REPO/experiments/analysis/verify_formation_sim.py" "$META" "$CSVDIR" \
      --controller "$CTRL" --min-coverage 0.85 2>&1 | tee -a "$OUT/smoke_${SC}_${CTRL}.log"; then
    append_row "$SC" "$CTRL" "PASS" "\`$(basename "$META")\` + \`$(basename "$(dirname "$CSVDIR")")\`"
    return 0
  fi
  append_row "$SC" "$CTRL" "FAIL" "see $OUT/smoke_${SC}_${CTRL}.log"
  return 1
}

run_one() {
  local CTRL=$1; shift
  bash "$MATRIX" one "$CTRL" 3 "$@"
}

client_aborted() {
  local LOG=$OUT/client_$1.log
  [ -f "$LOG" ] && grep -q 'ABORT: EKF does not agree with mocap' "$LOG"
}

mode=${1:-run}
write_header

if [ "$mode" = "verify" ]; then
  verify_newest B1 geo || true
  verify_newest B2 geo || true
else
  echo "=== B1 geo ==="
  run_one geo --scenario B1 --dz2 0.30 --path line || true
  if client_aborted geo; then
    append_row B1 geo "ABORT" "EKF/mocap gate — no sidecar; see client_geo.log"
  else
    verify_newest B1 geo || true
  fi
  echo "=== B2 geo ==="
  run_one geo --scenario B2 --dz2 0.30 --r 0.10 --path line || true
  if client_aborted geo; then
    append_row B2 geo "ABORT" "EKF/mocap gate — no sidecar; see client_geo.log"
  else
    verify_newest B2 geo || true
  fi
fi

cat >> "$REPORT" <<EOF

## Hardware blockers (unchanged)

- Third airframe + mocap volume / anchor for \`crazyflies.yaml\` three-robot roster.
- uSD on all three for thesis-grade metrics (current pipeline is 2-drone merge).
- Lab time: B1/B2 are **Priority B** in the flight checklist (\`docs/07\`), after 2-drone C.1/C.4 core.

## Interpretation

- Sim **PASS** means commanded geometry is realised under the sim physics backend — not that downwash compensation works on hardware.
- B1 under **geometric** may show **EXPECTED** ~59 mm vertical error (combined wash of two neighbours); INDI typically passes tighter — see docs/12 §4.

EOF

echo "Wrote $REPORT"
