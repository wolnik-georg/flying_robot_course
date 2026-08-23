#!/bin/bash
# Re-run only the cases that did not pass, and nothing else.
#
#   rerun_failures.sh            # re-run every FAIL / NO-DATA row
#   rerun_failures.sh --dry-run  # list what would be re-run
#
# A passing run is evidence and does not need repeating. The one exception is a change
# to the simulator itself: that invalidates every earlier result, and then the full
# matrix has to run again (`run_sim_matrix.sh all`). Re-running only failures is right
# whenever the FIX is to a scenario, a tolerance, or the harness -- not to the plant or
# the controller.
#
# Results are appended, so the file keeps the history. summarise_matrix.py collapses it
# to the latest verdict per case for the report.

REPO=/home/georg/Desktop/flying_robot_course
CASES=$REPO/experiments/analysis/sim_matrix_cases.txt
RESULTS=$REPO/experiments/sim_validation/matrix_results.md
DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

if [ ! -f "$RESULTS" ]; then
  echo "no results yet -- run: experiments/analysis/run_sim_matrix.sh all"; exit 1
fi

# Latest verdict per (scenario, controller). Later rows win, so a re-run supersedes.
mapfile -t FAILED < <(~/.pyenv/versions/flying_robots/bin/python - "$RESULTS" <<'PY'
import re, sys
latest = {}
for line in open(sys.argv[1]):
    if not line.startswith('|') or line.startswith('| Scenario') or line.startswith('|---'):
        continue
    c = [x.strip() for x in line.strip().strip('|').split('|')]
    if len(c) < 4:
        continue
    sid, ctrl, n, verdict = c[0], c[1], c[2], c[3]
    # A NO-DATA row records the arg string rather than a scenario id; pull the id out.
    m = re.search(r'--scenario\s+(\w+)', sid)
    if m:
        sid = m.group(1)
    params = c[9] if len(c) > 9 else ''
    key = (sid, ctrl, params)
    latest[key] = verdict
# A case can appear twice: once with its parameters, once as a NO-DATA row whose
# parameter column is empty. They are the same case, so collapse them -- otherwise the
# re-run happens twice.
bad = {k: v for k, v in latest.items() if v != 'PASS'}
with_params = {(s, c) for (s, c, p) in bad if p and p != '-'}
seen = set()
for (sid, ctrl, params), verdict in bad.items():
    if (not params or params == '-') and (sid, ctrl) in with_params:
        continue
    if (sid, ctrl, params) in seen:
        continue
    seen.add((sid, ctrl, params))
    print(f'{sid}|{ctrl}|{params}')
PY
)

if [ ${#FAILED[@]} -eq 0 ]; then
  echo "nothing to re-run -- every case passed."; exit 0
fi

echo "cases needing a re-run:"
for f in "${FAILED[@]}"; do echo "    $f"; done

[ $DRY -eq 1 ] && exit 0

# Map each failed (scenario, controller) back to its command line via the shared list.
while IFS='|' read -r sid ctrl params; do
  base=${sid%%[ab]}          # A1a / A1b both come from scenario A1
  while IFS='|' read -r cid cctrl cn cargs; do
    case "$cid" in \#*|'') continue;; esac
    cid=$(echo "$cid" | xargs); cctrl=$(echo "$cctrl" | xargs)
    cbase=${cid%%[ab]}
    if [ "$cbase" = "$base" ] && [ "$cctrl" = "$ctrl" ]; then
      # If the row carried params, only re-run the variant they match.
      if [ -n "$params" ]; then
        want=$(echo "$params" | sed 's/[^0-9.]//g')
        have=$(echo "$cargs"  | sed 's/[^0-9.]//g')
        [ "${want:0:4}" != "${have:0:4}" ] && continue
      fi
      echo "### re-run $cctrl $cn $cargs"
      bash "$REPO/experiments/analysis/run_sim_matrix.sh" one "$cctrl" "$cn" $cargs
    fi
  done < "$CASES"
done < <(printf '%s\n' "${FAILED[@]}")
echo RERUNDONE
