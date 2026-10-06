#!/usr/bin/env bash
# Desk verification before NS2 sign-test lab — read-only checks, no flash.
set -euo pipefail
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
FW_APP="$REPO/flying_drone_stack/firmware_app"
HOST="$FW_APP/host"
CS2="${CS2:-$HOME/Desktop/crazyswarm2}"
YAML="$CS2/crazyflie/config/crazyflies.yaml"
PACK="$REPO/docs/lab_session_pack_signtest.md"
PYFR="${PYFR:-$HOME/.pyenv/versions/flying_robots/bin/python}"

pass=0
fail=0
skip=0

ok() { echo "PASS: $*"; pass=$((pass + 1)); }
bad() { echo "FAIL: $*"; fail=$((fail + 1)); }
skp() { echo "SKIP: $*"; skip=$((skip + 1)); }

echo "=== lab_session_pack_verify (signtest pack) ==="
echo "REPO=$REPO"

# --- Pack doc exists ---
if [[ -f "$PACK" ]]; then ok "pack doc $PACK"; else bad "missing $PACK"; fi

# --- Paths referenced in pack (scripts) ---
for rel in \
  flying_drone_stack/tools/check_usd_deck.py \
  flying_drone_stack/tools/merge_usd_logs.py \
  experiments/analysis/post_flight_check.py; do
  if [[ -f "$REPO/$rel" ]]; then ok "exists $rel"; else bad "missing $rel"; fi
done

# --- merge_usd_logs.py --help ---
if python3 "$REPO/flying_drone_stack/tools/merge_usd_logs.py" --help >/dev/null 2>&1; then
  ok "merge_usd_logs.py --help"
else
  bad "merge_usd_logs.py --help"
fi

# --- post_flight_check.py --help ---
if "$PYFR" "$REPO/experiments/analysis/post_flight_check.py" --help >/dev/null 2>&1; then
  ok "post_flight_check.py --help (flying_robots)"
else
  bad "post_flight_check.py --help"
fi

# --- run_formation.py flags ---
RF="$CS2/crazyflie_examples/crazyflie_examples/run_formation.py"
if [[ -f "$RF" ]]; then
  ok "run_formation.py path"
  for flag in --list --scenario --dz --height --passes --hold --auto-center --yes; do
    if grep -qF -- "$flag" "$RF"; then ok "run_formation flag $flag"; else bad "run_formation missing $flag"; fi
  done
else
  bad "missing $RF"
fi

# --- crazyflies.yaml cf5 / cf_second ---
if [[ -f "$YAML" ]]; then
  ok "crazyflies.yaml"
  yaml_out=$(python3 - <<PY
import yaml
from pathlib import Path
p = Path("$YAML")
data = yaml.safe_load(p.read_text())
robots = data.get("robots", data)
cf5 = robots.get("cf5", {})
cf2 = robots.get("cf_second", {})
fp5 = cf5.get("firmware_params", {})
fp2 = cf2.get("firmware_params", {})
ig5 = fp5.get("indi_gains", {})
ig2 = fp2.get("indi_gains", {})
rn5 = fp5.get("rnn", {})
c5 = fp5.get("stabilizer", {}).get("controller")
c2 = fp2.get("stabilizer", {}).get("controller")
checks = [
    ("cf5 controller 6", c5 == 6, c5),
    ("cf5 indi_gains.ctrl_mode 0", ig5.get("ctrl_mode") == 0, ig5.get("ctrl_mode")),
    ("cf5 indi_gains.res_sign -1", ig5.get("res_sign") == -1, ig5.get("res_sign")),
    ("cf5 rnn.en 1", rn5.get("en") == 1, rn5.get("en")),
    ("cf_second controller 6", c2 == 6, c2),
    ("cf_second indi_gains.ctrl_mode 0", ig2.get("ctrl_mode") == 0, ig2.get("ctrl_mode")),
]
for name, ok, got in checks:
    print(f"{'PASS' if ok else 'FAIL'}: yaml {name} (got {got!r})")
PY
)
  echo "$yaml_out"
  if echo "$yaml_out" | grep -q "^FAIL:"; then bad "yaml spot checks"; else ok "yaml spot checks all PASS lines"; fi
else
  bad "missing yaml $YAML"
fi

# --- post_flight_check 2026-10-05 vs acceptance table ---
PFJ="$REPO/experiments/analysis/out/post_flight_check_2026-10-05.json"
if [[ -f "$PFJ" ]]; then
  ok "existing post_flight_check JSON"
else
  if "$PYFR" "$REPO/experiments/analysis/post_flight_check.py" \
    --date 2026-10-05 \
    --logs "$REPO/experiments/logs" \
    --bags "$REPO/experiments/logs/rosbags" \
    --md "$REPO/experiments/analysis/out/post_flight_check_2026-10-05.md" \
    --json "$PFJ" 2>/dev/null; then
    ok "post_flight_check.py --date 2026-10-05 run"
  else
    bad "post_flight_check.py --date 2026-10-05"
  fi
fi

if [[ -f "$PFJ" ]]; then
  PFJ="$PFJ" python3 - <<'PY'
import json, os
from pathlib import Path
j = json.loads(Path(os.environ["PFJ"]).read_text())
# Expected verdicts from docs/post_flight_check.md acceptance table (times as HH:MM:SS prefix)
expect = {
    "17:39:27": "CLEAN", "17:41:09": "CLEAN", "17:59:30": "CLEAN",
    "18:01:01": "NOT CLEAN", "18:02:38": "NOT CLEAN", "18:18:58": "NOT CLEAN",
    "18:34:10": "NOT CLEAN", "18:55:34": "NOT CLEAN",
    "19:17:04": "CLEAN", "19:19:27": "CLEAN", "19:23:26": "CLEAN", "19:21:54": "ABORTED",
}
flights = j.get("flights", j) if isinstance(j, dict) else j
if isinstance(flights, dict):
    flights = flights.get("flights", [])
mismatch = 0
for row in flights:
    t = row.get("time") or row.get("stamp") or ""
    for key, exp in expect.items():
        if key.replace(":", "-") in str(t) or key in str(t):
            got = row.get("verdict") or row.get("status")
            if exp == "CLEAN" and got != "CLEAN":
                print(f"FAIL: {key} expected CLEAN got {got}"); mismatch += 1
            elif exp == "ABORTED" and got != "ABORTED":
                print(f"FAIL: {key} expected ABORTED got {got}"); mismatch += 1
            elif exp == "NOT CLEAN" and got not in ("NOT CLEAN", "NOT_CLEAN"):
                print(f"FAIL: {key} expected NOT CLEAN got {got}"); mismatch += 1
            else:
                print(f"PASS: {key} -> {got}")
            break
import sys
sys.exit(1 if mismatch else 0)
PY
  pf_rc=$?
  if [[ $pf_rc -eq 0 ]]; then ok "post_flight verdicts vs acceptance table"; else bad "post_flight verdict mismatch"; fi
fi

# --- no "humble" in pack ---
if [[ -f "$PACK" ]]; then
  if grep -qi humble "$PACK"; then bad 'pack contains "humble"'; else ok 'pack has no "humble"'; fi
fi

# --- legacy checks (optional firmware host test) ---
if [[ -f "$HOST/test_residual_nn.py" ]]; then
  echo "--- test_residual_nn.py ---"
  (cd "$HOST" && python3 test_residual_nn.py) | tail -3 || bad "test_residual_nn.py"
else
  skp "test_residual_nn.py"
fi

echo "=== summary: pass=$pass fail=$fail skip=$skip ==="
[[ $fail -eq 0 ]]
