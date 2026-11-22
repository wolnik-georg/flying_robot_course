#!/usr/bin/env bash
# Lab PC: make sure the committed round-0 files (S0/S2: round.py --init of 2026-11-18, commit 99ec0351; S1/S3: round.py --init of 2026-11-22) are in place
# for lab_run.sh prep, and that this checkout is up to date. Safe to run more than once. Never runs round.py.
# Run without pulling first:
#   cd ~/georg/flying_robot_course && git fetch origin && git show origin/feature/auto-tuning:experiments/diffcotune/lab/fix_round0.sh | bash
REPO="${REPO:-$HOME/georg/flying_robot_course}"
BR=feature/auto-tuning
cd "$REPO" || { echo "STOP: no repo at $REPO"; exit 1; }
echo "repo:   $REPO"
echo "branch: $(git branch --show-current)   HEAD $(git rev-parse --short HEAD)   origin $(git rev-parse --short origin/$BR)"
[ "$(git branch --show-current)" = "$BR" ] || { echo "STOP: this checkout is not on $BR. Paste this output to the planning agent."; exit 1; }

echo "== pull"
if ! git pull --ff-only origin $BR; then
  echo "!! git pull failed (local changes in the way). Files that block it:"; git status --short | head -20
  echo "!! continuing with the round-0 files only; paste this output to the planning agent afterwards."
fi

echo "== round-0 files"
for C in S0 S1 S2 S3; do
  D=experiments/diffcotune/lab/$C/round_0
  mkdir -p "$D"
  for f in gains.yaml state.json; do
    git show origin/$BR:$D/$f > "$D/$f" || { echo "STOP: could not read $D/$f from origin"; exit 1; }
  done
done
declare -A WANT=( [S0]=ff01a61d6e3b80ba [S1]=2c3a4fc88e05772c [S2]=644e0e7222b2afdb [S3]=b1961b1b9ea7e543 )
BAD=0
for C in S0 S1 S2 S3; do
  SHA=$(sha256sum experiments/diffcotune/lab/$C/round_0/gains.yaml | cut -c1-16)
  echo "$C gains sha $SHA (must be ${WANT[$C]})"
  [ "$SHA" = "${WANT[$C]}" ] || BAD=1
done
[ "$BAD" = 0 ] || { echo "STOP: checksum mismatch"; exit 1; }
grep -q stack_guard experiments/diffcotune/lab/lab_run.sh && echo "lab_run.sh: current (has the safety checks)" || echo "!! lab_run.sh is OLD (pull failed?): paste this output to the planning agent"

echo
echo "OK. Next (as told by the planning agent): experiments/diffcotune/lab/lab_run.sh prep S1 0   (S0 and S2 are closed; S3 later)"
