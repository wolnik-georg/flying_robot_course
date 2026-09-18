#!/usr/bin/env bash
# ============================================================================
# find_active_marker.sh  --  RUN ON THE LAB PC (flightcontrol1)
#
# Purpose: locate the active-marker-deck configuration (a 4-point marker
# definition in motion_capture.yaml and/or a robot type like `cf21_active`
# in crazyflies.yaml, used by cf232_active) created ~April/May 2026.
#
# Established so far (2026-09-18):
#   - It is NOT in the crazyswarm2 repo on the laptop, on ANY branch, in ANY
#     commit, or in ANY object (reachable or unreachable). Verified by scanning
#     the entire object database: only 7 distinct motion_capture.yaml versions
#     ever existed, all byte-identical to upstream (1-point default_single_marker
#     p0 [0.0177184, 0.0139654, 0.0557585]), and only 2 robot_types sets ever
#     existed: {cf21, cf21_mocap_deck}. No cf21_active, ever.
#   - It is NOT in flying_robot_course either.
#   - The lab PC has LOCAL branches that were deleted from origin and never
#     existed on the laptop (e.g. tune-indi-on-crazyflie-thrust-upgraded,
#     restore-working) plus 37 stashes. Those objects exist ONLY here.
#     => This machine is the only remaining place it can be found in git.
#
# Usage:
#   bash ~/georg/flying_robot_course/debug/lab_logs/find_active_marker.sh
# Output is written to debug/lab_logs/debug.log; commit and push it afterwards.
# ============================================================================

LOG=~/georg/flying_robot_course/debug/lab_logs/debug.log
exec > >(tee "$LOG") 2>&1

SRC=~/georg/ros2_ws/src/crazyswarm2
cd "$SRC" || { echo "FATAL: $SRC not found"; exit 1; }

echo "############################################################"
echo "# 0. CONTEXT"
echo "############################################################"
echo "pwd: $(pwd)"
echo "current branch: $(git rev-parse --abbrev-ref HEAD)"
echo "HEAD: $(git log -1 --oneline)"
date

echo
echo "############################################################"
echo "# 1. ALL LOCAL BRANCHES (includes ones deleted from origin)"
echo "############################################################"
git branch -vv

echo
echo "############################################################"
echo "# 2. CONFIG ON EVERY LOCAL BRANCH"
echo "#    Looking for: a marker config with 4 points (p0..p3) other"
echo "#    than 'default'/'mocap_deck', and/or a robot type cf21_active"
echo "############################################################"
for b in $(git branch --format='%(refname:short)'); do
  echo "=================== BRANCH: $b ==================="
  echo "--- motion_capture.yaml marker_configurations:"
  git show "$b:crazyflie/config/motion_capture.yaml" 2>/dev/null \
    | sed -n '/marker_configurations:/,/dynamics_configurations:/p'
  echo "--- crazyflies.yaml robot_types:"
  git show "$b:crazyflie/config/crazyflies.yaml" 2>/dev/null \
    | sed -n '/^robot_types:/,/^all:/p' | grep -E "^  [a-z_0-9]+:|marker:|tracking:|dynamics:"
  echo "--- crazyflies.yaml robots -> type:"
  git show "$b:crazyflie/config/crazyflies.yaml" 2>/dev/null \
    | grep -E "^  cf[0-9_a-z]*:|^    type:"
  echo
done

echo
echo "############################################################"
echo "# 3. EVERY OBJECT IN THIS REPO'S DATABASE (reachable or not)"
echo "#    This is where deleted branches and dropped stashes live."
echo "############################################################"
git cat-file --batch-all-objects --batch-check='%(objecttype) %(objectname)' --unordered 2>/dev/null \
| awk '$1=="blob"{print $2}' | while read -r b; do
  if git cat-file -p "$b" 2>/dev/null | grep -qE "cf21_active|cf232|active_marker|activeMarker"; then
    echo "### MATCH in blob $b"
    git cat-file -p "$b" 2>/dev/null | grep -nE "cf21_active|cf232|active|marker" | head -25
    echo "--- (full first 60 lines) ---"
    git cat-file -p "$b" 2>/dev/null | head -60
    echo "=========================================="
  fi
done

echo
echo "############################################################"
echo "# 4. ANY marker_configurations BLOCK WITH MORE THAN ONE POINT"
echo "#    under a name other than default/mocap_deck/medium/big"
echo "############################################################"
git cat-file --batch-all-objects --batch-check='%(objecttype) %(objectname)' --unordered 2>/dev/null \
| awk '$1=="blob"{print $2}' | while read -r b; do
  if git cat-file -p "$b" 2>/dev/null | grep -q "marker_configurations"; then
    echo "--- blob $b"
    git cat-file -p "$b" 2>/dev/null | sed -n '/marker_configurations:/,/dynamics_configurations:/p'
    echo "=========================================="
  fi
done | sort -u

echo
echo "############################################################"
echo "# 5. CURRENT STASHES THAT TOUCH config/  (37 exist here)"
echo "############################################################"
git stash list | while IFS= read -r line; do
  ref="${line%%:*}"
  if git stash show --name-only "$ref" 2>/dev/null | grep -q "config/"; then
    echo "=================== $line ==================="
    git stash show -p "$ref" -- crazyflie/config/ 2>/dev/null | head -60
    echo
  fi
done

echo
echo "############################################################"
echo "# 6. STASH REFLOG -- includes stashes already POPPED/DROPPED"
echo "############################################################"
git reflog stash 2>/dev/null | head -60
echo "--- full stash ref history ---"
git log -g --format="%h %gd %gs" refs/stash 2>/dev/null | head -60

echo
echo "############################################################"
echo "# 7. UNREACHABLE / DANGLING COMMITS (dropped stashes land here)"
echo "#    Stash commits look like: 'WIP on <branch>: ...'"
echo "############################################################"
git fsck --unreachable --no-reflogs 2>/dev/null | grep commit | awk '{print $3}' | while read -r c; do
  msg=$(git log -1 --format='%s' "$c" 2>/dev/null)
  echo "--- $c : $msg"
  # does this commit's tree contain a config change of interest?
  if git show "$c" --stat 2>/dev/null | grep -q "config/"; then
    echo "    *** touches config/ ***"
    git show "$c" -- crazyflie/config/ 2>/dev/null | head -50
  fi
done | head -200

echo
echo "############################################################"
echo "# 8. OUTSIDE GIT: other workspaces / older checkouts / backups"
echo "############################################################"
echo "--- every motion_capture / crazyflies yaml on the machine ---"
sudo find / -name "motion_capture*.yaml*" -not -path "*/ros2_ws/build/*" 2>/dev/null
sudo find / -name "crazyflies*.yaml*" -not -path "*/ros2_ws/build/*" 2>/dev/null | head -40
echo "--- any file anywhere containing a marker_configurations block ---"
sudo grep -rl "marker_configurations" / --include=*.yaml 2>/dev/null
echo "--- any file anywhere mentioning cf21_active or cf232 ---"
sudo grep -rl "cf21_active\|cf232" / --include=*.yaml --include=*.py --include=*.md 2>/dev/null | head -20
echo "--- other workspaces / home dirs ---"
ls -la /home/
sudo find / -maxdepth 4 -type d \( -name "*crazyswarm*" -o -name "*catkin_ws*" -o -name "*ros2_ws*" -o -name "*ros_ws*" \) 2>/dev/null
echo "--- anything modified Mar-Jun 2026 that looks like config ---"
find ~ -newermt "2026-03-01" ! -newermt "2026-07-01" \( -name "*.yaml" -o -name "*.yml" \) 2>/dev/null | head -40

echo
echo "############################################################"
echo "# 9. REMINDER -- BRANCH STATE MUST BE FIXED BEFORE FLYING"
echo "############################################################"
echo "The lab PC was left on 'tune-indi-on-crazyflie-thrust-upgraded'."
echo "None of the 2026-09-18 fixes are active there. To restore:"
echo "    cd ~/georg/ros2_ws/src/crazyswarm2 && git checkout main && git pull"
echo "    cd ~/georg/ros2_ws && colcon build --symlink-install"
echo "Current branch right now: $(git rev-parse --abbrev-ref HEAD)"

echo
echo "############################################################"
echo "# DONE -- commit and push debug.log"
echo "#   cd ~/georg/flying_robot_course"
echo "#   git add debug/lab_logs/debug.log && git commit -m 'active marker search' && git push"
echo "############################################################"
