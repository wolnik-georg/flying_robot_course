#!/usr/bin/env bash
# Follow-up to find_active_marker.sh: that search grepped for "active_marker"/"activeMarker"
# and never matched -- the real name recovered from a dangling crazyflies.yaml blob is
# "active_deck" (type: cf21_active -> marker: active_deck). This finds the motion_capture.yaml
# blob(s) that actually DEFINE it, with real point coordinates, not just the ones that
# reference it by name.
LOG=~/georg/flying_robot_course/debug/lab_logs/debug.log
exec > >(tee "$LOG") 2>&1

cd ~/georg/ros2_ws/src/crazyswarm2 || exit 1

echo "############################################################"
echo "# every blob containing 'active_deck:' as a marker_configurations KEY"
echo "############################################################"
git cat-file --batch-all-objects --batch-check='%(objecttype) %(objectname)' --unordered 2>/dev/null \
| awk '$1=="blob"{print $2}' | while read -r b; do
  c="$(git cat-file -p "$b" 2>/dev/null)"
  if echo "$c" | grep -qE "^\s*active_deck:\s*$"; then
    echo "### blob $b"
    echo "$c" | sed -n '/marker_configurations:/,/dynamics_configurations:/p'
    echo "=========================================="
  fi
done

echo
echo "############################################################"
echo "# same blobs' surrounding crazyflies.yaml, if this IS one -- confirms cf21_active's"
echo "# full robot_types entry (tracking method, dynamics) alongside the marker"
echo "############################################################"
git cat-file --batch-all-objects --batch-check='%(objecttype) %(objectname)' --unordered 2>/dev/null \
| awk '$1=="blob"{print $2}' | while read -r b; do
  c="$(git cat-file -p "$b" 2>/dev/null)"
  if echo "$c" | grep -qE "type: cf21_active"; then
    echo "### blob $b (crazyflies.yaml referencing cf21_active)"
    echo "$c" | sed -n '/^robot_types:/,/^all:/p'
    echo "=========================================="
  fi
done
