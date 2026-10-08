#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

MODE="${1:-1}"
# mode 1 = default build (legacy, sentinel -> 0) must be bit-identical to the legacy logic;
# mode 2 = -DRPM_FILTER_HOLD_SENTINEL=1 (prepared sentinel hold-last-good)
DEF=""; [ "$MODE" = "2" ] && DEF="-DRPM_FILTER_HOLD_SENTINEL=1"
gcc -std=c11 -Wall -Wextra -O2 $DEF -I "$HERE/.." "$HERE/test_rpm_filter.c" -o "$WORK/test_rpm_filter"
"$WORK/test_rpm_filter" "$MODE"
