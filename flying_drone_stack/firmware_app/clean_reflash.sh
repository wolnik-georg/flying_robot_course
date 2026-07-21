#!/usr/bin/env bash
# Clean firmware rebuild + reflash for CF21BL brushless.
#
# Why "clean" matters right now: today the drone was flashed through THREE
# firmware states — original (traj.ci = PARAM_UINT8, TRAJ_MAX_SEGS=12) ->
# widened (traj.ci = PARAM_UINT16, TRAJ_MAX_SEGS=24) -> reverted back to the
# original source. The source is now byte-identical to the validated baseline,
# but an incremental `make` can flash a STALE object/binary left over from the
# u16 build (build/ and *.o are timestamp-cached). If the drone ends up running
# the u16 traj.ci while the CS2 server's cached TOC (or flight.py) treats it as
# u8 — or vice-versa — every coefficient-upload index write is packed at the
# wrong byte width, the coefficients scatter, the onboard trajectory evaluates
# to garbage/zero, and the drone just holds position (no figure-8 / circle
# motion). `make clean` removes all cached objects so the flashed binary is
# guaranteed to match the current (reverted) source.
#
# Run this ON THE LAB PC, from the firmware_app directory, with the radio
# connected and the drone on. DRONE=bl (CF21BL brushless) is the default.
#
# Usage: bash clean_reflash.sh
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== firmware_app clean reflash (CF21BL brushless) ==="
echo "dir: $SCRIPT_DIR"
echo

echo "=== Step 1: make clean (remove stale objects + build/) ==="
make clean
# belt-and-suspenders: also nuke the committed/leftover object files and build dir
rm -rf build app.o built-in.o traj_iface.o .built-in.o.cmd .app.o.cmd 2>/dev/null
echo "cleaned."
echo

echo "=== Step 2: make (full rebuild from reverted source) ==="
make
echo

echo "=== Step 3: verify the built binary exists and note its size ==="
if [ -f build/cf2.bin ]; then
  ls -la build/cf2.bin
else
  echo "ERROR: build/cf2.bin not produced — build failed above. Do NOT flash."
  exit 1
fi
echo

echo "=== Step 4: flash (make cload) ==="
make cload
echo

echo "=== Done. ==="
echo "IMPORTANT next steps before flying (order matters):"
echo "  1. Fully restart the CS2 server (kill and re-run 'ros2 launch crazyflie launch.py')"
echo "     so it re-downloads a FRESH parameter TOC from this freshly-flashed firmware."
echo "     If the drone's TOC is cached to disk (CachedCfFactory rw_cache='./cache'),"
echo "     delete that cache dir first so no stale u16/u8 param table survives."
echo "  2. Then retest:"
echo "     ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.05 --height 0.7 --onboard --brushless"
