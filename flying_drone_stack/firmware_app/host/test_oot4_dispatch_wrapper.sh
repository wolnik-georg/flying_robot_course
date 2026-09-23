#!/usr/bin/env bash
# Builds and runs test_oot4_dispatch_wrapper.c -- the only way to exercise controller=9's real
# firmware dispatch path (controllerOutOfTree4*, #ifdef CRAZYFLIE_FW-gated, never compiled into
# the host/SWIG build) without a real board. See that file's own header comment and
# omar_indi_reference_build_notes.md's "Firmware dispatch wrapper" section.
set -euo pipefail
CF=~/Desktop/crazyflie-firmware
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

INCS=(
  -I "$CF/src/modules/interface" -I "$CF/src/modules/interface/controller"
  -I "$CF/src/modules/interface/kalman_core" -I "$CF/src/modules/interface/outlierfilter"
  -I "$CF/src/hal/interface" -I "$CF/src/utils/interface/lighthouse" -I "$CF/src/utils/interface"
  -I "$CF/build/include/generated" -I "$CF/src/config" -I "$CF/src/drivers/interface"
  -I "$CF/src/platform/interface" -I "$CF/vendor/CMSIS/CMSIS/DSP/Include"
  -I "$CF/vendor/CMSIS/CMSIS/Core/Include"
)
# CONFIG_MODIFIED_CF_MASS mirrors app-config-bl / bindings/setup.py -- Omar's own
# brushless-established mass (his cf21blrpm_defconfig), not this project's own measurement.
# Update here too if that value ever changes.
DEFS=(-DCRAZYFLIE_FW -DUNIT_TEST_MODE -DCONFIG_PLATFORM_CF21BL -DCONFIG_MODIFIED_CF_MASS=42700)

gcc -c "${DEFS[@]}" "${INCS[@]}" "$CF/src/modules/src/controller/controller_omar_indi.c" \
    -fno-strict-aliasing -Wno-address-of-packed-member -o "$WORK/omar_indi.o"
gcc -c "${DEFS[@]}" "${INCS[@]}" "$HERE/oot4_dispatch_stubs.c" -o "$WORK/stubs.o"
gcc -c "${DEFS[@]}" "${INCS[@]}" "$HERE/test_oot4_dispatch_wrapper.c" -o "$WORK/wrapper.o"
gcc "$WORK/omar_indi.o" "$WORK/stubs.o" "$WORK/wrapper.o" -lm -o "$WORK/test_oot4"

"$WORK/test_oot4"
