# Building the reference `cffirmware` for `test_naindi_reference.py`

`~/Desktop/NA-INDI-firmware` is read-only (never modified in place — see
`LOCAL_MODIFICATIONS.md` and `feedback_dont_fork_upstream_repos` in project memory). To
compile Cobo & Briesewitz's own `controller_lee.c` for numerical comparison, build a
**throwaway copy** somewhere outside that checkout (e.g. a job scratch dir). The steps below
reproduce the `.so` used to validate `naindi.rs` (all 5 cases in `test_naindi_reference.py`
pass at ~1e-9 tolerance); the `.so` itself is a build artifact, not checked in.

```bash
SCRATCH=/tmp/na_indi_build   # any throwaway directory
cp -r ~/Desktop/NA-INDI-firmware/. "$SCRATCH/"

# Their vendor/ submodules are uninitialized (no network access assumed) -- reuse the
# already-checked-out copies from our own crazyflie-firmware tree (same upstream content).
cp -r ~/Desktop/crazyflie-firmware/vendor/CMSIS/.    "$SCRATCH/vendor/CMSIS/"
cp -r ~/Desktop/crazyflie-firmware/vendor/FreeRTOS/. "$SCRATCH/vendor/FreeRTOS/"
cp -r ~/Desktop/crazyflie-firmware/vendor/libdw1000/. "$SCRATCH/vendor/libdw1000/"

cd "$SCRATCH"
make cf2_defconfig
# CONFIG_MOTORS_DEFAULT_IDLE_THRUST defaults to 7000, which #errors unless arming is on:
sed -i 's/# CONFIG_MOTORS_REQUIRE_ARMING is not set/CONFIG_MOTORS_REQUIRE_ARMING=y/' build/.config
make oldconfig
```

Then patch `bindings/setup.py` (in the scratch copy only) to add the include paths and
defines their own `bindings/setup.py` doesn't set (their controller_lee.c is the only
reference file that `#include <motors.h>`, pulling in the full STM32 HAL/driver/deck stack
that stock `controller_lee.c` never needed):

```python
include = [
    ...,
    "src/lib/CMSIS/STM32F4xx/Include",
    "src/lib/STM32F4xx_StdPeriph_Driver/inc",
    "vendor/FreeRTOS/include",
    "vendor/FreeRTOS/portable/GCC/ARM_CM4F",
    "src/deck/interface",
    "src/deck/drivers/interface",
    "src/modules/interface/estimator",
    "src/modules/interface/lighthouse",
    "src/modules/interface/cpx",
    "src/modules/interface/p2pDTR",
    "src/utils/interface/kve",
    "src/utils/interface/tdoa",
    "src/drivers/bosch/interface",
    "src/drivers/esp32/interface",
    "src/lib/FatFS",
    "src/lib/STM32_USB_Device_Library/Core/inc",
    "src/lib/STM32_USB_OTG_Driver/inc",
    "src/lib/vl53l1",
    "src/lib/vl53l1/core/inc",
    "vendor/libdw1000/inc",
]
fw_sources = [
    ...,
    "src/modules/src/controller/nn.c",         # use_nn=0 in every test, but the symbols
    "src/modules/src/controller/nn_utils.c",   # (nn_forward, layer) must still link
]
cffirmware = Extension(
    "_cffirmware", include_dirs=include,
    sources=fw_sources + ["build/cffirmware_wrap.c", "bindings/host_stubs.c"],
    extra_compile_args=[
        "-O3", "-fno-strict-aliasing", "-Wno-address-of-packed-member", "-DUNIT_TEST_MODE",
        "-DSTM32F40_41xxx", "-DCONFIG_PLATFORM_CF21BL", "-DUSE_STDPERIPH_DRIVER",
        "-Isrc/config", "-include", "stm32f4xx_conf.h",
    ],
)
```

And add `bindings/host_stubs.c` (scratch-only, not part of their tree) providing the
firmware-side symbols `controller_lee.c` calls that this checkout doesn't otherwise link
(log/param subsystem, `pmGetBatteryVoltage`, `motorsGetRatio`, `usecTimestamp`), plus two
test-only hooks:

- `oot_test_set_rpm(m1,m2,m3,m4)` — the reference reads RPM via `logGetUint(logGetVarId("rpm","mN"))`;
  the stub's `logGetVarId` assigns ids 1-4 by name and `logGetUint` returns a Python-settable value.
- `oot_test_set_kappa_f(k1,k2,k3,k4)` — `kappa_f[4]` (in `power_distribution_quadrotor.c`) is a
  runtime param, zero by default in this checkout; must be set explicitly for `t1..t4` to be
  non-zero.
- `usecTimestamp()` — fixed 2000us (500Hz) increments, **not** real wall-clock. A host test
  loop can call this fast enough that two consecutive calls return the same microsecond,
  making `dt=0` in the reference's own un-guarded `(omega-omega_prev)/dt` → NaN. This is a
  host-timing artifact (real hardware never sees `dt=0` at a 2ms tick period), not a real
  discrepancy — `firmware_app/host/oot_host.c`'s own `usecTimestamp()` stub uses the same
  fixed-increment approach for exactly this reason.
- `paramGetVarId`/`paramGetUint` — `controllerLeeInit()` checks `paramGetUint(idDeckBcRpm)==1`
  once to decide whether `rpm_deck_available` is true for the whole run; the stub reports
  "present" unconditionally so the INDI RPM path is actually exercised.

```bash
cd "$SCRATCH" && rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python
```

Then run the comparison from this project:

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
python3 host/test_naindi_reference.py "$SCRATCH/build" ~/Desktop/crazyflie-firmware/build
```

## Result (2026-09-14)

All 5 hand-picked test vectors (exact hover, position error, combined roll+velocity error,
90° yaw + asymmetric RPM, a fully aggressive multi-axis case) match to ~1e-9 on thrust and
torque — the algorithmic port is numerically verified against the reference's own compiled C,
with mass/J/kt pinned identically on both sides via `naindi_test_set_j()` (our side, test-only
hook, see `naindi.rs`) and `oot_test_set_kappa_f()` (reference side).

One non-obvious fix needed during validation: `setpoint->mode.yaw` must be `modeAbs` in the
reference call, or `desiredYaw` falls into a different branch (`state->attitude.yaw +
attitudeRate.yaw*dt`) than the one `naindi.rs` implements. This surfaced the deviation
documented in `naindi.rs`'s module doc: the port only implements the `modeAbs` yaw branch
(consistent with this project's existing convention that Mode E/HLC setpoints are always
absolute — see `firmware_app/CLAUDE.md`), not the reference's velocity/disable yaw modes.
