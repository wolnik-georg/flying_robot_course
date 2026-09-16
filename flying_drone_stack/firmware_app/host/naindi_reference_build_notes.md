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

## NA-INDI hybrid (controller=8, `use_nn=1`) — addendum, 2026-09-16

`test_naindi_hybrid_reference.py`/`_naindi_hybrid_case_runner.py` extend this exact setup to
verify `naindi_hybrid.rs` (`controller=8`) against the SAME reference `controllerLee()`, this
time with `ctrl.use_nn = 7` (all three bits) instead of `0` — exercising the real trained
network in `nn.c`/`nn_utils.c`, not just linking it unused as the `controller=7` test did.

Two changes to the scratch build beyond the recipe above:

1. **`motorsGetRatio()` must become settable.** The NN's own input vector reads it directly
   (`controller_lee.c` line ~293); the base recipe's stub always returns 0. Add, alongside the
   existing `oot_test_set_rpm()`:
   ```c
   static uint16_t g_test_pwm_ratio[4] = {0, 0, 0, 0};
   uint16_t motorsGetRatio(uint32_t id) { return (id < 4) ? g_test_pwm_ratio[id] : 0; }
   void oot_test_set_pwm_ratio(uint32_t m1, uint32_t m2, uint32_t m3, uint32_t m4)
   {
     g_test_pwm_ratio[0] = m1; g_test_pwm_ratio[1] = m2;
     g_test_pwm_ratio[2] = m3; g_test_pwm_ratio[3] = m4;
   }
   ```
   and declare `oot_test_set_pwm_ratio` in the scratch `bindings/cffirmware.i` next to the
   existing `oot_test_set_rpm`/`oot_test_set_kappa_f` declarations (both occurrences).

2. **`usecTimestamp()`'s fixed-increment stub needs a real fix, not a cosmetic one.** With
   `use_nn` enabled, `controller_lee.c` calls `usecTimestamp()` **three times per active
   control tick**, not one: two extra calls (lines ~271/306) bracket the NN forward pass purely
   to compute an unused `nn_inference_time` diagnostic local, *before* the one call that
   actually feeds the control law (line 476, the attitude-INDI `dt` measurement).
   `naindi_hybrid.rs` makes no equivalent profiling calls, so the original "+2000us every call"
   stub advances the reference's clock **3x faster** than ours between real measurements,
   inflating `(omega - omega_prev)/dt` and showing up as torque-only mismatches (thrust matched
   to ~1e-8 throughout; the first, NN-blind attempt failed 3/6 cases, all on `tau`, all
   correlated with nonzero gyro/RPM-asymmetry — never on the two all-zero-gyro cases, which
   matched exactly). **Confirmed root cause before patching**: a direct ctypes-level call to
   the reference's own exported `nn_forward()` against our `naindi_hybrid_test_nn_forward()` on
   an identical input vector matched **bit-for-bit** — the network port itself was never the
   problem, only the surrounding test harness's fake clock.

   Fix: make the stub track a repeating 3-call cycle per active tick (profiling-start,
   profiling-end, real) and only advance on the 3rd call, so elapsed time between successive
   *real* measurements stays exactly 2000us regardless of the diagnostic calls in between. The
   one-time `usecTimestamp()` call inside `controllerLeeInit()` (line 161) is excluded from the
   cycle and always frozen:
   ```c
   static uint64_t g_fake_usec = 0;
   static int g_usec_call_idx = -1;   /* -1 = init call not yet seen */
   uint64_t usecTimestamp(void)
   {
     if (g_usec_call_idx < 0) { g_usec_call_idx = 0; return g_fake_usec; }
     int sub = g_usec_call_idx % 3;
     g_usec_call_idx++;
     if (sub == 2) { g_fake_usec += 2000; }
     return g_fake_usec;
   }
   ```

On our side, `naindi_hybrid.rs` needed one small addition to make the comparison possible at
all: its own `naindi_hybrid_test_set_j()` test-only inertia-override hook (mirroring, not
sharing, `naindi.rs`'s `naindi_test_set_j()` — kept as this file's own static per the module's
isolation requirement), since it had none before and the two sides' real per-platform `J` don't
match (`REFERENCE_GAINS` uses `16.57e-6/16.66e-6/29.26e-6`; the active `bl` platform build uses
`23.951e-6/23.951e-6/32.347e-6`).

### Result (2026-09-16)

After the clock fix, **all 6 hand-picked test vectors match to ~1e-9** on thrust and torque —
including a case deliberately set with `gyro != gyro_reg` (different `gyroNoLpf` vs. `gyro`
values) to catch a wiring swap between the two, which it would have caught had one existed.
`controller=8` (`naindi_hybrid.rs`, `use_nn=7`) is now numerically verified against the
reference's own compiled C, the same standard `controller=7` was held to. **Still never flown**
— numerical verification is a precondition for flying, not a substitute for it.

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
python3 host/test_naindi_hybrid_reference.py "$SCRATCH/build" ~/Desktop/crazyflie-firmware/build
```
