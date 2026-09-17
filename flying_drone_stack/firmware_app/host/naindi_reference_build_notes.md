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

## CS2 SIL closed-loop validation (controller=7) — reference-consistent build recipe, 2026-09-17

**This is a SIM-ONLY, throwaway build configuration. It must never be used to build
`cf21bl.bin`/`cf2.bin` for a real flash — see "Restoring the default build" below.**

### Why this exists

The static test vectors above (and `test_naindi_hybrid_reference.py`) only ever check a single
tick against a hand-picked state — they cannot catch a closed-loop instability. The first real
closed-loop test (2026-09-16, `docs/07` History (39)/(40)) wired `oot2`/`oot3` into the CS2 SIL
simulator and found both controllers crash a plain single-drone hover. Chasing why (2026-09-17)
found the SIL's simulated plant is a second, independent implementation
(`crazyflie_sim/backend/np.py`, NOT this project's own Rust simulator) with its own hardcoded
airframe constants that don't automatically track whichever platform `naindi.rs`/
`naindi_hybrid.rs` were compiled for.

### The two separate platform switches involved

| Switch | Governs | Default | For this test |
|---|---|---|---|
| `DRONE_PLATFORM` (Rust, `firmware_app/build.rs`) | `naindi.rs`'s own `JXX/JYY/JZZ`, `ARM_REF_DEFAULT`, `T2T_REF_DEFAULT` (and `lib.rs`'s equivalents for controller=6) | `bl` (brushless) if set, else standard/upgraded | **unset** (or `cf2`) |
| `OOT_PLATFORM` (C, `crazyflie-firmware/bindings/setup.py`) | The `platform_defaults.h` block `oot_arm_length()`/`oot_thrust2torque()` (`firmware_app/host/oot_host.c`) read — these are what `crazyflie_server.py`'s `_setup_oot` uses to sync the **simulated plant's** arm/thrust-to-torque to the controller | `CONFIG_PLATFORM_CF21BL` if unset | **`CONFIG_PLATFORM_CF2`** |

These are governed by **completely different mechanisms** (a Rust build-time env var vs. a
Python `setup.py` env var reading a **different** name) — easy to set one and assume the other
followed. Confirm both took effect before trusting a run:

```bash
python3 -c "
import sys; sys.path.insert(0, '/home/georg/Desktop/crazyflie-firmware/build')
import cffirmware as firm
print('arm_length:', firm.oot_arm_length())       # expect 0.046 (reference), not 0.050 (bl)
print('mass default:', firm.cvar.g_indi_mass)      # compile default only -- yaml still overrides this, see below
"
```

The plant's own **inertia** (`crazyflie_sim/backend/np.py`'s `Quadrotor.J`) is a THIRD thing,
hardcoded to the reference's own numbers (`16.571710e-6, 16.655602e-6, 29.261652e-6`) and
**never synced from the firmware at all** (`_setup_oot` doesn't include `J` in its physics
sync). This is what makes the standard/upgraded `DRONE_PLATFORM` choice load-bearing here: on
that branch `naindi.rs`'s own `JXX/JYY/JZZ` (`lib.rs`) already equal those same numbers, so the
controller's internal model and the plant's true inertia finally agree — not because the plant
was fixed, but because the controller was pointed at what the plant already was.

### The recipe

```bash
# 1. Rust side -- controller's own arm/t2t/J -> reference values
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu \
    --features residual_nn   # feature needed only if testing rnn.* too; harmless otherwise

# 2. C side -- plant's arm/t2t sync source -> reference values (SEPARATE env var!)
cd ~/Desktop/crazyflie-firmware
rm -f build/_cffirmware*.so build/cffirmware.py build/cffirmware_wrap.c
OOT_PLATFORM=CONFIG_PLATFORM_CF2 make bindings_python

# 3. Run the SIL hover (server_sim_naindi.yaml selects oot2; crazyflies_sim1.yaml is
#    required -- oot2/oot3 have no per-vehicle state-swap mechanism, one drone only)
source /opt/ros/humble/setup.bash && source ~/Desktop/crazyswarm2/install/setup.bash
cd ~/Desktop/crazyswarm2
ros2 launch crazyflie launch.py backend:=sim rviz:=False mocap:=False teleop:=False \
    crazyflies_yaml_file:=$(pwd)/crazyflie/config/crazyflies_sim1.yaml \
    server_yaml_file:=$(pwd)/crazyflie/config/server_sim_naindi.yaml &
sleep 6
ros2 run crazyflie_examples simple_flight -- --trajectory hover --duration 8 --height 1.0
# NAINDI_SCALED_GAINS=1  -- KR/KOMEGA scaled by J_real/J_ref (naindi.rs GAIN_TEST_OVERRIDE)
# NAINDI_REFERENCE_MASS=1 -- g_indi_mass -> 0.034 kg before the plant snapshot (crazyflie_server.py)
```

Recorded state: `crazyswarm2/state_naindi/<timestamp>/csv/cf231_active.csv`
(`server_sim_naindi.yaml`'s `record_states`).

### Results

| Config | Hover (t≈8-16s) | Landing (t≈16-19s) |
|---|---|---|
| `bl` platform, unscaled gains (2026-09-16 original) | **crashes** ~t=13s | — |
| `bl` platform, `NAINDI_SCALED_GAINS=1` (2026-09-17) | **crashes** ~t=12.6s, same signature | — |
| **`cf2`/reference platform** (this recipe), real mass/kt | **clean**, roll/pitch <1.2° | **crashes** ~t=17-19s, 30-45° |
| `cf2`/reference platform + `NAINDI_REFERENCE_MASS=1` | **worse** — oscillation from ~t=12s, tumble ~t=16s | (never reached cleanly) |

**Conclusion so far:** the arm/t2t/J platform mismatch was the hover crash's real cause — fixed
by this recipe. `KR`/`KOMEGA` scaling (tried first, before this was understood) was solving a
mismatch that didn't physically exist in the sim and made nothing better. Overriding mass alone
(`NAINDI_REFERENCE_MASS=1`, decoupling it from the still-real `kt1-4`) makes things **worse**,
not better — mass and kt must stay internally consistent (both real, since they were measured
together; there's no reference-project kt in the same RPM²-domain units to pair with a
reference mass). **The landing crash is still open and NOT mass/kt-driven** — next diagnostic is
the hover→land setpoint transition (integral windup, a velocity/acceleration discontinuity at
that segment boundary) or the still-untouched `KPOS_P/KPOS_D/KPOS_I` position gains (also
reference-literal, never scaled or tested).

### Restoring the default build

**Every session that runs this recipe must restore the real hardware default afterward** —
`OOT_PLATFORM`/`DRONE_PLATFORM` are easy to leave set in a shell and easy to forget:

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
make DRONE=bl                      # real ARM build, reseeds the Kconfig .config to CF21BL
DRONE_PLATFORM=bl RUSTFLAGS="-C panic=abort" cargo build --release \
    --target x86_64-unknown-linux-gnu --features residual_nn
cd ~/Desktop/crazyflie-firmware
rm -f build/_cffirmware*.so build/cffirmware.py build/cffirmware_wrap.c
make bindings_python                # OOT_PLATFORM unset -> CONFIG_PLATFORM_CF21BL default
```

Verified 2026-09-17: after restoring, `oot_arm_length()` reads back `0.050` (brushless) and a
`controller=6` geometric hover (`server_sim_geo.yaml`) flies exactly as before — `state_geo/`,
max roll/pitch `0.0°` through the full flight. **Nothing about controller=6 or our own INDI
(`controller=6`, `ctrl_mode` 0-3) is touched by any of this** — they only ever read `lib.rs`'s
own platform-gated constants, built the same way they always have been; `naindi.rs`/
`naindi_hybrid.rs` are a fully separate compiled path.
