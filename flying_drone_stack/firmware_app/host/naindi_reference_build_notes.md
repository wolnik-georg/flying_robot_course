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

## Closed-loop test of their ACTUAL compiled C — 2026-09-18

**The static test vectors above only ever check a single tick.** They prove the port is a
faithful *translation* of `controller_lee.c`, not that the algorithm is stable in a real
closed loop — that's a separate question the numeric comparison can't answer. This section
builds a standalone closed-loop harness (`host/naindi_reference_closed_loop.py`) that flies
**their actual compiled `controllerLee()`** — not `naindi.rs` — against the same rigid-body
plant physics (`crazyflie_sim.backend.np.Quadrotor`) the CS2 SIL uses, to separate "port bug"
from "their algorithm is genuinely unstable here".

### Build recipe (extends the static-test scratch build above)

The static-test scratch build's `bindings/setup.py`/`bindings/cffirmware.i` patches, `PLUS`:

1. **New host stub file, `bindings/host_stubs.c`** (the static test's patches embedded the
   equivalent code inline in `setup.py`'s docstring; this puts it in its own file since the
   closed-loop harness needs two more symbols). Provides `paramGetVarId`/`paramGetUint`
   (report the RPM deck "present" unconditionally), `logGetVarId`/`logGetUint` +
   `oot_test_set_rpm()` (their RPM read path), `oot_test_set_kappa_f()` (their `kappa_f[4]`
   is a runtime `PARAM_FLOAT`, zero by default), `motorsGetRatio`/`oot_test_set_pwm_ratio`
   (unused at `use_nn=0`, linked for completeness), `usecTimestamp()` (fixed +2000us per
   call, matching `ATTITUDE_RATE=500`), **and `pmGetBatteryVoltage()`** — a genuinely new
   requirement versus the static test: not called by `controller_lee.c` itself, but needed
   elsewhere in the link chain, and the static test never triggered it.
2. Add `bindings/host_stubs.c` to `sources=` in `bindings/setup.py`.
3. Add SWIG declarations for `oot_test_set_rpm`/`oot_test_set_kappa_f`/`oot_test_set_pwm_ratio`
   to `bindings/cffirmware.i` (both the `%{ %}` block and the plain declaration below it).
4. `cd $SCRATCH && rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python`

Verified this rebuild is still bit-for-bit correct before trusting it for anything new:
`test_naindi_reference.py`'s 5 static cases still pass at the same ~1e-9 tolerance.

### `naindi_reference_closed_loop.py`

Drives BOTH sides — `--which reference` (their `controllerLee()`) and `--which ours`
(`naindi.rs`'s `controllerOutOfTree2`) — through the **identical** tick loop, plant, and
trajectory, differing only in which compiled controller is called. This is the rigorous
comparison: a reference-only run leaves the trajectory-shape difference from the CS2 SIL
test as an uncontrolled variable, so an apples-to-apples same-harness run is the only way
to isolate "port bug" from everything else. `--which ours` needs the host `cffirmware`
rebuilt for the reference-consistent platform (`DRONE_PLATFORM` unset,
`OOT_PLATFORM=CONFIG_PLATFORM_CF2` — see the section above) to match the airframe on the
reference side; **remember to restore the default (`make DRONE=bl` + plain
`make bindings_python`) afterward**, same as every other experiment in this file.

Airframe: mass, kt, arm, t2t, J — **all** the reference authors' own values (kt is their real
measured `kappa_f` from `~/Desktop/NA-INDI/pwm2thrust.py`, not this project's), fully
self-consistent, matching the same config that flew `naindi.rs` clean through hover in the
CS2 SIL before crashing later (`state_naindi/2026-09-17_203435`).

**A real bug caught and fixed while building this harness, worth flagging:** `controllerLee()`
gates on `RATE_DO_EXECUTE(ATTITUDE_RATE=500, tick)`, which is `(tick % 2) == 0` at
`RATE_MAIN_LOOP=1000`/`ATTITUDE_RATE=500`. A raw incrementing tick counter (0,1,2,...) makes
this fire on only every other call, silently halving the effective control rate to 250 Hz
with no error — the controller just holds its previous output on odd ticks. Fixed by passing
`tick = 2 * loop_index`, so every call is a real execution at true 500 Hz.

### Results (`--duration 20`, climb 2s → hold 8s → land 2s → settle, matching the CS2 SIL
tests' own `--duration 8 --height 1.0`)

| Config | reference (their C) | ours (naindi.rs) |
|---|---|---|
| Bare position ramp (no feedforward) | clean, max roll/pitch 0.004° | clean, max roll/pitch 0.004° |
| + landing phase added | clean | clean |
| + minimum-jerk climb/land feedforward (real `sp.velocity`/`sp.acceleration`, not held at 0) | clean | clean |

**All four configurations are clean, and — critically — `reference` and `ours` are
numerically almost identical to each other in every one** (z-trajectories match to 5-6
significant figures). **This is strong, direct evidence against a port bug**: given the
identical plant, identical airframe, identical trajectory, the two compiled controllers
produce the same behavior. It does **not** reproduce the crash the CS2 SIL test found.

**What this means, honestly:** the crash is real (documented on video^Wlog, reproducible in
the actual ROS2 SIL every time) but this standalone harness — despite matching airframe,
gains, trajectory phases (climb/hold/land), and even feedforward shape — does not reproduce
it. Something about the *actual* CS2 SIL execution differs from this simplified
reimplementation in a way that matters. The leading unexamined candidate: `crazyflie_sil.py`
steps physics at 2 kHz but only *attempts* a controller call every other substep (~1 kHz),
and `RATE_DO_EXECUTE` gates that down again to the true 500 Hz — meaning the real SIL holds
each control output across **4** physics substeps (dt=0.0005 each) between genuine
recomputes, a different discretization than this harness's direct 1:1 dt=0.002 stepping.
Also un-replicated: the real HLC's own Poly4D/min-snap polynomial shape (this harness uses a
hand-picked quintic, not what `uploadTrajectory` actually generates) and any x/y motion in
the real flight (this harness is pure-Z). **Next step, if pursued: instrument the actual CS2
SIL run with per-tick logging (setpoint, `KI_ATT.i_error_att`, the exact physics-vs-control
call pattern) around the crash window, rather than continuing to guess-and-check with a
hand-rolled reimplementation** — the standalone harness has done what it usefully can:
ruled out a port bug as the explanation, not found the actual trigger.

## ROOT CAUSE FOUND AND FIXED — 2026-09-18

**The plan below worked on the first pass through it.** Summary for anyone who only reads
one section of this file: `crazyflie_sil.py`'s `setState()` populated `self.sensors.acc`
every tick but **never once set `self.state.acc`** — a plain omission, not a subtle
numerical issue. `naindi.rs` reads `state->acc` (not `sensors->acc`) for its position-INDI
residual term `a_imu` (module doc; `let acc = &st.acc;` at naindi.rs's line ~389) — so
`a_imu` was **exactly zero for the entire flight, every controller=7/8 SIL run to date**.
The INDI residual `a_res = a_imu - a_rpm` therefore equaled `-a_rpm` instead of a genuine
measured-vs-modeled comparison: roughly constant during steady hover (which is why hover
alone sometimes looked clean), but large and dynamically varying whenever commanded thrust
changes — climb, landing — which is exactly the failure window every single crash to date
was observed in.

**Confirmed by direct A/B, not just code-reading:** `naindi_reference_closed_loop.py` grew
a `--zero-state-acc` flag that replicates the bug standalone. With everything else already
matched to the real crashing config (real substeps, real motor lag, the real logged
setpoint sequence via `--replay-log`), `--zero-state-acc` **diverges at t=10.4s** — the
exact same window (t≈9.9-11.1s) the real logged crash actually happened in. Without it
(computing `state.acc` correctly), the identical run is completely clean. That is about as
clean an isolated cause-and-effect as a test like this can produce.

**Fix, `crazyflie_sim/crazyflie_sim/crazyflie_sil.py`'s `setState()`:** populate
`self.state.acc` too, not just `self.sensors.acc`. The two fields have different
conventions — `sensors.acc` (`sim_data_types.State`'s own field) is body-frame *specific
force* (reads `(0,0,1)` at hover, the accelerometer/thrust-reaction convention);
`state->acc` (what `controller_lee.c`/`naindi.rs` read) is world-frame,
**gravity-excluded**, comparable directly against `a_rpm` (which already has gravity
subtracted) — reads `(0,0,0)` at hover. Converting: rotate the body-frame specific force
into world frame, then subtract the gravity-cancelling `(0,0,1)` hover offset:
`state.acc_world = rotate(quat, sensors_acc_body) - (0, 0, 1)`.

**Result: the exact same previously-100%-crashing config — brushless airframe, default
gains, `server_sim_naindi.yaml` + `crazyflies_sim1.yaml`, unmodified — now flies completely
clean.** Max roll/pitch 0.003°/0.001° through climb, 8s hover, and into landing. Repeated:
a second hover run (clean) and figure8 (`--kt 0.008`) — also clean, `z` rock-solid at 1.0
throughout the actual maneuver (t=9.3-17.6s), normal ±5° banking roll, the larger 15-27°
swings only during the final landing descent, not a divergence. Two trajectory shapes now
confirmed for controller=7.

**`controller=8` (`naindi_hybrid.rs`) re-verified 2026-09-18, same fix, also clean** —
confirmed the same `state->acc` read at its own line ~369 before testing (not assumed from
controller=7's finding). `server_sim_naindi_hybrid.yaml` (`oot3`), same brushless airframe,
same default gains: max roll/pitch 0.106°/0.257° through hover and into landing (slightly
higher than controller=7's, consistent with the NN feedforward adding its own dynamics —
still fully stable, no divergence). **Both controller=7 and controller=8 are now confirmed
working in the real CS2 SIL.**

**Note on 2-drone:** not re-testable in this SIL as-is — `oot2`/`oot3` have no per-vehicle
state-swap mechanism yet (`crazyflie_sil.py`'s own `__init__` guard limits them to one
vehicle per sim run). The original 2-drone crash this investigation is sometimes conflated
with was a **real hardware** RPM-deck defect (docs/07 History (41), fixed via DShot) — a
separate issue from the SIL-only single-drone crash this fix addresses.

### Full controller audit — who reads `state.acc`, who reads `sensors.acc`, who is affected

Checked directly in the actual compiled source for every controller in the project, not
assumed by analogy — one subtle correction along the way: an earlier note here claimed
`lib.rs` "never reads either acc field," which was imprecise. It reads `sensors->acc` (via
`(*sensors).acc`, easy to miss with a naive grep for the literal substring `sensors.acc`),
just never `state->acc` — the distinction that actually matters, since `sensors.acc` was
never broken (only `state.acc` was).

| # | Controller | File | Reads | Affected by this fix? |
|---|---|---|---|---|
| 1 | PID | `controller_pid.c` | `sensors->acc.z` only | No |
| 2 | Mellinger | `controller_mellinger.c` | `sensors->acc.z` only | No |
| 3 | Stock INDI | `controller_indi.c`/`position_controller_indi.c` | `sensors->acc.{x,y,z}` only | No |
| 4 | Brescianini | `controller_brescianini.c` | no `.acc` at all | No |
| 5 | Stock Lee (`cf_second`) | `controller_lee.c` (crazyflie-firmware's own, NOT the reference's file of the same name) | no `.acc` at all | No |
| 6 | This project's own (`ctrl_mode` 0-3) | `lib.rs` | `sensors->acc` only, never `state->acc` | No |
| 7 | `naindi.rs` | our port of the reference's `controller_lee.c` | `state->acc` (line ~389) | **Yes — was broken, now fixed** |
| 8 | `naindi_hybrid.rs` | same reference file, `use_nn` on | `state->acc` (line ~369) | **Yes — was broken, now fixed** |

`setState()` populates `self.state.acc` for every simulated vehicle unconditionally
(it's generic, not controller-specific) — the fix changes what data is *available*, but
only controllers 7 and 8 ever read that field, so only they are behaviorally affected. All
six others read `sensors.acc` (already correct before this fix, untouched by it) or nothing
acceleration-related at all.

### Trajectory coverage — controller=8, 2026-09-18

Controller=7 had two trajectory shapes confirmed (hover, figure8). Controller=8 had only
hover. Ran figure8/circle/oval through the same real SIL (`server_sim_naindi_hybrid.yaml` +
`crazyflies_sim1.yaml`, unmodified default gains) to close that gap:

| Trajectory | Max \|roll\|/\|pitch\| during maneuver (t≈12-16s) | Trajectory-start transient (t≈10-12s) | Landing transient | z during maneuver | Verdict |
|---|---|---|---|---|---|
| figure8 (`--kt 0.008`) | 5-6° | 26.9°/13.2° | 22.2°/10.0° | 0.99-1.00 | clean |
| circle (`--kt 0.1`) | 4-9° | 39.4°/8.9° | 20.6°/28.0° | 1.00-1.01 | clean |
| oval (`--kt 0.008`) | 2-7° | 32.4°/0.9° | (run cut by harness timeout during hold, before landing) | 1.00 | clean |

All three: `z` never leaves the 0.99-1.01 m band during the actual maneuver — no divergence,
no crash. **The ~27-39° transient at trajectory start is not a controller=8-specific finding**:
re-checked controller=7's own figure8 log (`state_naindi/2026-09-18_101909`) and it shows the
same shape at the same timing (26.86° roll, t=10-12s) — this is the `go_to`→`start_trajectory`
setpoint discontinuity in `simple_flight.py` itself, common to both controllers, not something
either INDI port introduces. Landing-phase swings (15-28°) also match the pattern already
documented for controller=7. **Four trajectory shapes now confirmed clean for controller=7
(hover, figure8) and controller=8 (hover, figure8, circle, oval)** — no shape-dependent
instability found for either.

## Multi-vehicle support for controller=7/8 — 2026-09-18

Until now `oot2`/`oot3` were hard-limited to one vehicle per sim run: `naindi.rs` and
`naindi_hybrid.rs` each keep one process-global `static mut ST` (filters, integrators,
`timestamp_prev`) with no per-vehicle swap, unlike `controllerOutOfTree`'s own static, which
the simulator already swaps in/out per vehicle via `firm.oot_select_drone()` +
`oot_state_ptr()`/`oot_state_size()` (`lib.rs`) + `oot_select_drone()` (`oot_host.c`). Two
drones sharing either controller would have silently cross-contaminated their INDI filter
state — the exact bug class the existing `oot_select_drone` mechanism exists to prevent for
controller=6.

**Fix, mirroring the existing pattern exactly, nothing new invented:**
- `naindi.rs` / `naindi_hybrid.rs`: added `oot2_state_ptr`/`oot2_state_size` and
  `oot3_state_ptr`/`oot3_state_size` (same shape as `lib.rs`'s `oot_state_ptr`/
  `oot_state_size`), pointing at each module's own `static mut ST`.
- `oot_host.c`: generalized the old single-purpose `oot_select_drone` body into
  `oot_swap_select()` (one park/restore implementation, parameterized by pointer+size) and
  three independent slot pools (`g_pool_oot`/`g_pool_oot2`/`g_pool_oot3`), exposed as
  `oot_select_drone`/`naindi_select_drone`/`naindi_hybrid_select_drone`. Three pools because a
  sim run could in principle mix controllers across vehicles.
- `bindings/cffirmware.i`: exposed the two new `*_select_drone` functions (both `%{ %}` and
  plain-SWIG blocks, alongside the existing `oot_select_drone` line).
- `crazyflie_sil.py`: removed the `_oot2_count > 0` / `_oot3_count > 0` guard that raised
  `ValueError` on a second vehicle; `_oot2_count`/`_oot3_count` now assign each vehicle a slot
  index (`self._naindi_index`, same pattern as `self._oot_index`), and
  `firm.naindi_select_drone(self._naindi_index)` /
  `firm.naindi_hybrid_select_drone(self._naindi_index)` are called immediately before the
  controller, mirroring `firm.oot_select_drone(self._oot_index)` for `'oot'`.

### Results (`crazyflies_sim.yaml`, 2 vehicles, `backend: np` — no interaction model, isolates
the state-swap mechanism itself from any downwash-model question)

| Controller | Trajectory | cf231_active max\|roll\|/\|pitch\| | cf_second max\|roll\|/\|pitch\| | z range (both) | Cross-contamination? |
|---|---|---|---|---|---|
| controller=7 (`oot2`) | hover | 0.006°/0.001° | 0.006°/0.001° | 0.00-1.00 | None — final positions match each drone's own `initial_position` (0.3,0.0 vs 0.3,0.5) |
| controller=8 (`oot3`) | hover | 0.15°/0.36° | 0.15°/0.36° | 0.00-1.00 | None — same distinct final positions, same near-identical stats between drones |

Both drones track near-identically (as expected — identical trajectory, identical airframe,
`backend: np` has no coupling between them) and land at their own distinct commanded XY, not
each other's — the signature that each vehicle genuinely has its own controller state rather
than silently sharing one. **Single-drone regression check**: re-ran controller=7 hover through
`crazyflies_sim1.yaml` (1 vehicle) after this change — 0.002°/0.001°, matching the pre-change
result, confirming the `_naindi_index=0` path is unaffected.

**Not yet done**: a downwash-modeling backend (`neuralswarm`) with two `oot2`/`oot3` vehicles,
which is what would actually exercise the residual/interaction-force content this thesis cares
about, as opposed to two independent hovers. Also not yet done: a 2-drone maneuver (figure8/
circle/oval) for either controller — only hover tested so far for the 2-drone case.

### Still to do before this counts as fully closed

- This is a **simulator fix, not a hardware validation.** `naindi.rs`/`naindi_hybrid.rs`
  remain unflown — clearing the SIL is a precondition for flying, not a substitute for the
  same hardware-validation gate every other controller change goes through.
- 2-drone downwash-backend and 2-drone maneuver coverage still open (see above).
- The `--zero-state-acc`/`--real-substeps`/`--motor-tau`/`--replay-log` flags added to
  `naindi_reference_closed_loop.py` while chasing this are now genuinely useful diagnostic
  tools, not just one-off scaffolding — worth keeping for the next investigation like this.

## Investigation plan — root-causing the real-SIL-only gap, 2026-09-18

**Status when this was written:** the algorithm and the port are both confirmed correct
(closed-loop, not just single-tick) — see the section above. `naindi.rs` (controller=7)
still crashes in the real CS2/ROS2 SIL under every airframe/gain configuration tried; the
standalone harness cannot reproduce that crash despite matching airframe, gains, and
trajectory phases. **The remaining question is narrower than "is the algorithm right" — it's
"what does the real SIL do differently that this harness doesn't."** This section exists so
that question can be picked up at any point without re-deriving where things stand.

**Do not re-run more airframe/gain sweeps as the default next move.** That line is
exhausted (`docs/07`'s History, five separate parameter classes tested: attitude gains,
mass/kt together and separately, position gains, full reference-consistency). The next
useful step is observational, not another A/B flight.

### Ranked hypotheses

1. **Physics/control discretization mismatch (leading candidate).** `crazyflie_sil.py` steps
   physics at 2 kHz but only *attempts* a controller call every other substep (~1 kHz); the
   Rust port's own `RATE_DO_EXECUTE`-equivalent gate then halves that again to the true
   500 Hz. So the real SIL holds one control output across **4** physics substeps
   (dt=0.0005 each) between genuine recomputes. This harness steps physics 1:1 with control
   calls at dt=0.002 (500 Hz) — same nominal rate, different substep structure. A held
   command across a longer physics-only interval could behave differently under numerical
   integration, especially near a marginal stability boundary.
   - **Test:** modify `naindi_reference_closed_loop.py` to replicate the exact real
     substep/hold pattern — `dt_physics=0.0005`, attempt a controller call every 2nd
     substep, let the Rust-side rate gate decide whether it actually computes, hold the
     previous `Action` on every substep where the controller didn't run. Compare against
     the current 1:1 500 Hz version on the identical trajectory.
2. **Real HLC trajectory shape, not a hand-picked quintic.** `simple_flight.py`'s
   `uploadTrajectory` sends real Poly4D/min-snap coefficients (Richter/Mellinger-style),
   not necessarily matching this harness's minimum-jerk climb/land — different jerk/snap
   content, different segment boundary conditions.
   - **Test:** export the actual Poly4D coefficients `export_poly4d`/`simple_flight.py`
     generates for `--trajectory hover --height 1.0`, evaluate that polynomial directly for
     `sp.position/velocity/acceleration` in the harness instead of the quintic.
3. **x/y motion, not pure-Z.** The real flight starts from `crazyflies_sim1.yaml`'s
   configured initial position (nonzero x, per the very first crash CSVs read this
   session), and per-drone `goTo`/takeoff sequencing might introduce small x/y motion this
   harness never exercises (pure vertical climb from the origin).
   - **Test:** match the real initial position and any x/y setpoint content from a real
     log; cheap to try once hypothesis 1 or 2 are ruled out.
4. **State/sensor handoff mismatch.** Assumed but never directly confirmed: that
   `crazyflie_sil.py` hands the controller the plant's exact ground-truth state (no
   estimator, no noise, no delay) — matching what this harness constructs directly from
   `Quadrotor.state`. If that assumption is wrong (a KF/estimator stage, or some delay
   between physics and control not visible from reading the code), it would explain a gap
   no amount of trajectory/timing tuning could close from this harness alone.
   - **Test:** only worth pursuing after 1-3 are exhausted — would need direct
     instrumentation of a real SIL run, not further reasoning about the code.

### The single most useful concrete action, if picked up

**Instrument a real, currently-crashing CS2 SIL run with per-tick logging**, rather than
keep guessing from outside. Add a temporary, env-var-gated debug CSV to
`crazyflie_sil.py`'s `oot2` path (mirroring the existing `NAINDI_SCALED_GAINS`/
`NAINDI_POS_GAIN_SCALE` opt-in pattern) that dumps, every tick the controller actually
computes: `setpoint.position/velocity/acceleration`, `state.position/velocity`,
`sensors.gyro`, `control.thrustSi/torque`, and the wall-clock gap since the previous real
compute. Run the known-crashing config (brushless airframe, default gains,
`state_naindi/2026-09-17_202027`'s own setup) with this logging on, focused on the window
right before divergence.

This single log turns hypotheses 1, 2, and 3 above from guesses into directly observable
facts: the real dt-between-calls answers (1) immediately; the real setpoint trajectory
shape answers (2); the real position trace answers (3). **Whatever it shows, replay that
exact logged sequence (setpoint + sensors + rpm, tick by tick) through
`naindi_reference_closed_loop.py` in a new `--replay-log CSV` mode** — if replaying the
*real* recorded inputs into the *standalone* harness reproduces the crash, the discretization/
trajectory/motion difference is confirmed as the cause and can be fixed directly; if it
still doesn't crash, the gap is in the state/sensor handoff (hypothesis 4) or something
this plan hasn't anticipated, and that failure mode itself is new, useful information.

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

| Config | Result |
|---|---|
| `bl` platform, unscaled gains (2026-09-16 original) | **crashes** ~t=13s |
| `bl` platform, `NAINDI_SCALED_GAINS=1` (2026-09-17) | **crashes** ~t=12.6s, same signature |
| **`cf2`/reference platform** (this recipe), real (brushless) mass/kt | **clean hover**, roll/pitch <1.2°, then **crashes during landing**, t≈17-19s, 30-45° |
| `cf2`/reference platform + `NAINDI_REFERENCE_MASS=1` (0.034 kg, **our own** real kt — self-inconsistent mix) | **worse** — oscillation from ~t=12s, tumble ~t=16s |
| `cf2`/reference platform + `NAINDI_REFERENCE_MASS=1` (0.034 kg + **the reference's own** measured `kappa_f`, fully self-consistent — see below) | **still crashes**, but now DURING hover (~t=12.6-15.8s), not landing |

**The reference authors' own measured thrust constants exist** — not baked into
`controller_lee.c` as a literal (their `kappa_f[4]` is a plain runtime `PARAM_FLOAT`, zero by
default, meant to be set from a real calibration), but present in their own host-side tooling:
`~/Desktop/NA-INDI/pwm2thrust.py` and `LMCE/residual_calculation.py`, both hardcoding
`kappa_f = [2.139974655714972e-10, 2.3783777845095615e-10, 1.9693330742680727e-10,
2.559402652634741e-10]` (same `force[i] = kappa_f[i] * rpm²` formula `naindi.rs` uses). The
`NAINDI_REFERENCE_MASS=1` override in `crazyflie_server.py` now sets **both** `g_indi_mass`
(0.034 kg, `controller_lee.c`'s own `.mass`) and `g_indi_kt1-4` (the array above) together,
giving — for the first time — mass, kt, arm, t2t, and J **all** from the reference authors' own
airframe, fully self-consistent, no mixing with this project's own numbers anywhere.

**Result: still crashes.** Not better than the real-brushless-mass/kt run, and by a different
signature — a growing oscillation starting almost immediately after climb completes (~t=12.6s),
period **≈1.4s** (measured from 5 roll zero-crossings), tumbling by ~t=15.8s, well inside the
requested 8s hover — **this is not a landing event at all**, it happens entirely during hover.
That reframes the earlier "landing crash" finding: it isn't landing-specific — different
mass/kt combinations shift *when* an underlying marginal oscillation tips into a tumble, not
whether one exists. **Conclusion: this is not fixable by getting mass/kt "right" in any
combination tried so far — real-brushless and the reference authors' own numbers both
eventually diverge, just at different times.**

### Position-gain diagnostic, 2026-09-17 — position loop IS involved, but not the whole story

**Important clarification on what this override is for, before the result:** by this point
`KR`/`KOMEGA`/`KPOS_P`/`KPOS_D`/`KPOS_I`/mass/kt/arm/t2t/J had *all* already been set to the
reference authors' own values in the fully-self-consistent run above, and it still crashed —
so there is no "which of our numbers is wrong" question left to answer. `POS_GAIN_TEST_OVERRIDE`
(mirroring `GAIN_TEST_OVERRIDE`'s exact shape) exists purely to test whether the ~1.4s
oscillation tracks `KPOS_P` the way a real position-loop resonance would — a diagnostic to
localize the instability, not a step toward a flyable configuration. Default (`None`) leaves
`KPOS_P/KPOS_D/KPOS_I` at the reference's own literal `12.0/10.5/2.0`.

```bash
# crazyflie_sil.py: NAINDI_POS_GAIN_SCALE=<float> scales KPOS_P/D/I together by that factor
# (controller=7/oot2 only). Combine with NAINDI_REFERENCE_MASS=1 for the fully-consistent test.
NAINDI_REFERENCE_MASS=1 NAINDI_POS_GAIN_SCALE=0.25 ros2 launch ...
```

**Result:** at `NAINDI_POS_GAIN_SCALE=0.25` (`KPOS_P=3.0, KPOS_D=2.625, KPOS_I=0.5`), the
oscillation period measured **~1.85s** (6 zero-crossings, `state_naindi/2026-09-17_210144`),
up from the unscaled ~1.4s. A clean single-degree-of-freedom position-loop resonance predicts
`period ∝ 1/√KPOS_P`, i.e. quartering `KPOS_P` should roughly **double** the period (1.4→2.76s);
the actual shift was **~34%** (1.4→1.85s). **The position gain genuinely moves the oscillation
(ruling out "irrelevant"), but far more weakly than a pure position-loop mode would — this is a
coupled position+attitude oscillation, not a clean single-loop resonance.** Consistent with
that: max roll/pitch was similar or slightly worse at the softened gain (51°/60° vs ~50°/65° at
default), not better — softening the position loop alone didn't meaningfully improve stability,
which a purely-position-loop-driven instability should have shown clearly.

**Where this leaves things:** every single-parameter or single-loop-gain hypothesis tested so
far (attitude gains, airframe mass/kt/arm/t2t/J individually and fully self-consistent,
position gains) has been partially or fully refuted. The instability appears to be a genuine
coupled-loop phenomenon in this specific closed-loop combination (this Rust port + this
project's HLC/trajectory + this SIL's EKF/sensor model), not traceable to any one number being
wrong. Properly characterizing it from here would need either a real linearized closed-loop
stability analysis (root locus over the coupled position-attitude system) rather than further
single-variable A/B flights, or accepting that this needs its own dedicated investigation
before controller=7 is a realistic near-term candidate — not something to resolve with one more
quick parameter sweep.

Trajectory-transition-specific causes (integral windup or a setpoint discontinuity at the
hover→land handover, the original hypothesis) are now less well supported — the reference-mass
run's oscillation starts well before any landing transition and has the same qualitative shape
(growing, several-second oscillation into a tumble) as the real-mass run's landing crash. This
was not directly ruled out (the current SIL harness doesn't log the setpoint/`KI_ATT` state,
only position and attitude), but the position-loop-frequency match is the more specific,
better-supported lead right now.

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
