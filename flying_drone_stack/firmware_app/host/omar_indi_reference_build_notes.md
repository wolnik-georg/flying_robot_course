# Building the reference `cffirmware` for `test_omar_indi_reference.py`

Mirrors `naindi_reference_build_notes.md`'s recipe exactly, for `controller_omar_indi.c`
(`stabilizer.controller=9`) against `~/Desktop/crazyflie-firmware-omar`'s own, completely
unmodified `controller_lee.c`. Simpler than the NA-INDI case: no NN (`nn.c`/`nn_utils.c` not
needed), no `<motors.h>` include, and no gain/arm/t2t pinning required at all -- both sides are
built for the SAME platform (`CONFIG_PLATFORM_CF21BL`), so every airframe constant comes from
the identical `platform_defaults_cf21bl.h` logic on both sides automatically.

**Mass, current recipe (2026-09-23, revised):** both sides use `CONFIG_MODIFIED_CF_MASS=42700`
-- Omar's own brushless-established mass (his `cf21blrpm_defconfig`), not this project's
independently measured 41.0g. Operator decision: use exactly the airframe constant set he
established for brushless throughout, not a mix. Get this by seeding the scratch reference with
his own `cf21blrpm_defconfig` (not the bare `cf21bl_defconfig`, which has no mass override), and
by the `-DCONFIG_MODIFIED_CF_MASS=42700` compile define this project's own host build now
carries (`bindings/setup.py`, `LOCAL_MODIFICATIONS.md`) -- the embedded build's `app-config-bl`
and this host build's `.config` are two entirely separate files, confirmed directly (see "Bug #1
addendum" below), so the mass define has to be set in both places independently.

```bash
SCRATCH=/tmp/omar_indi_build   # any throwaway directory
rm -rf "$SCRATCH"; mkdir -p "$SCRATCH"
cp -r ~/Desktop/crazyflie-firmware-omar/. "$SCRATCH/"
cp -r ~/Desktop/crazyflie-firmware/vendor/CMSIS/.    "$SCRATCH/vendor/CMSIS/"
cp -r ~/Desktop/crazyflie-firmware/vendor/FreeRTOS/. "$SCRATCH/vendor/FreeRTOS/"
cp -r ~/Desktop/crazyflie-firmware/vendor/libdw1000/. "$SCRATCH/vendor/libdw1000/"

cd "$SCRATCH"
make cf21blrpm_defconfig   # HIS brushless target config -- already carries
                           # CONFIG_MODIFIED_CF_MASS=42700 via his own defconfig, no override needed
sed -i 's/# CONFIG_MOTORS_REQUIRE_ARMING is not set/CONFIG_MOTORS_REQUIRE_ARMING=y/' build/.config
make oldconfig
```

`controller_lee.c` is already in his own `bindings/setup.py`'s `fw_sources` and already
`%include`d in `bindings/cffirmware.i` (both files, no patching needed there) -- his repo was
already set up to host-test this exact file. Three things are still missing for a working host
build, all scratch-only, added as `bindings/host_stubs.c`:

- `logGetVarId`/`logGetUint` -- `controllerLee()` reads RPM via
  `logGetUint(logGetVarId("rpm","mN"))`, a genuinely different mechanism from `rpm_get_all()`
  (what this project's own `lib.rs`/`naindi.rs` use). `oot_test_set_rpm(m1,m2,m3,m4)` feeds it.
- `paramGetVarId`/`paramGetUint` -- `controllerLeeInit()` checks `paramGetUint(idDeckBcRpm)==1`
  once to decide whether the RPM branch is exercised at all; report "present" unconditionally.
- `usecTimestamp()` -- fixed +2000us per call (500 Hz), not real wall-clock, for the same reason
  as every other host stub in this file family: a host loop calling fast enough that two calls
  land in the same microsecond would make `dt=0` in the un-guarded `(omega-omega_prev)/dt`.

```python
# setup.py additions (scratch copy only):
sources = fw_sources + ["bindings/host_stubs.c", "build/cffirmware_wrap.c"]
extra_compile_args = [..., "-DCONFIG_PLATFORM_CF21BL"]
```

```bash
cd "$SCRATCH" && rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python
```

Then, on **this project's own tree** (not the scratch copy): `controller_omar_indi.c` was added
to `crazyflie-firmware/bindings/setup.py`'s `fw_sources`, and `%include "controller_omar_indi.h"`
to `bindings/cffirmware.i` (both persistent changes — see `LOCAL_MODIFICATIONS.md`). The same
three host stubs were added to this project's own `firmware_app/host/oot_host.c`, reusing the
**existing** `oot_set_rpm()`/`g_host_rpm[]` array for the RPM read path (one physical quantity,
one place to set it, whichever mechanism a given controller happens to use) rather than adding a
second, parallel array.

```bash
cd ~/Desktop/crazyflie-firmware
rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python
```

Then run the comparison from this project:

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
python3 host/test_omar_indi_reference.py "$SCRATCH/build" ~/Desktop/crazyflie-firmware/build
```

## Numerical result (2026-09-22)

**All 6 hand-picked test vectors match to `d=0.00e+00`** on both thrust and torque — not merely
within tolerance, bit-for-bit identical. Stronger than the Briesewitz comparison's ~1e-9 (which
needed explicit gain/mass/kt pinning to isolate "is the translation correct" from "different
airframe constants") — here there is nothing to pin, since the ported file reads the exact same
platform macros on both sides by construction. This confirms the integration (renaming,
Kconfig wiring, the two small backports in `math3d.h`/`stabilizer_types.h`, the platform-constant
addition in `platform_defaults_cf21bl.h`) introduced **zero** behavioral difference from his
source.

## Firmware dispatch wrapper — actually exercised, not just compiled (2026-09-23)

Every test above, and the SIL wiring below, calls the explicit-self core functions
(`controllerOmarIndiInit`/`controllerOmarIndi`) directly. The functions real hardware's
`controller.c` dispatch table actually calls — `controllerOutOfTree4Init`/`Test`/
`controllerOutOfTree4`, a thin wrapper around a static `g_self` — are gated behind
`#ifdef CRAZYFLIE_FW`, which the host/SWIG build never defines (confirmed:
`hasattr(cffirmware, 'controllerOutOfTree4Init')` is `False` there). That code path had only
ever been *compiled* (as part of `make DRONE=bl`), never *executed*, by anything in this project.

Closed with a standalone isolated compile, now a permanent, reusable test:
`host/test_oot4_dispatch_wrapper.sh` (builds `controller_omar_indi.c` a second time with
`-DCRAZYFLIE_FW` plus `oot4_dispatch_stubs.c`, links `test_oot4_dispatch_wrapper.c`'s `main()`,
which calls `controllerOutOfTree4Init()` → `controllerOutOfTree4Test()` →
`controllerOutOfTree4()` directly). Result (2026-09-23, revised for the 42700 mass): `thrustSi=
0.418887` — exactly `CF_MASS × g = 0.0427 × 9.81 = 0.418887` (pure gravity feedforward; `indi`
defaults to `0` in `g_self`'s struct initializer and there is no PARAM subsystem in this isolated
harness to override it, so this is the geometric-only path). Confirms the wrapper correctly
threads through to the real control law with the right compiled-in constants — the dispatch path
is not just untested code sitting in the binary. (Originally run 2026-09-22 against the stock
`CF_MASS=0.0393` fallback, `thrustSi=0.385533` — before either mass override reached the host
build at all; see the Bug #1 addendum below.)

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
./host/test_oot4_dispatch_wrapper.sh
```

## CS2 SIL closed-loop validation

Wired into `crazyflie_sil.py` as `'oot4'`, mirroring the `'lee'` controller's pattern exactly
(`controllerOmarIndi()` takes an explicit `self` struct, so **no `select_drone`-style state-swap
plumbing is needed at all** — unlike `'oot'`/`'oot2'`/`'oot3'`, each vehicle just owns its own
struct instance, the simplest of the four out-of-tree controllers wired into this SIL).

### Bug #1 (caught before it shipped): plant mass/kt sync

`crazyflie_server.py`'s `_setup_oot()` builds the simulated plant's mass/kt from `g_indi_mass`/
`g_indi_kt1-4` (`traj_iface.c`'s globals) for `'oot'`/`'oot2'`/`'oot3'` — but
`controller_omar_indi.c` never reads those globals; it reads `CF_MASS`/`MOTORRPM2FORCE` from the
platform header instead. Naively adding `'oot4'` to the existing sync path would have built a
plant with `g_indi_mass` (0.041 kg, this project's own measured brushless mass) while the
controller's internal model used `CF_MASS` (0.0393 kg, the stock default) — a silent ~4% mass
mismatch, precisely the class of bug this whole `_setup_oot()` method exists to prevent. Fixed
with a dedicated branch reading `oot_omar_mass()`/`oot_omar_kt_equiv()` (two new `oot_host.c`
getters) instead. Confirmed in the launch log: `simulated airframe taken from firmware:
mass=0.0393 kg` — the controller's own number, not the unrelated `g_indi_mass`.

**Bug #1 addendum (2026-09-23) — the mass value itself, and a separate config-drift bug found
while revising it:** the `0.041 kg`/`0.0393 kg` numbers above were both narrated on 2026-09-22,
before the operator decision to use Omar's own brushless mass throughout. That decision (see the
top of this file) superseded `g_indi_mass=0.041` as the *intended* plant value — it was never the
right number to sync `CF_MASS` against in the first place, independent of Bug #1's sync-path fix,
which remains correct and unaffected. Revising it surfaced a second, independent bug: the host/
SWIG build's own `build/.config` never had `CONFIG_MODIFIED_CF_MASS` set at all — confirmed via
`grep CONFIG_MODIFIED_CF_MASS build/.config` showing `# CONFIG_MODIFY_CF_MASS is not set` even
after `app-config-bl` (the *embedded* build's Kconfig merge) had `=42700`. The embedded and host
builds are two entirely separate `.config` files; setting one never touched the other. Every prior
host/SIL/numerical run of controller=9 had silently used the stock `CF_MASS=0.0393` fallback
regardless of what `app-config-bl` said. Fixed by adding
`-DCONFIG_MODIFIED_CF_MASS=` + `os.environ.get("OOT_CF_MASS_UG", "42700")` directly to
`bindings/setup.py`'s compile args (mirroring the existing `OOT_PLATFORM` `-D` pattern), and by
seeding the *reference* scratch build from Omar's own `cf21blrpm_defconfig` (which propagates the
value through the normal Kconfig→autoconf.h path with no `-D` hack needed). Both sides now read
`CF_MASS=0.0427 kg` — confirmed in the re-run launch log: `simulated airframe taken from firmware:
mass=0.0427 kg`.

**Unit-conversion detail worth flagging for anyone touching this again**: his `MOTORRPM2FORCE`
convention is `force = MOTORRPM2FORCE * (rpm in RAD/S)^2`; this project's plant/`g_indi_kt*`
convention is `force = kt * (rpm in REV/MIN)^2`. `oot_omar_kt_equiv()` folds in the
`(2*pi/60)^2 ≈ 0.010966` conversion factor once, in C, rather than duplicating it in Python —
getting this wrong would silently under-thrust the simulated plant by ~91x.

### Bug #2 (found via standalone diagnostic, fixed 2026-09-23): the height deficit

The first pass through every configuration below showed a reproducible ~15-20% steady-state
height deficit (hover plateaued at z≈0.82m against a commanded 1.0m). **Not** a property of
`controller_omar_indi.c`, his gains, or the airframe — a second SIL wiring bug.
`crazyflie_sil.py`'s `self.kt`/`self.thrust_max` (needed by the plant's `pwm_to_rpm()`/
`pwm_to_force()` to invert a commanded PWM back into a physically consistent force) are set for
every other out-of-tree controller but were **never set for `'oot4'`** — an omission. With
`self.kt` left at its class default of `None`, the plant silently fell back to a *different*,
generic thrust polynomial (IMRCLab's own system-ID fit, meant for a different airframe/mixer
entirely) instead of exactly inverting this controller's own thrust model — plant and controller
no longer agreed on how much force a given command produced, and a P/D-only position loop
(`Kpos_I=0`) has nothing to cancel a steady mismatch like that.

**Found by a standalone diagnostic, not by guessing**: `host/omar_indi_height_deficit_diag.py`
reuses the exact same `Quadrotor` plant class as the CS2 SIL, correctly passed `kt`/`arm`/`t2t`
from the start (the way a standalone script naturally would) — and reached the commanded height
**exactly** (`z=1.0000`) under every gain/`indi`-mode combination tried. That "disagrees with the
real SIL" result was the actual signal: the bug lived in the SIL wiring, not the control law —
the same shape of finding (simplified harness clean, real SIL not) that led to the `state.acc`
discovery for controller=7/8. The original working theory (his `Kpos_I=0`) was wrong; kept as a
general-purpose diagnostic tool regardless, per its own updated docstring.

**Fix**: add `self.kt = [firm.oot_omar_kt_equiv()] * 4` and
`self.thrust_max = firm.oot_thrust_max()` to the `'oot4'` branch — the same two getters the
Bug #1 fix already established. `self.thrust_max` crashed the server outright the first time
this was actually tested (`TypeError: float * NoneType` in `pwm_to_force()`) — caught
immediately, not a silent gap.

### Methodology note: exclude landed (z≈0) rows before reporting max roll/pitch

Found while checking the 3-drone downwash result below: a vehicle sitting motionless on the
ground after landing, with a near-zero commanded thrust, can show its logged attitude quaternion
drift smoothly through a large apparent angle (up to 103° observed) with **zero** physical
consequence — it's on the ground, not tumbling. `z < 0.05` rows must be excluded before quoting
a max-attitude figure, or a completely benign post-touchdown artifact reads as a stability
finding. Applied retroactively to every number below; confirmed it changed nothing for any
`backend: np` run (all landed promptly, short idle tail) — only the longer-running 3-drone
`neuralswarm` run had a long enough post-landing idle period for the artifact to dominate the
naive whole-recording max.

### Final, verified results

All results below are post-fix (both bugs), in-flight-only (landed rows excluded per the note
above).

**`backend: np`** (no interaction model — isolates the controller/plant wiring from any downwash
question):

| Config | Robots | Trajectory | Max \|roll\|/\|pitch\| | z during maneuver | Verdict |
|---|---|---|---|---|---|
| `crazyflies_sim1.yaml` | 1 | hover | 0.0°/0.0° | **1.0000 m** — exact | clean |
| `crazyflies_sim1.yaml` | 1 | figure8 (`--kt 0.008`) | 25.2°/20.1° | peak **1.02 m** | clean |
| `crazyflies_sim1.yaml` | 1 | circle (`--kt 0.1`) | 38.5°/29.9° | peak **1.02 m** | clean |
| `crazyflies_sim.yaml` (2 robots) | 2 | hover | 0.0°/0.0° (both) | both **1.0000 m**, distinct XY maintained | clean, no cross-contamination |
| `crazyflies_sim3.yaml` (3 robots) | 3 | hover | 0.0°/0.0° (all three) | all **1.002 m**, distinct XY maintained | clean, no cross-contamination |

**`backend: neuralswarm`** (real coupled downwash-interaction model, `server_sim_omar_indi_dw.yaml`,
`--ros-args -p use_sim_time:=true` — required for this ~4x-slower-than-real-time backend, per
`docs/09_Simulation.md`, confirmed not controller-specific):

| Scenario | Robots | Geometry | Max \|roll\|/\|pitch\| (in-flight) | Realized relative geometry (steady hold, t=17-23s) | Verdict |
|---|---|---|---|---|---|
| A1 (vertical stack, dz=0.5m) | 2 | stacked | 30.1°/0.0° (climb transient) | dx=dy≈0.000 (perfect lateral); dz≈-0.64m vs -0.5m target (~14cm off) | clean, no NaN; modest z offset plausibly real downwash coupling, not divergence |
| C1 (side-by-side coplanar) | 3 | no vertical offset | 37.9°/0.0° (climb transient) | **[0,0,0] on all three pairs, exactly** | clean, no NaN, no cross-contamination, perfect station-keeping |

The formation script's own `run_formation.py` end-of-flight "realised relative geometry" check
measures a single sample at whatever point it considers "settled" — for A1 this landed on a
mid-landing-transition sample and reported a misleading 500mm z-error; the numbers above are
from the actual steady hold window, read directly from the recorded CSVs. C1's near-zero
vertical offset makes it the less demanding case for downwash coupling (drones are laterally
separated, closer to the neighbour-gate boundary than truly stacked) — consistent with its
cleaner realized-geometry result. A1's ~14cm z deviation is a genuine open question (is it real
downwash pushing the bottom vehicle down and the top vehicle up, or a station-keeping gain
characteristic?) but is a modest, bounded, non-divergent effect, not a stability concern.

Zero NaN, zero divergence, across every `np` and `neuralswarm` configuration tried. Confirmed
per-vehicle state isolation directly from the recorded CSVs in every multi-drone run (each
vehicle's logged trajectory starts from and tracks its own configured `initial_position`, never
a shared/averaged one) — the "no `select_drone` needed" design claim holds up under real
multi-vehicle runs, including under real downwash coupling, not just in theory.

### Remaining finding, unaffected by any fix above

**Trajectory-start roll/pitch transients** (25-38° on figure8/circle/A1/C1 climb). Not new or
`oot4`-specific — this file's sibling doc (`naindi_reference_build_notes.md`) already documents
the identical pattern for controller=7/8 and traces it to `simple_flight.py`'s own
`go_to`→`start_trajectory` setpoint discontinuity, common to every controller tested through
this harness.

## Status — what's closed, what genuinely isn't

**Closed** (software/sim only, everything above): numerical verification, both host-dispatch
paths (explicit-self and the real firmware wrapper), single-drone across 3 trajectory shapes,
2-drone and 3-drone with no interaction model, 2-drone and 3-drone with real coupled downwash.
Two real integration bugs found and fixed, not papered over.

**Cannot be closed without hardware, and is not being attempted here**:
- **Real flight.** Zero hardware testing of any kind. This remains the actual point of the C.0
  gate every other controller in this project goes through — sim-clean is a precondition, never
  a substitute.
- **Gain tuning for our real airframe.** Every run above uses his default gains (`Kpos_P=7`,
  `Kpos_D=4`, `KR`/`Komega`/`KI`, `J` — the standard CF2.1 ETHZ inertia, not this airframe's
  measured `23.951e-6/23.951e-6/32.347e-6`). Nothing has been tuned or validated against this
  project's real dynamics.
- **`MOTORRPM2FORCE` bench verification.** His CF21BL calibration is used as-is; this project's
  own brushless `kt` was never independently bench-calibrated (still a placeholder per
  `project_brushless_commissioning` memory), so there is nothing on our side to cross-check his
  number against. `arm_length`/`THRUST2TORQUE` *are* independently confirmed identical — see
  `docs/41_Pure_INDI_Implementation_Comparison.md` §8.
- The A1 ~14cm z-offset under real downwash — plausible explanations exist (see above) but it
  hasn't been isolated from a real gain/station-keeping characteristic.

No decision has been made about whether this controller is pursued further, per
`docs/41_Pure_INDI_Implementation_Comparison.md` §7. This document only establishes that
everything achievable without a real drone has now been done, and done to the same standard
`controller=7`/`8` were held to.
