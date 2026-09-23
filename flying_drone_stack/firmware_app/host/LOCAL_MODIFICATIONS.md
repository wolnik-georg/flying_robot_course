# Local modifications to `crazyflie-firmware`

**Decision (2026-08-23): these stay exactly as they are. Intentional. Do not remove, rebase or
clean them.** This question is closed — the file exists so it does not come up again.

`~/Desktop/crazyflie-firmware` tracks **bitcraze upstream** and is deliberately not forked
(see the reasoning in the project memory: a fork has to be kept in sync and adds a repository to
reason about, which is not worth it for a small additive change). The consequence is that
several files carry uncommitted local changes, plus two new untracked files. That is the
intended steady state, not an untidy loose end.

Each file now carries a `LOCAL MODIFICATION -- INTENTIONAL, DO NOT REVERT` comment in place.
This manifest is the durable copy, because comments in a tree we do not own can be wiped by a
checkout or an upstream pull.

---

## The modifications

| File | What it changes | Consequence if lost |
|---|---|---|
| `bindings/setup.py` | Links the out-of-tree controller into the SIL build; renames six colliding symbols (`peer_get_all` added 2026-08-23 for the residual network -- the host has no `peer_localization`, so `oot_host.c` injects peers instead); sets `CONFIG_PLATFORM_CF21BL`. **2026-09-18: added `controller_indi.c`/`position_controller_indi.c` to `fw_sources`** — Bitcraze's own stock INDI wasn't compiled into the host build at all before this; needed to wire it into the SIL as a third reference point (see `controller_indi.h`'s row below and `docs/07`) | **The simulator does not build.** Without the platform define it builds but silently uses the wrong airframe — `THRUST_MAX` 0.1125 N/motor instead of 0.2 — and attitude INDI never leaves the ground. Losing the `controller_indi.c` addition just removes the `indi` controller option (`crazyflie_sil.py` — a separate, `flying_robot_course`-tracked change) |
| `bindings/cffirmware.i` | Exposes `controllerOutOfTree*`, the RPM/log helpers, the airframe constants, the gain globals and the `rnn.*` residual-network globals + peer injection. **2026-09-18: added `%include "controller_indi.h"`** (was never SWIG-exposed before, despite the airframe-matching filter tweak below existing since July). **2026-09-18: added `naindi_select_drone`/`naindi_hybrid_select_drone`** (per-vehicle state-swap for controller=7/8, mirroring `oot_select_drone` — see "Multi-vehicle support for controller=7/8" below) | Simulator cannot select or configure our controller |
| `src/deck/drivers/src/usddeck.c` | `MAX_USD_LOG_VARIABLES_PER_EVENT` 20 → **48** (was 40 until 2026-09-21). **2026-09-21 evening:** `usd.runTag` param + log (`runTag` uint32) for session pairing — see `docs/39`. Config uses all **48** slots (47 prior channels + `usd.runTag` as the 48th; `indi.e_r_norm` retained). | **⚠️ The most dangerous one to lose.** The thesis `usd_thesis_config.txt` records **48** variables including dual RPM, `indi.e_r_*`, and `usd.runTag`. At the stock limit of 20 (or an old 40 build) extra lines are **skipped** (`Skip log variable … out of storage` in DEBUG only) — missing columns, no flight abort. Verify with `grep MAX_USD_LOG_VARIABLES_PER_EVENT src/deck/drivers/src/usddeck.c` → **48** before C.1 |
| `src/modules/interface/controller/controller_indi.h` | Filter cutoff and `g1`/`g2` re-derived for the CF21BL airframe through stock INDI's legacy output path (July 2026 investigation) — `STABILIZATION_INDI_FILT_CUTOFF` 8.0 (stock) → **70.0 Hz**, matching the standard/upgraded platform's own `fc_bw`, not the brushless-flown 60Hz (see the file's own comment for why). **2026-09-18: actually run closed-loop for the first time** (via the new `indi` SIL controller above) — this is the config already flown clean on real hardware, confirmed apples-to-apples, not a mismatch | The stock-INDI comparison is no longer on equal terms with ours |
| `src/modules/interface/controller/controller.h`, `src/modules/src/controller/controller.c`, `src/modules/src/Kconfig` (2026-09-14) | Adds `ControllerTypeOot2` / `CONFIG_CONTROLLER_OOT2` — a **second, independent** out-of-tree controller slot (`stabilizer.controller=7`) alongside the existing `ControllerTypeOot` (`=6`, our geometric/INDI, `ctrl_mode` 0-3). Exists so a byte-faithful Rust port of Cobo-Briesewitz's NA-INDI (`firmware_app/src/naindi.rs`) can fly without any risk of interfering with our own controller — separate enum value, separate dispatch row, separate Rust module, no shared state | Controller 7 does not exist / does not build; falls back silently to whatever `ControllerType_COUNT`-indexed garbage or a build error, depending on how it's lost |
| Same three files (2026-09-16) | Adds `ControllerTypeOot3` / `CONFIG_CONTROLLER_OOT3` — a **third, independent** out-of-tree controller slot (`stabilizer.controller=8`). Ports the SAME reference file as Oot2 (`controller_lee.c`) but with `use_nn` enabled and their real trained network included (`firmware_app/src/naindi_hybrid.rs` + `naindi_hybrid_weights.rs`) — true NA-INDI, not their plain INDI. Own enum value, own dispatch row, own Rust module, no shared state with Oot or Oot2. **Compiles, links, and is now numerically verified — 6/6 test vectors match the reference's own compiled `controller_lee.c` (`use_nn=7`) to ~1e-9 on thrust and torque (2026-09-16, `test_naindi_hybrid_reference.py`). Still never flown — do not fly without clearing the hardware-validation gate.** | Controller 8 does not exist / does not build |
| `src/modules/interface/stabilizer_types.h`, `src/hal/src/sensors_bmi088_bmp3xx.c` (2026-09-14) | Adds `sensorData_t.gyroNoLpf` (the pre-LPF gyro), populated right after `sensorsAlignToAirframe` and before `applyAxis3fLpf` overwrites `sensorData.gyro` in place — ported verbatim from NA-INDI-firmware, which added the same field for the same reason. Operator's explicit instruction (2026-09-14): controller=7 must read exactly the signal their reference does, not a filtered substitute, "no exceptions apart from mass/inertia/kt" | Controller 7's attitude-INDI angular-acceleration term silently falls back to the regular filtered gyro — no build error, just a quiet fidelity loss to the port. `bindings/cffirmware.i` also needs `%include "stabilizer_types.h"` to still see the new field (it already does, no separate change needed there beyond the controller=7 entry points) |
| Same three files + `src/modules/src/controller/Kbuild` (2026-09-22) | Adds `ControllerTypeOot4` / `CONFIG_CONTROLLER_OOT4` — a **fourth, independent** out-of-tree controller slot (`stabilizer.controller=9`). Unlike Oot2/Oot3, this one is **not a Rust port** — it's a literal C copy of the supervisor's own `controller_lee.c` (`~/Desktop/crazyflie-firmware-omar`), verified byte-identical after normalizing renamed identifiers (`controllerLee*`→`controllerOmarIndi*`/`controllerOutOfTree4*`, `ctrlLee`→`ctrlOmarIndi` param/log groups — pure renames, zero control-law lines touched, done to avoid colliding with our own stock `controller_lee.c`/`ControllerTypeLee`). See `docs/41_Pure_INDI_Implementation_Comparison.md`. Own enum value, own dispatch row, own C file (`controller_omar_indi.c`/`.h`), no shared state with Oot/Oot2/Oot3. `app-config-bl` now also enables `CONFIG_CONTROLLER_OOT3` (previously off) purely so the enum stays contiguous and Oot4 lands at 9, not 8 — does not change any existing flight config. **Compiles for the brushless target. 2026-09-22, same day: numerically verified 6/6 to `d=0.00e+00` against his own compiled build (`firmware_app/host/test_omar_indi_reference.py`), and SIL-clean single/2/3-drone (`backend: np`, `firmware_app/host/omar_indi_reference_build_notes.md`). NEVER FLOWN, not queued to fly — same C.0 hardware-validation gate as controller=7/8.** | Controller 9 does not exist / does not build |
| `bindings/setup.py`, `bindings/cffirmware.i` (2026-09-22) | Adds `controller_omar_indi.c` to `fw_sources` and `%include`s `controller_omar_indi.h`, exposing `controllerOmarIndi_t`/`controllerOmarIndiInit`/`controllerOmarIndi` to the host/SWIG build — same pattern already used for stock `controller_lee.c`. Also exposes two new `oot_host.c` getters, `oot_omar_mass()`/`oot_omar_kt_equiv()`, used by `crazyflie_server.py`'s plant-sync path (see below). **2026-09-23**: adds `-DCONFIG_MODIFIED_CF_MASS=42700` to `extra_compile_args` — a real config-drift bug, not cosmetic: this host build's own `.config` never had `CONFIG_MODIFIED_CF_MASS` set (confirmed directly, independent of `firmware_app/app-config-bl`'s own override, which lives in a completely separate Kconfig `.config` that never reaches this build), so every host/SIL test of controller=9 had silently been running on the stock 39.3g fallback, not the 42.7g actually flashed to hardware. `42700` = Omar's own brushless-established mass (his `cf21blrpm_defconfig`), matching `app-config-bl` — see that file's own comment for why this project's independently measured 41.0g was deliberately NOT used (operator decision 2026-09-23: use exactly the airframe set he established for brushless, not a mix). Update both places together if this value ever changes | Every host/SIL test of controller=9 silently runs on the wrong (stock) mass, undetected — exactly the kind of test/flight config drift this manifest exists to prevent |
| `firmware_app/host/oot_host.c` (2026-09-22) | Extends `logGetVarId`/`logGetUint` to resolve `("rpm","m1".."m4")` against the **existing** `g_host_rpm[]` array (already written by `oot_set_rpm()`) instead of always reporting "not present" — controller=9 reads RPM via this log-based path, a genuinely different mechanism from `rpm_get_all()` (what `lib.rs`/`naindi.rs` use); reusing the same array means the existing RPM-injection call in `crazyflie_sil.py` feeds both paths with no new setter. Also adds `paramGetVarId`/`paramGetUint` (report the RPM deck "present" unconditionally — controller=9's `Init()` checks this once) and `oot_omar_mass()`/`oot_omar_kt_equiv()` (return `CF_MASS`/`MOTORRPM2FORCE`-converted-to-this-project's-kt-convention, for the plant sync below). All additive — every other group/name/controller's behavior through these functions is unchanged | Controller 9's Init crashes (undefined `paramGetVarId`) or silently reads zero RPM forever; sim plant airframe mismatches the controller's internal model |
| `crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py`, `crazyflie_server.py` (2026-09-22, `flying_robot_course`-tracked, not a `crazyflie-firmware` local mod) | Adds the `'oot4'` controller option: per-vehicle `controllerOmarIndi_t()` instance (no `select_drone` needed, unlike oot/oot2/oot3 — see `controllerOmarIndi()`'s explicit-self signature), `indi=3` set once at Init, RPM injection reusing the existing `oot_set_rpm()` call. `_setup_oot()` gets a **dedicated** physics-sync branch for `'oot4'` (`oot_omar_mass()`/`oot_omar_kt_equiv()`) rather than falling through to the `g_indi_mass`/`g_indi_kt*`-based default, which this controller never reads — that fallthrough would have built a plant ~4% off the controller's real mass, caught before it shipped. New configs: `crazyflie/config/server_sim_omar_indi.yaml` (`backend: np`) and `server_sim_omar_indi_dw.yaml` (`backend: neuralswarm`, real coupled downwash, mirrors `server_sim_naindi_dw.yaml`). **2026-09-23 fix**: `self.kt`/`self.thrust_max` (needed by `pwm_to_rpm()`/`pwm_to_force()` to invert a commanded PWM back into a physically consistent force) were missing from the `'oot4'` branch entirely — `self.thrust_max` staying `None` crashed the server outright on the first real `takeoff()` call (`TypeError`), and before that was caught, the missing `self.kt` alone had already produced a silent, reproducible ~15-20% steady-state height deficit (plant using a generic wrong thrust curve, not this controller's own model). Both now set from `oot_omar_kt_equiv()`/`oot_thrust_max()`. See `omar_indi_reference_build_notes.md`'s "Bug #2" section for the full diagnostic trail | Controller 9 not selectable in the CS2 SIL at all, or selectable but silently flying a mismatched plant / crashing on takeoff |
| `src/platform/interface/platform_defaults_cf21bl.h` (2026-09-22) | Adds `MOTORRPM2FORCE` (previously entirely absent from this tree) and an unconditional `THRUST2TORQUE` fallback, both copied verbatim from the supervisor's own `platform_defaults_cf21bl.h` — his `controller_omar_indi.c` reads these directly, so using his numbers (not a substitute) keeps the port airframe-correct on our brushless platform, which he had already parameterized for both standard and brushless before we touched anything | Controller 9 won't compile (`MOTORRPM2FORCE` undefined), or compiles but silently uses the wrong (generic/upgraded-platform) torque constant |
| `src/modules/interface/math3d.h`, `src/modules/interface/stabilizer_types.h` (2026-09-22) | Adds `vadd5()` (math3d.h) and `setpoint_t.attitudeAcc` (stabilizer_types.h) — his source assumes a newer Bitcraze upstream commit than this tree is pinned to; both are plain upstream helpers/fields (also present verbatim in NA-INDI-firmware, same signature/name), not customizations of any kind, backported only so his file compiles unmodified. Read/written by nothing except controller=9 | Controller 9 does not compile |

---

## Recovering them

The bindings changes are preserved as a patch in this repository:

```bash
cd ~/Desktop/crazyflie-firmware
git diff --quiet bindings/ && \
  git apply ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/cffirmware_bindings.patch
make bindings_python
```

The `usddeck.c` and `controller_indi.h` changes are **not** in that patch — regenerate it if you
want them covered:

```bash
cd ~/Desktop/crazyflie-firmware
git diff bindings/ > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/cffirmware_bindings.patch
```

The `ControllerTypeOot2` slot (controller.h/controller.c/Kconfig) has its own patch:

```bash
cd ~/Desktop/crazyflie-firmware
git apply ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_controller_slot.patch
```

Regenerate it after editing those three files with:

```bash
cd ~/Desktop/crazyflie-firmware
git diff src/modules/interface/controller/controller.h src/modules/src/controller/controller.c src/modules/src/Kconfig \
  > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_controller_slot.patch
```

The `gyroNoLpf` addition (stabilizer_types.h/sensors_bmi088_bmp3xx.c) has its own patch too:

```bash
cd ~/Desktop/crazyflie-firmware
git apply ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_gyro_no_lpf.patch
```

Regenerate it after editing those two files with:

```bash
cd ~/Desktop/crazyflie-firmware
git diff src/modules/interface/stabilizer_types.h src/hal/src/sensors_bmi088_bmp3xx.c \
  > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_gyro_no_lpf.patch
```

After applying either patch, `bindings/cffirmware.i` must also carry the `controllerOutOfTree2*`
declarations (already in `cffirmware_bindings.patch` above) before `make bindings_python` will
expose `gyroNoLpf`/controller=7 to the host tests — it picks the new struct field up automatically
via its existing `%include "stabilizer_types.h"`, no separate `.i` change needed for the field.

`naindi_controller_slot.patch` and `cffirmware_bindings.patch` now also carry the `ControllerTypeOot3`
/ `CONFIG_CONTROLLER_OOT3` slot (2026-09-16, controller=8, `naindi_hybrid.rs`) — same two commands
above regenerate both after further edits. `naindi_hybrid_weights.rs` and `naindi_hybrid.rs`
themselves live in `firmware_app/src/` (the `flying_robot_course` repo) and need no patch. `app-config`
needs `CONFIG_CONTROLLER_OOT3=y` alongside the existing two flags (already committed there).

### Controller 9 (Omar's INDI, literal C port) — recovery

`naindi_controller_slot.patch` also now carries `ControllerTypeOot4`/`CONFIG_CONTROLLER_OOT4`
(controller.h/controller.c/Kconfig/Kbuild — the same regenerate command above, now including
`src/modules/src/controller/Kbuild`, covers it):

```bash
cd ~/Desktop/crazyflie-firmware
git diff src/modules/interface/controller/controller.h src/modules/src/controller/controller.c \
  src/modules/src/Kconfig src/modules/src/controller/Kbuild \
  > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_controller_slot.patch
```

The math3d.h/stabilizer_types.h backport (`vadd5`, `attitudeAcc`) is folded into
`naindi_gyro_no_lpf.patch` (same regenerate command as the gyroNoLpf row, now including
`src/modules/interface/math3d.h`):

```bash
cd ~/Desktop/crazyflie-firmware
git diff src/modules/interface/stabilizer_types.h src/hal/src/sensors_bmi088_bmp3xx.c \
  src/modules/interface/math3d.h \
  > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/naindi_gyro_no_lpf.patch
```

The `platform_defaults_cf21bl.h` addition has its own patch:

```bash
cd ~/Desktop/crazyflie-firmware
git apply ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/omar_indi_platform_defaults.patch
# regenerate after further edits:
git diff src/platform/interface/platform_defaults_cf21bl.h \
  > ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/omar_indi_platform_defaults.patch
```

`controller_omar_indi.c` and `controller_omar_indi.h` are **new, untracked** files — not covered
by any `git diff`-based patch. Their recovery copies are
`firmware_app/host/controller_omar_indi.{c,h}.snapshot`; restore with:

```bash
cp ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/controller_omar_indi.c.snapshot \
   ~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c
cp ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/host/controller_omar_indi.h.snapshot \
   ~/Desktop/crazyflie-firmware/src/modules/interface/controller/controller_omar_indi.h
```

Regenerate the snapshots after any further edit to either file (should be rare — the whole point
is that these stay a literal copy of his source, renamed identifiers only). `app-config-bl` needs
`CONFIG_CONTROLLER_OOT3=y` + `CONFIG_CONTROLLER_OOT4=y` (already committed there, in the
`flying_robot_course` repo, no patch needed).

Two host-only files complete the picture (already committed to the `flying_robot_course` repo,
not local modifications to `crazyflie-firmware` itself, so no patch needed for them):
- `firmware_app/host/oot_host.c` provides a `usecTimestamp()` stub (wall-clock microseconds) —
  the real one is STM32-only (`usec_time.c`, TIM7) and controller=7's attitude-INDI dt needs it
  even on the host.
- `crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py` sets `sensors.gyroNoLpf` alongside
  the existing `sensors.gyro` (same ground-truth value — the sim applies no LPF to begin with).

## Checking they are still in place

```bash
cd ~/Desktop/crazyflie-firmware && git status --short
```

Expect these files modified (setup.py, cffirmware.i, usddeck.c, controller_indi.h, controller.h,
controller.c, Kconfig, Kbuild, stabilizer_types.h, sensors_bmi088_bmp3xx.c, math3d.h,
platform_defaults_cf21bl.h) plus two new untracked files (controller_omar_indi.c, .h). **If that
list is empty, the modifications have been wiped** — re-apply before building or flying. A quick
functional check:

```bash
grep MAX_USD_LOG_VARIABLES_PER_EVENT src/deck/drivers/src/usddeck.c   # must read 48
python3 -c "import cffirmware as f; print(f.oot_thrust_max())"        # must print 0.2
grep ControllerTypeOot2 src/modules/interface/controller/controller.h # must be present
grep ControllerTypeOot3 src/modules/interface/controller/controller.h # must be present
grep ControllerTypeOot4 src/modules/interface/controller/controller.h # must be present
grep gyroNoLpf src/modules/interface/stabilizer_types.h               # must be present
grep MOTORRPM2FORCE src/platform/interface/platform_defaults_cf21bl.h # must be present
test -f src/modules/src/controller/controller_omar_indi.c             # must exist
python3 -c "import cffirmware as f; print(f.controllerINDI)"          # must not error (2026-09-18)
```

## Bitcraze's stock INDI wired into the SIL — 2026-09-18, a real result

Third reference point for the controller=7 SIL investigation (`docs/07`): a structurally
different INDI (`controller_indi.c`, pure gyro-differentiation, no RPM feedback at all — confirmed
by grep, no `rpm`/`Rpm`/`RPM`/`motorsGetRatio` symbol anywhere in the file) from both this
project's own (`lib.rs`, Tal & Karaman) and the Cobo-Briesewitz port (`naindi.rs`). Wired as a new
`indi` controller option in `crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py` (new config:
`crazyswarm2/crazyflie/config/server_sim_stock_indi.yaml`) — cheap, since it needs no RPM
injection plumbing, unlike `oot`/`oot2`/`oot3`.

**Result: completely clean.** Same single-drone hover through `crazyflies_sim1.yaml`, same SIL,
same trajectory that crashes `oot2` (controller=7) every time — max roll/pitch **exactly 0.0°**
through climb, a full 8s hover, and landing (`crazyswarm2/state_stock_indi/2026-09-17_214312`).
Matches the operator's real-hardware report (flew clean, worse tracking than either INDI above,
no oscillation) using the *same* filter-matched config (the `controller_indi.h` row above) that
was actually flown, not a mismatched comparison.

**What this settles:** the SIL itself — same physics substrate, same trajectory, same EKF/sensor
model — does not inherently produce instability. A clean flight here for a third, independent
controller is strong evidence against a sim-vs-reality gap explaining `naindi.rs`'s crash;
the instability found in `controller=7` looks like a genuine property of that specific
port+gains combination, not an artifact of this simulator.
