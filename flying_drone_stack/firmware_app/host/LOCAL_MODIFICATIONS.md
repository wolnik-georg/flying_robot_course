# Local modifications to `crazyflie-firmware`

**Decision (2026-08-23): these stay exactly as they are. Intentional. Do not remove, rebase or
clean them.** This question is closed — the file exists so it does not come up again.

`~/Desktop/crazyflie-firmware` tracks **bitcraze upstream** and is deliberately not forked
(see the reasoning in the project memory: a fork has to be kept in sync and adds a repository to
reason about, which is not worth it for a small additive change). The consequence is that nine
files carry uncommitted local changes. That is the intended steady state, not an untidy loose end.

Each file now carries a `LOCAL MODIFICATION -- INTENTIONAL, DO NOT REVERT` comment in place.
This manifest is the durable copy, because comments in a tree we do not own can be wiped by a
checkout or an upstream pull.

---

## The seven modifications

| File | What it changes | Consequence if lost |
|---|---|---|
| `bindings/setup.py` | Links the out-of-tree controller into the SIL build; renames six colliding symbols (`peer_get_all` added 2026-08-23 for the residual network -- the host has no `peer_localization`, so `oot_host.c` injects peers instead); sets `CONFIG_PLATFORM_CF21BL` | **The simulator does not build.** Without the platform define it builds but silently uses the wrong airframe — `THRUST_MAX` 0.1125 N/motor instead of 0.2 — and attitude INDI never leaves the ground |
| `bindings/cffirmware.i` | Exposes `controllerOutOfTree*`, the RPM/log helpers, the airframe constants, the gain globals and the `rnn.*` residual-network globals + peer injection | Simulator cannot select or configure our controller |
| `src/deck/drivers/src/usddeck.c` | `MAX_USD_LOG_VARIABLES_PER_EVENT` 20 → 40 | **⚠️ The most dangerous one to lose.** The thesis logging config records **34** variables including `indi.a_res_*`. At the stock limit of 20 the log is **silently truncated** — no error, no warning, just missing columns. A flight campaign could be lost before anyone noticed |
| `src/modules/interface/controller/controller_indi.h` | Filter cutoff and `g1`/`g2` re-derived for the CF21BL airframe through stock INDI's legacy output path (July 2026 investigation) | The stock-INDI comparison is no longer on equal terms with ours |
| `src/modules/interface/controller/controller.h`, `src/modules/src/controller/controller.c`, `src/modules/src/Kconfig` (2026-09-14) | Adds `ControllerTypeOot2` / `CONFIG_CONTROLLER_OOT2` — a **second, independent** out-of-tree controller slot (`stabilizer.controller=7`) alongside the existing `ControllerTypeOot` (`=6`, our geometric/INDI, `ctrl_mode` 0-3). Exists so a byte-faithful Rust port of Cobo-Briesewitz's NA-INDI (`firmware_app/src/naindi.rs`) can fly without any risk of interfering with our own controller — separate enum value, separate dispatch row, separate Rust module, no shared state | Controller 7 does not exist / does not build; falls back silently to whatever `ControllerType_COUNT`-indexed garbage or a build error, depending on how it's lost |
| Same three files (2026-09-16) | Adds `ControllerTypeOot3` / `CONFIG_CONTROLLER_OOT3` — a **third, independent** out-of-tree controller slot (`stabilizer.controller=8`). Ports the SAME reference file as Oot2 (`controller_lee.c`) but with `use_nn` enabled and their real trained network included (`firmware_app/src/naindi_hybrid.rs` + `naindi_hybrid_weights.rs`) — true NA-INDI, not their plain INDI. Own enum value, own dispatch row, own Rust module, no shared state with Oot or Oot2. **Compiles and links; `controllerOutOfTree3Init/Test` pass and the NN forward pass produces sane non-degenerate output on a host smoke test (2026-09-16) — NOT YET numerically verified against the reference's own compiled C. Do not fly.** | Controller 8 does not exist / does not build |
| `src/modules/interface/stabilizer_types.h`, `src/hal/src/sensors_bmi088_bmp3xx.c` (2026-09-14) | Adds `sensorData_t.gyroNoLpf` (the pre-LPF gyro), populated right after `sensorsAlignToAirframe` and before `applyAxis3fLpf` overwrites `sensorData.gyro` in place — ported verbatim from NA-INDI-firmware, which added the same field for the same reason. Operator's explicit instruction (2026-09-14): controller=7 must read exactly the signal their reference does, not a filtered substitute, "no exceptions apart from mass/inertia/kt" | Controller 7's attitude-INDI angular-acceleration term silently falls back to the regular filtered gyro — no build error, just a quiet fidelity loss to the port. `bindings/cffirmware.i` also needs `%include "stabilizer_types.h"` to still see the new field (it already does, no separate change needed there beyond the controller=7 entry points) |

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

Expect exactly these nine files modified (setup.py, cffirmware.i, usddeck.c, controller_indi.h,
controller.h, controller.c, Kconfig, stabilizer_types.h, sensors_bmi088_bmp3xx.c). **If that list
is empty, the modifications have been wiped** — re-apply before building or flying. A quick
functional check:

```bash
grep MAX_USD_LOG_VARIABLES_PER_EVENT src/deck/drivers/src/usddeck.c   # must read 40
python3 -c "import cffirmware as f; print(f.oot_thrust_max())"        # must print 0.2
grep ControllerTypeOot2 src/modules/interface/controller/controller.h # must be present
grep ControllerTypeOot3 src/modules/interface/controller/controller.h # must be present
grep gyroNoLpf src/modules/interface/stabilizer_types.h               # must be present
```
