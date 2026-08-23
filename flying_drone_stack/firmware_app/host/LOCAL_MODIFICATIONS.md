# Local modifications to `crazyflie-firmware`

**Decision (2026-08-23): these stay exactly as they are. Intentional. Do not remove, rebase or
clean them.** This question is closed — the file exists so it does not come up again.

`~/Desktop/crazyflie-firmware` tracks **bitcraze upstream** and is deliberately not forked
(see the reasoning in the project memory: a fork has to be kept in sync and adds a repository to
reason about, which is not worth it for a small additive change). The consequence is that four
files carry uncommitted local changes. That is the intended steady state, not an untidy loose end.

Each file now carries a `LOCAL MODIFICATION -- INTENTIONAL, DO NOT REVERT` comment in place.
This manifest is the durable copy, because comments in a tree we do not own can be wiped by a
checkout or an upstream pull.

---

## The four modifications

| File | What it changes | Consequence if lost |
|---|---|---|
| `bindings/setup.py` | Links the out-of-tree controller into the SIL build; renames six colliding symbols (`peer_get_all` added 2026-08-23 for the residual network -- the host has no `peer_localization`, so `oot_host.c` injects peers instead); sets `CONFIG_PLATFORM_CF21BL` | **The simulator does not build.** Without the platform define it builds but silently uses the wrong airframe — `THRUST_MAX` 0.1125 N/motor instead of 0.2 — and attitude INDI never leaves the ground |
| `bindings/cffirmware.i` | Exposes `controllerOutOfTree*`, the RPM/log helpers, the airframe constants, the gain globals and the `rnn.*` residual-network globals + peer injection | Simulator cannot select or configure our controller |
| `src/deck/drivers/src/usddeck.c` | `MAX_USD_LOG_VARIABLES_PER_EVENT` 20 → 40 | **⚠️ The most dangerous one to lose.** The thesis logging config records **34** variables including `indi.a_res_*`. At the stock limit of 20 the log is **silently truncated** — no error, no warning, just missing columns. A flight campaign could be lost before anyone noticed |
| `src/modules/interface/controller/controller_indi.h` | Filter cutoff and `g1`/`g2` re-derived for the CF21BL airframe through stock INDI's legacy output path (July 2026 investigation) | The stock-INDI comparison is no longer on equal terms with ours |

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

## Checking they are still in place

```bash
cd ~/Desktop/crazyflie-firmware && git status --short
```

Expect exactly these four files modified. **If that list is empty, the modifications have been
wiped** — re-apply before building or flying. A quick functional check:

```bash
grep MAX_USD_LOG_VARIABLES_PER_EVENT src/deck/drivers/src/usddeck.c   # must read 40
python3 -c "import cffirmware as f; print(f.oot_thrust_max())"        # must print 0.2
```
