# Lab runbook — alternative pure-INDI shakedown (controllers 9 → 7 → 8)

**Date:** 2026-09-28  
**Scope:** Desk-prepared execution plan only. **Human operator flies; agent does not.**  
**Study drone:** `cf5` (brushless, `cf21_active`). **Reference partner (2-drone only):** `cf_second` — **must stay `stabilizer.controller: 5` (stock Lee) always.**  
**Technical background:** [docs/41_Pure_INDI_Implementation_Comparison.md](../41_Pure_INDI_Implementation_Comparison.md) (read before/after lab, not mid-session).

**Standing gate (same as every prior first flight of new control-law code):** solo hover first → watch attitude on live telemetry → **land immediately** on any **growing** roll/pitch oscillation. Sim-clean is a precondition, not a substitute for this gate.

**Order this session:** **c=9 (Omar C port)** first, then **c=7 (NA-INDI)**, then **c=8 (NA-INDI+NN)** only if the prior controller’s solo + 2-drone steps were clean **and** you choose to continue same session.

---

## 0. Before you touch hardware (once per session)

### 0.1 Repos and build (lab PC)

```bash
cd ~/Desktop/flying_robot_course && git pull
cd ~/Desktop/crazyswarm2 && git pull
cd ~/Desktop/crazyswarm2 && colcon build --packages-select crazyflie_examples crazyflie_py --symlink-install
source ~/Desktop/crazyswarm2/install/setup.bash
```

### 0.2 Firmware build verified on desk (2026-09-28)

From `flying_drone_stack/firmware_app`, **`make DRONE=bl`** succeeds with all OOT slots enabled in merged `.config`:

- `CONFIG_CONTROLLER_OOT=y` → **controller 6** (`lib.rs`)
- `CONFIG_CONTROLLER_OOT2=y` → **controller 7** (`naindi.rs`)
- `CONFIG_CONTROLLER_OOT3=y` → **controller 8** (`naindi_hybrid.rs`)
- `CONFIG_CONTROLLER_OOT4=y` → **controller 9** (`controller_omar_indi.c`)

Symbols present in `build/cf21bl.elf`: `controllerOutOfTree`, `controllerOutOfTree2/3/4`, etc.

**One reflash** of `cf5` with this brushless binary exposes **6, 7, 8, and 9**; switching among 7/8/9 afterward is **yaml only** (no second flash unless firmware source changed).

### 0.3 What the repo cannot tell you (check live)

| Question | Repo-only answer |
|----------|------------------|
| Which firmware image is **on cf5 right now**? | **Unknown.** No on-drone flash timestamp in git. Latest logged flight meta (`A1_2026-09-28_17-47-50.meta.json`) shows **cf5 @ controller 6** — not proof of image age. **Assume reflash required** before selecting 7/8/9. |
| Mocap healthy? | **Unknown from repo.** 2026-09-26 facility fault blocked flying; confirm tracking before arming today. |
| `cfloader` / radio sees cf5? | **Operator check** on pad. |

### 0.4 Per-robot yaml beyond `controller` (do not “tune” for shakedown)

| Controller | Extra yaml / compile knobs |
|------------|----------------------------|
| **9** | **No** runtime mass/gain overrides. Omar’s defaults are **in his C source**. **`CF_MASS=42700` (42.7 g)** is **compile-time** in `firmware_app/app-config-bl` (`CONFIG_MODIFY_CF_MASS`), not a yaml field. Platform thrust/arm constants are his brushless defaults in firmware. **`indi_gains.ctrl_mode` on cf5 does not apply** (not our OOT6 law). RPM: c=9 reads **`rpm.m*`** log path / deck presence — **`indi_gains.rpm_source` does not switch Omar’s C file** (still leave `rpm_source: 1` on cf5 for consistency / uSD). |
| **7 / 8** | **No** extra yaml beyond **`stabilizer.controller: 7` or `8`**. Keep **`indi_gains.rpm_source: 1`** on cf5 (DShot — required for RPM-based INDI). **`ctrl_mode` unused** by 7/8; leave as-is. Mass/inertia for Rust ports are **in Rust**, not `CF_MASS`. |
| **cf_second** | **Only** `controller: 5`. Never change. Stock firmware on standard CF2.1 — **do not flash cf5’s brushless OOT image onto cf_second**. |

### 0.5 Mandatory **`cf_second` pin check** (before **every** arm)

Open `~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml` and confirm:

```yaml
  cf_second:
    firmware_params:
      stabilizer:
        controller: 5   # MUST stay 5 — stock Lee reference partner
```

After CS2 is up, optional readback (replace name if your ROS graph differs):

```bash
# Expect stabilizer.controller == 5 on cf_second only
ros2 param get /cf_second/params stabilizer.controller
```

**If cf_second is not 5 → do not launch.**

### 0.6 Solo vs 2-drone yaml switches

**Solo shakedown (step 3 for each controller):**

- `cf5`: `enabled: true`
- `cf_second`: **`enabled: false`** (current desk yaml, 2026-09-25 — keeps spare out of mocap volume)

**2-drone A1 (step 5, only if solo clean):**

- Set **`cf_second: enabled: true`** (required — disabled drone in volume breaks mocap association; see yaml comments)
- Re-run **§0.5** pin check
- **Role mapping:** ROS service order puts **`cf5` first → A1 “bottom”**, **`cf_second` → “top”** (matches C.1 merges). Printed `[formation]` plan must show `cf5 [bottom]`, `cf_second [top]`.

### 0.7 Start CS2 (hardware)

Use your usual lab launch (OptiTrack + `crazyflie_server` + mocap). Example skeleton — **use the same command you used for 2026-09-23 C.1** if it differs:

```bash
source ~/Desktop/crazyswarm2/install/setup.bash
ros2 launch crazyflie launch.py backend:=cflib
```

---

## Controller 9 — Omar literal C port (`ControllerTypeOot4`)

**Reminder:** Gains are the **reference author’s untouched defaults**. This is a **shakedown**, not a science flight — it may not work on this airframe.

### 9.1 Reflash cf5 (once per session, before first 7/8/9 flight)

Power on **cf5** on the pad; radio on channel **80**, URI **`radio://0/80/2M/E7E7E7BB02`**.

```bash
cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app
make DRONE=bl cload CLOAD_ARGS='-w radio://0/80/2M/E7E7E7BB02'
```

If `cload` fails, explicit flash of the brushless artifact:

```bash
cfloader flash ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build/cf21bl.bin stm32-fw \
  -w radio://0/80/2M/E7E7E7BB02
```

Power-cycle cf5 after flash; reconnect CS2.

### 9.2 Edit `crazyflies.yaml` — cf5 only

File: `~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml`

**Change (cf5 block, ~line 180):**

```yaml
        controller: 6   # 2026-09-21: C.1 / default study config -- our OOT geometric+INDI
```

**→**

```yaml
        controller: 9   # 2026-09-28: Omar INDI shakedown — restore 6 after session
```

**In the same review, confirm unchanged:**

```yaml
  cf_second:
    ...
    firmware_params:
      stabilizer:
        controller: 5   # permanent stock-Lee pin
```

Do **not** change shared `all:` `stabilizer.controller`. Leave `indi_gains.ctrl_mode: 0` on cf5 unless you know you need otherwise (ignored by c=9).

Restart or reload CS2 so yaml params push to cf5.

### 9.3 Solo hover (cf5 only)

**CLI note:** `simple_flight.py` has **`--trajectory hover`**, not `--mode`. It has **no `--drone` flag** — with only **cf5 enabled**, the sole enabled robot is flown.

```bash
source ~/Desktop/crazyswarm2/install/setup.bash
ros2 run crazyflie_examples simple_flight -- \
  --trajectory hover \
  --pin-controller \
  --height 1.0 \
  --duration 8
```

**Why `--pin-controller`:** For `controller != 6`, takeoff/trajectory/land stay on **one** controller config (no mid-air switch to ramp controller 6 / geometric — 2026-09-09 lesson).

**Console checks before/at takeoff:**

- `[simple_flight] crazyflies.yaml (trajectory): stabilizer.controller=9 ...`
- `[simple_flight] --pin-controller: one config from takeoff to landing...`
- Ramp line should show **controller=9** throughout (not 6).

### 9.4 Watch for / abort if (solo, live radio log / RViz)

Use the same attitude stream you used for prior C.0 gates (`/cf5/...` log topics or RViz).

| Signal | Abort if |
|--------|----------|
| **Roll / pitch magnitude** | After initial climb settles (~3–5 s into hover), **|roll| or |pitch| grows flight-over-flight** (e.g. under 5° → 10° → 15°+) instead of staying bounded like prior geometric hovers (~few °). |
| **Oscillation character** | Clear **increasing buzz/wobble** in roll/pitch traces — not the small steady limit-cycle you already tolerate on c=6, but **clearly worsening** over 1–2 s. |
| **Height** | Runaway climb or drop vs ~1 m target (setpoint diverges while motors saturate). |
| **RPM (cf5)** | **All four** motor RPM streams **near zero or frozen** while armed and flying (c=9 needs RPM via deck/log path). |
| **Mocap** | Pose freeze, jump, or geofence breach. |

**Abort action:** `ros2 run crazyflie_examples` emergency if needed; else let script land; **do not** proceed to 2-drone; **do not** advance to c=7/8 this session.

### 9.5 Two-drone A1 (only if §9.3 clean)

1. Set **`cf_second: enabled: true`** in yaml; restart CS2.
2. **§0.5** again — **`cf_second` controller must be 5**.
3. Sim matrix for alt INDI used **A1, dz=0.5 m**; lab C.1 often uses **dz=0.30 m**. Match sim-style **short hold** with **`--hold 10`**:

```bash
ros2 run crazyflie_examples run_formation -- \
  --auto-center --yes \
  --scenario A1 --dz 0.30 --hold 10
```

Before arming, read printed roster:

- `cf5` → **`bottom`**, controller **9** (from yaml)
- `cf_second` → **`top`**, controller **5**

Same **growing oscillation** abort criteria on **cf5** (study drone). If **cf_second** misbehaves but cf5 is fine, land and diagnose — do not assume “partner issue” is safe to ignore for a shakedown.

---

## Controller 7 — NA-INDI faithful port (`naindi.rs`)

**Reminder:** Gains are the **reference author’s untouched defaults**. Shakedown only — may not work.

**Skip entire section** unless controller **9** solo (+ optional 2-drone) was clean **and** you continue.

### 7.1 Reflash

**None** if §9.1 already done this session with current `firmware_app` build.

### 7.2 Edit `crazyflies.yaml` — cf5 only

```yaml
        controller: 9   # (or 6 if restoring from earlier)
```

**→**

```yaml
        controller: 7
```

**Confirm again:** `cf_second` → `controller: 5` unchanged.

### 7.3 Solo hover

```bash
ros2 run crazyflie_examples simple_flight -- \
  --trajectory hover \
  --pin-controller \
  --height 1.0 \
  --duration 8
```

Expect **`stabilizer.controller=7`** in script banner.

### 7.4 Watch for / abort if

Same as **§9.4**, plus for c=7:

- **`indi.a_res_*`** on cf5 should **not** read identically **zero** throughout hover if RPM is healthy (INDI uses RPM force model). All-zero **`a_res`** with live RPM → treat as **measurement failure**, not “no disturbance.”

### 7.5 Two-drone A1 (if solo clean)

Enable cf_second (§0.6), **§0.5**, then:

```bash
ros2 run crazyflie_examples run_formation -- \
  --auto-center --yes \
  --scenario A1 --dz 0.30 --hold 10
```

---

## Controller 8 — NA-INDI + trained NN (`naindi_hybrid.rs`)

**Reminder:** Gains and **network weights** are the **reference defaults** baked into firmware — not retrained for this lab. Shakedown only.

**Skip** unless **7** cleared solo (+ optional 2-drone) and you continue.

### 8.1 Reflash

**None** if §9.1 done.

### 8.2 Edit `crazyflies.yaml` — cf5 only

```yaml
        controller: 7
```

**→**

```yaml
        controller: 8
```

**Confirm:** `cf_second` → `controller: 5`.

### 8.3 Solo hover

```bash
ros2 run crazyflie_examples simple_flight -- \
  --trajectory hover \
  --pin-controller \
  --height 1.0 \
  --duration 8
```

### 8.4 Watch for / abort if

Same as **§7.4** (including **`a_res`** / RPM sanity).

### 8.5 Two-drone A1 (if solo clean)

```bash
ros2 run crazyflie_examples run_formation -- \
  --auto-center --yes \
  --scenario A1 --dz 0.30 --hold 10
```

---

## Close-out — A1 failure @ cf5 / controller=9 (2026-09-28 evening)

**Flight:** `A1_2026-09-28_19-18-32` — cf5 `controller=9`, cf_second `controller=5`. Operator: **cf5 motors never spun**; cf_second flew normally. Radio CSV: `experiments/logs/A1_cf5_2026-09-28_19-18-32.csv` is **all-zero** state/attitude/thrust (only `vbat` live); `A1_cf_second_…` shows normal climb. Sidecar: `A1_2026-09-28_19-18-32.meta.json` (`gains_apply=0` on cf5).

### Root cause (traced in code + logs, not the modeAbs gate)

- **Ruled out:** Omar `controller_omar_indi.c` only producing thrust under `modeAbs` — real mechanism, but **not** this incident if cf5 had received the same active HLC setpoints as cf_second (cf_second @ Lee on the identical `run_formation.py` path flew).
- **Ruled out:** firmware/supervisor allow-list on `stabilizer.controller=9` — no such check in `supervisor.c` / server; solo hover @ c=9 reportedly clean same session.
- **Found:** `run_formation.py` `apply('takeoff', _RAMP_CONTROLLER=6, …)` still pushed the full shared **`all:` `indi_gains.*` / `pos_gains.*` block** to any drone without a **per-key** override. cf5’s yaml pins only `stabilizer.controller`, `indi_gains.ctrl_mode`, `rnn.en` — **not** the ~15 INDI gain keys — so cf5 @ **controller 9** (where those params are **inert**, `gains_apply=0`) was hit with a large Param burst **immediately before `arm()` / per-drone `takeoff()`**, on the **same single Crazyradio** already carrying two drones’ custom log topics (`crazyflies.yaml` bandwidth note). That matches **cf5 stuck on HLC `nullSetpoint` / motors off** while cf_second’s planner ran — upstream **formation script + radio budget**, not the Omar control law.
- **Contrast:** `simple_flight.py --pin-controller` pins the ramp to **9** for solo and does not have this “pretend ramp=6 but vehicle is 9, still push OOT gains” pattern.

### Fix (crazyswarm2)

`crazyflie_examples/run_formation.py`:

1. **`_gains_apply_to_drone()`** — skip `indi_gains` / `pos_gains` writes in `apply()` when `resolve()` says that drone’s effective controller **≠ 6** (same rule as log meta `gains_apply`).
2. **Pre-arm abort** if any `DroneLogger` already sees short/empty `/state` or all-zero position (would have flagged cf5 before arming on a re-run).
3. **150 ms spacing** between per-drone `takeoff()` / `goTo()` calls (HL services are `call_async` without ack).

### Re-verification (desk / SIL only — no hardware flight by agent)

- `experiments/analysis/test_run_formation_gain_skip.py` — confirms cf5@9 / cf_second@5 skip gain push; shared OOT6 drone still receives gains.
- Operator should re-run **2-drone A1** after `colcon build --symlink-install` on CS2; expect cf5 `/state` healthy pre-arm, fewer Param writes before takeoff, and nonzero thrust if HL activates.

### Controllers 7 / 8 — same failure class?

**At equal upstream risk** for **2-drone `run_formation.py`** with cf5 pinned to **7** or **8**: they do **not** use Omar’s `modeAbs` gate (`naindi.rs` / `naindi_hybrid.rs` assume HLC absolute position), but they **would** have received the same useless OOT gain flood pre-takeoff before this fix. **No separate control-law patch** — the `apply()` gain skip covers **7/8/9** and stock Lee alike whenever effective controller ≠ 6. Remaining operational requirement: **2-drone radio headroom** (yaml 20 Hz logging, second dongle if needed) is unchanged.

---

## Verification pass — param counts + SIL (2026-09-28 night)

### Task 1 — What was measured

**Param volume (`apply('takeoff', 6, 0, …)`), installed `crazyflies.yaml`:**

| Drone | eff. controller | BEFORE fix (`f25470a^`) | AFTER fix (`f25470a`) |
|-------|-----------------|-------------------------|------------------------|
| cf5 | 9 | **21** gain `setParam` writes | **0** |
| cf_second | 5 | **21** | **0** |

Tool: `experiments/analysis/measure_run_formation_takeoff_params.py` (mirrors `resolve()` + gain loops; 17 shared `indi_gains` + 4 `pos_gains` keys, minus none on cf5/cf_second for those keys).

**SIL 2-drone A1** (`run_formation.py`, `crazyflies.yaml` with cf5 + cf_second enabled — same roster as hardware A1):

| cf5 yaml pin | Server profile | Meta sidecar | `verify_formation_sim.py` | cf5 z (record_states) |
|--------------|----------------|--------------|---------------------------|------------------------|
| 9 | `server_sim_omar_indi.yaml` (oot4) | `A1_2026-09-28_20-43-19.meta.json` | **PASS** (dz RMSE 0.1 mm) | 0 → **1.00 m** |
| 7 | `server_sim_naindi.yaml` (oot2) | `A1_2026-09-28_20-46-01.meta.json` | **PASS** (RMSE 0.3 mm) | 0 → **1.00 m** |
| 8 | `server_sim_naindi_hybrid.yaml` (oot3) | `A1_2026-09-28_20-48-19.meta.json` | **PASS** (RMSE 0.4 mm) | 0 → **1.00 m** |

Harness: `experiments/analysis/run_formation_alt_controller_sil.sh {oot4|oot2|oot3}`; logs under `experiments/sim_validation/run_formation_sil_*`.

### Task 1 — Honest limits (do not overstate)

| Claim | Status |
|-------|--------|
| Gain-skip logic drops inert OOT param traffic to **0** for cf5 @ 9/7/8 and cf_second @ 5 | **Confirmed** (bench script on real yaml) |
| Fix does not break 2-drone A1 in SIL; both drones climb and hold commanded stack geometry | **Confirmed** (three SIL runs above) |
| Original hardware failure was **caused by** param burst **starving HL CRTP** on one Crazyradio | **Still inferred only** — no trace of `stabilizer.mode_*` / onboard HLC state from the failed flight, and **SIL does not model radio contention or packet loss** |
| Pre-arm `/state` abort would have blocked the 2026-09-28 flight | **Unknown** — that flight’s all-zero CSV may be logger-only; uSD was not in repo |

**Hardware still required:** one 2-drone A1 re-fly after `colcon build` with **`f25470a`** to confirm cf5 motors spin and (ideally) uSD or TOC log shows non-idle HLC setpoint modes during takeoff.

### Task 2 — Controllers 7 / 8 (rigor, not assertion)

**Yaml shape (installed config):** cf5 pins `stabilizer.controller`, `indi_gains.ctrl_mode`, `indi_gains.rpm_source`, `rnn.en` only — **no** per-key `indi_gains.kr` / `kt*` / etc. (same as c=9). A pin to **7** or **8** would be the same override pattern; bench script reports **21 → 0** writes for hypothetical cf5@7 and cf5@8.

**Generalization of `_gains_apply_to_drone()`:** predicate is **`eff_ctrl == 6`**, not “controller == 9” — **7/8/9/Lee** all skip when ≠ 6. **SIL-confirmed** for **7** and **8** via separate A1 runs (meta `per_drone.cf5.controller` 7 and 8, verify **PASS**, cf5 reached **~1 m**).

**SIL caveat:** both vehicles run the **same** sim controller profile (`oot2`/`oot3`/`oot4` from server yaml); yaml’s cf_second@5 pin does not switch sim physics to stock Lee. This run validates **`run_formation.py` + gain-skip + HL sequence**, not mixed-firmware physics fidelity.

### Edge cases noted

- **cf_second @ 5** also went from 21 → **0** gain writes after the fix (correct: Lee ignores OOT gains; also reduces radio noise).
- **Pre-arm logger gate:** drones at exactly `[0,0,0]` with valid `/state` could false-abort (`‖pos‖ < 1e-4`); lab pads usually offset from origin.
- **SIL param warnings:** `indi_gains.ctrl_mode` / `rpm_source` `setParam` KeyError on oot4/oot2/oot3 sim (expected — those params are not in the sim TOC); unrelated to gain-skip.

---

## After session — restore C.1 / default study config

Regardless of outcome, before C.1 or geometric collection:

1. **`cf5` → `stabilizer.controller: 6`**, **`indi_gains.ctrl_mode: 0`**
2. **`cf_second` → `controller: 5`** (unchanged)
3. Set **`cf_second: enabled: true`** if you need 2-drone C.1; **`false`** only for solo debug
4. Commit yaml changes **only if** you intentionally want lab state saved (default: revert locally)

Log in a new `docs/lab_sessions/YYYY-MM-DD.md`: controllers tried, flash time, yaml snippet, abort/continue decisions, **`stabilizer.controller` readback** for both drones.

---

## Quick reference — file paths

| Item | Path |
|------|------|
| Robot yaml | `~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml` |
| Brushless app config | `~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/app-config-bl` |
| Flash binary | `~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build/cf21bl.bin` |
| cf5 URI | `radio://0/80/2M/E7E7E7BB02` |
| cf_second URI | `radio://0/80/2M/E7E7E7E7E9` |
| Doc 41 | `~/Desktop/flying_robot_course/docs/41_Pure_INDI_Implementation_Comparison.md` |

---

## Operator pushback (2026-09-28 night) — root cause weaker than stated, open for tomorrow

Two problems raised with the "gain burst starves HL radio" story, both valid, neither yet resolved:

1. **`cf_second` (controller=5, stock Lee) has received the identical ~21-write inert gain burst on every 2-drone flight this whole project** — including dozens of prior successful C.1 flights — and has never failed to take off. If the burst alone were sufficient to starve HL commander traffic and prevent takeoff, it should have broken those flights too. It never did. So "any non-controller-6 drone hits this bug" is too broad a claim; something **specific to `cf5`/controller=9 being flown for the first time** is more likely at least part of the real story, not the gain burst in isolation.
2. **Why didn't the original (weeks-ago) SIL validation catch this?** Resolved, not a contradiction: `docs/41`'s controller=9 SIL testing used a direct SIL driver/backend config, never `run_formation.py` itself — it validated the *controller's flight dynamics* (hover, trajectory tracking, downwash), not the *launch script's* takeoff/arm/param-push sequence. That code path had never been SIL-tested with any alternative controller before last night. Two different parts of the system, two different tests — the earlier "sim-clean" result still stands and isn't contradicted.

**Status: the fix (skip inert gains for non-controller-6 drones) is real, correct, and SIL-verified — but not proven to be *the* explanation for `cf5`'s specific failure**, given point 1. Treat as "a good fix for a real launch-script bug" rather than "solved." **Tomorrow's hardware re-fly of `cf5`/controller=9 is the actual test** — if it works, that's real evidence for the fix; if it still fails, the cause is something else specific to controller=9 and this investigation restarts from there.

## Plan — night of 2026-09-28 / next lab session

- **Tonight (desk, not lab):** all C.1-planned scenarios are now collected (21+23+28 Sep) — retrain Neural-Swarm2 on the complete dataset, re-run LOO/eval, check the new A4/A1/A2 folds particularly, since those are new data.
- **Tomorrow (lab):** re-fly `cf5`/controller=9 solo + 2-drone A1 first (the actual open test from tonight); if clean, continue the alt-INDI ladder (7, then 8) using the same runbook. If `cf5` still fails, stop and re-open the root-cause investigation — don't assume the fix worked without seeing it fly.

---

## 2026-09-29 — cf5 connect assert (`uart_syslink.c:549`, queue overflow)

**Symptom (lab, reproducible):** `cf5` with `stabilizer.controller: 9` asserts during **connection**, before `run_formation.py` or takeoff:

`SYS: Assert failed at .../uart_syslink.c:549` (`ASSERT(0); // Queue overflow`).

**Not the same bug as 2026-09-28 `run_formation.py` gain flood** — that fix (`f25470a`) only paces/skips gains on `apply('takeoff', …)` **after** connect. This failure is in the **routine connect-time `firmware_params` sync**.

### Mechanism (desk, confirmed with numbers)

| Item | cf5 | cf_second (reconstructed) |
|------|-----|---------------------------|
| **Yaml `firmware_params` pushed on connect** | **44** | **42** |
| **Measured burst (2026-09-29 `debug/lab_logs/debug.log`)** | **44 writes in ~0.00033 s** | *No comparable log in repo* — same `crazyflie_server.cpp` loop would burst similarly |
| **cf5-only yaml keys** | `indi_gains.rpm_source`, `rnn.en` | — |

**`ctrlOmarIndi` (~22 params on brushless firmware) is not pushed on connect** — not in yaml; the “extra Omar param group” hypothesis for **host write count** is **refuted**. TOC on cf5 is **423 entries** (brushless OOT firmware); cf_second uses stock Lee firmware (smaller TOC, no OOT4).

**Firmware queue:** `STATIC_MEM_QUEUE_ALLOC(syslinkPacketDelivery, 8, …)` in `uart_syslink.c`. On checksum OK, if the queue is full → assert (line ~549). **`uartslkEnableIncoming()` is set `true` in `system.c` before `deckInit()`** — so the “consumer not ready” flag is **not** the gating issue; the problem is **depth + drain rate** while the STM32 is still busy (deck/IMU init, tests) and the nRF51 forwards a **near-simultaneous** CRTP/param storm from the host.

**Why cf5 and not cf_second (working hypothesis, not fully A/B logged):** same unpaced server code, but cf5 is **brushless + dual deck (uSD + RPM) + heavier boot**, and receives **two extra yaml writes** plus post-connect **extra log blocks** (e.g. DShot `rpm` topic). Any of those can widen the boot window where 8 syslink RX slots are insufficient — not “44 vs 42 params alone.”

Tooling: `experiments/analysis/analyze_connect_param_burst.py`.

### Fix applied (host / vendored crazyswarm2 — **not firmware**)

**Layer:** pace **connect-time** parameter application in **`crazyswarm2/crazyflie/src/crazyflie_server.cpp`** (upstream ROS driver — **flagged as vendored patch**, same repo family as `f25470a`).

After building `set_param_map`, **`change_parameter` is called once per entry with `150 ms` sleep between writes** (same spacing as last night’s takeoff gain pacing). ~44 × 150 ms ≈ **6.5 s extra connect time per robot** — acceptable vs boot assert.

**Not changed:** `controller_omar_indi.c`, `uart_syslink.c` queue depth (8 left as-is).

**Desk verify:** `colcon build --packages-select crazyflie` clean; `experiments/analysis/test_connect_param_pacing.py` checks the pacing block exists.

### Next step (hardware — operator only)

1. Pull/build **crazyswarm2** with this commit on the lab PC; `colcon build --packages-select crazyflie`.
2. `ros2 launch …` with **cf5 @ controller 9** — confirm **no** `uart_syslink.c:549` assert and normal connect.
3. If connect is clean → proceed with **solo + 2-drone A1** (still the real test of `f25470a` + controller=9).
4. If assert persists → stop; consider firmware queue bump (document in `LOCAL_MODIFICATIONS.md`) or longer pacing — do not fly.

### Deploy verification marker (2026-09-29 desk)

**Problem:** With `f7856e2` on disk, **flightcontrol1** logs still showed **44 `setParam` writes in ~0.3 ms** — the paced binary was **not** what `ros2 launch` was running (wrong workspace install / stale `colcon` output). There was **no log line** to prove which `crazyflie_server` binary was live.

**Fix (crazyswarm2, after `f7856e2`):** grep the launch log for:

1. **Once at node startup:**  
   `CS2_CONNECT_PARAM_PACE_V1 connect-param pacing: ENABLED (150ms per firmware_params write on connect)`
2. **Per robot at connect:**  
   `[cf5] connect-param pacing: ENABLED (150ms) applying N firmware_params from yaml`  
   then after ~**(N−1)×150 ms**:  
   `[cf5] connect-param pacing: finished N writes in X.XX s`

If those lines are **missing** but `Update parameter` lines still stack in **<1 ms**, the old unpaced binary is still running — rebuild **`crazyflie`** in the workspace you **`source install/setup.bash`** from, then relaunch.

**2026-09-29 `debug/lab_logs/debug.log` (flightcontrol1):**

| Check | Finding |
|-------|---------|
| Pacing marker | **Absent** — confirms **unpaced** server on lab PC |
| cf5 param burst | **44 writes in ~0.32 ms** (same as pre-fix) |
| cf_second | **Zero lines** — no connect attempt visible in this file |
| End of file | **Truncated** at line 110 mid–second-boot `DECK_DISCOVERY: deckctrl` (blank tail) — **not** enough to call a firmware hang vs incomplete terminal capture |
| Assert | **No** `uart_syslink.c:549` in this capture; after unpaced burst, **firmware reboot** console resumes (~223 ms later) then log **cuts off** |

**Next lab capture:** save **full** `ros2 launch` stdout until connect completes or fails; confirm **`CS2_CONNECT_PARAM_PACE_V1`** before interpreting boot SYS lines. If cf_second is enabled in yaml, expect **`[cf_second] Requesting parameters...`** before or after cf5 depending on connect order — its absence here is **unexplained** (disabled yaml, blocked on cf5 connect, or truncated log).

---

## 2026-09-29 — actual root cause found, fixed, and controller=9 flown for real

**The connect assert was never a pacing/syslink problem.** Both the takeoff-gain-flood theory
and the syslink-queue theory (above) were plausible but wrong — neither was ever confirmed with
direct evidence, and both missed something that had been printed in every single boot log from
the start: `[ERROR] [cf5] Could not find param deck/bcRpm`.

**Root cause:** `controllerOmarIndiInit()` reads `paramGetUint(paramGetVarId("deck","bcRpm"))`.
His reference firmware registers that param (`NA-INDI-firmware/.../rpm.c`); ours never did. The
invalid `varid` tripped `ASSERT(PARAM_VARID_IS_VALID(varid))` at `param_logic.c:524` the instant
`stabilizer.controller` was set to 9 — assert, reboot, repeat. Explains everything: only `cf5`
(only drone on controller=9), reflashing the *same* image never helped (bug was in the firmware
source itself, unaffected by reflashing), and why solo hover had looked "clean" the first night —
those runs had actually been controller=6 (confirmed from flight meta), so this code path never
ran.

**Fix:** backported the missing `PARAM_GROUP(deck) { bcRpm }` registration verbatim from the
reference `rpm.c` (`crazyflie-firmware` commit local to this project, documented in
`flying_drone_stack/firmware_app/host/LOCAL_MODIFICATIONS.md`). Read-only presence flag, no
effect on any other controller. Reflashed `cf5` — **connected clean, no assert, no reboot loop.**

**Second finding, same evening:** connecting didn't mean his INDI was active. `controller_omar_indi.c`
defaults `.indi = 0` and gates every INDI term behind it; the CS2 SIL has always set `indi=3` at
init, so hardware and sim had never tested the same code path. Added `ctrlOmarIndi.indi: 3` to
`cf5`'s yaml override (no reflash needed — runtime param).

**Flown, same session, A1 dz=0.30, 2 reps each:**

| config | realized separation | error vs 0.30m commanded |
|---|---|---|
| Our geometric | 0.47 m | 0.17 m |
| Omar `indi=0` | 0.89 m | 0.59 m |
| **Omar `indi=3`** | **0.30 m** | **0.04 m** |

Omar's INDI beats our geometric ~4-5× on this metric, same day/conditions, no confound. First
real, working hardware result for this controller. Full writeup:
`docs/41_Pure_INDI_Implementation_Comparison.md` §9. Data: `experiments/logs/omar_indi_2026-09-29_merged/`.

**Loose end, not yet closed:** the two earlier "fixes" (`run_formation.py` gain-skip pacing,
`crazyflie_server.cpp` connect-time pacing) are still in place and add real latency (~6.5s to
every connect) for problems that turned out not to exist. Candidates to revert once there's time
to confirm nothing now depends on them.

**Not yet tested:** controller=7/8 (`naindi.rs`/`naindi_hybrid.rs`) — worth checking for the same
class of missing-param dependency before their first hardware attempt, rather than rediscovering
it the same way.

## 2026-09-29 — controller=7 first hardware attempt: failed, no logs, cause unknown

Checked first: `naindi.rs` does **not** share controller=9's missing-param bug class — no
`paramGetVarId`/`paramGetUint` calls anywhere; its RPM read (`rpm_get_all()`) already validates
the log-id and falls back to 0 instead of asserting. Cleared to fly on that basis.

**Two consecutive solo-hover attempts, both failed the same way:** "one motor spin fast and then
it flips on the ground." **No radio or uSD logs were captured for either attempt** — a desk-side
scan (`experiments/analysis/analyze_controller7_sep29.py`) confirmed 0 of 6 same-evening `cf5`
radio CSVs tag `controller=7` (all are `=9` or `=6`). **Root cause could not be investigated and
remains completely unknown.** controller=7/8 work was paused here by operator decision — not
abandoned, but deprioritized in favor of the option below.

## 2026-09-29 — decision: build controller=10 instead of continuing to debug controller=7

Operator's reasoning: controller=9 (Omar's own INDI) is now proven working on real hardware.
`naindi.rs` (controller=7) is a Rust port of a **different** reference (Cobo-Briesewitz's
NA-INDI) — debugging it further doesn't get any closer to a Rust-native version of the algorithm
that's actually known to work. Faster path: build a new, faithful Rust port of
`controller_omar_indi.c` itself — a new slot, `controller=10`.

**Built same evening** (`ControllerTypeOot5`, `firmware_app/src/omar_indi_rust.rs`): numerically
verified 7/7 against the C reference (worst delta 7.15e-07), SIL single-drone clean. RPM
availability deliberately uses this project's own safe `rpm_get_all()`/`oot_rpm_logs_available()`
pattern instead of Omar's fragile `paramGetVarId` probe — the one intentional, documented
deviation. Full build notes: `docs/41` §12.

**Pre-flight gap found and fixed before ever reflashing:** the initial build had **no**
yaml-settable `indi` bitmask param at all — `Init()` unconditionally zeroed it every
controller-select, which would have silently repeated controller=9's own two-day "shipped with
indi=0, flew plain geometric" bug. Added `PARAM_GROUP(ctrlOot5) { indi }` (default `3`, not `0`)
before this ever reached hardware. Verified: host 7/7 unaffected, real `make DRONE=bl` build
clean.

## 2026-09-29 — controller=10 first hardware attempt: motors spun, bottom drone never lifted off

Flown **directly to A1** (2-drone), skipping the usual solo-first step — explicit operator
decision, with the real difference from controller=9's own precedent flagged at the time:
controller=9 had a confirmed working basic flight before its first A1; controller=10 had zero
real hardware time at all before this attempt.

**Result:** `cf_second` (top, controller=5) flew normally. `cf5` (bottom, controller=10) — motors
visibly spun, drone never left the ground.

**uSD pulled and analyzed** (both cards, both A1 attempts, `experiments/logs/usd_raw/A1_controller10_2026-09-29_{19-02-37,19-04-22}_merged.csv`):
- `cf5.z` stayed flat at −0.024 m the entire ~10s flight (normal small EKF noise, estimator not
  stuck — ruled that theory out directly).
- **Real finding:** `motor_m1..4` (PWM) show a **single-motor-dominant pattern** in both
  attempts — one motor (m1 in flight 1, m4 in flight 2) ramps from idle to ~30-50k PWM while the
  *other three sit flat at idle* the whole time. Peak 4-motor thrust computed from RPM (1.37 N)
  was well above what's needed to lift the 42.7 g airframe — **not** an underpowered-thrust
  problem, a torque/allocation problem.
- `tau_x/y/z`/`a_res_*` read dead-zero throughout — confirmed this is a **telemetry gap, not real
  data** (the same columns carry real non-zero values in a known-good controller=6 flight from
  Sep 15). Neither `controller_omar_indi.c` nor `omar_indi_rust.rs` had ever called this
  project's `indi_tau_write`/`indi_a_res_write`/`indi_e_r_write` bridge. **Fixed** (additive only,
  no control-law change) so the *next* attempt has real torque/residual telemetry — this one
  didn't.

**Bug found and fixed, real but not confirmed as the cause:** the controller's lazy-init branch
(`controllerOutOfTree5()`) took a live `&mut` reference into its static state, then called
`Init()` — which mutates the same static through a separate path — while that reference was
still held. Real aliasing UB (the compiler's own warning flagged exactly this), and the *only*
code path that had ever exercised it was real hardware's first tick — the host numerical test
always pre-initializes manually, so this branch had literally never run in any test. Fixed by
reordering (check + Init fully before any `&mut` into the static is created). **Honesty check:**
reverted the fix and re-ran a cold-start repro on the host build — both versions produced
identical, correct, non-zero output. Could not reproduce tonight's failure either way, so this
fix is real and worth keeping, but **not confirmed** as what actually happened on hardware.

**Full code audit against `controller_omar_indi.c`, line by line:** position loop, `R_des`
construction, `eR`, the differential-flatness `omega_des`/`omega_des_dot` block, INDI residual
terms, torque assembly — **no further discrepancy found**, consistent with the 7/7 numerical
match. **One real structural risk identified**, shared with the C reference itself (not a
Rust-port bug): `omega_des`/`omega_des_dot` divide by `thrust_si/mass` and `|yc × zb|` with no
floor — near-zero thrust (exactly the ground/takeoff-ramp phase this flight was stuck in) with
any nonzero commanded jerk can blow one axis up into a huge single-motor torque command,
structurally matching the observed symptom. **Not confirmed** as the cause — the identical code
is unguarded in the C reference and flies fine on controller=9. **Guarded anyway**
(`MIN_B1`/`MIN_C3` floors, commit `f4e9256e`) — a deliberate, documented deviation from strict
fidelity to Omar's source, since the risk is real regardless of whether it's this flight's actual
cause. 7/7 numerical unaffected, real firmware build clean.

**Net result: root cause still open.** Two real, worthwhile fixes are in (the UB, the guard) plus
the telemetry gap closed — but none of this is proven to be what actually happened. The only way
to know is another flight with the new telemetry in place.

## 2026-09-29 — Z-only position integral: built, tested, decided against (for now)

Separate track, `docs/51_Z_Only_Integral.md`. Built a new, separate Z-only integral for the
geometric controller (distinct from the joint XY+Z integral `docs/50` already closed), with
conditional-integration anti-windup. SIL A/B showed materially more correction than the old
joint integral at the same disturbance sweep (~63 mm vs ~0.6 mm at −200 mN) — looked like a real
win.

**Gain sweep (`ki_z` 8→64) corrected that framing.** The "mean error" table looked like a clean
win as gain increased, but the transient trace revealed why: the **peak dip at disturbance
onset is unchanged by gain** (~127 mm at −200 mN regardless of `ki_z`) — only *recovery speed*
changes, and even the fastest tested gain takes ~9 s to recover, longer than a real A1 hold
(~5–10 s). **Verdict: do not enable.** Gain-tuning is a recovery-speed knob, not a
disturbance-rejection knob, and doesn't address the thing that actually matters (the initial
sag). `ENABLE_Z_INTEGRAL` stays `false`, default gains unchanged.

**Validation note:** two real problems were found and fixed in the desk-work Cursor produced this
session — a stray `Co-authored-by` commit trailer (removed, this project never adds one) and a
missing `+` prefix in `cffirmware_bindings.patch` that would have broken re-applying that patch
against a fresh `crazyflie-firmware` checkout (fixed). A doc inaccuracy was also corrected —
`docs/51` originally claimed the Z-integral was geometric-only; it was in fact wired into both
the geometric and INDI control paths (left in place, corrected in the doc, since it's inert while
the flag is off).

---

## Close-out (2026-09-29, end of session)

**Yaml state left for next lab session:** `cf5` = `controller: 10`, `ctrlOot5.indi: 3` explicit.
`cf_second` = **disabled** (solo-first discipline, reset after the direct-to-A1 attempt failed).

**Next lab session — controller=10:**
1. `git pull` both repos (commits through `de3f673c` / `99e6b33`) — brings the UB fix, the
   `omega_des`/`omega_des_dot` guard, and the new `tau`/`a_res`/`e_r` telemetry.
2. `cd flying_drone_stack/firmware_app && make cload` — reflash `cf5`.
3. Confirm clean connect, spot-check `ctrlOot5.indi` reads `3` before arming.
4. **Solo hover only** — `cf_second` is already left disabled. Watch closely; this is still
   unflown-clean code, same discipline as any first attempt.
5. If it fails again: pull uSD immediately, run `experiments/analysis/check_omar_rust_telemetry.py`
   on the merged CSV — this time it should show real `tau`/`a_res` values, which is the actual
   missing piece from tonight's analysis.
6. If it flies clean: re-enable `cf_second`, move to A1, then resume the broader comparison plan
   (dz=0.20/0.50, other scenarios) that was paused before this whole shakedown started.

**Not lab-blocked, but currently deprioritized, not forgotten:**
- controller=7/8 — paused, no root cause found, no logs captured. Revisit only if controller=10
  doesn't pan out, or time allows a properly-logged retry.
- The two "loose end" pacing fixes from the wrong-theory chase (`run_formation.py` gain-skip,
  `crazyflie_server.cpp` connect pacing) — still add ~6.5s to every connect for problems that
  turned out not to exist. Candidates to revert, not urgent.

**Desk work: fully exhausted for both active tracks.** Nothing further to investigate on
controller=10 or the Z-integral without new flight data — see `docs/07`'s Next-action line.
