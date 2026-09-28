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
