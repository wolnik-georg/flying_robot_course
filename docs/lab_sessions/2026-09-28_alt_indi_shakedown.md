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
