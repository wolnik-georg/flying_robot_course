# 44 — controller=9 (Omar pure INDI) — hardware validation agent prompt

**Purpose:** Copy from **`--- PROMPT START ---`** through **`--- PROMPT END ---`** into Claude (or another agent) to **audit readiness and refine the flight ladder** before any lab execution.  
**Not the same as:** `docs/26` (superseded “retrained NA-INDI” plan) · **This is** `stabilizer.controller=9` = **`controller_omar_indi.c`** (literal C port of supervisor’s `controller_lee.c`).

**Operator intent:** After **C.1** closes (**A4 ×4**), evaluate Omar’s INDI on **our brushless cf5** as a candidate for the core comparison — **sim/numerical work is done; hardware is zero**.

---

## PROMPT START

You are auditing **hardware readiness** for **`stabilizer.controller=9`** (Omar INDI, `ControllerTypeOot4`) on a Master’s thesis stack: **cf5** = study drone (brushless, `cf21_active`), **cf_second** = **permanently stock Lee (`controller: 5`)** top vehicle only.

**Phase A (this conversation):** verify files, configs, and gaps against the filesystem; produce a **go/no-go checklist** and **ordered flight ladder** (single-drone → 2-drone). **Do not assume sim-clean implies hardware-safe.**

**Phase B (lab, only after operator approves Phase A):** execute flights — out of scope unless operator says “execute now”.

### Disambiguation (controller numbers)

| `stabilizer.controller` | Implementation |
|-------------------------|----------------|
| **6** | Our geometric / INDI (`lib.rs`, `ctrl_mode` 0–3) — **C.1 collection uses `ctrl_mode: 0` on cf5** |
| **7** | Briesewitz NA-INDI plain port (`naindi.rs`) |
| **8** | Same + `use_nn` hybrid |
| **9** | **Omar literal C** (`controller_omar_indi.c`) — **no `ctrl_mode`; full INDI inside his law** |

### What is already done (verify, don’t re-litigate)

Ground in these sources:

| Claim | Where to verify |
|-------|-----------------|
| C port built, enum **9**, `CONFIG_CONTROLLER_OOT4=y` in **`flying_drone_stack/firmware_app/app-config-bl`** | `app-config-bl`, `~/Desktop/crazyflie-firmware` Kconfig/dispatch patches in **`flying_drone_stack/firmware_app/host/naindi_controller_slot.patch`** |
| **`controller_omar_indi.c`** on disk | `~/Desktop/crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c` (+ `.h`); recovery notes in **`LOCAL_MODIFICATIONS.md`** if untracked |
| Numerical **6/6** vs Omar repo | `flying_drone_stack/firmware_app/host/test_omar_indi_reference.py` |
| Dispatch wrapper | `flying_drone_stack/firmware_app/host/test_oot4_dispatch_wrapper.sh` |
| SIL **`oot4`**: single-drone (hover/figure8/circle-class), 2-drone, 3-drone; **`np` + `neuralswarm`** | `crazyswarm2/crazyflie/config/server_sim_omar_indi.yaml`, `server_sim_omar_indi_dw.yaml`; notes in **`firmware_app/host/omar_indi_reference_build_notes.md`**, summary **`docs/41` §8** |
| **`CF_MASS=42700` (42.7 g)** for Omar path | `app-config-bl`, host `-DCONFIG_MODIFIED_CF_MASS=42700` in bindings (2026-09-23 fix) |
| **Never flown on hardware** | **`docs/41` §8**, **`docs/07`**, `omar_indi_reference_build_notes.md` “Cannot be closed without hardware” |

### What is NOT prepared (expected — agent must confirm)

| Item | Expected state | Action before flight |
|------|----------------|----------------------|
| **`crazyflies.yaml`** | **cf5** still **`stabilizer.controller: 6`**, **`ctrl_mode: 0`** (C.1) | **Per-robot override** → **`controller: 9`** on **cf5 only** for Omar session; **do not** change shared `all:` or **cf_second** (`controller: 5`) |
| **Lab yaml pre-set** | Intentionally **not** committed to 9 | Operator sets at lab; document read-back of `stabilizer.controller` over radio |
| **Gain tuning** | Omar’s **default gains** (his ETHZ-style blocks) — **not** retuned for this airframe | Treat first hops as **exploratory**; abort on growing oscillation (same discipline as c=7 ladder) |
| **`MOTORRPM2FORCE` / kt** | Copied from Omar’s brushless defaults; **no independent bench cross-check on our side** | Flag in thesis limitations; watch hover thrust/height |
| **RPM telemetry path** | **Different from c=6/7:** c=9 reads RPM via **firmware log API** (`rpm.m*`), Init checks **`deck/bcRpm` param**; **`rpm_source` in `traj_iface.c` does not apply** to Omar’s C file | **Verify non-zero RPM in logs** on first hover (`motor.m*_rpm` and/or `rpm.m*` per what's actually wired); **`a_res` semantics may differ** — confirm which vars matter for your abort criteria |
| **`simple_flight.py`** | Treats **`controller != 6`** as “stock-like” for **`pos_gains`** (yaml passes through, no GEOMETRIC_POS_GAINS swap) | Use **`--pin-controller`** so takeoff/trajectory/land **never mid-air switch** controller or gains (2026-09-09 lesson) |
| **Real hardware vs SIL** | SIL uses **`crazyflie_server.py` + `oot4`**; hardware uses **`stabilizer.controller=9` param**, not ROS `controller: oot4` | Do not confuse sim yaml with radio params |

### Sequencing vs C.1 (operator policy — confirm)

1. **Finish C.1 first:** **A4 ×4** with **cf5 @ controller 6, ctrl_mode 0** (geometric) — do not mix Omar into training merges unless explicitly intended.
2. **Omar ladder on a separate block** (same day OK **after** A4 + desk merge, or dedicated session):
   - Reflash / confirm firmware includes **OOT4**
   - Switch **cf5 → controller 9**
   - Single-drone **`simple_flight`** (pinned): **hover → figure8 or circle** (low, in volume)
   - If clean: **2-drone `run_formation`**: **A1 hover** then **A8** (same order sim-tested for c=7/8/9); **cf_second stays Lee**

### Pre-flight verification checklist (agent: mark pass/fail from files + operator confirms on lab PC)

**Firmware tree (`~/Desktop/crazyflie-firmware`)**

- [ ] `controller_omar_indi.c` / `.h` present; **`CONFIG_CONTROLLER_OOT4=y`** in effective config used by **`flying_drone_stack/firmware_app`** build
- [ ] `MAX_USD_LOG_VARIABLES_PER_EVENT` **48** (`usddeck.c`) if logging thesis uSD config
- [ ] **`CONFIG_MODIFIED_CF_MASS=42700`** in **`app-config-bl`** matches intent for Omar
- [ ] Recent **`make cload`** (or `DRONE=bl`) on **cf5** after any firmware change

**Repos synced on lab PC**

- [ ] `flying_robot_course` — manifests, docs
- [ ] **`crazyswarm2`** — includes **`oot4`** SIL support (for optional pre-lab sim replay only) + **`simple_flight` / `run_formation`**
- [ ] **`colcon build`** if crazyswarm2 changed

**Radio / yaml (`crazyswarm2/crazyflie/config/crazyflies.yaml`)**

- [ ] **cf5** enabled, **`type: cf21_active`**, **`rpm_source: 1`**, uSD logging plan unchanged
- [ ] **`stabilizer.controller: 9`** only under **cf5** for Omar runs (restore **6** for C.1 geometric)
- [ ] **cf_second**: **`controller: 5`**, no DShot rpm topic bleed

**Abort criteria (hardware)**

- Growing roll/pitch oscillation within **~1 s** of trajectory start or handoff
- **`a_res_*` all zero** (if you rely on them — may not apply same way on c=9)
- Mocap dropout / geofence breach
- Motor saturation or landing detection failure

**Logging**

- uSD on for any flight that counts; radio = QA (`docs/07` policy)
- Record **`stabilizer.controller`**, firmware git dirty state, yaml snippet in lab session note

### Suggested commands (templates — operator fills drone name)

**Single-drone (after yaml → controller 9 on cf5):**

```bash
# Example — confirm exact flags in simple_flight.py --help on lab PC
ros2 run crazyflie_examples simple_flight -- --drone cf5 --mode hover --pin-controller
# then figure8 / circle at conservative speed/height inside FLIGHT_SPACE
```

**2-drone (after single-drone pass):**

```bash
ros2 run crazyflie_examples run_formation -- --auto-center --yes \
  --scenario A1 --dz 0.30 --hold 15
# then A8 per docs/29 / sim matrix — cf5 must be controller 9, cf_second Lee
```

**Optional desk replay before lab (sanity, not substitute for hardware):**

```bash
# From crazyswarm2, with rebuilt cffirmware bindings
ros2 launch crazyflie_sim crazyflie_server.py config:=server_sim_omar_indi.yaml ...
```

### Agent deliverables (Phase A reply)

1. **Go / no-go / go-with-caveats** table (firmware, yaml, scripts, sequencing).
2. **Gap list** — anything missing vs **`LOCAL_MODIFICATIONS.md`** or **`omar_indi_reference_build_notes.md`**.
3. **Refined ladder** (minimum flights to call “validated enough to compare” vs “needs retune”).
4. **Explicit conflicts** with tomorrow’s plan (A4, restore c=6, when to flash).
5. **Open items** that cannot be verified without lab (mark “operator check”).

### Rules

- Do not change **`flying_drone_stack/tools/residual/`** unless operator expands scope.
- Do not silently set **`controller: 9`** in committed yaml — lab override only unless operator asks to commit.
- No invented flight outcomes; no push unless operator asks.
- No **`Co-Authored-By` / Cursor / Claude session trailers** on any commit.

### Key doc map

| Doc | Role |
|-----|------|
| **`docs/41_Pure_INDI_Implementation_Comparison.md` §8** | Summary + “never flown” |
| **`docs/26_Controller9_NA_INDI_Retrained.md`** | **Disambiguation banner only** |
| **`firmware_app/host/omar_indi_reference_build_notes.md`** | Build, SIL, bugs fixed, open hardware items |
| **`firmware_app/host/LOCAL_MODIFICATIONS.md`** | Firmware manifest + OOT4 row |
| **`firmware_app/host/naindi_reference_build_notes.md` § “Prerequisites for a real controller=7/8 flight”** | **Template ladder** (adapt for 9) |
| **`docs/07_Thesis_Progress_Checklist.md`** | Parallel track, not blocking C.1 |
| **`docs/25_C1_Data_Collection_Plan.md`** | Finish **A4** before treating collection as closed |

--- PROMPT END ---

## Local note

After hardware runs, add **`docs/lab_sessions/YYYY-MM-DD.md`** + restore **cf5 → controller 6, ctrl_mode 0** for C.1/C.4 unless Omar is chosen as primary INDI.
