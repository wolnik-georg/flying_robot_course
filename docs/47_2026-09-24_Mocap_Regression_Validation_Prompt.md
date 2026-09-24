# 47 — 2026-09-24 mocap / regression validation agent prompt

**Purpose:** Copy from **`--- PROMPT START ---`** through **`--- PROMPT END ---`** into Claude (or another agent) to **validate whether Sep-24 lab failures are explained by our repo changes, mocap/ops, or both**.  
**Companion docs:** `docs/46_2026-09-24_Regression_Code_Delta_Handoff.md`, `docs/lab_sessions/2026-09-23.md`, `experiments/logs/usd_raw/2026-09-24_PAIRING.md`.

**Operator intent:** Last **good** lab day = **2026-09-23** (C.1 merges). **2026-09-24**: A4 ×3 + A1 (+ retry) — motors ramp then drones “go nuts” / crash; operator suspects **something on our side** changed since yesterday. Desk already archived uSD and cleared SD cards for a clean re-fly.

---

## PROMPT START

You are validating a **hardware regression** on a two-Crazyflie mocap lab (Crazyswarm2 + custom geometric/INDI firmware on **cf5**, stock Lee on **cf_second**). Your job is **Phase A only: read repos + logged data, rank hypotheses, produce a verdict table and a minimal fix list**. Do **not** plan new flights unless the operator asks.

### Repos (expected paths on lab/desk PC)

| Repo | Remote | Baseline “good day” | Head at handoff (2026-09-24 desk) |
|------|--------|---------------------|-----------------------------------|
| `flying_robot_course` | `github.com:wolnik-georg/flying_robot_course` | Sep-23 C.1 artifacts | **`dc1883f`** |
| `crazyswarm2` | (operator’s fork/clone) | Sep-23 evening with geofence commits | **`4d90e41`** |

Run `git log -1 --oneline` on both and flag if lab PC is behind.

**Uncommitted (desk only, may not match lab PC):** `crazyswarm2/crazyflie/config/crazyflies.yaml` — **cf5 `enabled: false`**, **cf_second only** for solo stock-Lee hover test. **Do not assume** this was the yaml during Sep-24 formation flights (both were likely enabled).

---

### Symptom summary (2026-09-24)

- **Scenarios flown:** A4 lemniscate ×3 (~17:17, 17:20, 17:23), A1 stack hover (~17:33, retry ~17:36).
- **Operator observation:** takeoff / motor ramp OK, then violent behaviour or crash within seconds.
- **Radio CSVs** in `flying_robot_course/experiments/logs/*2026-09-24*.csv`: huge `pos_*` jumps, sometimes **y ≈ +40 m**, **27 m** single-tick on cf_second A1.
- **On-drone uSD** (authoritative for *firmware* EKF + controller): **`stateEstimate.*` stays in lab volume** (max |pos| ≈ **1.6 m** on any archived file); **`ctrltarget.*` sane** (lemniscate ±0.75 m, stack z 0.18–1.30 m). **No uSD file** shows “commanded flight to y=40 m”.
- **Missing uSD:** card slots **THESIS1 `thesis36–39`**, **THESIS2 `thesis48–51`** were **0 B** — worst crashes may have **no onboard log**.

**Good reference (2026-09-23):**  
`experiments/logs/usd_raw/2026-09-23_THESIS1/cf5_A1_thesis19_2026-09-23_17-22-06.bin` — max |Δ stateEstimate| **< 1 mm**, hover z ≈ 1.23 m, tgt 1.30 m.

---

### Archived data you must use

1. **Full SD snapshot (pre-refly):**  
   `flying_robot_course/experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/`  
   - `THESIS1/` — 32 files, original names `thesis00`…`thesis35` (cf_second card)  
   - `THESIS2/` — 43 files (cf5 card)  
   - `README.md` — wipe procedure  

2. **Pairing notes:** `experiments/logs/usd_raw/2026-09-24_PAIRING.md`

3. **Radio + meta (QA only, do not treat as ground truth for EKF):**  
   `experiments/logs/A4_*_2026-09-24_*.csv`, `A1_*`, `*.meta.json`

4. **Analysis tool:**  
   `experiments/analysis/mocap_pose_proof.py` — decodes uSD via `flying_drone_stack/tools/decode_usd_log.py` (`stateEstimate.*`, `ctrltarget.*`).

**Decode example:**
```bash
cd ~/Desktop/flying_robot_course
python3 flying_drone_stack/tools/decode_usd_log.py \
  experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/THESIS2/thesis37
python3 experiments/analysis/mocap_pose_proof.py \
  experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/THESIS2/thesis37
```

---

### uSD findings you must confirm (numbers)

Recompute; do not trust prose alone.

| File | Role | Key fact |
|------|------|----------|
| `THESIS2/thesis37` | cf5 | **~1.71 m** single-step in **stateEstimate** at log start; **z → −1.62 m**; **ctrltarget z ≈ 0.05–0.10 m** |
| `THESIS1/thesis34` + `THESIS2/thesis46` | top + bottom, paired **`run_tag=1790184079`** | Lemniscate ~29 s; bottom est z ≈ **0.98** vs tgt **1.00**; top est drops to ≈ **0.98** while **tgt z = 1.30** after ~8 s; max est step top **~0.37 m** |
| `THESIS1/thesis31–33` + `THESIS2/thesis43–45` | stack segments | max est steps **≤ 27 mm**; paired by `run_tag` |
| Good ref `2026-09-23 … thesis19` | cf5 | max step **< 1 mm** |

**Cross-check:** No archived uSD has |x|,|y|,|z| > ~2 m. If you find one, update the hypothesis set.

**Radio vs uSD:** Radio can show **multi-metre teleports** while uSD on the **same nominal flight index** does not — treat as **clock mis-pairing and/or ROS logging path**, unless you time-align (`usd_start_s` in CSV header vs uSD `t[]`).

---

### Code changes since 2026-09-23 (you must read diffs)

#### `crazyswarm2` — **only** formation/safety touched; **not** `simple_flight.py` / `flight.py`

| Commit | When (CEST) | Change |
|--------|-------------|--------|
| `3657aa7` | Sep 23 18:44 | `formations/safety.py`: **`FLIGHT_SPACE['z']` (0.0, 1.70) → (0.1, 1.30)** |
| `7d9d9e3` … `c50ca82` | Sep 23 eve | `run_formation.py`: **`apply_a7_lab_defaults`**, **`apply_a2_lab_defaults`** (height ~0.45, radius 0.40, rotate 90°) |
| `66c9c65` | Sep 23 21:02 | Cached trajectory CSVs only |
| `7cb419a` | Sep 24 13:57 | `scenarios.py`: A2 default radius **0.75 → 0.40**; stronger A2 hook in `run_formation.py` |
| `e2f85e5` | Sep 24 **17:24** | **`clamp_height_for_mocap_z(args, sc)`** after `scenarios.build()` |
| `4d90e41` | Sep 24 **17:32** | A2: force **`args.auto_center = True`** |

**Critical timing:** First bad A4 radio stamp **17:17:37** is **before** commits `e2f85e5` / `4d90e41` — lab may have run **without** height clamp unless they pulled mid-session.

**Read:**
```bash
cd ~/Desktop/crazyswarm2
git diff 66c9c65..4d90e41 -- crazyflie_examples/crazyflie_examples/run_formation.py
git show 3657aa7 -- crazyflie_examples/crazyflie_examples/formations/safety.py
git show 7cb419a -- crazyflie_examples/crazyflie_examples/formations/scenarios.py
```

**Sep-24 A4 meta (radio QA):** anchor height **1.0 m**, dz **0.30**, top target **z ≈ 1.30 m** — equals **`FLIGHT_SPACE` z_max 1.30** after Sep-23 cap → plausible **mocap track loss at ceiling** for top drone even if setpoints are “correct” per scenario.

#### `flying_robot_course` — **no flight-runner changes** Sep 23–24

| Commit | Runtime? |
|--------|----------|
| `8e803b9` | **`app-config-bl` CF_MASS 41000 → 42700** — only if **brushless firmware reflashed**; project convention: **Lee on cf_second = separate std build** |
| `d707d80` | `residual_nn.rs` comment only |
| `dc1883f` | uSD snapshot + `mocap_pose_proof.py` |
| `bf90f88` etc. | Sep-24 radio logs only |

**Read:**
```bash
cd ~/Desktop/flying_robot_course
git show 8e803b9 -- flying_drone_stack/firmware_app/app-config-bl
```

---

### Hypotheses to validate or falsify

For each hypothesis, state **CONFIRMED / REJECTED / INCONCLUSIVE**, evidence, and **what would change the verdict**.

| ID | Hypothesis | How to test |
|----|------------|-------------|
| **H1** | **Mocap rigid-body / marker association** broken (extra drone in volume, wrong body, dropped markers) → bad external pose → EKF `stateEstimate` | Motive logs; yaml enabled vs physical presence; compare to Sep-23 session notes (`docs/lab_sessions/2026-09-19.md`, `2026-09-23.md`); 4-marker **`cf21_active` / `active_deck`** |
| **H2** | **A4/A1 commanded top above mocap z cap** (height 1.0 + stack dz) after **`FLIGHT_SPACE` z_max=1.30** → track loss, controller fights | Replay `clamp_height_for_mocap_z` logic with Sep-24 meta; check uSD top **thesis34/35** z vs tgt 1.30 |
| **H3** | **Lab ran stale `run_formation`** (no pull/build before 17:17) | Lab shell history / git reflog; compare printed `[formation]` clamp messages vs commit `e2f85e5` |
| **H4** | **Our formation code generates insane setpoints** | uSD **`ctrltarget.*`** on thesis34–35, 31–33 — already look sane; verify programmatically |
| **H5** | **Firmware regression** (mass 42.7g, etc.) | Read `stabilizer.controller` + flash date per drone; only relevant if cf5 reflashed Sep 23+ |
| **H6** | **Radio `/state` is wrong but uSD EKF is fine** | Time-align one flight; if aligned and still diverge, H6 rejected |
| **H7** | **Disabled drone in volume stole markers** (documented failure mode) | Was cf5 disabled while physically present? Sep-24 formation likely **both enabled** |

---

### Required deliverables (your reply)

1. **Verdict table** — H1–H7 with CONFIRMED/REJECTED/INCONCLUSIVE + 1-line evidence each.

2. **Code attribution** — List every Sep 23–24 commit that **could** affect Sep-24 flights; explicitly list files that **did not change** (`simple_flight.py`, `flight.py`, mocap bridge).

3. **Root cause statement** (≤ 5 bullets) separating:  
   - **Must fix before flying** (mocap / yaml / height)  
   - **Should fix in software** (if any bug found in `clamp_height` or geofence)  
   - **Red herrings** (radio-only teleports, etc.)

4. **Minimal re-fly protocol** (single drone first):  
   - `simple_flight` hover, **one** enabled robot, mocap stable, uSD non-zero file  
   - Pass criterion: uSD max |Δ stateEstimate| **< 0.05 m** over 15 s hover (Sep-23 ref **< 0.001 m**)

5. **Open questions** for the operator (only if blocked).

---

### Constraints

- **Do not** recommend merging **2026-09-24** data into C.1 training until pose verified.
- **Prefer uSD `stateEstimate` + `ctrltarget`** over radio CSV for EKF/controller conclusions.
- **SD cards were wiped** after snapshot; new logs should start from low `thesisNN` indices.
- Controller map: **cf5** study = **6 / ctrl_mode 0** for C.1; **cf_second** = **5** (Lee). Sep-24 solo test may use **cf_second only**.

---

### Suggested work order

1. `git pull` both repos; record SHAs.  
2. Read diffs listed above.  
3. Run `mocap_pose_proof.py` on **thesis37**, **thesis34**, **thesis46**, and **Sep-23 thesis19**.  
4. Read `A4_2026-09-24_17-17-37.meta.json` and `A1_2026-09-24_17-33-49.meta.json`; compare to `FLIGHT_SPACE` and scenario params.  
5. Inspect `crazyflies.yaml` + `motion_capture.yaml` + `formations/safety.py` for consistency.  
6. Write deliverables 1–5.

---

## PROMPT END

---

## After validation (operator)

- If prompt doc should be on lab PC: `git pull` in `flying_robot_course` (commit **`docs/47_...`** when pushed).
- Re-fly: see §Minimal re-fly protocol in prompt; local yaml may have cf5 disabled for solo Lee.
