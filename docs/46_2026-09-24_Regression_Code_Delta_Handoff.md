# 2026-09-24 — “Something broke since yesterday” — code delta handoff

**For a follow-up agent:** compare this against git on **lab PC** (`git pull` + `git log`) and against **physical mocap** (Motive rigid bodies, markers in volume). Symptom day: **2026-09-24** A1/A4 crashes / bad pose; last **good** full lab day: **2026-09-23** C.1 (`docs/lab_sessions/2026-09-23.md`).

**uSD conclusion (already on desk):** `ctrltarget.*` sane; `stateEstimate.*` jumps / wrong z on-card → **pose/EKF path**, not formation setpoint generation. See `experiments/logs/usd_raw/2026-09-24_PAIRING.md`, snapshot `2026-09-24_SD_CARD_SNAPSHOT/`.

---

## 1. Timeline

| When | What |
|------|------|
| **2026-09-23** | Successful C.1 flights; **same day** geofence / A2/A7 lab defaults landed in `crazyswarm2` (`3657aa7` … `c50ca82`, cache `66c9c65`). |
| **2026-09-24 ~13:57** | `7cb419a` — A2 library default radius 0.75 → 0.40 in `scenarios.py` + `run_formation.py` hooks. |
| **2026-09-24 ~17:17–17:33** | Bad A4/A1 radio + some uSD (see repo `experiments/logs/*2026-09-24*`). |
| **2026-09-24 ~17:24–17:32** | `e2f85e5`, `4d90e41` — geofence follow-ups (`clamp_height_for_mocap_z`, A2 `auto_center`). **Pushed after first A4 stamp.** |
| **2026-09-24 desk** | `flying_robot_course` logs + uSD snapshot commit `dc1883f`; **no** `flight.py` / `simple_flight.py` changes. |

---

## 2. `crazyswarm2` — commits since 2026-09-23 (code)

| Commit | Time (CEST) | Files | Summary |
|--------|-------------|-------|---------|
| `3657aa7` | Sep 23 18:44 | `formations/safety.py` | **`FLIGHT_SPACE['z']`**: `(0.0, 1.70)` → **`(0.1, 1.30)`** (commanded/mocap cap). |
| `5bbc2cf` | Sep 23 18:51 | (related) | z ceiling narrative / margin. |
| `7d9d9e3` | Sep 23 18:56 | `run_formation.py` | `apply_a7_lab_defaults` (rotate, height, geofence). |
| `6ca12d5` | Sep 23 19:05 | A7 rotate default. |
| `906f37d` | Sep 23 19:25 | A7 height/length. |
| `c50ca82` | Sep 23 19:29 | `run_formation.py` | **`apply_a2_lab_defaults`** (h≈0.45, r=0.40, rotate 90°). |
| `66c9c65` | Sep 23 21:02 | data cache | Trajectory CSV cache only. |
| `7cb419a` | Sep 24 13:57 | `scenarios.py`, `run_formation.py` | A2 default **radius 0.40** in library; stronger A2 lab hook. |
| `e2f85e5` | Sep 24 17:24 | `run_formation.py` | **`clamp_height_for_mocap_z()`** after `scenarios.build()`. |
| `4d90e41` | Sep 24 17:32 | `run_formation.py` | A2 force **`--auto-center`**. |

**Unchanged since Sep 23:** `flight.py`, `simple_flight.py`, `crazyflie_server.py`, `crazyflies.yaml` on **remote** (local uncommitted: cf5 `enabled: false` for solo Lee test).

**Not in git:** Motive layout, marker sets, who was **enabled** in yaml during Sep 24 flights (both drones were enabled for formation).

---

## 3. `flying_robot_course` — commits since 2026-09-23 (runtime-relevant)

| Commit | Runtime impact |
|--------|----------------|
| `8e803b9` (Sep 23) | **`app-config-bl` `CONFIG_MODIFIED_CF_MASS` 41000 → 42700** — affects **only if brushless firmware reflashed**; **Lee on cf_second uses separate std build** per project convention. |
| `d707d80` | `residual_nn.rs` **comment only** (RAM investigation). |
| `eedabc3` / `69c3af0` | Desk **`rpm_source_quality.py`** — analysis only. |
| `dc1883f` | uSD snapshot + **`mocap_pose_proof.py`** — analysis only. |
| Many `* logs` | Radio/meta Sep 24 — data, not code. |

**No changes** to `run_formation`, CS2 launch, or mocap bridge in this repo.

---

## 4. Hypothesis ranking (for agent to verify)

### A. Mocap / association (strongest; matches uSD)

- Documented rule: drone **in volume** must be **enabled** in yaml **or removed** (`crazyflies.yaml` comments, Sep 19–23 sessions).
- Sep 24 formation: **cf5 + cf_second both enabled** — extra markers / wrong rigid-body assignment → bad **external pose → EKF `stateEstimate`**.
- **Check:** Motive: only expected bodies; no duplicate markers; 4-point **`active_deck`** on each enabled drone.

### B. Lab PC **did not rebuild / pull** before Sep 24

- If old `run_formation` ran without `clamp_height_for_mocap_z`, **A4 @ height 1.0 + top z** may command **above `FLIGHT_SPACE` z_max 1.30** → track loss at ceiling (matches top uSD z drop vs tgt 1.30 on `thesis34/35`).
- **Check:** lab `git log -1` on `crazyswarm2`, time of `colcon build` vs flight stamps.

### C. Geofence / `FLIGHT_SPACE` (formation CLI only)

- Tighter **z (0.1, 1.30)** affects **`formations/safety.py` checks** and **`run_formation` height clamp** — **does not** patch firmware EKF or Motive.
- Unlikely sole cause of **instant** crash on takeoff; can explain **top drone** fighting **z** limit mid-lemniscate.

### D. Firmware mass 42.7 g (only if cf5 reflashed Sep 23+)

- Would affect **controller 9** or any stock controller on **brushless app-config-bl** build — **not** typical Sep 24 path (cf5 **c=6**, cf_second **c=5** on std build).
- **Check:** `param stabilizer.controller` + which binary was flashed on each drone.

### E. **`simple_flight` / radio logging**

- **No code changes** Sep 23–24. Radio showing huge jumps while uSD in-room → treat radio as **unsynced or ROS `/state` stream**, confirm with **uSD + Motive**, not radio alone.

---

## 5. Agent double-check checklist

1. **Lab PC git SHAs:** `crazyswarm2` ≥ `4d90e41`? `flying_robot_course` ≥ `dc1883f`?
2. **`colcon build --packages-select crazyflie_examples`** after pull?
3. **Sep 24 flight commands** — A4 height/dz vs `FLIGHT_SPACE` (`formations/safety.py`); was **`clamp_height_for_mocap_z`** active?
4. **Yaml at flight time:** both drones enabled? cf5 **out of volume** for solo tests?
5. **Motive:** rigid body names ↔ `motion_capture.yaml` / `cf21_active` **`active_deck`** geometry unchanged since Sep 23?
6. **Reproduce:** single drone **`simple_flight` hover**, uSD max |Δpos| ≪ 0.1 m before any formation.
7. **Do not** merge Sep 24 into C.1 until pose verified.

---

## 6. Key diffs to read

```bash
cd ~/Desktop/crazyswarm2
git diff 66c9c65..4d90e41 -- crazyflie_examples/crazyflie_examples/run_formation.py
git diff 3657aa7^..3657aa7 -- crazyflie_examples/crazyflie_examples/formations/safety.py
git show 7cb419a -- crazyflie_examples/crazyflie_examples/formations/scenarios.py

cd ~/Desktop/flying_robot_course
git show 8e803b9 -- flying_drone_stack/firmware_app/app-config-bl
python3 experiments/analysis/mocap_pose_proof.py \
  experiments/logs/usd_raw/2026-09-24_SD_CARD_SNAPSHOT/THESIS2/thesis37
```

---

## 7. Local-only (not pushed)

- `crazyswarm2`: `crazyflies.yaml` — **cf5 `enabled: false`**, cf_second only (solo stock Lee).
