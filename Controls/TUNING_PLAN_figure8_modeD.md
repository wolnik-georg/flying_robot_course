# Complete tuning plan — figure-8 mode 1 onboard (Mode D)

Step-by-step commissioning guide for CS2 onboard figure-8 with full INDI (`ctrl_mode: 3`).
Work through phases in order; **lock each phase before moving to the next**.

---

## Prerequisites (done — keep as-is)

- [x] Sync coef upload (`_set_param_sync`, 8 ms/coef)
- [x] `cmdFullState` keepalive at 20 Hz during traj
- [x] OOT controller 6, `run_eval_mode=onboard_d`
- [x] `ctrl_mode: 3` (full INDI: position + attitude)
- [x] Landing / back-to-back flight cleanup working
- [x] **Discard pre-upload-fix logs** for tuning decisions

Feedforward is valid only with complete onboard upload: jerk/snap → `alpha_desired()` on firmware.

---

## Flight command (every test)

```bash
colcon build --packages-select crazyflie_examples && source install/setup.bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.02 --onboard
```

## Analysis after each flight

```bash
cd ~/Desktop/flying_robot_course/Controls
PY=~/.pyenv/versions/flying_robots/bin/python3
$PY analyze_flight.py --csv logs/<file>.csv --type figure8
$PY analyze_indi.py logs/<file>.csv
```

**Always check both scripts.** Path RMSE alone is not enough for INDI commissioning.

| Script | Key metrics |
|--------|-------------|
| `analyze_flight` | Phase-aligned XY, geom XY, **t_shift**, roll/pitch err vs flatness (traj lap), speed RMSE |
| `analyze_indi` | Hover roll/pitch peaks, **τ_sat**, α filter lag, **τ-vs-J·α** correlation & residual |

---

## Phase 0 — Baseline reference (lock before sweeping)

| Param | Value |
|-------|-------|
| `ctrl_mode` | 3 |
| `fc_bw` | 90 |
| `kr` / `kw` | 1050 / 87 |
| `kr_z` / `kw_z` | 1050 / 87 |
| Firmware | Block O: KP=28, KV=3 |

**Reference metrics:** phase-aligned XY ~4.9 cm, geom ~8.8 cm, t_shift ~+90 ms, τ_sat <5%

**Reference log:** `figure8_mode1_kt0.02_2026-06-17_21-24-32.csv`

- [x] Phase 0 locked

---

## Phase 1 — Inner loop: attitude INDI gains (KR/KW) — COMPLETE

| | |
|--|--|
| **What it tunes** | Attitude tracking, α_ref loop, torque response |
| **How** | yaml only — **no reflash** |
| **Keep fixed** | `fc_bw=90`, `ctrl_mode=3` |

### KR/KW ladder (flown 2026-06-18)

| Step | KR | KW | Log | Phase-aligned XY | Done |
|------|-----|-----|-----|------------------|------|
| 0 ✓ | 1050 | 87 | `…21-24-32` (Jun17) | **4.9 cm** | [x] |
| 0b | 1050 | 87 | `…19-07-58` (Jun18 repeat) | 6.6 cm | [x] |
| 1 | 1150 | 91 | `…19-09-28` | 6.6 cm | [x] |
| 2 | 1260 | 95 | `…19-10-36` | 7.8 cm | [x] |
| 3 | 1320 | 98 | `…19-11-36` | 7.7 cm | [x] |
| 4 | 1400 | 101 | — | **not flown** | [x] skipped |

### Phase 1 results — path + INDI combined (fc_bw=90)

| Run | KR | Path | Roll err* | Pitch err* | Roll pk† | Pitch pk† | τ↔J·α r (R/P) | τ δ roll |
|-----|-----|------|-----------|------------|----------|-----------|---------------|----------|
| **Jun17 KR1050** | 1050 | **4.9 cm** | 5.0° | 2.8° | **21°** | **14°** | 0.02 / 0.05 | 0.86 mN·m |
| Jun18 KR1050 | 1050 | 6.6 cm | 4.8° | 3.0° | 32° | 18° | 0.15 / 0.48 | 0.91 |
| Jun18 KR1150 | 1150 | 6.6 cm | 4.7° | 2.7° | 33° | 17° | 0.16 / 0.28 | 0.88 |
| Jun18 KR1260 | 1260 | 7.8 cm | **4.4°** | **2.7°** | 30° | 16° | 0.14 / 0.31 | 0.92 |
| Jun18 KR1320 | 1320 | 7.7 cm | 4.7° | 2.8° | 31° | 17° | 0.09 / 0.12 | **1.13** |

\* roll/pitch err vs flatness planned during figure-8 lap (phase-aligned)  
† hover-window peaks from `analyze_indi`

### Phase 1 conclusions

- **Locked: KR1050 / KW87** — best combined path + attitude peaks (Jun17 log).
- Higher KR (1150→1320): path RMSE **worsens**; τ_sat still <5% but **τ-vs-J·α alignment degrades** at KR1320.
- KR1260 had best traj attitude err (4.4° roll) but **worst path** (7.8 cm) → inner loop stiffer, outer loop still lagging.
- Jun18 session worse than Jun17 at same KR1050 (6.6 vs 4.9 cm; 32° vs 21° roll peak) → **single-lap variance ~1–2 cm / ~10°**.
- `analyze_indi` “HEADROOM AVAILABLE” on τ_sat is **not sufficient** to raise KR further.
- **KR1400 not needed** — trend clearly negative above 1050.

| Locked KR/KW | Log file | Phase-aligned XY |
|--------------|----------|------------------|
| **1050 / 87** | `figure8_mode1_kt0.02_2026-06-17_21-24-32.csv` | **4.9 cm** |

- [x] Phase 1 complete — KR/KW locked: **1050 / 87**

---

## Phase 2 — Inner loop: α filter (fc_bw) — SKIPPED

| | |
|--|--|
| **Decision** | **Skip** — keep `fc_bw=90` |
| **Rationale** | Prior sweep (40–120 Hz) found 90 best for path. Jun18 KR ladder showed **identical** α filter lag (~0–20 ms roll/pitch) at all KR steps. No fc_bw signal from Phase 1. |

| Step | fc_bw | Status |
|------|-------|--------|
| 0 ✓ | **90** | locked |
| 1 | 85 | skipped |
| 2 | 95 | revisit only if Phase 3 KV causes attitude noise |

- [x] Phase 2 complete — fc_bw locked: **90** (no further sweep)

---

## Phase 3 — Outer loop: position gains (KV ablation) — **NEXT**

| | |
|--|--|
| **What it tunes** | Path lag (+90 ms), velocity tracking, corner performance, cascaded attitude |
| **How** | **reflash** `flying_drone_stack/firmware_app/src/lib.rs` |
| **Keep fixed** | yaml: `ctrl_mode=3`, `fc_bw=90`, `kr/kw=1050/87` |

### Why KV is next

Phase 1 showed geom XY ~8.8–9.4 cm (shape OK) while phase-aligned XY is 5–8 cm → **~3 cm is timing/lag**, not figure-8 shape. KV=3 gives very low position damping (ζ_pos ≈ 0.28 with KP=28). Raising KV attacks the bottleneck INDI cannot fix.

### Firmware edit location

File: `flying_drone_stack/firmware_app/src/lib.rs`  
Active block: **Block O** (lines ~400–406). Comment out Block O, uncomment the test block, rebuild, reflash.

### KV ablation ladder

| Step | Block | KP_XY | KV_XY | KP_Z | KV_Z | Reflash | Notes | Done |
|------|-------|-------|-------|------|------|---------|-------|------|
| 0 ✓ | O | 28 | 3 | 30 | 7 | — | current baseline | [x] |
| 1 | O+ | 28 | **4** | 30 | 7 | **yes** | first KV raise | [ ] |
| 2 | O+ | 28 | **5** | 30 | 7 | **yes** | expect t_shift ↓, speed RMSE ↓ | [ ] |
| 3 | O+ | 28 | **6** | 30 | 7 | **yes** | stop if wobble | [ ] |
| alt | **Q** | **36** | **4** | 30 | 7 | **yes** | if KV≥5 oscillates | [ ] |

**One flight per KV step.** Log meta will not capture KP/KV — note block name in filename or a `# meta:` comment in flight logger if available.

### Reflash workflow (each KV step)

```bash
# 1. Edit lib.rs — change KV_X/KV_Y (keep Block O KP=28, comment/uncomment blocks)
cd ~/Desktop/flying_robot_course/flying_drone_stack
# build + flash per your usual workflow (cfloader / make cload)

# 2. yaml unchanged — confirm still:
#    ctrl_mode: 3, fc_bw: 90, kr/kw: 1050/87

# 3. Fly + analyze
cd ~/Desktop/crazyswarm2
colcon build --packages-select crazyflie_examples && source install/setup.bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.02 --onboard

cd ~/Desktop/flying_robot_course/Controls
PY=~/.pyenv/versions/flying_robots/bin/python3
$PY analyze_flight.py --csv logs/<file>.csv --type figure8
$PY analyze_indi.py logs/<file>.csv
```

### Phase 3 primary metrics (in order of importance)

1. **t_shift** — should move toward 0 ms (currently +90 ms)
2. **Phase-aligned XY RMSE** — target ~3.5–4.5 cm
3. **Speed RMSE** (analyze_flight) — should decrease
4. **Roll/pitch err vs flatness** (traj lap) — should improve with better velocity
5. **Geom XY** — may drop slightly if lag shrinks
6. **τ_sat, hover peaks** (analyze_indi) — watch for oscillation; stop if roll pk >22° consistently

### Stop rules

- XY oscillation / thrust spikes during figure-8
- Phase-aligned RMSE worsens **2 steps in a row**
- t_shift still >+60 ms after KV=6 → try **Block Q** (KP=36, KV=4)
- If KV=4 alone wobbles → try Block Q instead of pushing KV higher

| Locked block | KP_XY | KV_XY | Log file |
|--------------|-------|-------|----------|
| | | | |

- [ ] Phase 3 complete — firmware block locked: _____

---

## Phase 4 — Feedforward accuracy (KT motor coeffs)

| | |
|--|--|
| **What it tunes** | Position INDI `a_model`, RPM→torque mapping |
| **How** | yaml `indi_gains.kt1`–`kt4` — **no reflash** |
| **When** | Only if after Phase 3 you still see: |

- poor τ-vs-J·α correlation in `analyze_indi`
- τ_sat rising oddly despite moderate KR
- position INDI not helping (mode 3 bit 0)

**Method:** per-motor hover/bench calibration → update kt1–kt4 in yaml

- [ ] Phase 4 skipped (not needed)
- [ ] Phase 4 complete — KT updated

---

## Phase 5 — Secondary / usually skip

| Knob | Action |
|------|--------|
| KR > 1050 | **don't** — Phase 1 showed path + τ alignment degrade |
| `KI_ATT` | leave 0 unless steady attitude bias remains |
| `ctrl_mode` | stay 3 (geom = reference only) |
| `mass` | update only if payload changes |

---

## Phase 6 — Validation & stress test

### 6a — Confirm at kt=0.02

- [ ] 2–3 laps with **all locked gains**
- Target: phase-aligned XY stable within ~0.5 cm of Phase 3 best

### 6b — Faster trajectory

```bash
ros2 run crazyflie_examples flight -- --trajectory figure8 --mode 1 --kt 0.1 --onboard
```

- [ ] Same locked stack flown at kt=0.1

---

## Full parameter map

| Phase | ctrl_mode | fc_bw | kr/kw | KP/KV | kt1–4 | Reflash? |
|-------|-----------|-------|-------|-------|-------|----------|
| 1 KR/KW ✓ | 3 | 90 | 1050/87 locked | — | — | No |
| 2 fc_bw ✓ | 3 | 90 locked | locked | — | — | No |
| 3 KV/KP | 3 | locked | locked | ✓ | — | **Yes** |
| 4 KT | 3 | locked | locked | locked | ✓ | No |
| 6 validate | 3 | locked | locked | locked | locked | No |

---

## Decision metrics cheat sheet

| Metric | Good | Stop / investigate |
|--------|------|-------------------|
| Phase-aligned XY RMSE | ↓ trend | ↑ 2 steps in a row |
| Geom XY RMSE | ~8.5–9 cm (lag-limited) | sudden +1 cm jump |
| **t_shift** | toward 0 ms | stuck >+80 ms after KV sweep |
| τ_sat | <5% ideal, <15% ok | >15–20% |
| Roll/pitch peaks (hover) | <22° / <18° | above limits |
| τ-vs-J·α r (roll/pitch) | higher better | <0.1 with high τ δ |
| Speed RMSE | ↓ with KV | oscillation |

---

## Expected outcome trajectory

```
Phase 0–1:   4.9 cm aligned @ KR1050, fc90, KV3  (DONE)
Phase 2:     fc_bw=90 locked, skipped fine-tune   (DONE)
Phase 3:     ~3.5–4.5 cm + t_shift ↓ + cleaner velocity plots (KV)  ← NOW
Phase 4:     polish if feedforward still off
Phase 6:     confirm robustness at kt=0.1
```

---

## yaml template (locked inner loop)

File: `crazyflie/config/crazyflies.yaml` → `robots/<your_drone>/indi_gains`

```yaml
indi_gains:
  ctrl_mode: 3
  fc_bw: 90.0
  kr: 1050.0
  kw: 87.0
  kr_z: 1050.0
  kw_z: 87.0
```

---

## Current status / notes

**Phase:** 3 — KV ablation (outer position loop)

**Next action:** Reflash **Block O+ with KV=4** (KP=28 unchanged), fly figure-8, run both analyzers.

**Locked inner loop:**

| Param | Value |
|-------|-------|
| ctrl_mode | 3 |
| fc_bw | 90 |
| kr / kw | 1050 / 87 |
| kr_z / kw_z | 1050 / 87 |
| Firmware block | O (KV=3) → next: O+ KV=4 |

**Best log (Phase 1):** `figure8_mode1_kt0.02_2026-06-17_21-24-32.csv`

**Best phase-aligned XY:** 4.9 cm

---

*Last updated: 2026-06-18*
