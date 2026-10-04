# 60 — Toy harness vs compiled SIL: why limit cycle here but not there?

Status 2026-10-04. **Investigation only** — no root-cause claim.

## Summary (baseline validity first)

| Question | Answer | Confidence |
|---|---|---|
| **Is the 1-axis harness faithful to `lib.rs`?** | **No** — several harness-only structures (scalar `sin(θ)` attitude, **2-tick command dead time**, **`rpm_delay[1]` base**, no SO(3)/position/residual) | **FACT (code)** |
| **What is necessary for harness LC at kr 2400?** | **τ_act ≈ 44 ms AND plant command dead time ≥ 2 ms** at 1 kHz; RPM 1-sample lag alone **not** sufficient; **τ=44 with dead=0 → calm** | **SIMULATION (bisect)** |
| **Does full SIL reproduce flight LC without extras?** | **No** — gyro **~0.7 °/s** (doc 58/59) | **SIMULATION (HIGH)** |
| **Smallest SIL add-on that approaches flight gyro RMS?** | **2 ms command dead time on plant** (`cmd_dead_ticks=2`) → **~233 °/s**, f_dom **~7.4 Hz** (not 4.7–5.7); Omar stays **~0 °/s** | **SIMULATION (MEDIUM)** |

**Baseline validity:** **PARTIAL / NOT VALIDATED for full flight match** — amplitude **near** band (233 vs 260–290), frequency and height error **not** matched; **cannot** treat doc-58 grid as factor-validated. **JUDGEMENT:** Toy harness LC is **dominated by plant dead-time + lag**, not reproduced by motor_tau alone in compiled SIL; **“linear 44 ms lag alone”** is **refuted** for SIL; harness **overstates** dead time vs current firmware+SIL ordering.

| Source | Gyro RMS [°/s] | f_dom [Hz] | Notes |
|---|---:|---:|---|
| Flight ours (doc 56 §3) | 260–290 | 4.7–5.7 | MEASUREMENT |
| Harness baseline (bisect) | σ≈4.3 rad/s (**~247 °/s**) | **7.9** | SIMULATION, 1-D |
| SIL compiled (no gap) | **~0.7** | ~2.3 | SIMULATION |
| SIL + `cmd_dead_ticks=2` | **~233** | **~7.4** | SIMULATION |

---

## 1 — Fidelity audit (harness vs `lib.rs`)

| Element | Harness (`indi_loop_rate_harness.rs`) | `lib.rs` (`controller_step`) | Equal? |
|---|---|---|---|
| Attitude error | `sin(θ)` scalar | `eR` from `Rd,R` (SO(3)) L1969–1975 | **No** |
| α_meas | BW on ω then diff; `filt_order=1` path L171–176 | Same pattern L1998–2006 when `filt_order=1` | **Partial** |
| α_ref | `-KR*sin(θ) - KW*ω` | `-KR*eR - KW*e_ω` + `alpha_des` L2068–2072 | **Partial** |
| **τ_cur base** | **`rpm_delay[1]`** (1-step old **applied torque**) L185–186 | **`rpms_to_torque` same tick** from RPM L2107–2108 | **No** |
| **Plant path** | **2-tick dead time** on τ_cmd + 1st-order τ L192–197 | SIL: **instant** RPM feedback from plant L603–605,728–729 `crazyflie_sil.py` | **No** |
| BW on τ base | `bw_tau` on base L186 | `bw_tau_*` on `tau_current_raw` L2150–2156 | **Partial** |
| Clamp | `TAU_CLAMP` 0.014 N·m L187 | `tau_xy_max` / mixer scale (different path) | **Partial** |
| Position / residual | absent | full outer loops | **No** |
| Loop rate | 1 kHz L169 | 1 kHz stabilizer | **Yes** |

**JUDGEMENT (HIGH):** Harness limit cycle can be **artifact of dead-time + stale base** not present together in compiled SIL.

---

## 2 — Harness bisection (NEW `indi_harness_bisect.rs`)

Flown-like defaults: kr/kw 2400/170, fc_bw 206, filt_dt_us 1000, prewarp, τ=44 ms.

| Case | σ [rad/s] | f [Hz] | LC? |
|---|---:|---:|---|
| baseline | **4.27** | **7.88** | yes |
| **no_dead (dead=0)** | **0.00** | — | **no** |
| dead=1 | 0.04 | 9.6 | no |
| no_rpm_lag | 4.22 | 7.88 | yes |
| no_bw filters | 0.13 | 9.6 | no |
| tau=0 | 0 | — | no |
| tau=44, **dead=0** | **0** | — | **no** |
| tau=44, **dead=2** | **4.27** | **7.88** | **yes** |
| kr ceiling (bisect) | ~**1640** | — | (harness) |

**Necessary set (minimal):** **actuator lag τ≈44 ms + ≥2 ms command dead time** (at 1 kHz). RPM 1-sample base lag **not** necessary (LC without it).

Output: `experiments/analysis/out/indi_harness_bisect/bisect.json`.

---

## 3 — Element presence: harness vs SIL vs hardware

| Element | Harness | Full SIL (default) | Hardware estimate | Evidence |
|---|---|---|---|---|
| Motor lag τ≈44 ms | yes | yes (`motor_tau=0.044`, plant) | yes (bench §16) | HIGH bench |
| **Command dead ~2 ms** | **2 ticks @ 1 kHz** | **0** (torque/RPM applied same step) | **~2–4 ms** extra phase (investigation §16 DShot/ESC) | MEDIUM doc |
| Stale RPM vs gyro | `rpm_delay[1]` | current `state.rpm` → `oot_set_rpm` | logs: τ vs α **~1 ms** (doc 59) | MEASUREMENT |
| Gyro LPF 80 Hz | no | no in SIL | BMI088 ~80 Hz | MEDIUM firmware |
| 500 Hz output hold | optional harness | no | yes (Rust INDI hold) | MEDIUM code |
| Amplitude-dependent ESC | no | no | §16 describing function | MEDIUM investigation |
| Downwash / position | no | NS2 yes | yes | FACT |

---

## 4 — SIL gap experiments (NEW `indi_harness_sil_gap_*.py`)

Compiled controller, A1 stack, `motor_tau=0.044`, independent toggles from Step 3.

| Gap config | Ours gyro RMS | f_dom | Omar gyro |
|---|---:|---:|---:|
| none | 0.7 | 2.3 | 0.0 |
| **cmd_dead_ticks=2** | **233.4** | 7.43 | 0.0 |
| rpm_lag_samples=1 | 0.7 | 2.3 | 0.0 |
| dead2 + rpm_lag1 | 182.2 | 6.86 | 0.0 |
| combo + spool 1.25 | 196.6 | 6.86 | 0.0 |

**SIMULATION:** **2 ms plant command delay alone** raises gyro to **flight-order** magnitude; **does not** match frequency band or z error (−36 cm in that run). **Not** full baseline validation.

**RPM delay/hold alone (doc 59):** insufficient — consistent with bisect (dead time dominates).

Outputs: `out/indi_harness_bisect/sil_gap_results.json`, `sil_gap_summary.md`.

---

## 5 — Grid on “validated” plant

**Skipped as validated.** Partial gyro match only; **no** illustrative full doc-58 grid committed here (would need `cmd_dead_ticks=2` + frequency tuning — **would be tuning**, excluded by charter).

---

## 6 — Consequences & lab test

**(i) Harness faithful?** **No** for compiled path — dead-time/stale-base coupling.

**(ii) Necessary for harness LC:** τ + **command dead time** (not RPM log delay at 1 ms scale).

**(iii) SIL missing:** primarily **plant-side command latency** vs **instant** measured RPM feedback.

**(iv) Flight reproduction:** **Partial** with `cmd_dead_ticks=2` (amplitude only).

**(v) “Linear 44 ms lag alone”:** **Refuted** in SIL and bisect (τ with dead=0 calm); investigation’s **effective 0.23∠−83° @ 6.3 Hz** is **amplitude-dependent / dead-time** narrative, **not** pure first-order pole.

**Smallest high-value measurement (text):** **Round-trip latency** from stabilizer torque command sample to effective rotor thrust change (bench: current command step + optical RPM / load cell), target **~2–4 ms** — separates SIL instant plant from hardware; **effort:** one bench session, **value:** explains harness vs SIL gap directly.

**Logging:** extend card with `indi.tau_*`, `power.m*`, `gyro.*` (patch in `docs/lab_prep_log_filter_params.patch`).

---

## Files

**Created:** `docs/60_INDI_Harness_vs_SIL_Bisection.md`, `flying_drone_stack/tests/indi_harness_bisect.rs`, `experiments/analysis/indi_harness_bisect_run.py`, `indi_harness_sil_gap_sim.py`, `indi_harness_sil_gap_run.py`, `out/indi_harness_bisect/**`.

**Read (not modified):** `indi_loop_rate_harness.rs`, `test_indi_actuator_lag.rs`, `lib.rs`, `crazyflie_sil.py`, docs 56–59, investigation §15–16.

**Modified existing files:** none. **Committed/pushed:** none.
