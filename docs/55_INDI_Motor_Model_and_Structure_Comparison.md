# INDI motor / thrust / torque model & structure comparison (investigation)

**Date:** 2026-10-04 · **Scope:** read-only code/config/log analysis. No root-cause claim.  
Builds on [`docs/41`](41_Pure_INDI_Implementation_Comparison.md), [`docs/53–54`](53_INDI_Loop_Rates_and_Oscillation_Ledger.md), investigation [`flying_drone_stack/docs/investigation_indi_oscillation_2026-07-21.md`](../flying_drone_stack/docs/investigation_indi_oscillation_2026-07-21.md).

---

## One-screen summary

| Topic | Ours (c=6, ctrl_mode=3) | Omar C (c=9) | Omar Rust (c=10) | Verdict (confidence) |
|---|---|---|---|---|
| **Attitude law shape** | Replaces geometric torque with **τ = τ_cur + j_scale·J·Δα** when bit1 set | **Geometric τ + additive** `(τ_rpm_f − J·α_f)` | Same as Omar C (Rust port) | **Large structural diff** (HIGH, code) |
| **Flown inner gains** | kr/kw **2400 / 170** [1/s², 1/s] on α chain | KR/Kω **0.007 / 0.00115** [Nm] on θ,ω | Same constants in `omar_indi_rust.rs:61–62` | **Not comparable units** (HIGH) |
| **J in INDI / gyro** | **23.951e-6** kg·m² (`lib.rs:232`, `j_scale=1`) | **16.572e-6** (`controller_omar_indi.c:60`) | Same as C in Rust `omar_indi_rust.rs:50–54` | **+45% roll/pitch J in ours** (HIGH) |
| **Mass in model** | **0.041** kg runtime (`g_indi_mass`, yaml) | **CF_MASS 0.0427** kg build (`app-config-bl:49`) | **0.0427** hardcoded Rust | **~4%** (HIGH) |
| **RPM→force** | Per-motor **kt1..4** ~4.1e-10 N/RPM² | Scalar **MOTORRPM2FORCE** | Same scalar `omar_indi_rust.rs:65` | **~4% mean vs scalar** (MEDIUM, calc) |
| **Arm / t2t / mixer** | ARM_M, TORQUE_RATIO → same mixer | ARM_LENGTH, THRUST2TORQUE | ARM, T2T same numeric | **No difference** at output (HIGH) |
| **Mixer path** | `controlModeForceTorque` | Same | Same | **Identical** (HIGH, `power_distribution_quadrotor.c:148–150`) |
| **Position residual sign** | **res_sign=+1** → **adds** a_res (`traj_iface.c:441`, `lib.rs:1929`) | **model−meas** → net **subtracts** meas−model (`controller_omar_indi.c:244–252`) | Same as C | **Opposite default** (HIGH) |
| **Loop rate** | **1 kHz** stabilizer, no hold | **500 Hz** `RATE_DO_EXECUTE` (`controller_omar_indi.c:156–158`) | Same 500 Hz `omar_indi_rust.rs:43–44` | **2× rate diff** (HIGH) |
| **A1 gyro activity (20 Hz radio, 6–13 s)** | **~311°/s RMS** (n=2) | **~130°/s** (n=2; τ log **zero**) | **~129°/s** (n=2) | **Ours ~2.4×** (MEDIUM, log) |
| **500 Hz uSD gyro (docs/54 batches)** | **4.6–5.7 Hz** peaks, high PSD | **~3.4–3.7 Hz**, low PSD | **~4.0 Hz** | **Ours louder, higher band** (MEDIUM, paired uSD) |
| **Linear nominal mode (illustrative)** | **√kr/(2π) ≈ 7.8 Hz** | **√(KR/J)/(2π) ≈ 3.3 Hz** geo core | Same as C | Aligns order-of-magnitude with flight/harness (MEDIUM, calc) |

**Bottom line (JUDGEMENT, MEDIUM):** Output **motor/thrust path is shared**; differences that plausibly matter for **attitude stability** are **law structure (replacement vs additive)**, **inner-loop gain scale/units**, **inertia used in INDI**, **position residual sign**, and **outer-loop stiffness** — not a different mixer or t2t map. **Nothing here proves** which single factor causes the A1 shake.

---

## 1 — Master comparison table

Evidence tags: **F**=FACT (file), **C**=CALCULATION, **J**=JUDGEMENT, **A**=ASSUMPTION.  
Reference columns **Stock INDI** / **NA-INDI** from [`docs/41`](41_Pure_INDI_Implementation_Comparison.md) only (not re-flown on CF21BL).

### 1.1 Physical & thrust model constants

| Row | Ours (c=6) | Omar C (c=9) | Omar Rust (c=10) | Stock / NA (ref) |
|---|---|---|---|---|
| Mass [kg] | **0.041** `g_indi_mass` / yaml `crazyflies.yaml:651` | **CF_MASS** from build **0.0427** `app-config-bl:49` → `controller_omar_indi.c:54` | **0.0427** `omar_indi_rust.rs:49` | NA 0.034 hardcoded; stock n/a |
| Jxx,Jyy,Jzz [kg·m²] | **23.951e-6**, **23.951e-6**, **32.347e-6** `lib.rs:232–236` (`drone_bl`) | **16.572e-6**, **16.656e-6**, **29.262e-6** `controller_omar_indi.c:60` | Same as C `omar_indi_rust.rs:50–54` | NA uses CF2.1 nano values |
| j_scale | **1.0** flown meta / yaml `655` | n/a (fixed J in C struct) | n/a | — |
| Arm [m] (√2/2·L) | **0.0353553** `lib.rs:260` | **0.707×0.050** `controller_omar_indi.c:367`, `platform_defaults_cf21bl.h:46` | **0.035355** `omar_indi_rust.rs:66` | — |
| Thrust→torque ratio [m] | **0.0056927884** `lib.rs:270` | **THRUST2TORQUE** same `platform_defaults_cf21bl.h:68` | **T2T** same `omar_indi_rust.rs:67` | — |
| RPM→force | **kt_i·rpm²** per motor `lib.rs:1781–1784`; kt≈4.1e-10 meta | **MOTORRPM2FORCE·(ω_rad/s)²** `controller_omar_indi.c:186–189`, scalar `platform_defaults_cf21bl.h:66` | Same formula `omar_indi_rust.rs:243–245` | NA per-motor κ_f; stock no RPM |
| THRUST_MAX / motor [N] | **0.2** CF21BL `platform_defaults_cf21bl.h:81`; upgrade kit `app-config:26` | Same firmware | Same | — |

### 1.2 Thrust / torque chain (INDI internals)

| Row | Ours | Omar C / Rust | Notes |
|---|---|---|---|
| τ from RPM (roll) | `ARM_M·(F3+F4−F1−F2)` `lib.rs:1487–1490` | Same pattern `controller_omar_indi.c:368–371` / `omar_indi_rust.rs:463–467` | **F: same mixer inverse** |
| τ_current base | RPM² unless `act_tau>0` or `ff_free` `lib.rs:2103–2110` | Filtered **τ_rpm** only when `indi&2` | Ours: optional act_dyn; Omar: always RPM when bit set |
| Measured angular accel | α from ω diff + **fc_bw=206** BW (and optional notch) `lib.rs:1991–2013` | α from ω diff, **30 Hz** BW on α `controller_omar_indi.c:378–386` | **Different filter depth** |
| INDI torque combine | **τ = τ_cur + j_scale·J·(α_ref−α_meas)** replaces geo path `lib.rs:2174–2213` | **u += τ_rpm_f − J·α_f** on top of geo `controller_omar_indi.c:391–392` | **Large structural F** |
| α reference | **α_des − kr·eR − kw·eω** (kr=2400 flown) `lib.rs:2068–2071` | No equivalent α_ref; geo uses **−KR·eR − Kω·eω** `controller_omar_indi.c:357–362` | Different formulation |
| RPM source | **DShot** flown `rpm_source=1` meta | `rpm_get_all` / log vars; deck probe in C | **F** |
| RPM guard | Optional, **default off** `lib.rs:580` | None in Omar C | — |
| Position a_res | **a_meas − a_model**; use **+res_sign·a_res** default **+1** `lib.rs:1787–1929` | **a_rpm_f − a_imu_f** (model−meas) `controller_omar_indi.c:244–252` | Net: Omar **subtracts** (meas−model) **F** docs/41 |

### 1.3 Output & power distribution

| Row | All variants using SI force/torque |
|---|---|
| controlMode | **controlModeForceTorque** — ours `lib.rs:2590`; Omar `controller_omar_indi.c:400` |
| Mixer | **Identical** `power_distribution_quadrotor.c:95–117` — per-rotor force, clip at 0, scale THRUST_MAX |
| Saturation | **Uniform scale-down** (preserve mix) `power_distribution_quadrotor.c:160–189` — roll/pitch/yaw not prioritized separately |
| Battery thrust comp | Optional Kconfig; same firmware tree — **no variant-specific fork found** |

**JUDGEMENT (HIGH):** Any torque command in Nm that reaches `controlModeForceTorque` sees the **same plant gain g≈1** through the mixer (until motor clip). Differences are **inside** each controller’s τ estimate and **gain/law**, not a hidden Omar-only thrust map.

### 1.4 Law structure & flown toggles (ours)

| Feature | Flown Oct-02 A1 (meta + yaml) | Source |
|---|---|---|
| ctrl_mode | **3** (full INDI) on ours flights meta | CSV `# meta:ctrl_mode=3` |
| kr/kw | **2400 / 170** | meta + `crazyflies.yaml:600–603` |
| kr_geo/kw_geo | **0.01 / 0.0011** (geo only) | meta |
| fc_bw, filt_dt_us, filt_prewarp | **206, 1000, 1** | yaml `640–642`; docs/54 validation |
| filt_order, filt_tau, dt_usec | **1, 1, 1** | yaml `666–680` |
| res_fc, res_clamp | **80, 10** | yaml `645–646` |
| res_sign | **+1** (default, not in meta) | `traj_iface.c:441` |
| notch_en | **0** (default; meta f0/bw logged) | yaml `779` |
| ff_free, act_tau | **0, 0** | yaml `660`, `367` |
| clamp_en | **11** | yaml `735` |
| pos kp/kv | **64/48, 5/7** + **ki_z=16** | meta; yaml |
| Omar indi bitmask | **ctrlOmarIndi.indi=3** / **ctrlOot5.indi=3** | yaml `205–220` (required for INDI paths) |
| Attitude KI | **Off** ours (`KI_ATT=0` `lib.rs:516`) | Omar **KI=0.03 on** `controller_omar_indi.c:73` |

**Unrecorded in Oct-02 radio meta (LOW):** `filt_dt_us`, `filt_prewarp`, `notch_en`, `res_sign` — see [`docs/lab_prep_log_filter_params.patch`](lab_prep_log_filter_params.patch) (not applied).

### 1.5 Omar-only / ours-only (high level)

| | Ours only | Omar only |
|---|---|---|
| | Snap/jerk flatness α_des; Z integral; vel glitch guard; optional notch/BW on τ & α; RNN hook; separate kr_geo | Full Lee **−J(ω×ω_r − R^T R_des ω̇_des)** term always; **500 Hz hold**; scalar MOTORRPM2FORCE; attitude **KI** |

---

## 2 — Control effectiveness (plant gain g)

**CALCULATION** (`experiments/analysis/out/indi_model_compare/plant_constants.json`):

| Quantity | Roll / pitch | Yaw | Thrust |
|---|---:|---:|---:|
| Mixer scale (τ_cmd → force diff) | **ARM ≈ 0.03536 N/Nm-equivalent** | **T2T ≈ 0.00569** | **0.25·thrustSi per motor** |
| g vs Omar at same SI τ_cmd | **1.00** | **1.00** | **1.00** |
| RPM model at hover RPM | mean **kt** vs **MOTORRPM2FORCE** → **−4.2%** force at same RPM | same | — |
| INDI **increment** scale (J used) | **J_ours/J_omar ≈ 1.45** on roll/pitch | **Jzz ratio ≈ 1.11** | n/a |

**FACT:** Per-motor **kt spread** on flown config ≈ **±1.3%** around mean (`meta kt1..4`).

**JUDGEMENT (MEDIUM):** No **>10–20%** mismatch at the **mixer output** between variants. The **>10%** effect appears in **internal INDI feedback** (ours applies **~45% larger** torque increment per rad/s² error than Omar’s **J·α** term would, if errors were equal — **A:** same α error).

**Does NOT show:** ESC nonlinearity, prop flex, or downwash changing physical g in flight.

---

## 3 — Common-units loop comparison (linear, illustrative)

Script: `experiments/analysis/indi_model_compare_loop.py` → `out/indi_model_compare/loop_margins.json`, `fig_loop_ol_bode.png`.

**ASSUMPTIONS:** single-axis roll; τ_act=**44 ms**; **2 ms** delay; small-angle; Omar model = **geometric core only** (additive INDI path omitted); Butterworth at flown cutoffs.

| Variant | Nominal ω_n | ζ (nominal) | Nominal mode freq | Investigation cross-check |
|---|---:|---:|---:|---|
| Ours flown kr=2400, kw=170 | **49.0 rad/s** | **1.74** | **√kr/(2π) ≈ 7.80 Hz** | Host harness ~**7.88 Hz** (`docs/54`) |
| Ours kr≈**800** (sim ceiling cite) | 28.3 rad/s | 1.74 | **≈ 4.50 Hz** | Investigation §16 **kr_max≈800** with lag (sim) |
| Omar geo KR=0.007, J=16.57e-6 | 20.6 rad/s | 0.082* | **≈ 3.27 Hz** | *ζ from KR,KW,J — geo heavily rate-damped in flight |

**CALCULATION:** Flown **kr=2400** is **~3×** the investigation’s **sim-derived ~800** stable ceiling (`investigation_indi_oscillation_2026-07-21.md` §16) — consistent with **limit-cycle** near **5–8 Hz** when lag lowers the realized frequency below √kr/(2π).

**INCONCLUSIVE:** Open-loop Bode **phase margins** from the script are **not reliable** (phase wrapping); use **nominal frequencies** and sim/flight only.

**Does NOT show:** Full SO(3) coupling, position loop, downwash, or Omar’s **parallel** INDI path.

---

## 4 — Log evidence (Oct-02 A1, no new flights)

### 4.1 Radio CSV (20 Hz), steady **6–13 s** after file start

Source: `experiments/analysis/out/indi_model_compare/log_stats_a1_oct02.json` (n per variant as flown).

| Variant | n | gyro RMS [deg/s] | |τ| max mean [Nm] | a_res RMS [m/s²] | Notes |
|---|---:|---:|---:|---:|---|
| Omar C | 2 | **130** | **0** (log dead) | **0** | **F:** τ columns zero — telemetry gap docs/41 §13 |
| Omar Rust | 2 | **129** | **0.008** | **2.54** | τ telemetry present |
| Ours c=6, mode 3 | 2 | **311** | **0.046** | **2.60** | **~5× Omar τ amplitude** vs Rust |

**JUDGEMENT (MEDIUM):** **Roll/pitch std** similar across variants (~5–7°) — oscillation shows in **rates**, not necessarily large mean attitude error. **Thrust column** near zero in these logs (normalized setpoint field) — **not** useful for saturation study.

### 4.2 500 Hz uSD (batch-level, docs/54)

| Batch | Variant | Gyro-x peak band | Comment |
|---|---|---|---|
| 18:40 | Omar C | **~3.4–3.7 Hz** | LOW per-flight pairing |
| 18:53 | Omar Rust | **~4.0 Hz** | MEDIUM pairing |
| 19:18 | Ours | **~4.6–5.7 Hz**, high PSD | LOW/MARGINAL pairing |

**FACT:** **~24–26%** of 2–20 Hz gyro energy in **5.5–7.5 Hz** for ours vs **~3–6%** Omar (`usd_spectrum_paired.json`).

**Missing for ours on these uSD configs:** trustworthy **`indi.tau_*`** (only `ctrlOmarIndi.*`); **RPM channels** not analysed in this pass — **effective g from RPM↔τ cross-check omitted**.

### 4.3 A1 downwash / saturation / windup

| Check | Result | Confidence |
|---|---|---|
| Motor clip from logs | **Not established** (no motor PWM/RPM in radio CSV) | LOW |
| τ clamp hits (ours max ~0.05 Nm vs limit 0.045 yaml) | **Near or at τ_xy clamp** on ours | MEDIUM |
| Omar τ | **Well below** clamp | MEDIUM |
| Position I windup | Omar **KI_pos=0**; ours **KI_P=0.05** + **ki_z=16** | **Different** outer integral — **J:** downwash may pump **a_res** on ours with **res_sign=+1** |

---

## 5 — Ranked differences (plausible stability effect)

| Rank | Difference | Size | Evidence |
|---|---|---|---|
| 1 | **Attitude law:** full **τ replacement** + Tal **α** loop vs **geometric + small additive INDI** | **Large** | `lib.rs:2174–2213` vs `controller_omar_indi.c:357–392` — **HIGH** |
| 2 | **Inner gain scale:** flown **kr=2400** vs investigation **~800** ceiling with lag | **Large** | Investigation §16; meta — **MEDIUM** |
| 3 | **Position loop:** **stiff KP/KV (64/48)** + **res_sign=+1** vs Omar **7/4** and subtractive residual | **Large** (outer) | yaml/meta vs Omar C — **MEDIUM** |
| 4 | **J in INDI increment (+45% roll/pitch)** | **Medium (~15–45%)** | `lib.rs:2204–2207` vs Omar J — **HIGH** |
| 5 | **Inertia / mass mismatch Omar model vs brushless** | **Small–medium** | CF21BL build uses Omar’s **nano J** — **HIGH** but Omar **flies calm** → **J alone does not explain** — **J** |
| 6 | **RPM→force scalar vs per-motor kt (~4%)** | **Small** | `plant_constants.json` — **MEDIUM** |
| 7 | **Mixer / t2t / arm** | **None** | Same `power_distribution_quadrotor.c` — **HIGH** |
| 8 | **1 kHz vs 500 Hz** (already studied docs/53–54) | **Small for shake freq** | uSD ~505 Hz all — **MEDIUM** |
| 9 | **Filter meta / notch defaults** | **Unclear** | Flown filt corrected; notch off — **MEDIUM** |

**Plain result:** **No single motor-map or mixer difference** explains “ours worst.” The strongest **code-backed** distinctions are **law architecture** and **gain/residual/outer-loop** stacking — still **not proof** of root cause.

### Open questions & bench tests (proposals only)

1. **SIL back-to-back:** same disturbances, swap only **law structure** (additive vs replacement) with matched bandwidth — **medium effort, high value**.
2. **Log `indi.tau_*` + motor RPM on uSD** for c=6 flights — **low effort**.
3. **res_sign=-1** and **kr sweep toward ~800** on hardware — **high effort**, needs safety protocol.
4. **Measure g(ω)** from chirp on bench (τ_cmd → gyro α) per variant build — **medium effort**.

---

## Relation to docs/53–54

- **Loop rate / filt_dt mismatch alone** — **does not** separate variants; **shared ~500 Hz logging** (docs/54).
- **500 Hz spectra** — **partially** resolve 5–7 Hz band; **Ours louder** than Omar in paired batches (**MEDIUM** pairing).
- **Motor model at mixer** — **this doc:** **no difference**; focus shifts to **control law & gains**.

---

## Files

**Created:**  
`docs/55_INDI_Motor_Model_and_Structure_Comparison.md`  
`experiments/analysis/indi_model_compare_plant.py`  
`experiments/analysis/indi_model_compare_loop.py`  
`experiments/analysis/indi_model_compare_logs.py`  
`experiments/analysis/out/indi_model_compare/plant_constants.json`  
`experiments/analysis/out/indi_model_compare/loop_margins.json`  
`experiments/analysis/out/indi_model_compare/log_stats_a1_oct02.json`  
`experiments/analysis/out/indi_model_compare/fig_loop_ol_bode.png`

**Read (principal):**  
`flying_drone_stack/firmware_app/src/lib.rs`, `traj_iface.c`, `omar_indi_rust.rs`,  
`~/Desktop/crazyflie-firmware/.../controller_omar_indi.c`, `power_distribution_quadrotor.c`,  
`platform_defaults_cf21bl.h`, `~/Desktop/crazyswarm2/crazyflie/config/crazyflies.yaml`,  
Oct-02 `experiments/logs/A1_cf5_*.csv`, `docs/41`, `docs/54`, investigation §16,  
`out/indi_loop_rates/usd_spectrum_paired.json`.

**Modified / committed:** **No existing file modified.** **No commit or push.**
