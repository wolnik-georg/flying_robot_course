# 74 — Our INDI vs Omar C: the two control laws, term by term (2026-10-12)

Why this page: the input replay (`docs/64`, meeting 2026-10-12 §3) shows that the same logged inputs give a different thrust and different torques for our INDI and for Omar C. The two are built on the same idea, so the question is *where exactly* they differ. This is a reading of the code, not a new measurement; code references are given so every line can be checked.

- **Ours:** `flying_drone_stack/firmware_app/src/lib.rs`, `controller_step()` with `indi_gains.ctrl_mode = 3` (`stabilizer.controller 6`). Study configuration = the `all:` block of `crazyflies.yaml` plus `ctrl_mode 3`, `res_sign +1`, `ki_z 0`.
- **Omar C:** `crazyflie-firmware/src/modules/src/controller/controller_omar_indi.c` (`stabilizer.controller 9`, `ctrlOmarIndi.indi 3`, study value `Kpos_Iz 1.5`). Omar Rust (`controller 10`) is a line-by-line port of it (replay output differences 1e-7 / 1e-9).

## 1. What is the same
- Geometric core (Lee): desired thrust vector `F_d` from the position loop, desired attitude `R_d` with `z_b ∝ F_d` and the yaw direction `x_c`, attitude error `e_R = ½ (R_dᵀR − RᵀR_d)^∨` — identical formula (`lib.rs:1969`, `controller_omar_indi.c:316–318`).
- Thrust `T = m · (F_d · b_z)` with the current body axis `b_z` (`lib.rs:1948–1949`, `controller_omar_indi.c:268`).
- Both measure the unmodelled force and torque from the **motor RPM** (force `k · rpm²` per motor, moment through the same mixer: arm `√2/2 · 0.050 m`, yaw ratio 0.005693) and compare it with the accelerometer / gyro.
- Both feed forward the trajectory jerk and snap that the high-level commander puts into the setpoint.

## 2. Position loop → desired acceleration and thrust
Common part: `a_d = a_ref + g·ẑ + K_P (p_ref − p) + K_D (v_ref − v)`.

| | ours (`lib.rs:1875–1930`) | Omar C (`controller_omar_indi.c:209–277`) |
|---|---|---|
| position gains `K_P` / `K_D` | 64 (xy), 48 (z) / 5 (xy), 7 (z) | 7 / 4 (all axes) |
| position integral | none in INDI mode (`ki_z 0`; 16 with `ctrl_mode 3` caused the liftoff tumble) | z only, `K_I,z · ∫e_z`, study value 1.5 (default 0) |
| velocity error | XY low-pass 10 Hz + clamp | clamp only (±100) |
| residual force | `a_res = a_meas − a_model` (measured − model) | `a_indi = a_rpm − a_imu` (model − measured) |
| how it enters `F_d` | `F_d = a_d + s · a_res`, **`s = res_sign = +1`** → `a_d + (a_meas − a_model)` | `F_d = a_d + a_indi` = `a_d − (a_meas − a_model)` |
| mass in `T = m F_d·b_z` | 0.041 kg | 0.0427 kg (`CONFIG_MODIFIED_CF_MASS`, his brushless value) |

The sign of the residual term is **opposite**. Omar C subtracts the measured-minus-model residual (the textbook INDI increment, `m a = f_thrust + f_res + m g`). We add it: `res_sign +1` is the sign that flew the July campaign, `−1` (the textbook sign) diverged in hover on 2026-09-09 and was not isolated further (`lib.rs:1881–1923`, `docs/41` §3). The study rule keeps `+1` with `ctrl_mode 3` (`docs/study_parameter_decisions.md`).

How the residual itself is computed:

| | ours (`lib.rs:1780–1833`) | Omar C (`:245–260`) |
|---|---|---|
| model acceleration | `(Σ kₜ,ᵢ · rpmᵢ² / m) · b_z − g`, per-motor `kₜ` (mean 4.10e-10 N/RPM²) | `(f_rpm / m) · R ẑ − g`, one constant 3.91e-8 N/(rad/s)² = 4.29e-10 N/RPM² (4.5 % more) |
| measured acceleration | body accelerometer rotated to world (+ 206 Hz pre-filter) | `9.81 · state.acc` (estimator, world frame) |
| conditioning | norm clamp 10 m/s² and 80 Hz Butterworth on each side, then subtract | 30 Hz Butterworth (2nd order) on each side, no clamp |
| RPM source | DShot telemetry (with the invalid-reading fix) | optical RPM deck |

## 3. Attitude loop → torque
Both can be written as an **incremental INDI law**: `τ = τ_rpm,f + J (α_ref − α_f)` (measured torque from the RPM plus an increment that drives the measured angular acceleration `α_f` to the reference).

- **Ours** (`lib.rs:2062–2072, 2174–2217`): `α_ref = α_des − k_R e_R − k_ω e_ω`, `τ = τ_rpm,f + J (α_ref,f − α_f)`.
- **Omar C** (`controller_omar_indi.c:371–414`): `τ = τ_Lee + τ_rpm,f − J α_f` with `τ_Lee = −K_R e_R − K_ω e_ω − K_I ∫e_R + ω×Jω − J(ω̂ ω_r − RᵀR_d ω̇_d)`. This is the same incremental form with `J α_ref = τ_Lee`.

So the structure is the same; the differences are in the reference, the gains, the filters and the constants:

| | ours | Omar C |
|---|---|---|
| attitude reference `α_ref` | `α_des − k_R e_R − k_ω e_ω`; no gyroscopic term, no integral | full Lee moment: `−K_R e_R − K_ω e_ω − K_I∫e_R + ω×Jω −` rate feed-forward terms |
| attitude gains | `k_R = 2400 s⁻²`, `k_ω = 170 s⁻¹` (≈ `J·k_R = 0.057 N·m` per rad) | `K_R = 0.007 N·m` (= 420 s⁻² with his J), `K_ω = 0.00115 N·m·s` (= 69 s⁻¹) → ours is 5.7× stiffer in angle, 2.5× in rate (8× in torque units) |
| inertia `J` (kg·m²) | 23.95e-6, 23.95e-6, 32.35e-6 (Busetto 2025, brushless) | 16.57e-6, 16.66e-6, 29.26e-6 (the Crazyflie 2.0 system-identification values (Förster, ETH), hard-coded in his source) |
| angular-acceleration filter `α_f` | 2nd-order Butterworth, 206 Hz (rate filtered, then differenced; for a linear filter this equals differencing first) | 2nd-order Butterworth, 30 Hz (rate differenced with the measured `dt`, then filtered) |
| filter on the reference and on `τ_rpm` | the same filter on `α_ref`, `α_meas` and `τ_rpm` (matched phase) | 30 Hz on `τ_rpm` and `α`; the reference `τ_Lee` is **not** filtered |
| notch | optional 6.9 Hz notch (on in the 10-02 flight configuration used for the replay, off in the study configuration) | none |
| torque limits | `|τ_xy| ≤ 45 mN·m`, `|τ_z| ≤ 2.5 mN·m` | none |
| loop rate | every stabilizer tick, 1 kHz | `RATE_DO_EXECUTE(ATTITUDE_RATE)` = 500 Hz, output held in between |
| `τ_rpm` constants | per-motor `kₜ`, arm 0.0354 m, yaw ratio 0.005693 | one `MOTORRPM2FORCE`, arm `0.707·0.050`, `THRUST2TORQUE` 0.005693 (same mixer) |

> **Correction 2026-10-13:** the replay numbers quoted in this section used a harness without the estimator acceleration for Omar C; see `docs/76` (equal-parameter replay) for the corrected comparison: with equal parameters the thrust law of ours, Omar C and Tal & Karaman agrees to 0.005 N / 0.0013 N.

## 4. What this says about the replay result (`docs/64`, meeting §3)
- **Thrust, ours 19 % lower (0.333 vs 0.412 N), same shape and same peaks:** about one third of the level difference is the mass (0.041 vs 0.0427 kg; replaying ours with Omar's mass cuts the RMS difference from 0.080 to 0.052 N, `indi_replay_harmonize.py`). The rest comes from the law: opposite residual sign, different position gains and different residual filtering. These were not separated one by one.
- **Torques, different correlation (≈ 0.4):** the attitude reference, gain scale and filters differ (table above): ours is 5.7× / 2.5× stiffer, Omar C adds the gyroscopic and integral terms and filters only the measurement side. Which of these causes the 2.8× larger pitch torque of ours was not isolated.
- **Omar C = Omar Rust:** same code, hence the 1e-7 / 1e-9 differences.

## 4b. Direct check against Tal & Karaman (arXiv 1809.04048v2, `docs/papers/tal2018indi.pdf`, read 2026-10-12)
Paper frame is NED (`τ b_z` = specific thrust, `g i_z` points down); everything below is translated to our ENU frame. "Matches" = the same equation, "deviates" = a different formula or value. Paper gains are Table III ("platform-independent"), filter and rate from Section V.

| # | Paper (equation) | ours (`lib.rs`) | Omar C |
|---|---|---|---|
| 1 | position PD + acceleration feedback (17): `a_c = K_x e_x + K_v e_v + K_a (a_ref − a_f) + a_ref`; `K_x = diag(18, 18, 13.5)`, `K_v = diag(7.8, 7.8, 5.9)`, `K_a = diag(0.5, 0.5, 0.3)` | **deviates:** no `K_a` term; `K_P` 64/64/48, `K_D` 5/5/7 | **deviates:** no `K_a` term; `K_P` 7, `K_D` 4 |
| 2 | measured acceleration (16): `a_f = LPF(R a^b + g i_z)` (rotated into the inertial frame, then filtered) | matches in structure (`R·acc·g − g`, then 80 Hz); extra 206 Hz filter in the body frame before rotating | matches in structure (estimator acceleration) |
| 3 | thrust from motor speeds (5): `τ b_z` from `G1 ω∘2` (encoder speeds, filtered the same way as the acceleration) | matches in structure (RPM² · `kₜ`, DShot telemetry instead of encoders) | matches in structure (one force constant, deck RPM) |
| 4 | incremental thrust (20): `(τ b_z)_c = (τ b_z)_f + a_c − a_f`, i.e. `F_d = a_c + g ẑ − (a_meas − a_model)`: the residual is **subtracted** | **deviates: adds it** (`res_sign +1`) | matches (subtracts) |
| 5 | thrust magnitude (21): `T_c = −m ‖(τ b_z)_c‖₂` (norm of the vector) | **deviates:** projection `m (F_d · b_z)` on the current body axis | **deviates:** same projection |
| 6 | attitude command: incremental quaternion relative to the current attitude (22)–(26), error angles `ξ_e = 2 arccos(ξ_c^w)/√(1 − ξ_c^w²) · ξ_c^{xyz}` (27) | **deviates (small):** Lee error `e_R = ½ (R_dᵀR − RᵀR_d)^∨` (= sin θ instead of θ; identical for small errors) | same as ours |
| 7 | attitude PD (28): `Ω̇_c = K_ξ ξ_e + K_Ω (Ω_ref − Ω_f) + Ω̇_ref`; `K_ξ = diag(175, 175, 82)` s⁻², `K_Ω = diag(19.5, 19.5, 19.2)` s⁻¹; damping on the **filtered** rate `Ω_f` | form matches; **gains 2400 s⁻² (13.7×) and 170 s⁻¹ (8.7×)**, yaw 2400 / 170; damping on the **raw** gyro | **deviates:** Lee moment in torque units (≈ 420 s⁻², 69 s⁻¹, i.e. 2.4× / 3.5× the paper), raw gyro, plus gyroscopic, integral and rate terms |
| 8 | feed-forward references (14), (15): `Ω_ref`, `Ω̇_ref` from jerk and snap, including the yaw and `Ω×Ω` terms | **deviates (small):** `Ω_ref` from the high-level commander, `Ω̇_ref` with yaw and `Ω×Ω` terms dropped (zero for zero yaw rate) | his own flatness formulas with the cross terms |
| 9 | INDI torque (31): `μ_c = μ_f + J (Ω̇_c − Ω̇_f)`, no gyroscopic term, **`Ω̇_c` is not filtered**, `μ_f` filtered | form matches (`τ_rpm,f + J(α_ref − α_f)`); **deviates: `α_ref` is filtered too** (`alpha_ref_filt`) | reference not filtered (matches); but the reference is the Lee moment (gyroscopic and integral terms inside) |
| 10 | filters (Sec. V): 2nd-order Butterworth **30 Hz** on motor speeds and IMU, identical on paired signals | **deviates:** 206 Hz (res 80 Hz) | matches (30 Hz) |
| 11 | no integral action anywhere (III-B) | matches in INDI mode (`ki_z 0`) | **deviates:** attitude `K_I` 0.03 and, in the study, z integral 1.5 |
| 12 | motor control (32)–(36): encoders at ~5 kHz, nonlinear effectiveness inversion, integral motor-speed loop | not present (stock mixer, DShot telemetry) | not present |
| 13 | rates (Sec. V): control and IMU at 2000 Hz | 1000 Hz | 500 Hz |
| 14 | — (not in the paper) | extras: velocity-error 10 Hz low-pass, torque / thrust clamps, optional notch, residual clamp | none |

**Result:** our INDI is **not** an exact implementation of the paper. It follows the paper's attitude structure ((28) reference with feed-forward, (31) increment, no gyroscopic term, no integral in INDI mode) but deviates in the position residual sign (4), the filtered attitude reference (9), the filter cutoff (10), the gains (1, 7) and several smaller points (5, 6, 8). Omar C matches the paper on the sign (4), the unfiltered reference and the 30 Hz filter (10) and is closer in gain scale (7), but replaces the paper's attitude law by the Lee moment (7, 9, 11) and has no `K_a` (1). Neither follows the paper's motor-speed loop (12) or the 2 kHz rate (13). So the differences between the two implementations cannot be explained by "ours follows the paper, Omar's does not"; each deviates from it in different places, and rows 4, 9, 10 (sign, reference filter, cutoff) are the ones where ours is the one that deviates.

## 4c. Why ours is not the paper's controller (from the code and the commit history, not from anyone's recollection)
- **Built as "Lee geometric outer loop + INDI inner loop", not as a port of the whole paper.** The simulation controller (`src/controller/indi.rs`, April–May 2026, the template for the firmware) states it: the outer position loop is identical to the geometric controller, the inner INDI loop replaces only the torque law. That is why we have the thrust projection, Lee's `e_R`, no `K_a` term and no incremental attitude command.
- **Position INDI was added on top later** as `a_res = a_meas − a_model` (the thesis signal) with a runtime sign; the sign kept is the one that flew in July (`lib.rs:1881–1923`), not the paper's.
- **Filtering of `α_ref`** was added for phase matching (`lib.rs:2074`), following the stock Bitcraze `controller_indi.c` style; the paper filters measurements only.
- **Hardware limits:** no motor encoders, so the paper's motor-speed loop (32)–(36) cannot exist; 1 kHz loop instead of 2 kHz.
- **Simplifications:** `α_des` without the yaw and `Ω×Ω` terms.

## 4d. What would have to change to make ours the paper's controller (gains and filter cutoffs excluded)
In `lib.rs` `controller_step()` unless noted. Numbers refer to the rows of §4b.
1. **Residual sign and thrust command (4):** build the command from Eq. (20), `(τ b_z)_c = (τ b_z)_f + a_c − a_f`, i.e. `res_sign −1`, with `a_f` and `(τ b_z)_f` filtered by the identical filter after rotating to the inertial frame (remove the extra body-frame accelerometer pre-filter).
2. **Acceleration feedback (1):** add `K_a (a_ref − a_f)` to the position PD, Eq. (17).
3. **Thrust magnitude (5):** `T_c = m ‖(τ b_z)_c‖₂` instead of the projection onto the current `b_z`, Eq. (21).
4. **Attitude command and error (6):** incremental attitude `ξ_c` relative to the current attitude (22)–(26) and error angles `ξ_e` (27) instead of `R_d` and Lee's `e_R`.
5. **Feed-forward (8):** `Ω_ref` and `Ω̇_ref` from (14), (15) with the yaw rate / acceleration and `Ω×Ω` terms, using the filtered measured `τ_f`, `τ̇_f`, `ξ`, `Ω_f` (needs jerk, snap, yaw rate and yaw acceleration references).
6. **Damping (7):** use the filtered rate `Ω_f` in `K_Ω (Ω_ref − Ω_f)` (`ENABLE_GYRO_FB_FILTER`), Eq. (28).
7. **Reference (9):** do not filter `Ω̇_c` (remove `alpha_ref_filt`), Eq. (31).
8. **Measured torque (9):** `μ_f = G1 (ω∘2)_f + G2 ω̇_f` including the rotor-inertia term, Eq. (5).
9. **Remove what the paper does not have (14):** velocity-error low-pass, residual / torque / thrust / tilt clamps, optional notch, z integral. The clamps are safety limits on this vehicle; the paper handles saturation in its motor-level feasibility step (34).
10. **Not possible on this hardware (12, 13):** the encoder-based motor-speed loop with nonlinear effectiveness inversion (32)–(36) (optical encoders at ~5 kHz, direct motor-speed control, 2 kHz loop). A Crazyflie has DShot RPM telemetry and the stock mixer only, so "100 % the same" cannot be reached; items 1–9 are the closest feasible version.

Risk: with `res_sign −1` full-INDI hover diverged on 2026-09-09 (cause not isolated); any of these changes needs the simulation first and the C.0 hardware gate before it counts.

## 5. Not covered
- A step-by-step ablation (changing one term of ours to Omar's value and replaying again) — possible with the replay harness (`indi_replay_harmonize.py` already does it for the mass), not done.
- Closed-loop behaviour: the replay is open loop, it shows the two laws' outputs on the same inputs, not which one flies better.

## 6. Implemented: the isolated Tal & Karaman controller (2026-10-12) — `stabilizer.controller 11` (Rust), `12` (C)

Our INDI (6), Omar C (9) and Omar Rust (10) are **untouched**. Items 1–9 of §4d are a **new, separate controller**; item 10 uses DShot RPM instead of encoders and sends thrust + torque to the stock mixer.

| | |
|---|---|
| select | yaml `stabilizer.controller: 11` (Rust, `firmware_app/src/tk_indi.rs`) or `12` (C, `crazyflie-firmware/src/modules/src/controller/controller_tk_indi.c`); both read the same `ctrlTk.*` params and the RPM bridge of `traj_iface.c` |
| firmware | **new image** `firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk.bin` (sha256 `cb38f3f4981d8fa6b9c85c9e26d5645b5535040328d4ef99926a1c267037f8ca`) = the unified usd96 image with the filtered-RPM log (`rpmf`) + `CONFIG_CONTROLLER_OOT6/OOT7` (without the slots: `…usd96_rpmf_rnn_iz_sentinel_ON.bin`). Built in a scratch copy; the seven existing images are byte-identical to before (sha256 checked). Not flashed. Controller numbers 6–10 are unchanged |
| defaults | Table III gains (`ctrlTk.kx_xy 18, kx_z 13.5, kv_xy 7.8, kv_z 5.9, ka_xy 0.5, ka_z 0.3, kxi_xy 175, kxi_z 82, kom_xy 19.5, kom_z 19.2`), 30 Hz 2nd-order Butterworth (`ctrlTk.fc`), no limit of any kind inside the law |
| frames | the law runs in the paper's NED / FRD frames (`tk_indi.rs` header); inputs and outputs are converted from / to ENU / FLU; ENU yaw → NED heading `π/2 − ψ` |
| mass, thrust constant | `indi_gains.mass` and `indi_gains.kt1..4` (the same yaml values as our controller 6), arm length and torque ratio as in `lib.rs` |

**Equations → code.** (16) `a_f` = LPF of `R a^b + g i_z`; (5) `τ_f`, `(τ b_z)_f`, `μ_f = G1 (ω∘2)_f + G2 ω̇_f` from the DShot speeds; (17) PD + `K_a (a_ref − a_f)`; (20) `(τ b_z)_c = (τ b_z)_f + a_c − a_f`; (21) `T_c = m‖(τ b_z)_c‖`; (22)–(26) incremental attitude `ξ_c = ξ̄_c ∘ ξ_ψ`; (27) `ξ_e`; (14), (15) flatness feed-forward with the measured `τ_f, τ̇_f, ξ, Ω_f` (4×4 solve, `Ṡ` analytic); (28) `Ω̇_c` with the filtered `Ω_f`; (31) `μ_c = μ_f + J (Ω̇_c − Ω̇_f)`, `Ω̇_c` unfiltered, no gyroscopic term.

**What is still not the paper (all unavoidable or a guard, none changes the law):**
1. Motor loop (32)–(36): no encoders, stock mixer. Rate 1 kHz instead of 2 kHz (`dt = 1 ms`, filters designed for 1 kHz, derivatives are differences of the filtered signals, identical filter on paired signals).
2. Eq. (25) **as printed** gives the yaw quaternion `[1, 0, 0, −n̄¹/n̄²]`, which has half-angle tangent `tan φ`, i.e. it doubles the yaw angle error for small errors; it still converges to the reference. `ctrlTk.yaw_lit 1` (default) = as printed, `0` = half-angle form `φ = atan(−n̄¹/n̄²)`.
3. Guards only against division by zero: `‖s_c‖ → 0`, `|τ_f| < 0.5 m/s²` or singular 4×4 → no feed-forward (the matrix of (14)/(15) is singular at `τ = 0`, i.e. on the ground before the motors spin), `1 − i_z·v → 0` → rotation about x. Filters start at the first sample instead of 0.
4. No motor-speed source (`rpm_ok` 0) → IMU-only increment (`τ_f` from the filtered acceleration, `μ_f = J Ω̇_f`); logged in `ctrlTk.rpm_ok`.
5. Other setpoint modes than position (cfclient attitude, thrust) → motors off. Only the study's position setpoints are supported.

**Rotor inertia `J_rz` (G2 term, yaw row).** The paper has it, no number exists in our files. Estimated from our own flights: regression `J_zz Ω̇_z = a Σ s_i F_i + J_rz Σ s_i ω̇_i` (30 Hz filtered, DShot speeds) on the 2026-10-08 A8 logs, both drones, 6 flights: `J_rz ≈ 2.3–3.0e-8 kg m²` in 8 of 12 fits (the cf5 fits with a poor yaw correlation give 1–5e-9; the `a` coefficient of the same fit is biased by the telemetry delay, so this is an estimate, not a calibration). A geometric guess from bell + propeller gives 1e-7, 4× more. **Default `ctrlTk.jrz = 2.5e-8`.** Sensitivity in simulation (plant with 2.5e-8 reaction torque, noise on): controller value 0 … 5e-8 (0 … 2×) all fly with the same error; 1e-7 (4×) never reaches altitude. So the term is not critical within a factor 2; if the yaw channel misbehaves in the lab the first thing to try is `ctrlTk.jrz 0`.

**Verification.**
- Flatness (14)/(15): `experiments/analysis/test_tk_indi_reference.py` against a true trajectory (`Ω` from `RᵀṘ`, `Ω̇` and `τ̇` by differences): worst error 6e-8 rad/s²; this test found that the paper's yaw (heading of the projection of `b_x`) differs from the usual `x_c` construction.
- `experiments/analysis/tk_indi_reference.py` is a literal NumPy version of every equation above (float64). `flying_drone_stack/firmware_app/host/test_tk_indi_rust_vs_ref.py`: 600 ticks of time-varying inputs (all filters, differences, feed-forward): Rust **and** C vs the reference, worst thrust difference 1.0e-6 N and torque difference 4.3e-8 N m, both yaw variants (PASS).
- Closed loop in the SIL (`experiments/analysis/tk_indi_sil_run.py`, one drone, CS2 plant with 44 ms motor lag and the rotor reaction torque; position / gyro noise of the NS2 harness plus 0.03 g accelerometer and 60 rpm speed noise), **paper gains, no retuning**:

| run | rms xy | z offset | max tilt | |
|---|---|---|---|---|
| hover 12 s, no noise | 0.001 cm | −0.005 cm | 0.01° | Rust = C |
| A8 (31 s), no noise | 0.018 cm | −0.006 cm | 1.2° | |
| A1, no noise | 0.001 cm | −0.005 cm | 0.01° | |
| hover, noise | 0.24 cm | −0.05 cm | 2.9° | Rust 0.2414 = C 0.2414 |
| A8, noise | 0.22 cm | −0.015 cm | 5.5° | Rust 0.2220 = C 0.2220 |
| circle 0.4 m/s, noise | 0.22 cm | −0.017 cm | 5.1° | Rust = C |
| circle 0.8 m/s, noise | 0.26 cm | −0.040 cm | 12.0° | Rust = C |
| figure-8 (lemniscate) 0.4 m/s, noise | 0.22 cm | −0.024 cm | 6.2° | Rust = C |
| figure-8 (lemniscate) 0.8 m/s, noise | 0.21 cm | −0.015 cm | 11.7° | Rust = C |

(circle and figure-8 = scenario A2 with `--kwargs '{"path":"circle","speed":0.8}'`, `"lemniscate"`; an oval is not a path of the scenario library, not run.)

**What this does not say.** The SIL has one drone, no downwash, no estimator delay and a perfect acceleration signal; the first fits of the rotor-inertia term and of the thrust constant come from other controllers' flights. It shows that the law as implemented is correct and stable at the paper's gains, not that it flies on the Crazyflie. Open before a flight: the 2026-09-09 hover divergence with `res_sign −1` (cause never isolated, §4d), the thrust-constant error (`kt` ±14 % turns directly into a z offset, because this law has no integral action), the DShot speed latency (not compensated), and the C.0 gate (`docs/11`). First flight: hover, then A8, `ctrlTk.jrz 0` as the fallback.

**Hardware status (2026-10-13).** First flown on the Crazyflie today. (1) The first attempts did not run it: cf5 had the non-`_tk` image, so `stabilizer.controller` 11 and 12 were silently ignored and controller 6 with INDI position gains flew (the 1–1.5 Hz oscillation seen there is that ring-up). Check with `flying_drone_stack/tools/check_active_controller.py` that the console says "Using OutOfTree6 (11) controller" / "OutOfTree7 (12)". (2) After the reflash the C port (12) runs, but it flipped at takeoff in 2 of 2 hovers: pitch +31° and roll −18° within 0.16 s of lift-off, upside down at 1.2 s. Before any attitude error the torque command is −0.0023 N·m on x and y and grows while two motors spin up and two stay at idle, which equals the torque implied by the RPM asymmetry itself; the last frozen thrust command was 2.0 N (hover 0.4 N). Working hypothesis, not proven: no ground handling in the law (the RPM-based torque estimate and the measured angular acceleration disagree on the ground), no limits, no integral. Prepared, not flown: takeoff on the geometric ramp and switch at altitude. The rotor-inertia term (`jrz`) and the gains have not been tested in flight. Details: `lab_sessions/2026-10-13.md` §7.

**yaml block** (under `firmware_params:` of the drone; all other keys as in the study yaml):
```
stabilizer:
  controller: 11      # 12 = C version, same law
ctrlTk:                # everything below is the default
  kx_xy: 18.0
  kx_z: 13.5
  kv_xy: 7.8
  kv_z: 5.9
  ka_xy: 0.5
  ka_z: 0.3
  kxi_xy: 175.0
  kxi_z: 82.0
  kom_xy: 19.5
  kom_z: 19.2
  fc: 30.0
  jrz: 2.5e-08
  yaw_lit: 1
```
Needs `deck.bcRpm 1` or the DShot source like the other INDI controllers, and the `_tk.bin` image above.
