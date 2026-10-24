> One-page state and resume guide: `PROGRESS_AND_RESUME.md` (2026-10-20).

# FBL onboard port — specification (Stage 3 preparation)

Created 2026-10-19. Status: **SPEC ONLY + one finished desk piece (the ROM lookup table).** No firmware file is changed, nothing is flashed or flown. Building the port beyond the table needs the user's and the supervisor's go (scope decision of Strategy 3, meeting 2026-10-22). Context and results so far: `docs/fbl/STATUS.md`, bundle analysis `docs/73_FBL_Bundle_Analysis.md`.

## 1. Goal
Strategy 3 on the drone: the authors' feedback-linearization law (thrust and body-rate/torque from acceleration and jerk commands, compensated by a downwash residual) running in the same firmware tree as S0–S3, using only what we have today: the bundle's inference side, our residual fit (Stage 1) and our data. The authors' training script and hardware gains, when they arrive, replace the fitted C_D / network and the gains; nothing else.

## 2. What maps onto the existing Omar C controller (code read 2026-10-19)
| FBL element (bundle `compute_fbl_command`) | Omar C controller (`controller_omar_indi.c`, 500 Hz tick) | Consequence |
|---|---|---|
| thrust = m (a_cmd + g e3 − a_res) · z_measured | thrust = m (a_d + a_indi) · R e3, with a_indi = a_rpm_f − a_imu_f (the NS2 slot is `a_d − a_nn`) | the same form: FBL's residual enters like `a_nn`, with C_D · a1(z0, r0) from the table |
| desired body rate from corrected jerk: ω_xy = e3 × Rᵀ m (j_cmd − j_res) / thrust, yaw rate reconstructed | `omega_des` from `setpoint->jerk` (flatness), `omega_des_dot` from `setpoint->snap` | jerk and snap exist in the setpoint; FBL needs the **feedback jerk** (Eq. 18 with the acceleration estimate) and the residual jerk subtracted |
| torque = J (K_rate (ω_des − ω) − α_res), K_rate = 238/238/140 | torque from attitude error (rotation error, `KR`, `Komega`, INDI moment term) | **different inner loop**: FBL is a pure rate loop, no attitude-error term. This is the main new code |
| position PD + integral (56/45, 38/21.6, 48/3), acceleration estimate (alpha-beta-gamma on velocity) | PD 7/4, z integral 1.5 | new gains, new estimator; the bundle's gains are for a 30 g vehicle in simulation |
| residual from two peers | `peer_get_all()` already used by NS2 (`ns2_residual.c`, per-peer velocity by differencing) | reuse the peer handling |

## 3. Architecture (proposal)
1. **`fbl_rom_table.c/.h`** (generated, **done**, `experiments/fbl/out/stage3/rom_table_v1/`): per-unit-C_D acceleration a1(z0, r0) and its two partials, 60 × 60 × 3 floats = 42.2 KiB flash. Grid: z0 0.05–2.0 m uniform, r0 0–0.6 m quadratic (the jet is narrow, ~0.1 m).
2. **`fbl_residual.c/.h`** (new): gate (neighbour above, z0 0.05–1.4 m with a linear fade from 1.2 m, r0 < 0.6 m), geometry from own and peer position/velocity, bilinear lookup, `acc_z = C_D · a1`, `jerk_z = C_D (∂a1/∂z0 · ż0 + ∂a1/∂r0 · ṙ0)`. Force-only, z only. Evaluated at the NS2 cadence (100 Hz) or every controller tick (cost to be measured; expected tens of µs). Reference implementation: `experiments/fbl/stage3_port/fbl_rom_runtime.py` (float32, same steps).
3. **FBL law** inside `controller_omar_indi.c` as an additive branch behind a parameter (`ctrlOmarIndi.fbl`, default 0). With `fbl = 0` the outputs must stay bit-identical to the current build (same proof obligation as for NS2). Alternative: a new controller slot (13); rejected as default because logging, filters, peers and the tree stay shared with S0–S3.
4. **Acceleration estimate** for the jerk command: the bundle's alpha-beta-gamma filter on the velocity (its constants 0.5/0.4/0.1 are per update, i.e. rate-dependent). Alternative: the onboard filtered IMU acceleration `a_imu_f` already in the controller. Open (D4).
5. **Parameters / logs** under the existing `ctrlOmarIndi` group plus `fbl.*` (C_D, gains, gate). uSD logging is at the 96-variable cap: new log variables replace unused ones or are added to the radio log only (to be budgeted).

## 4. Constants and assumptions to carry
- Vehicle: mass 0.0427 kg, arm 0.05 m, inertia `oot_inertia()`, k_thrust 3.91e-8. The bundle's ROM function takes the arm length from its own `quad_params` and divides by ITS mass (0.03 kg) in the residual class — **the table uses our mass explicitly** (found 2026-10-19: using the bundle class unchanged makes the ROM 1.42× too large; Stages 0–1 were unaffected, Stage 2 was rerun, see STATUS).
- C_D 0.43 (median of the Stage 1 folds, range 0.426–0.434); the paper's hardware value 0.236 is for a different vehicle/fit. Propeller radius assumed equal to the bundle's 45 mm (unverified).
- Rate loop ≥ 500 Hz: at 100 Hz the bundle's rate gain is marginal and a 10 ms position delay diverges (Stage 2).
- Motor headroom: INDI-commanded thrust is only 64–72 % delivered at crossings on our drones (`s3_indi_ns2_decision_2026-10-18.md`); FBL asks for stiff gains and may saturate the same way. Gains are tuned with a thrust limit in the simulation first.

## 5. Gates (pass numbers marked (you define) are not invented)
| # | Gate | State |
|---|---|---|
| G1 | table vs the authors' ROM: acceleration error, jerk error, gate edges | **PASSED 2026-10-19**: 3000 states, acc max error 0.005 m/s², jerk p99 0.03, max 0.14 m/s³ (steep r0 flank); table acceleration differentiated numerically vs table jerk: max deviation 0.24 m/s³ at a peak of 5.1 |
| G2 | C `fbl_residual` vs the Python reference, host build, same inputs | **PASSED 2026-10-19** (`experiments/fbl/stage3_port/c/`, `make test`): 3077 cases incl. gate edges, fade band, r0 = 0 / 0.6, neighbour below, 1–3 neighbours; tolerance acceleration 1e-5 + 1e-5·|ref|, jerk 1e-4 + 1e-5·|ref| (proposal, to be confirmed by you); worst error 2.2 % (acceleration, 7e-7 m/s²) and 3.3 % (jerk, 7.6e-6 m/s³) of the tolerance; peer-velocity differencing rules and the zero-output cases tested; compiles clean for the ARM target (1.5 KB code + the 42 KiB table) |
| G3 | C FBL law vs `compute_fbl_command` on logged and random inputs | **PASSED 2026-10-19** (`stage3_port/c/fbl_law.c`, `make test_law`; authors' code imported unmodified as the reference): 3000 random command cases (tilt up to 46°, thrust below the 0.01 N floor, yaw singularity) and 6 × 400 steps of the position law (alpha-beta-gamma estimate, integral, repeated time stamps); thrust error ≤ 1.2e-7 N, body rates ≤ 1.1e-2 rad/s (worst 23 % of the tolerance 1e-4 + 2e-5·|ref|), torque ≤ 4.6e-5 N m (69 %), acceleration command ≤ 6e-5 m/s², jerk command ≤ 7e-3 m/s³ (51 %); 20 cases within 0.01 of the yaw singularity (about ±90° tilt) are not compared — float32 and float64 legitimately differ there (denominator 1e-7 / 1e-2); ARM code 1.1 KB. Open: integration into the controller (G4) |
| G4 | `fbl = 0` bit-identical to the current Omar C build on logged flights | **PASSED 2026-10-19** (`host/test_omar_ns2_off_identical.py`, build before the patch vs. build with `fbl_integration.patch`, `ctrlFbl.en = 0`): 4 of 4 replay cases byte-identical (the same sha256 as in the NS2 gate) |
| G5 | closed-loop simulation of the C code in the Stage 2 plant: dip and tracking within a band of the Python result | **first step done 2026-10-19** (`stage3_port/c_in_the_loop.py`: the real firmware controller via the host bindings, rigid-body plant of our vehicle, A8 under a scripted top vehicle, NS2 v2 as downwash truth): crossing dip at 0.1 / 0.3 / 0.5 m/s: Omar geometric (`fbl.en 0`) −22.9 / −10.8 / −7.1 cm; FBL without residual −3.7 / −1.9 / −1.6 cm (Python Stage 2: −3.7 / −1.9 / −1.7); FBL + ROM residual (C_D 0.43) −0.60 / −0.34 / −0.41 cm (Python: −0.67 / −0.34 / −0.41). Agreement with the Python result within 0.1 cm; band to be confirmed by you. Still to do: gain tuning with delay, noise, thrust limit and motor lag; handover at takeoff/landing |
| G6 | bench (timing, `check_active_controller.py`, params), then solo hover, then A8 / A1 in the tree as S4 | needs the go; lab |
| G7 | stock rate-PID inner loop (`ctrlFbl.inner 1`, paper hardware Sec. 5.2) | **PASSED on the desk 2026-10-24** (`STOCK_INNER_LOOP_2026-10-24.md`): bit-exact vs stock `controllerPid` rate mode (`host/test_fbl_inner_stock.py`, 5 × 3000 ticks); `inner 0` / S0–S3 bit-identical (`host/test_fbl_inner0_identical.py`, `test_omar_ns2_off_identical.py`); simulation with the real stock PID (`stage3_port/stock_inner_sim.py`): preset 1 stable 15–60 ms lag, yaw < 2°, steady offset < 2 cm with torque disturbances. Lab: open |
| G8 | force network (ROM + small net, paper hardware) and α-β-γ at the measurement rate | **PASSED on the desk 2026-10-24**: `make test_nn` (bundle `_acceleration_nn_analytical_jvp`, 9- and 4-neuron nets), `make test_export` (weights through the weak symbol), `make test_abg` (bundle estimator on new samples only); trained weights: open |

## 6. Steps and effort (desk unless stated)
1. Spec + table — **done** (this document, 0.5 day).
2. `fbl_residual.c` + host test G2 — **done 2026-10-19** (files in `experiments/fbl/stage3_port/c/`: `fbl_residual.h/.c`, `test_fbl_residual.c`, `gen_test_vectors.py`, `Makefile`; not in the firmware tree).
3. **Integration done 2026-10-19:** `firmware_patches/fbl_integration.patch` (additive branch in `controller_omar_indi.c`, parameters and logs `ctrlFbl.*`, handover at z_on 0.15 m / z_off 0.10 m, tilt latch 60°, finite check, rate limit 20 rad/s, thrust limit) applied to the firmware tree, host bindings rebuilt, ARM image builds clean (-Werror, 584 104 B, flash 57 %, RAM 76 %, candidate `study_tree_v2_fbl_candidate.bin`, sha256 `a96ad225…`, not flashed). FBL law branch + test G3/G4 — 4–5 days (largest piece: rate loop without the attitude-error term, yaw reconstruction, thrust floor, safe start/landing behaviour). **Law as a standalone C module with host test (G3) done 2026-10-19** (`fbl_law.c/.h`); remaining: the integration branch in `controller_omar_indi.c` behind the `fbl` parameter, the bit-identical check with `fbl = 0` (G4), limits/saturations of the commanded rates and the safe start/landing.
4. Simulation of the C code, gain tuning with delay, noise and thrust limit (G5) — 3–4 days.
5. ARM image, bench, hover, A8/A1 (lab, G6) — 1 week with the lab slot. Total ≈ 3–4 weeks of work after the go, sooner if the authors send hardware gains and their network.

## 6b. Notes from the law port (2026-10-19)
- The bundle integrates the position error with `t − 0` at its first call (it assumes the clock starts at 0); the C port uses 0 at the first call (identical when the clock starts at 0, safe otherwise).
- Near thrust ≈ 0 the rate inversion saturates at the 0.01 N floor and the commanded rates become huge (hundreds of rad/s); near the yaw singularity (±90° tilt) the reconstructed yaw rate is ill-conditioned. Both are outside the flight envelope but the controller needs explicit limits on the commanded body rates and a safe behaviour at start and landing (open item of step 3).
- Not yet in the C law: the ESO / disturbance estimate option of the bundle (not used), torque residual (force only).

## 7. Open decisions (user / supervisor)
- D1 branch in controller 9 (`fbl` flag, recommended) or new slot 13.
- D2 ROM only first (recommended: the network adds nothing, Stage 1/2) or include a fitted `net_f` (tiny, trivial to add to C).
- D3 C_D: keep 0.43 (fitted on our label, which contains thrust-model error and ground effect) or refit from FBL flights once they exist. **2026-10-22:** KNODE on our logs with calibrated thrust gives ≈ 0.55 (0.65 with the raw command); **decided 2026-10-22: first flights at 0.55 per flight via `ctrlFbl.cd`, firmware default unchanged.**
- D4 acceleration estimator: bundle's alpha-beta-gamma (retuned for 500 Hz) or the controller's `a_imu_f`.
- D5 gates and fade (z0 1.2–1.4 m, r0 0.6 m): the fit data ends at z0 1.3 m.
- D6 **decided 2026-10-19 (user): Strategy 3 is part of the study**, so steps 2–5 are committed; target: FBL flying in the tree early enough to join the tier-1 study (deadline 26 Feb 2027).

## 8. Files
`experiments/fbl/stage3_port/` — `rom_table_lib.py` (table build + reference lookup), `fbl_rom_runtime.py` (reference of the onboard residual), `build_rom_tables.py` (generator + G1 test, writes `out/stage3/rom_table_<tag>/` with npz, `fbl_rom_table.c/.h`, `MODEL_CARD.json`, `report.md`; refuses to overwrite). The C files are NOT in the firmware tree.
