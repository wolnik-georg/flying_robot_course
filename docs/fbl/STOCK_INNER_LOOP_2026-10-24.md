# FBL with the Stock Rate-PID Inner Loop (`ctrlFbl.inner = 1`) — 2026-10-24

## Overview
- **What changed:** FBL's thrust + body rates (Eqs. 15, 9, 10) can now go into the stock Crazyflie rate PID, exactly as the paper's hardware did (Sec. 5.2), instead of our P-only Eq. 19 torque loop. Parameter `ctrlFbl.inner` (0 = as flown 10-24, default; 1 = stock).
- **Also new (both default off):** `ctrlFbl.abg_on_new` (α-β-γ estimator only on a new EKF velocity), `ctrlFbl.nn_en` (ROM + small force network, the paper's hardware residual; needs a weights file).
- **Gates:** host 1:1 against `controller_pid.c` bit-exact; `inner = 0`, S0–S3 bit-identical to the build before; simulation: lateral offset and yaw drift removed; bundle network term matches the authors' code.
- **Simulation result (preset 1, 44 ms motor lag):** with cf5's torque need, steady lateral offset 18 cm → 0.03 cm, max yaw 27° → 0.3°; stable 15–60 ms lag. The bundle gains (preset 0) are unstable above 27 ms lag with either inner loop.
- **Not done (needs you / the lab):** nothing flashed or flown; image built (`study_tree_v2_fbl_stockinner_candidate.bin`, sha256 `5b20fe3a…`); crazyswarm2 branch `study_FBL_stock` created and pushed 2026-10-24 (`ctrlFbl.inner 1`, preset 1, cd 0.55).

Evidence levels as in the hardware review: **V** verified, **S** strong (simulation / data), **H** hypothesis, **U** unknown.

---

## 1. Simulation: stock inner loop vs the flown one

![traces, constant torque](assets/stock_inner_2026-10-24/traces_const_torque.png)

![summary](assets/stock_inner_2026-10-24/summary_offset_yaw.png)

| preset 1, 44 ms lag | steady xy offset [cm] inner 0 → 1 | max \|yaw\| [deg] inner 0 → 1 | tilt max [deg] inner 1 | handover tilt [deg] inner 1 |
|---|---|---|---|---|
| no disturbance (A1 / A4 / A8) | 0.07 → 0.02 | 0.1 → 0.03 | 0.2–1.3 | 0.3–0.6 |
| constant torque | 18.1 → 0.02–0.03 | 26.5–26.9 → 0.19–0.29 | 0.2–1.3 | 0.6–1.0 |
| slowly varying torque (sign flips) | 18.6–19.0 → 0.22–0.27 | 9.2–10.2 → 1.23–1.28 | 0.3–1.3 | 0.8–1.1 |

- Plant: real firmware controller via the host bindings, actuation through the real mixer (`powerDistribution` → cap → force-linear uint16), 4 ms delay, motor lag, noise, takeoff from the ground under Omar with FBL switch-on at 0.15 m, NS2 v2 downwash (z only), ROM C_D 0.55. [S]
- **Gate 2 PASS** (stable everywhere; with torque: yaw < 2°, steady offset < 2 cm).
- Same result with the other inertia value (J of Omar's controller) in the plant. With the state held 4–8 ms as on the EKF: unchanged (offset 0.03–0.07 cm).
- Not reproduced in this plant (also not with `inner = 0`): the A1 0.8–1.0 Hz xyz oscillation and the A4 0.125 trips seen on hardware (no lateral / torque downwash model). [H for their cause]

![lag margin](assets/stock_inner_2026-10-24/lag_margin.png)

- Preset 1 with the stock loop: stable for motor lag 15 / 30 / 44 / 60 ms (44 ms = measured rotor time constant, `docs/09_Simulation.md`). [S]
- Preset 0 (bundle "Deployed" gains 56/38/48, 45/21.6/3): stable up to 27 ms, unstable from 30 ms (A4, A8; A1 at 44 ms) with the stock loop; with `inner = 0` unstable at 44 ms too. Not retuned (rule: no retuning beyond 1:1). [S]

---

## 2. What `inner = 1` does (sign and unit table)

| Step (each 500 Hz tick, `fbl_inner_stock()` in `controller_omar_indi.c`) | Stock `controller_pid.c` in rate mode | Ours |
|---|---|---|
| yaw setpoint | `attitudeDesired.yaw = capAngle(yaw + attitudeRate.yaw · dt)`, `yawMaxDelta` | `yaw_sp = capAngle(yaw_sp + deg(ω_z,des) · dt)`, same clamp |
| yaw attitude PID → yaw rate desired | `CorrectAttitudePID` (6 / 1 / 0.35) | same call; roll / pitch attitude PIDs reset as stock |
| rate PID | `CorrectRatePID(gyro.x, −gyro.y, gyro.z, rate_roll, rate_pitch, yaw_rate)` deg/s | `(gyro.x, −gyro.y, gyro.z, deg(ω_x,des), −deg(ω_y,des), yaw_rate)` |
| output | `control->yaw = −yaw`, `controlModeLegacy` | same |
| thrust | uint16 per motor, force-linear with battery compensation | `T / (4 · THRUST_MAX) · 65535` (THRUST_MAX 0.2 N) |
| zero thrust | zero commands, reset all PIDs and the yaw setpoint | same |
| switch-on | stock resets on the ground (thrust 0) | in the air at 0.15 m: reset all PIDs, yaw setpoint = current yaw, rate PIDs primed with the current gyro (else the D term on the measurement kicks); **only deviation** |

- Gains: stock CF21BL defaults (`pid_rate.*` 200 / 400 / 2.5, yaw 120 / 16.7; `pid_attitude.yaw_*` 6 / 1 / 0.35), not copied into `ctrlFbl`; FBL's `kr_xy`, `kr_z` and `J` are unused with `inner = 1`. In N m/(rad/s) equal to the authors' CF2 + upgrade defaults within 4 %. [V arithmetic]
- Logged torque (`ctrlOmarIndi.torque*`) with `inner = 1` = the torque the legacy commands produce (mixer inverse), so the existing analysis scripts keep working.

---

## 3. Gates

| Gate | What | Result |
|---|---|---|
| 1a | `test_fbl_inner_stock.py`: FBL `inner = 1` vs stock `controllerPid` (rate mode, same w_des and uint16 thrust), 5 sequences × 3000 ticks incl. yaw across ±180° and a gyro-tracking case with 86 % unsaturated ticks | **bit-exact** (int16 commands, thrust, 4 motors) [V] |
| 1b | `test_fbl_inner0_identical.py`: S0, S1, S2, S3, FBL `inner = 0` (preset 1 and 0, through the handover) vs the build before | **identical** sha256, 6 of 6 [V] |
| 1c | `test_omar_ns2_off_identical.py` (logged flights, Omar indi 0 / 3) | **identical**, 4 of 4 [V] |
| 2 | `stock_inner_sim.py` (78 runs) | **PASS** for preset 1 (§1); preset 0 unstable above 27 ms [S] |
| 3 | `make test`, `make test_law`, `make test_abg`, `make test_nn`, `make test_export` in `experiments/fbl/stage3_port/c/`; firmware copies of `fbl_law.*`, `fbl_residual.*` byte-identical to the C port | **ALL PASSED** [V] |
| 4 | network term (`fbl_nn_pair`) vs bundle `DownwashResidual._acceleration_nn_analytical_jvp`: sample checkpoint (9 neurons) and a 4-neuron net, 3000 states each; exported weights through the weak symbol | **PASS** (worst 0.22 of tolerance; as accurate as the bundle's float32 against float64) [V] |
| 5 | ARM image with `-Werror`, patch round trip | built (583 580 B, flash 56 %, RAM 76 %); `fbl_stock_inner_loop.patch` applies on a clean checkout after the series and reproduces the live tree [V] |

---

## 4. The other 1:1 items

- **α-β-γ at the measurement rate (`abg_on_new`)**: the flown estimator updates every 2 ms tick while the EKF velocity changes only every ~5.6 ms; on repeated samples it produces large acceleration / jerk spikes (host test: driving it every tick against the measurement-rate reference gives jerk errors up to 592 m/s³). `abg_on_new = 1` reproduces the bundle estimator driven at the measurement rate (`test_abg`, worst 0.60 of tolerance). In the simulation with the EKF-like hold it brings no benefit (offset 0.5 vs 0.05 cm). Default stays 0; which rate the authors used is U.
- **Inertia**: 2.395e-5 / 2.395e-5 / 3.235e-5 = Busetto et al. 2025, measured on a CF2.1 Brushless (45 g with Flow + AI deck; `firmware_app/src/lib.rs` l. 230); 1.657e-5 / … = Förster's CF2.0 identification (Omar's source, `controller_lee.c`). The Busetto value is the right one for cf5 [S: same platform, not measured on cf5]. Matters only for `inner = 0` (P-only gain is J·kr). Defaults unchanged.
- **Yaw-rate input of Eq. 10**: stays `setpoint->attitudeRate.yaw` = the HLC's body ω_z (not ψ̇_ref); ≈ 0 for our plans. [V]
- **Preset 0** = the bundle values exactly (56/45, 38/21.6, 48/3, 238/140); mass and J stay ours. [V]
- **Force network (paper hardware: ROM + 4-neuron net)**: C evaluation + jerk in `fbl_residual.c` (inside the ROM gate and fade, ours; the bundle has no gate), weights via `export_fbl_nn_weights.py` → `fbl_nn_weights.c` (weak symbol; absent = ROM only), switch `ctrlFbl.nn_en`. Training: `stage4_knode/knode_train.py setup_ode(..., hidden_f=4, train_torque_net=False)`. **No trained weights yet**: the paper's data are FBL flights without the network (§5 card).
- **C_D**: stays 0.55 (our KNODE fit, 2026-10-22). The paper's value 0.236 comes from a manual closed-loop adjustment (Chee 2024: "until the nominal-ROM controller performs reasonably") with other ROM constants (ours is 15 % stronger per unit C_D); not transferable. Optional in the lab: their manual method = short C_D sweep with `inner = 1`, ROM only.

---

## 5. Data card for the force network (prepared, not flown)

| Item | Paper (Sec. 5.2) | Our card |
|---|---|---|
| controller | FBL without the network (ROM on) | FBL `inner = 1`, preset 1, cd 0.55, `nn_en = 0` |
| formations | static top + stacked, 0.4 m separation, 0.4 m/s | A3 (static top, bottom translates underneath) dz 0.4 speed 0.4; A2 (stacked, same path) dz 0.4 speed 0.4 |
| amount | 28 s (6 720 points) | ≥ 28 s in the gate: A3 × 2, A2 × 2 (each ~15–20 s) |
| OOD check | 0.3 m | A3 and A2 at dz 0.3, × 1 each |
| logs | – | uSD on both drones, log started before takeoff |

---

## 6. Cannot be 1:1 (documented)
- FBL on a laptop with Vicon 120 Hz and 400 Hz radio commands vs onboard 500 Hz with the EKF (~180 Hz effective state).
- Vehicle: CF21BL brushless, 42.7 g, larger J, vs CF2.1 + upgrade, 36 g.
- Their firmware version / Kconfig, N → uint16 map, exact gains and network weights (not published).
- Test cases: A4 jet edge and A8 crossing are outside their tested regime.
- Ours only: Omar below 0.15 m, in-air switch-on (rate PIDs primed), tilt latch 60°, body-rate limit 20 rad/s, thrust clamp, residual gate.

## 7. What is still open before the first flight, measured against the authors' bundle, training script and paper

**A. Lab steps (no code left):** flash the image on both drones, bench (`fbl_bench_check.py`: `inner`, `abg_on_new`, `nn_en` present; untested tool change), branch `study_FBL_stock`, readback must say `inner loop: STOCK rate PID`, uSD log started before takeoff, A1 → A8 → A4 (`LAB_PLAN.md` NEXT).

**B. Where our FBL still differs from what the authors flew**

| # | Item | Authors (bundle / paper / training script) | Ours now | Status |
|---|---|---|---|---|
| 1 | Control law (Eqs. 15, 9, 10, 17, 18, α-β-γ) | bundle code | same code in C (G3, bit-level tolerances); thrust law confirmed on the flight logs | **same** [V] |
| 2 | Inner loop | thrust + body rates to the onboard rate PID (Sec. 5.2) | `inner 1` = stock CF21BL rate PID, bit-exact vs `controller_pid.c` | **same structure** [V]; their interface (rpyt RATE / `manual`), firmware version, PID parameters and signs are not published [U] |
| 3 | Command path / timing | laptop, Vicon 120 Hz, commands over the radio at 400 Hz, rates held between packets | onboard 500 Hz from the EKF (~180 Hz effective) | **cannot be the same** (study setup) [V] |
| 4 | Position / integral gains | hardware gains not published; bundle "Deployed" 56/38/48, 45/21.6/3 | preset 1 (0.25× position loop) because preset 0 is unstable in our plant above 27 ms motor lag (ours 44 ms measured) | **largest remaining deviation** [S]; only the authors can say which gains flew |
| 5 | Force residual | ROM (their constants, C_D 0.236 by manual tuning) + 4-neuron force network trained on 28 s of their data | ROM table (our vehicle, C_D 0.55 from our KNODE fit), network slot ready but **no trained weights** | **open**: data card §5 → train (`knode_train.py`, Chee thesis App. B loop, `hidden_f=4`) → export → `nn_en 1` [V tools, data missing] |
| 6 | Training data recipe | "FBL unaugmented by learned residual dynamics", static + stacked, 0.4 m, 0.4 m/s | card: FBL `inner 1`, ROM on, A3 + A2 at dz 0.4 / 0.4 m/s, dz 0.3 OOD | whether their data had the ROM on is not stated [U] |
| 7 | ROM geometry | 30 g, arm 0.046, propeller radius 45 mm hard-coded | 42.7 g, arm 0.050, propeller radius assumed 45 mm | radius **unverified** (`STATUS.md`) [U] |
| 8 | α-β-γ rate | constants per update; their update rate (120 / 400 Hz) not published | per tick (500 Hz, as flown) or `abg_on_new 1` (measurement rate) | [U]; no difference in simulation |
| 9 | Yaw command | yaw of Eq. 10 sent as a rate (likely) | ω_z of Eq. 10 integrated into the stock yaw setpoint | what they sent as yaw rate is not published [U] |
| 10 | Thrust map | laptop N → uint16, battery handling unknown | force-linear uint16 with battery compensation | [U] |
| 11 | Start / stop | not published | Omar below 0.15 m, FBL switches on in the air (rate PIDs primed, integral starts there) | ours [V] |
| 12 | Safety layers | none published | tilt latch 60°, rate limit 20 rad/s, thrust clamp, residual gate | ours [V] |
| 13 | Test cases | static / stacked at 0.3 / 0.4 m | A1, A8 crossing, A4 jet edge | A8 / A4 outside their regime [V] |

**C. Not explained yet (may or may not go away with `inner 1`):** the A1 0.8–1.0 Hz xyz oscillation and the A4 0.125 trips of 2026-10-24 (not reproduced in simulation with either inner loop; the plant has no lateral / torque downwash). [H]

**D. What only the authors can settle** (no email decided 2026-10-22, so these stay documented assumptions unless that changes): items 2 (interface / parameters), 4 (hardware gains), 6, 7 (if from their vehicle), 8, 9, 10; their network weights and data; whether the 5 cm hardware RMSE is 3D or z.

**So "100 % the authors' controller" is reachable for the code and the inner-loop structure (done), not for the gains (unpublished, bundle gains unstable here), the trained network (needs our data card and training), and the laptop / Vicon timing (study setup).**

## Questions:
- First hardware flights with `inner = 1`: preset 1 (sim-stable at 44 ms) — agreed? Preset 0 is unstable at our motor lag in simulation.
- Make `inner = 1` the default after the bench / hover check?

## Next Steps:
- [ ] You: flash `study_tree_v2_fbl_stockinner_candidate.bin` (sha256 `5b20fe3a112066996ca44be7c6131531afb296217cd55c9a887f86886e4f4f7b`), bench, hover, then A1 / A8 / A4 (`FIRST_FLIGHT_CHECKLIST.md`, `LAB_PLAN.md`).
- [ ] Data card §5 → train the 4-neuron force net → export → `nn_en` flights.
- [ ] Optional: uSD swap for `ctrlFbl.yaw_sp` and the rate-PID integrators (`LAB_PLAN.md`).

Files: firmware patch `flying_drone_stack/firmware_patches/fbl_stock_inner_loop.patch`; host tests `flying_drone_stack/firmware_app/host/test_fbl_inner_stock.py`, `test_fbl_inner0_identical.py`; C tests `experiments/fbl/stage3_port/c/` (`test_fbl_law_abg.c`, `test_fbl_nn.c`, `test_fbl_nn_export.c`, generators `gen_law_vectors_abg.py`, `gen_nn_vectors.py`); simulation `experiments/fbl/stage3_port/stock_inner_sim.py`, plots `stock_inner_plots.py`; exporter `export_fbl_nn_weights.py`.
