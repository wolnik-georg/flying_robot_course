# Plan — Omar C controller tree with NS2 on top (Topic 1), then NS2 retrain and data (Topic 2)

Created 2026-10-14. Parent document: `docs/plan_topics_2026-10-14.md` (decisions). Status: **Phase 0 (desk reading) and Phase 1 (spec: `docs/ns2_c_module_spec.md`) done; waiting for your read of the spec before Phase 2.** Confirmed 2026-10-14: the five proposals of §3.

## 0. Ground rules (do not break)
1. **Do not touch any other controller.** Not our controller 6 / `lib.rs`, not Omar Rust (10), not Tal & Karaman (11, 12), not the frozen ours (`docs/77`, tag `ours-indi-frozen-2026-10-13`). Their behaviour, parameters and flown results stay exactly as they are.
2. **Omar's numerical code is not modified.** The only change inside `controller_omar_indi.c` is a small additive hook that is a no-op when `rnn.en = 0`. Proof obligation: with `rnn.en = 0` the outputs are bit-identical to the current build on logged inputs.
3. All changes to the `crazyflie-firmware` tree are preserved as patches in `flying_drone_stack/firmware_patches/` (same workflow as the existing ones, README order list extended). Never fork the upstream tree.
4. The logging chain stays as is: same `usd` config, same variable names (`rnn.pred_x/y/z`, `rnn.*` params), now produced under controller 9 too. 96-variable cap respected.
5. No acceptance criterion is invented: the pass numbers below marked **(you define)** are placeholders until you set them. Nothing is flown before the desk gates pass.

## 1. What Phase 0 found (code facts, 2026-10-14)
**Omar C (`controller_omar_indi.c`, 584 lines, controller slot 9 = `controllerOutOfTree4`)**
- Single law with `indi` bitmask: bit 1 position INDI, bit 2 torque INDI; `indi = 0` is the geometric (Lee) controller. `Kpos_P` 7, `Kpos_D` 4, `KR` 0.007/0.007/0.008, `Komega` 0.00115/0.00115/0.002, `KI` 0.03 (attitude integral, in the law), `mass` = `CF_MASS`, filters 30 Hz (4 × Butterworth-2). `Kpos_Iz` (z integral) is our opt-in addition (default 0, bit-identical). `Kpos_P/D` and attitude gains are shared by `indi` 0 and 3 → shared position gains are already satisfied.
- Executes only every `ATTITUDE_RATE` = 500 Hz tick (`RATE_DO_EXECUTE`; main loop 1000 Hz), `dt = 1/500`.
- Position loop: `a_d = acc_d + Kv·ev + Kp·ep + Ki-term(z)`; INDI term `a_indi = a_rpm_f − a_imu_f` (= −measured residual); `thrust = m · (a_d + a_indi)·R e3`; desired rotation from `F_d = a_d + a_indi`. **NS2 enters here: `a_d − a_nn`.** Same place as `a_indi`, no other change.
- Needs the deck RPM (`deck.bcRpm 1`) for `indi` ≠ 0; geometric (`indi = 0`) does not.
- Params group `ctrlOmarIndi`, logs group `ctrlOmarIndi` (gains, rpy, errors, omega, `a_rpm_f`, `a_imu_f`, `tau_*`, `f_rpm`).

**NS2 as it is today (Rust, `residual_nn.rs`, called from `lib.rs`)**
- Deep-Sets: `phi_S` (6→25→40→40→20) for each neighbour, `phi_G` (4→25→40→40→20) ground effect (always on), `rho_S` (20→40→40→40→1); ReLU, last layer linear. Used weights 3 675 + 3 625 + 4 161 = **11 461** (the file layout of 19 297 also holds the unused L variants).
- Gate per neighbour: |dx| < 0.2 m, |dy| < 0.2 m, |dvx| < 1.5 m/s. Up to 3 neighbours, summed in `rho` input.
- Output: `faz` in grams → `· 9.81/1000 / mass` → acceleration, **z only**, clamp ±8 m/s²; non-finite → 0 with clamp flag.
- Peers: `peer_get_all()` already exists **in C** (`firmware_app/traj_iface.c`, uses `peerLocalizationGetPositionByIdx`); relative velocity by differencing against previous sample with timestamp (first-seen / unchanged / gap ≥ 500 ms rules in `lib.rs:rnn_predict_eval`).
- Params `rnn.*` (`en`, `ready`, `div`, `n/wi/wv/begin/end`, `rst`, `us_*`), logs `rnn.pred_x/y/z`, `rnn.clamped` are **already in C** (`traj_iface.c`). Evaluation decimation `rnn.div` = 10 → 100 Hz; prediction is always computed and logged, used only if `rnn.en = 1`.
- Weights flash-resident for study images (`residual_nn_flash`, built from `ns2_v1.npz`); old bench timing 504 / 533 µs per evaluation (Rust).

**Build / tree**
- Omar C and the firmware core live in `~/Desktop/crazyflie-firmware` (patched tree); our app (`firmware_app`: Rust lib + `traj_iface.c`) is linked into the same image. Core code can call app symbols (that is how `controllerOutOfTree*` works). Host SIL bindings already compile Omar C (`omar_indi_control`), `peer_get_all` is a host stub with injected peers.
- `collision_avoidance.c` is compiled into the firmware (relevant for Topic 2).
- Image today 498 KB; flash headroom to be confirmed from the map file in Phase 5.

## 2. Phases
| # | Phase | Output | Gate to the next phase |
|---|---|---|---|
| 1 | **Specify the C module** `ns2_c` (inputs, outputs, state, constants, weight header generator) | `docs/ns2_c_module_spec.md` (exact function signatures, units, frames, state layout) | you read it |
| 2 | **Write it:** `ns2_residual.c/.h` in `firmware_app/` (weights as `const float[]` in flash, generated from `ns2_v1.npz` by a script with sha256 in the header), per-instance peer history, gate, clamp, `mass` from the controller | module + generator script | compiles on host and ARM |
| 3 | **Hook into Omar C:** additive lines in `controllerOmarIndi` (evaluate every `rnn.div` main ticks, always log, `a_d − a_nn` only if `rnn.en = 1 && ready`); reuse `g_rnn_*` / `rnn_pred_write` from `traj_iface.c` so params and logs are unchanged; patch into `firmware_patches/` | patch + README order | `rnn.en = 0` ⇒ bit-identical outputs to the current Omar C on logged inputs |
| 4 | **Equivalence tests (host):** C vs Rust `ResidualNet` with the same weights and inputs (random relative states, gate edges, 0–3 neighbours, ground term, clamp, NaN), and vs the numpy reference (`neuralswarm.py` path, `docs/52`) | test script + report | max abs difference ≤ tolerance **(you define; I'd propose ~1e-5 relative, float32)** |
| 5 | **Closed-loop check (SIL, no hardware):** Omar C + NS2 in the existing two-drone harness (`ns2_closed_loop_sil_sim.py` with the Omar controller), sign negative control (flip sign must get worse), `rnn.en = 0` equals the old Omar C run | report in `docs/` | direction and size consistent with the ours + NS2 SIL (dip −5.5 → −1.0 cm for ours; expect the same direction, not the same number) |
| 6 | **Firmware build:** apply patches on a clean tree, host bindings build, ARM image, sha256, manifest entry for both drones, flash headroom, stack/timing review | image + manifest | builds clean from the patch list |
| 7 | **Bench (no flight):** `check_active_controller.py` shows "Using OutOfTree4 (9)"; `rnn.ready` 1; `rnn.us_max` per evaluation inside Omar's 2 ms slot; peers present ⇒ `rnn.pred_*` non-zero with the other drone placed in the gate; both cf5 and cf_second | bench log | timing and logging OK **(you define the timing limit)** |
| 8 | **Lab validation on the tree:** solo hover, A8, S0–S3 one flight each with `--study`; then freeze v1.0 of the tree | lab doc | clean flights, no instabilities |
| 9 | **Re-fly baselines** (this doubles as Topic 2 training data) | data | — |

Topic 2 follows (see §4). Thesis updates only after phase 8 (D-11 and Ch1/Ch4 per `thesis_revision/STATUS.md`).

## 3. Design decisions that belong to Phase 1 (proposals, not yet confirmed)
1. **NS2 evaluated and logged under every strategy of the tree (S0–S3), used only when `rnn.en = 1`.** Same as the current ours/NS2 behaviour, keeps the compute load identical across strategies (fair timing), and gives the predicted-vs-measured residual comparison on every flight.
2. **Weights flash-resident only** (no CRTP upload path in C): weights are frozen per image; a retrain means a regenerated header and a new image. Avoids ~77 KB RAM and the upload protocol.
3. **Evaluation rate:** keep 100 Hz, counted in main-loop ticks (`tick % 10 == 0`, Omar's controller already runs on even ticks) so `rnn.div` keeps its meaning.
4. **Mass:** the network output (N) is divided by the controller's own mass (Omar's `mass`, 0.0427), as `g_indi_mass` is in the Rust path.
5. **Same parameter and log names** (`rnn.*`) so `run_formation`, `upload`/preflight scripts and the uSD config need no change; the `upload_residual_weights.py` path is simply not used for the C tree.

## 4. Topic 2 — NS2 retrain and diverse data (after Topic 1 phase 8)
| # | Step | Detail |
|---|---|---|
| T2.1 | **Coverage definition (you define)** | dz × speed × lateral offset grid for the study settings (dz 0.25 / 0.5 / 0.75; speeds 0.2–0.5; offset 0 / 0.1) plus A8; v1 thin spots: dz 0.75, A8 |
| T2.2 | **Adapt the random-waypoint script** | from IMRCLab `random_flight.py`: two drones, our logging and geofence, `colAv.ellipsoidZ` ≈ 0.12 so dz 0.25 is reachable, goals of one drone biased toward the other's xy so the pair is in the gate more often than ~9 % of the time, controller from the yaml |
| T2.3 | **Safety review of the script** | `colAv` hooks the setpoint in `stabilizer.c`; check peers present, bbox inside our arena, abort rules, fresh batteries; a first run with a single drone / far apart |
| T2.4 | **Collect** | on the Omar tree: structured grid + random flights, S0 (`indi = 0`, no NS2) as the geometric data source; uSD from both drones |
| T2.5 | **Retrain** | training target = Omar's residual (`a_imu_f − a_rpm_f`, 30 Hz, deck RPM, mass 0.0427, his k_t); same architecture; adapt `run_c2_lab_retrain.py` labels; leave-one-out gate as before; the pass rule is **(you define)** |
| T2.6 | **Deploy** | weights → C header → image → flash → S2 and S3 flights |

## 5. Risks and checks
- **Timing:** one evaluation ~0.5 ms in Rust; Omar's slot is 2 ms (500 Hz) but the main loop is 1 ms; confirm worst case on the bench (phase 7) including the ground-term and 3 neighbours.
- **Sign:** NS2 prediction estimates the measured residual `a_meas − a_model`; Omar subtracts it. Checked by the SIL negative control (phase 5) before any flight.
- **Gate coverage with Omar's RPM deck:** the deck dropped motors in earlier 2-drone flights (RPM filter not active for Omar C); watch `rpm` quality in every bench and flight log.
- **Two-drone flight with the same controller on both:** cf_second also runs Omar geometric; NS2 is enabled only on cf5; downwash source dynamics change slightly relative to all earlier flights (cf_second was our geometric).
- **Host/ARM parity:** C module must behave identically in the bindings and on the ARM (float, no `-ffast-math` differences); covered by phases 4 and 7.

## 6. What I need from you before Phase 1
1. Confirm the four proposals in §3 (especially 1 and 2).
2. Tolerances and pass numbers marked **(you define)** — or tell me to propose them for your approval before each gate.
