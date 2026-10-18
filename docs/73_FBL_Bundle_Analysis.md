# 73 — Learned-FBL code bundle (Hsieh et al. 2026): analysis — WORK IN PROGRESS

> **Update 2026-10-18:** own pipeline built on this analysis (Stages 0–2: definitions, residual fit on our 73-flight bank, closed loop); results in `docs/fbl/STATUS.md`.

**Status:** all phases done (2026-10-09). Read-only on the bundle (hashes verified at the end), one document, uncommitted until you say so. **Read-only on the bundle.** Source bundle: `~/Desktop/learned_fbl_core` (identical to `~/Desktop/learned_fbl_core.zip`), received from Pei-An Hsieh as a "standalone bundle of the core code" (simplified, not the repository). Paper: `docs/papers/hsieh2026flatness.pdf` (arXiv 2607.12275**v2**, 17 Aug 2026, 15 pp).

## Summary (for the supervisor)
- **What we received:** a standalone bundle (12 files, 2 888 lines, 114 learned parameters) with the *inference side* of the Hsieh et al. 2026 method: pairwise downwash residual (physics model + two tiny networks) with **analytic derivatives**, the three-equation FBL control mapping, an acceleration estimator, a sample **simulation** checkpoint and a 57-line example. **Not included:** training code/data, the hardware checkpoint (4-neuron force net, C_D 0.236), the hardware gains, the reference generator, any licence text.
- **Does the code do what the paper says? Yes.** The example runs (0.329 N), the analytic derivatives match finite differences (≤ 0.8 %), the control mapping matches an independent numerical derivation of the flatness maps (1e-11 rad/s), and a closed-loop test on my own plant reproduces the paper's ordering nominal > physics model > physics+network at ≥ 400 Hz control rate.
- **What it does *not* give us:** a drop-in controller for our drones. Everything is tied to a **30 g, Crazyflie-2.x geometry** (mass, inertia, propeller radius, jet constants hard-coded in three files); the sample network is a correction from the physics model to **one specific simulated plant** (it hurts if the plant matches the physics model); the gains are for a fast rate loop (**unstable at 100 Hz**, fine at ≥ 400 Hz) and give a **soft position loop** (the method relies on feed-forward).
- **Fit with our stack:** good. Same conventions (ENU, quaternion [x,y,z,w], SI). Our firmware already has the building blocks: flat-output trajectory with jerk and snap, `omega_desired` from jerk (Eq. 15 without residual), peer positions with differenced relative velocity (NS2), a 500–1000 Hz out-of-tree controller that outputs torque.
- **Port effort for Strategy 3 (estimate):** onboard port in our controller ≈ 4–6 weeks including retraining the residual on our data and lab validation — **does not fit before the first study session (Nov)**; options and a staged plan in §9. Cheapest honest path now: ask the authors for the missing pieces (draft email §10), prepare the table-based residual and a simulation-only baseline, decide with the supervisor whether Strategy 3 enters the study as a later tier or as a simulation/literature comparison.

## 0. Inventory and integrity (phase 0)
| Item | Result |
|---|---|
| Files | 12 files, 2 888 lines, 172 KB: 10 `.py`, 1 `.md` (guide, 124 lines), 1 `.pth` |
| Zip vs folder | `diff -rq` → identical |
| Integrity | SHA-256 of every file recorded before any test (scratchpad `fbl_hashes_before.txt`; re-checked at the end) |
| Dependencies | numpy, scipy (`Rotation`), torch only. No CasADi/Acados/ROS/simulator. Tested by the authors with PyTorch 2.7.1; here: system python 3.10 + torch 2.5.1(cu121), pyenv `flying_robots` python 3.12 + torch 2.14 CPU / numpy 2.4.2 / scipy 1.17 |
| Packaging | no `__init__.py` anywhere (implicit namespace packages; the example puts the bundle root on `sys.path`); **no licence / copyright header** → ask the authors about licence and citation before reuse in the thesis |
| Legacy markers | `fbl_control.py` "legacy simulator interface" (telemetry slots, ROM fallback if checkpoint absent, allocation fallback); `flatness_utils.py` "legacy flat-output reconstruction and MPC warm-start helpers" (not needed for the feedback law); `nn_models_full_only_zr.py` "legacy training wrapper uses neighbor attitude" |
| Not included | training scripts, datasets, the optional ESO (`use_eso` must stay False), the original simulator |

### Layout
```
examples/learned_fbl_inference.py        one residual + controller evaluation (no simulator)
KNODE/Models/sample_model.pth            sample ROM+NN checkpoint
KNODE/Training/nn_models_full_only_zr.py ODEFunc (2 nets) + re-exports (entry point)
KNODE/Training/downwash_residual.py      DownwashResidual: ROM + NN -> (acc, jerk, ang_acc)
KNODE/Training/residual_derivatives.py   analytic ROM + NN derivatives (time derivative of the flow frame)
KNODE/Training/nominal_dynamics.py       vehicle parameters, geometry/quaternion helpers, nominal ODE
KNODE/Training/Velocity_Field/{force,torque}_batch_fo.py   ROM force/torque (+ partial derivatives)
KNODE/Training/flatness_utils.py         legacy flatness helpers
simulator_multi_python/Controllers/fbl_control.py            compute_fbl_command (stateless) + FBLControl wrapper
simulator_multi_python/Controllers/acceleration_estimator.py velocity-based acceleration estimator
LEARNED_FBL.md                           getting-started guide
```

## 1. What the authors say the bundle is (guide `LEARNED_FBL.md`, read)
- Reading order: example → `compute_fbl_command` → `ODEFunc` → `DownwashResidual` → analytic derivatives.
- Data flow: position feedback → commanded acceleration and jerk (paper Eqs. 17–18); residual inference → world-frame disturbance acceleration, its time derivative, and body angular acceleration; control mapping → thrust and roll/pitch rates compensated (Eq. 15), yaw rate (Eq. 10), torque with body-rate feedback (Eq. 19); thrust projected onto the measured body z-axis (Sec. 3.3, footnote 2). `compute_fbl_command` is stateless, returns thrust [N], desired body rates [rad/s], torque [N·m]; rotation matrix maps body → world.
- Input layout: 30 entries per pair = ego state (13) | controls (4, unused) | neighbour state (13); state = position(3), velocity(3), quaternion **[x,y,z,w]**(4), body rates(3). NN features **[radial distance, vertical distance]**; vertical distance unsigned → assumes the neighbour is the **upper** vehicle; runtime inference assumes the upper vehicle upright (flow axis +world-z, zero derivative); the original `ODEFunc.forward` uses the neighbour's actual body-z instead.
- Frames: local x radially toward the neighbour, local z along the positive flow axis; force net predicts a local **acceleration** correction (rotate to world, do not divide by mass again); ROM force is divided by mass; torque net predicts a scaled local torque correction (rotated into the ego body frame with normalised inverse inertia); the controller receives body angular acceleration and multiplies by inertia.
- Architecture: two `Linear(2,9) → Tanh → Linear(9,3)` nets (`net_f`, `net_t`) = 2 × (27 + 30) = **114 learned parameters**; the analytic NN derivative assumes exactly this architecture.
- Sample checkpoint and example = ROM+NN (`use_nominal=True`, `use_nn=True`); example constants: mass 0.03 kg, inertia diag(1.43e-5, 1.43e-5, 2.89e-5), rate gains diag(238, 238, 140); expected thrust ≈ 0.329 N (they tested PyTorch 2.7.1; a tensor-copy warning may appear).
- Legacy arguments (`C_d`, `S` of `FBLControl` do not override `nominal_dynamics.py`); the standalone control function is the recommended integration entry point.

## 2. The paper (v2) in the points that matter for us
- Method: residual restricted to joint positions/velocities ξ = [p¹,v¹,…,pᴺ,vᴺ] keeps the augmented dynamics **differentially flat**; corrected maps: thrust and z-axis (14), body rates (15) with the residual derivative from the chain rule, torque (16); cascaded FBL: position PID → a (17), jerk by differentiation (18), ω_r from (15)+(10), torque = −K_ω(ω−ω_r) − Rᵀ f̂ᵗ (19); acceleration from an α-β-γ filter on p, v.
- Residual: f̂ = f̂_d(h₀) + f̂_Θ(h₀), h₀ = [Δz, Δr] (pairwise ROM from Chee et al. 2024: turbulent-jet airflow + disk drag, App. A; tiny tanh MLP rotated by R_F^W); trained with KNODE (integrated-state loss, Adam).
- Experiments: simulation (Crazyflie mass/inertia, 9-neuron nets, 18 s of data, FBL 100 Hz, Tables 1–4, 20× lower solve time than Acados NMPC: 0.99 vs 27.6 ms) and hardware (two **stock Crazyflie 2.1 with thrust-upgrade bundles, 36 g**, Vicon 120 Hz, **laptop i5, commands at 400 Hz over the radio, CrazyROS**; **thrust + angular-velocity commands, only the force residual compensated**, torque residual left to the onboard rate PID; ROM C_D = 0.236; force net 2 layers × 4 neurons; 28 s / 6 720 training points at 0.4 m, 0.4 m/s; test 0.4 m (in-distribution) and 0.3 m (OOD), 60 runs = 5 repeats × 12 cases; Learned-FBL ≈ 5 cm RMSE, 10 cm z_max; window demo 1.1 → 0.1 m).
- **Version note:** our summary `docs/papers/summaries/hsieh2026flatness.md` was written from **v1** (14 Jul 2026); the PDF in the repo is **v2** (17 Aug 2026). Differences seen so far: the intro now says "20× less compute" (v1: "7×"). The summary should be re-checked against v2 and updated (not done yet; no change made).

## 3. Phase 1 — the learned model (read line by line: `nn_models_full_only_zr.py`, `downwash_residual.py`, `residual_derivatives.py`; weights inspected with `weights_only=True`)

### 3.1 Checkpoint `KNODE/Models/sample_model.pth`
- Contains only `state_dict` (no optimizer, no metadata, no scaling constants). Archive name inside: `quad_bot_Mar1_1_with_dw_only_zr` ("quad_bot … with downwash, only zr" — the bottom quad, trained 1 March).
- 8 tensors, float32, **114 parameters** = `net_f` (57) + `net_t` (57); both `Linear(2,9) → Tanh → Linear(9,3)`.
- Ranges: `net_f` weights −3.5…+0.9 (output acceleration scale ≈ m/s²); `net_t` weights ±10, bias up to +7 (the torque net outputs a *scaled* torque, multiplied afterwards by the normalised inverse inertia).
- This is the **simulation / ROM+NN configuration (9 neurons for force and torque)**. The paper's *hardware* residual was a force-only net with 4 neurons and a re-identified drag coefficient (C_D 0.236) — the bundle does **not** contain that checkpoint.

### 3.2 `ODEFunc` (training wrapper; `nn_models_full_only_zr.py`)
- Holds `net_f`, `net_t` (init N(0, 0.1²), zero bias), `NominalModel`, `inv_inertia = nominal_model.inv_inertia_normalized` (J⁻¹/‖J⁻¹‖_F, paper's M_τ scaling).
- `forward(t, y)` on (B,1,30): nominal ODE + `delta_f` added to velocity-derivative slots 3:6 and `delta_t` to angular-acceleration slots 10:13. Features [r, z] from `point_line_distance(ego, top, z_bm)` with **z_bm = actual body-z of the upper (neighbour) vehicle** (training wrapper) — **runtime uses the constant world +z instead** (upright approximation). → documented train/runtime mismatch (the paper's "near-hover upper vehicle" assumption); matters when the upper drone tilts (e.g. A8 crossings, A2/A5 circles; ours reached 21–35° roll at crossings).
- Re-exports the flatness helpers so old imports keep working; nothing else.

### 3.3 `DownwashResidual` (runtime; `downwash_residual.py`)
- `forward(t, y) → (acc [B,3] world, acc_dot [B,3] world, ang_acc [B,3] body)`; input y = (30,) | (B,30) | (B,1,30) = ego(13) | controls(4, unused) | neighbour(13).
- `use_nominal` adds the ROM; `use_nn` adds the nets. Sample/example: both on.
- **Acceleration:** `acc = ROM + NN`. ROM: `−downwash_force_fo(z0, r0, −vz=0, C_d, S)/m · e_z(world)` — ego vertical velocity is set to **zero** in the ROM (kinematic features only via Δz, Δr). NN: `R_F^W @ net_f([r0, z0])` — **no extra 1/m** (the checkpoint already outputs m/s²; the paper's M_a = R_F^W/m is folded into the weights).
- **Angular acceleration (body):** ROM torque (`downwash_torque_fo`) → ego body (Rᵀ) → physical J⁻¹; NN torque: `R_F^W @ net_t` → body (Rᵀ) → **normalised** J⁻¹. The controller multiplies by J again.
- **Flow frame (`flow_frame_to_world`):** columns [x,y,z]; z = normalised flow axis (+world-z at runtime), y = normalise(z × x̂), x = normalise(y × z), with x̂ = (neighbour − ego)/(‖·‖ + 1e-8). **Difference to the paper:** Eq. (20) has e_z = **−e₃**, e_y = e_z × Δp/‖·‖, e_x = e_y × e_z; the code's +z frame is the paper's frame rotated by 180° about x (y and z flipped). Consistent inside the bundle (weights are tied to the code convention — the guide warns not to flip axes), but any re-implementation or retraining must use the code's convention or flip the output components.
- Options: `profile` (timing prints), `compile_nn` (torch.compile, off by default).

### 3.4 Analytic derivatives (`residual_derivatives.py`)
- `network_value_and_jvp`: value `W2·tanh(W1 f + b1) + b2`, derivative `W2·diag(1−tanh²)·W1·f_dot` — exact for this architecture, no autograd.
- Time derivative of the features: z0 = |d·z_bm|, dz0/dt = sign(t)·(d_dot·z_bm); r_vec = d − t z_bm, dr0/dt = (r_vec·r_vec_dot)/r0 (r0 clamped ≥ 1e-12); d = p_ego − p_top, d_dot = v_ego − v_top. With z_bm constant its derivative is exactly zero (they removed a lever-arm blow-up that scaled with separation × upper-vehicle attitude rate).
- ROM jerk: `−(1/m)·(dF/dt · z_bm)`, dF/dt = F_z0·dz0/dt + F_r0·dr0/dt (partials from `downwash_force_and_derivs_fo`, phase 2).
- NN jerk: `dR_vw/dt @ f_nn + R_vw @ df_nn/dt`, with the Gram–Schmidt frame differentiated analytically (x̂, y, x each normalised, derivatives projected).
- **Only the acceleration residual has a time derivative.** No derivative of the torque residual (paper Eq. 19 uses the torque residual only as feedforward), none of the second derivative.
- Tangent = relative velocity of the two vehicles only (ξ̇ in the paper also contains accelerations; for position-only features the directional derivative needs just velocities — consistent with the guide).

### 3.5 Observations and risks from reading (to be quantified in phase 4)
1. **Small-radius singularity:** y-axis normalisation uses ‖z × x̂‖ = r0/‖r̂‖ (clamped ≥ 1e-8) and its derivative scales with ‖r̂‖/r0 → for a nearly aligned stack (r0 of a few cm at 0.5 m separation, i.e. our A1 and the stack scenarios) the NN **direction and jerk term can jump**; also the NN's local-x (radial) output has no defined direction at exact alignment. Needs a numerical sweep of acc/jerk vs r0 (phase 4) and a decision on a floor/blend for r0 → 0 before any use on hardware.
2. **Upright-upper-vehicle assumption** (runtime) vs actual attitude (training wrapper); unquantified for our scenarios with moving upper drone.
3. **Neighbour must be the upper vehicle** (z0 unsigned, ROM force always downward on ego): fine for cf5 (bottom) with cf_second above; wrong if roles/heights swap.
4. **ROM drops ego vertical velocity** (vz = 0) — drag from vertical motion of the ego is ignored.
5. **dtype mixing:** several `.float()` casts; float64 inputs give float32 NN terms — harmless but mixed precision; tangent/JVP stay in input dtype.
6. **Checkpoint scope:** the sample net is the simulation 9-neuron force+torque model with its own C_d/S constants in `nominal_dynamics.py` (to be read in phase 2) — not the hardware model; for our brushless 41 g drone everything (mass, inertia, C_D, nets) would have to be re-identified/re-trained, and no training code is included.

### 3.6 Paper ↔ code map so far
| Paper | Code |
|---|---|
| (20) flow frame, h₀ = [Δz, Δr] | `flow_frame_to_world`, `NominalModel.point_line_distance` (phase 2) — frame sign convention differs (see 3.3) |
| (21) f̂ = f̂_d + f̂_Θ | `acceleration_split_jvp` / `angular_acceleration` (ROM + NN) |
| (22) NN, M_a, M_τ | `net_f`, `net_t`; M_a folded into weights; M_τ = `inv_inertia_normalized` |
| (15) f̂ᵃ derivative via chain rule (paper: automatic differentiation) | **analytic** `ResidualDerivativeMixin` (no autograd) |
| (24)–(25) ROM jet + disk drag | `force_batch_fo.py`, `torque_batch_fo.py` (phase 2) |
| (23) KNODE loss, Adam | not included (only `ODEFunc.forward` for an external trainer) |
| (14)–(19) flatness maps and controller | `fbl_control.py` (phase 3) |

## 4. Phase 2 — dynamics, ROM and flatness helpers (read line by line: `nominal_dynamics.py`, `Velocity_Field/force_batch_fo.py`, `torque_batch_fo.py`; `flatness_utils.py` read fully for structure, skimmed for the loop/batch variants)

### 4.1 Constants (hard-coded, three copies)
| Constant | Value | Where |
|---|---|---|
| mass | 0.03 kg | `NominalModel`, `quad_params` in both velocity-field files (3 copies); the controller example passes its own mass separately |
| inertia | Ixx = Iyy = 1.43e-5, Izz = 2.89e-5 kg·m² | same 3 places (+ example) |
| arm length / rotor | 0.046 m / k_thrust 2.3e-8, k_drag 7.8e-11, ω_max 2500 rad/s | same 3 places (stock Crazyflie 2.x numbers; not used by the residual itself except arm length) |
| ROM drag coefficient | **C_d = 0.3** (`nominal_dynamics.py`), jet spread S = 0.0997; function default `C_d = 0.236` is **overridden** by the module constant 0.3 in every call | `nominal_dynamics.py`, force/torque defaults |
| "ground truth" drag | `C_d_GT = 0.55`, `S_GT = 0.0997` — imported by `nn_models_full_only_zr.py` but **not used** | **paper says the simulated plant used C_D = 0.7** (and hardware ROM 0.236) → discrepancy to ask about |
| jet constants | U_j = 1, Bd = 6.84, z₀ = −3.08 (comment: "different from the paper, this should be correct"), N_r = 4 propellers, r_prop(velocity) = 0.0225 m, ρ = 1.293 (cancels) | `force_batch_fo.py` |
| hover induced velocity | U_H = sqrt(m g / (2 ρ π r² N_r)) ≈ 4.2 m/s (hand calculation, to be confirmed in phase 4) | uses m = 0.03 |
| integration disk | radius arm + r_prop + 0.001 = **0.072 m** (force, r_prop 0.025) vs **0.0695 m** (torque, r_prop 0.0225); normalising length l = 2(arm + r_prop) = 0.137 m (force) vs 0.139 m (torque, +0.002) | inconsistent between the two modules (≈ 1.5 %) |

### 4.2 ROM downwash force (paper App. A, Eqs. 24–25)
- Jet velocity at normalised axial distance ζ = Δz/l − z₀ and radial distance R: `V = U_H · Bd / ζ / (1 + (√2−1)·(R/(l S ζ))²)²` (matches Eq. 24); force `F = ½ C_d ρ ∮ (V(Δz, R(r,θ)) − k·vz)² dA` over the lower vehicle's disk (disk point at (r,θ) → jet-axis distance R = sqrt(r0² + r² ± 2 r0 r cosθ)).
- Quadrature: **10 × 10 trapezoid grid in polar coordinates** (θ endpoints duplicated, float64); the analytic derivative function `downwash_force_and_derivs_fo` differentiates the **same discretised sum** ("must match", derivative of the quadrature, not of the continuous integral). The force version uses `+2 r0 r cosθ`, the torque version `−2 r0 r cosθ` — equivalent for a continuous integral, but not identical on the 10-point θ grid (small quadrature difference).
- `vz` (ego velocity along the jet axis) is multiplied by 0 in both training and runtime → the ROM sees **no ego velocity** (kinematic features Δz, Δr only).
- Sign/direction: runtime/training use `a_ROM = −F/m · e_z` (world +z axis), i.e. a downward acceleration on the lower vehicle.
- ROM torque: ∫ (−p × e_z)·V² dA with p the disk point in the flow frame, rotated to the world by the flow-frame matrix, then (in `DownwashResidual`) to the ego body frame and multiplied by the physical J⁻¹.
- **Possible inconsistency (to verify numerically in phase 4):** in `rxz_velocity_field_squared_polar` the in-plane disk points (flow frame F) are crossed with `z_bm_batch`, a **world-frame** vector. This is only equivalent when the flow axis equals world z (the runtime case, where `z_bm = [0,0,1]`); in the training wrapper with a tilted upper vehicle the torque ROM mixes frames. Also `torch.tensor(z_bm_batch)` copy construct → the "tensor-copy warning" the guide mentions.

### 4.3 `NominalModel` (training ODE)
- Rigid-body ODE with thrust/torque inputs (u[0] thrust, u[1:4] torque), quaternion kinematics `q̇ = ½ Gᵀ ω`, gravity [0,0,−9.81], plus the ROM acceleration (slots 3:6) and angular acceleration (slots 10:13) of the downwash; control and neighbour states have zero derivative (the neighbour is treated as an exogenous trajectory in KNODE training).
- `point_line_distance(point, line_point1, line_direction)`: r0 = distance of the ego from the neighbour's thrust axis, **z0 = ‖neighbour − foot point‖, unsigned** → ROM and network cannot tell "neighbour above" from "neighbour below" (the guide's "assumes the supplied neighbour is the upper vehicle").
- `quaternion_to_rotation_matrix` standard [x,y,z,w] → body-to-world; hard-coded `torch.tensor` on the default device; `torch.pinverse` for the diagonal inertia.

### 4.4 `flatness_utils.py` (legacy, offline)
- `flat_output_to_state_and_control(+_batch)`: the full differential-flatness inversion from p, ṗ, p̈, jerk, snap, ψ, ψ̇, ψ̈ to state (13) and control (thrust, 3 moments) with optional `a_downwash` (subtracted like gravity: f_des = m·a + [0,0,m g] − m·a_dw → thrust ↑ when the residual points down) and `tau_downwash`; fallbacks for near-zero thrust (`trim_thrust 0.2943 N` = 0.03·9.81) and for n = ‖b3×a_ψ‖ < 1e-8.
- `flat_trajectory_to_state_and_control(+_loop)`: builds a trajectory over a horizon for the **MPC warm start**; the batch path places the top vehicle directly above the bottom one (**r0 = 0**) and **sets the angular-acceleration residual to zero** for the warm start; loop and batch paths differ numerically (documented by the authors: "existing behaviour retained"); `FLAT_TRAJ_PROFILE` env var for timing.
- **Not used by the feedback law** (`fbl_control.py` does not import it; only `nn_models_full_only_zr.py` re-exports it).

### 4.5 Findings of phase 2
1. The residual model is hard-wired to **one vehicle**: 30 g, Crazyflie-2.x geometry (arm 0.046 m, prop radius 0.0225/0.025 m, N_r = 4) in three duplicated copies, plus the jet constants (Bd, z₀, S). Porting to our 41 g CF2.1 Brushless means changing mass, inertia, arm, propeller radius, C_d (identification as in Chee et al.) and **retraining the two nets** — none of which the bundle supports with code (no training loop, no data).
2. **Two different hardware masses appear in the story:** ROM and example 30 g, paper hardware 36 g with battery and markers; our drones 41 g. Because U_H and the division by m both use 0.03, the ROM *acceleration* is formally independent of the vehicle's actual mass (it represents identical vehicles), but the controller's own mass/thrust model must be right.
3. Open inconsistencies (to ask the authors): `C_d_GT 0.55` vs paper's 0.7; force vs torque module constants (disk radius, l); world/flow-frame mix in the torque ROM for tilted neighbours; z0 unsigned.
4. The velocity-field code is slow-looking but fixed-size (10 × 10 points per sample, float64, `meshgrid` rebuilt on every call) — the real cost is for phase 4 timing; an onboard port would need either a lookup table in (Δz, Δr) (2-D, smooth) or an analytic approximation instead of the quadrature.

## 5. Phase 3 — the controller (read line by line: `fbl_control.py`, `acceleration_estimator.py`)

### 5.1 `compute_fbl_command` — stateless core (matches the paper)
Inputs: rotation R (body→world), body rates ω, acc_cmd, jerk_cmd, yaw, yaw_rate, residual acc / jerk (world) / angular acc (body), mass, gravity, inertia J, rate gains K (acceleration gains). Steps:
1. `z = R e₃`; **thrust = m·(a_cmd + g e₃ − a_res)·z** — Eq. 14a, but **projected on the measured body z-axis** (footnote 2) instead of the norm; the residual acceleration (downwash points down) is subtracted → thrust increases.
2. `ω_xy = (e₃ × Rᵀ·m·(j_cmd − j_res)) / max(thrust, 0.01 N)` — Eq. 15 (the 0.01 N floor only protects this division).
3. `ω_z = (ω_x (zᵀx̄) + yᵀ x̄̇)/(xᵀ x̄)` — Eq. 10 (heading x̄ from yaw, yaw_rate); set to 0 when |xᵀx̄| < 1e-6.
4. `τ = J·(K (ω_des − ω) − α_res)` — Eq. 19 with the torque residual as body angular acceleration; the paper's torque gain is K_ω = J K.
Not included by design: gyroscopic term ω × Jω, feed-forward ω̇_r (paper: exact ω̇_r would need the second derivative of the residual). No thrust/torque saturation, no non-negativity of thrust (projection can go negative for extreme tilt).

### 5.2 `FBLControl` — stateful wrapper (legacy simulator contract)
- **Vehicle (hard-coded):** m = 0.03 kg, J = diag(1.43e-5, 1.43e-5, 2.89e-5), arm 0.046 m, k_thrust 2.3e-8, k_drag 7.8e-11, ω_rotor ∈ [0, 2500] rad/s, trim 1790 rad/s; X-configuration allocation matrix (simulator-specific; irrelevant for our firmware, which has its own allocation).
- **Gains ("Deployed position/integral and body-rate gains" comment):** position kp 56 (xy) / 45 (z); velocity kd 38 / 21.6; integral ki 48 / 3.0; body-rate K = diag(238, 238, 140) s⁻¹ → torque gains J·K = 3.4e-3 / 3.4e-3 / 4.05e-3 N·m·s/rad. (In the hardware experiment the rate loop was the firmware's own PID, so 238/140 describe the simulation torque law.)
- **Position law (Eqs. 17–18):** `a_cmd = a_r − ki∫e − kp e − kd ė`, `j_cmd = j_r − ki e − kp ė − kd (â − a_r)` with â from the estimator; ė uses the measured velocity.
- **Acceleration estimator (`acceleration_estimator.py`):** four options (dirty derivative, second-order filter, **α-β-γ (default α 0.5 [unused], β 0.4, γ 0.1)**, Kalman); the α-β-γ filter is fed **velocity only** (the paper says "positions and velocities") — v̂ ← v_pred + β·res, â ← â + (γ/dt)·res. Because β and γ are dimensionless the discrete poles are fixed at 0.75 ± 0.19j; the continuous bandwidth therefore scales with the call rate: ≈ 25 ± 25j rad/s at 100 Hz, ≈ 51 ± 50j at 200 Hz, ≈ 102 ± 101j at 400 Hz.
- **Residual (`_predict_residual`):** builds the 30-vector for each neighbour (ego[:13], zeros(4), neighbour[:13]) and **sums** the pairwise residuals (superposition for N > 2, untested in the paper); every neighbour is assumed to be an upper vehicle; **zeroes the yaw component of the angular-acceleration residual** ("causes yaw drift") — a hack that also shows that **there is no yaw-angle feedback**: yaw is only handled through the heading constraint (Eq. 10) and rates.
- **Optional:** ESO (`use_eso`, not in the bundle), profiling hooks.
- **Interface:** `update(t, state{x,v,q[xyzw],w,u}, flat_output{x,x_dot,x_ddot,x_dddot,yaw,yaw_dot}, drones, …)`; the reference must provide **position, velocity, acceleration and jerk** (jerk-level feedforward); returns motor speeds (sim-specific) plus thrust/moment.

### 5.3 Closed-loop character of the gains (my calculation, no bundle code)
| Loop | Polynomial | Roots |
|---|---|---|
| FBL position error, xy | s³ + 38 s² + 56 s + 48 | −36.5, **−0.75 ± 0.87j** (ω_n ≈ 1.15 rad/s, ζ ≈ 0.65) |
| FBL position error, z | s³ + 21.6 s² + 45 s + 3 | −19.3, −2.26, **−0.07** (time constant ≈ 14 s) |
| our geometric, xy (kp 40, kv 8) | s² + 8 s + 40 | −4 ± 4.9j (ω_n 6.3, ζ 0.63) |
| our geometric, z (kp 30, kv 10, ki_z 16) | s³ + 10 s² + 30 s + 16 | −4.66 ± 1.4j, −0.67 |
- **Reading:** the large acceleration-error gain (kd = 38) makes the inner loop fast but leaves a **soft effective position loop** (≈ 1 rad/s) — disturbance rejection comes mainly from the **feed-forward residual** (the paper's point), not from feedback; with the bundle's z-integral gain (3.0) a constant z offset (mass/thrust model error) would decay with a ≈ 14 s time constant. On our hardware we needed ki_z = 16 to remove a 10 cm hover bias (docs/07) → the gains would have to be **re-tuned for our vehicle** (and the hardware gains of the paper are not stated).
- The structure has **no attitude-error feedback**: attitude follows from the jerk → rate command and the thrust projection on the *measured* z; with a fast rate loop (K = 238 s⁻¹ ≈ 38 Hz) this is the cascaded FBL of the paper; with our firmware's rate loop (and its latency) it must be verified.

### 5.4 Comparison with our controllers (structure, not performance)
| Aspect | Our geometric (S0) | Our INDI (ctrl_mode 3) | NS2 (S2) | Learned FBL (this bundle) |
|---|---|---|---|---|
| Position law | PID → desired force vector | same + incremental terms | geometric + network acc. | PID → a_cmd **and j_cmd** (3rd-order error dynamics) |
| Attitude | desired attitude from force + SO(3) error feedback (kr, kw) | incremental torque | as geometric | **no attitude error**: ω_des from jerk (Eq. 15) + rate feedback |
| Residual use | none | measured RPM-based a_res in the loop | network acc. as feed-forward on the thrust vector (`res_sign −1`) | learned/ROM residual acc **and its derivative** as feed-forward, torque residual as feed-forward |
| Needs | p, v, q, ω; ref p, v, a | + rotor RPM, IMU | + neighbour p, v | p, v, q, ω, **estimated acceleration**, neighbour p, v; ref p, v, a, **j**, ψ, ψ̇ |
| Yaw | attitude loop | attitude loop | attitude loop | heading constraint, no angle feedback (yaw drift hack) |
| Frames/units | ENU, quaternion [x,y,z,w], SI | same | same | **same** (ENU, [x,y,z,w], body→world R, SI) |
- Frame/convention compatibility with our firmware is good (ENU, [x,y,z,w], body-frame rates, N / rad/s / N·m).

### 5.5 Findings of phase 3
1. The three equations that matter (14a thrust projection, 15 rates, 19 torque) are small and implementable in our out-of-tree controller (no autograd needed); the cost is in the residual evaluation (ROM quadrature) and its analytic derivative.
2. **Needs the neighbour's state in the loop** (position and velocity) — our NS2 peer-packet path already delivers exactly this at 100 Hz (relative position/velocity), including the hold/resync logic.
3. **Needs jerk in the reference** and an **estimated acceleration** (α-β-γ on velocity): our trajectory source (poly4d polynomials) can supply jerk analytically; the estimator's bandwidth scales with the call rate and must be re-tuned for 500 Hz.
4. **Gains and model parameters are for a 30 g vehicle in simulation** (mass 0.03 kg in thrust and inertia); not transferable to the 41 g brushless drone without identification, and the paper's hardware gains are not published.
5. **Robustness gaps in the stateful wrapper:** integral accumulates with `dt = t − prev_t` where `prev_t` starts at 0 (a large first step if time does not start at 0), no integral clamp/anti-windup, no thrust floor/saturation inside `compute_fbl_command`, yaw not regulated (yaw-residual zeroed), residuals summed over neighbours although trained on pairs.
6. The hardware paper used the **rate-command interface** (thrust + rates) with only the force residual — the torque branch (Eq. 19 with torque residual) is a simulation feature; for us the torque branch is natural because our out-of-tree controller outputs torques directly.

## 6. Phase 4 — runtime checks (harness: `experiments/analysis/fbl_bundle_check/fbl_checks_part1.py`, `fbl_checks_part2.py`, results in `…/out/*.json`)
All runs on a **copy** of the bundle (`PYTHONDONTWRITEBYTECODE=1`, pyenv python 3.12, torch 2.14 CPU, single thread); the original was never executed in place (hashes re-verified: unchanged).

| # | Check | Result |
|---|---|---|
| 1 | Authors' example | runs; **thrust 0.32892 N** (guide: ≈ 0.329 N); residual acc (7e-4, 8e-5, **−1.154**) m/s², jerk (−7e-6, 2e-4, −0.030) m/s³, **angular acceleration (−17.3, 45.8, −0.5) rad/s²**, torque (2.5e-4, −6.6e-4, 1.4e-5) N·m; only the known tensor-copy warning |
| 2 | Analytic residual jerk vs central finite difference (300 random pairs, float64 nets, h = 5 ms) | **correct**: median relative error 1e-4, max 0.8 % (ROM 0.7 %, NN 0.8 %, total 0.7 %); NN JVP vs `torch.autograd` 1.2e-7 |
| 3 | ROM force: 10×10 quadrature vs fine integration (300×720) | quadrature error **≤ 0.55 %** (largest at r0 = 0, dz 0.2); the analytic partials dF/dz0, dF/dr0 equal the finite difference of the code's own force to 4 digits; U_H = 4.23 m/s; **ROM downwash ≈ 1.66 m/s² at dz 0.3 m (0.050 N = 17 % of the 0.294 N weight), 1.26 m/s² at 0.5 m, ≈ 0 beyond r0 ≈ 0.3 m** |
| 4 | r0 → 0 sweep (dz 0.5 m) | NN horizontal output ≈ 0.001 m/s² (the NN is effectively vertical at small r0; f_z −1.0 m/s² at r0 = 0) → the acceleration is benign. The NN **jerk** from the frame derivative grows ∝ 1/r0: with 0.3 m/s lateral relative speed |jerk_NN| = 0.00 / 0.01 / 0.04 / 0.13 / 0.41 / 4.1 m/s³ at r0 = 0.1 / 0.03 / 0.01 / 0.003 / 0.001 / 0.0001 m. **At r0 exactly 0 (z_bm ∥ r̂) the jerk is garbage: 8×10¹² m/s³** (clamp 1e-8 in the frame normalisation) → needs a guard (floor on r0 or drop the horizontal terms) in any port; with mocap noise r0 is almost never exactly 0, but a controller fed commanded/identical xy would hit it |
| 5 | ROM torque with a tilted flow axis (world z_bm vs flow-frame disk points) | my suspicion from reading was **negligible in practice**: ≤ 0.5 % difference to an independent reference for tilts up to 30° |
| 6 | Timing, this laptop CPU, PyTorch single thread | full residual (acc + jerk + ang. acc, ROM + NN) **1.34 ms** (acc 0.60, ang 0.69; ROM 0.86, NN 0.50); `compute_fbl_command` **0.034 ms**. The paper reports 0.99–1.3 ms solve time → consistent; the bulk is Python/torch overhead and the 10×10 quadrature, not the 114-parameter nets |
| 7 | Eq. 15 vs an independent numerical derivative of the flatness rotation (circle, three residual cases: none, constant, time-varying) | **`compute_fbl_command` matches to 1e-11 rad/s** (also the residual-jerk term; dropping it gives a 3.6e-2 rad/s error). **The legacy `flatness_utils.flat_output_to_state_and_control` returns `omega_des` with the opposite sign** (its `_vee_skew` returns −ω for S = hat(ω)) and ignores the residual jerk → its `omega_des`/moment must not be reused |
| 8 | Closed-loop smoke test, `FBLControl` from the bundle on my own 6-DoF plant, bottom drone translating beneath a fixed top drone at ≤ 0.4 m/s, downwash force + torque from the ROM with a **mismatched drag coefficient** | see below |

### 6.1 Closed-loop smoke test (my plant, not the authors' simulator)
- **The rate loop gain 238 s⁻¹ is discretely unstable at a 100 Hz control rate** (K·dt = 2.38 > 2): body rates oscillate ±2.5 rad/s, the torque command saturates two rotors at the limit and the realised thrust is clipped (z sags and recovers over ≈ 14 s). At **400 Hz (K·dt = 0.6) and 500 Hz (0.48) the loop is clean** (rates < 0.4 rad/s). The paper's hardware rate was 400 Hz (radio) and the simulation "100 Hz" — the gains in the bundle are for the fast loop.
- At 500 Hz, plant C_D = 0.55 (the code's `C_d_GT`), force + torque disturbance, RMSE / z_max / mean z error:
  | Controller | RMSE | z_max | mean z |
  |---|---|---|---|
  | nominal FBL | 7.8 mm | 30 mm | −2.3 mm |
  | ROM (C_d 0.3) | 3.7 mm | 14 mm | −1.1 mm |
  | **ROM + NN (sample weights)** | **1.6 mm** | **2.7 mm** | −0.7 mm |
  Same ordering as the paper's Tables 2–4 (nominal > ROM > learned), but **much smaller numbers than the paper's simulation (16 mm RMSE)**, because my plant has no estimator noise, a fixed top drone and a disturbance that is exactly the ROM family. At 100 Hz (unstable rate loop) my errors are ≈ 30 mm RMSE / 40–56 mm z_max, of the same size as the paper's reported sim numbers — I do not claim this explains them.
- **The sample network is tied to its training plant:** plant C_D = 0.30 (ROM exact) → ROM alone is near-perfect (0.5 mm) and **the NN hurts** (4.3 mm); C_D = 0.55 → NN helps most; C_D = 0.70 → ROM 5.7 mm, ROM+NN 2.4 mm (still helps). So the learned part is essentially a correction from the ROM (C_d 0.3) to a specific plant — consistent with `C_d_GT = 0.55` in the code (the paper text says 0.7). It is **not a generic downwash model**; for our vehicle it has to be retrained.
- Torque residual on/off changes little at 500 Hz (the torque feed-forward only matters for the plant torque disturbance; the paper's hardware ignores it).
- Earlier finding confirmed: with the bundle's z gains the nominal controller recovers from a 3.6 cm sag only slowly (z pole at −0.07), visible at 100 Hz.

## 7. Frames, units and signals — bundle versus our stack
| Item | Bundle | Our firmware / stack | Status |
|---|---|---|---|
| World frame | ENU, z up | Crazyflie/mocap frame, z up | compatible |
| Quaternion | [x, y, z, w], R maps body → world | same (stateEstimate) | compatible |
| Body rates | rad/s, body frame | gyro internally converted; 50 Hz low-pass on the feedback gyro (`GYRO_FB_FC_HZ`) | check units/filter delay when porting |
| Units | SI (m, N, kg, N·m) | SI | compatible |
| Control rate | 100 Hz (sim, unstable with K = 238), 400 Hz (hardware) | loop runs at ≈ 1 kHz (comments in `lib.rs`; logs at 500 Hz) → K·dt = 0.24 | favourable |
| Reference | p, v, a, **jerk**, ψ, ψ̇ | poly4d on board: pos, vel, acc, **jerk, snap** (`eval_axis`) | available |
| ω_des from jerk | Eq. 15 (+ residual jerk) | `omega_desired(acc, jerk, yaw)` (same flatness formula, no residual term, ψ̇ = 0) | partly there |
| Acceleration estimate | α-β-γ on velocity | IMU + INDI estimates; no α-β-γ | to add (bandwidth scales with rate) |
| Neighbour state | p, v of the upper drone | `peer_get_all` positions at ≈ 100 Hz + timestamp; relative velocity by differencing, held between packets (NS2 path) | available; velocity noise ≈ √2·σ_p/Δt ≈ 0.1 m/s for σ_p = 1 mm, Δt = 10 ms — filtering needed because the jerk residual multiplies it by ∂a/∂r ≈ 5 s⁻² |
| Thrust/torque out | thrust [N], torque [N·m] (+ rate command) | OOT controller outputs thrust and torque (geometric/INDI) | compatible |
| Neighbour above/below | unsigned z0 → must be the upper vehicle | cf5 below cf_second in all study scenarios | compatible (document the assumption) |

## 8. Gap analysis — what is missing for Strategy 3 beyond the bundle
1. **A residual for our vehicle.** The 114-parameter network and the ROM constants belong to a 30 g simulated Crazyflie. For our 41 g brushless drone we need: mass, inertia, arm length, propeller radius, a drag coefficient C_D (system identification as in Chee et al. 2024), and **retrained networks**. The bundle has no training loop or data format. Two ways: (a) re-implement the paper's KNODE loss (integrated-state MSE, Adam) around `ODEFunc.forward` (needs a differentiable ODE solver and ≈ 30 s of FBL flights without learned residual at 0.4 m / 0.4 m/s, per the paper); (b) **supervised fit** of the same two-input structure to the measured residual `a_res` that our logs already contain (C.1 bank, NS2 data), which avoids differentiating IMU data by using the RPM-based `a_res` labels we trust (corr 0.85–0.92 with the NS2 prediction).
2. **The control law inside our firmware** (new mode or controller): Eqs. 14a/15/10/19 + position law (3rd-order, gains re-tuned: the bundle's z-integral gain 3.0 is far too slow, we needed ki_z = 16), α-β-γ (or equivalent) acceleration estimate, rate loop, thrust/torque limits, mode handling for takeoff/landing (as with INDI: geometric ramp).
3. **A cheap onboard residual.** The ROM evaluation (10×10 quadrature, float64, meshgrid per call) is not firmware-friendly; plan: pre-compute the local residual **f(Δz, Δr) (3 components) and its partials as 2-D tables** (e.g. 56 × 31 grid × 9 floats ≈ 60 kB of the ≈ 548 kB free flash) with bilinear interpolation, plus the analytic frame rotation/derivative (a few dozen flops). The network is then folded into the table (same derivative structure), removing ROM and network cost altogether.
4. **Guards** the bundle lacks: r0 → 0 (frame normalisation, jerk blow-up), exact alignment, sign of Δz (upper vs lower), residual summed over N neighbours although trained on pairs, thrust floor and saturation, integral windup and first-step `dt`, yaw regulation (the bundle zeroes the yaw residual because of drift and has no yaw-angle feedback).
5. **Validation chain:** host unit tests against the bundle (my harness gives reference numbers), host SIL (our harness cannot run different controller modes per drone — the bottom drone would have to run FBL with the top drone scripted from its trajectory), single-drone hover/figure-8 (residual off), A8/A1 at low speed, then the study protocol.
6. **Unknown hardware details** the paper does not publish: hardware gains, α-β-γ coefficients, how thrust was mapped for the rate-command interface, training-data trajectory details.

## 9. Porting options for Strategy 3
| Option | What | Effort (estimate, uncertain) | Pros | Cons / risks |
|---|---|---|---|---|
| **A. Off-board, as in the paper** | FBL + residual in Python on the lab PC, thrust + rate commands streamed through Crazyswarm2 (400 Hz radio) | 1.5–3 weeks (rate-command interface in the cpp backend, thrust mapping, safety) | matches the authors' setup, quick residual swaps, torch available | radio load and latency/jitter with two drones (our radio saturated before), not comparable in loop architecture with the three onboard strategies (fairness), new low-level interface in CS2 |
| **B. Onboard port in our out-of-tree controller (recommended target)** | new FBL mode in `lib.rs` reusing `omega_desired`, peers, trajectory; residual as tables; gains tuned in SIL and bench | **≈ 4–6 weeks** incl. data/training (1–1.5), port + host tests (1.5–2), SIL/bench/hover (1), lab tuning on A8/A1 (1–1.5) | same architecture as S0–S2 → fair comparison; no radio dependence; uses our logging and analysis | most work; competes with the study schedule; risk that tuning needs several lab sessions |
| **C. Simulation-only** | run the bundle's FBL in our SIL as a theoretical baseline; cite the paper's hardware numbers | ≈ 1 week | cheap, no hardware risk | not a hardware result for Strategy 3; limited value for the thesis comparison |
| **D. Wait for the full repository** | authors announced a cleanup | 0 | may bring training code and hardware checkpoint | unknown date |
**Staged plan if Strategy 3 is pursued (B):** Stage 0 (desk, ≈ 1 week, no lab): send the questions (§10), build the residual tables from the bundle + our vehicle constants, implement the FBL law in numpy against my harness, add it to the SIL with a scripted top drone. Stage 1 (≈ 1.5 weeks): firmware mode + host tests. Stage 2 (1 lab session): single-drone hover/figure-8 with residual off, gains and estimator. Stage 3 (1–2 sessions): two-drone A8/A1 at low speed, then the retrained residual. Gate after each stage with the supervisor. Realistically ready for study flights **in December or later** — either as a late tier (after tier 1 of the study) or as a documented limitation.

## 10. Questions for the authors (draft e-mail — not sent)
> Dear Pei-An, dear authors,
> thank you very much for the code bundle — it is clear and we could reproduce the example (thrust 0.329 N), the analytic derivatives and the control mapping. For adapting the method to our setup (Crazyflie 2.1 Brushless, 41 g, onboard controller at ≈ 1 kHz, OptiTrack) we have a few questions:
> 1. **Licence / citation:** under which licence may we use and adapt the bundle in a master's thesis?
> 2. **Hardware model and gains:** could you share the hardware checkpoint (force-only network with 4 neurons, C_D = 0.236), the hardware controller gains (position/velocity/integral, rate loop) and the α-β-γ coefficients and control rate? In the bundle the rate gain (238 s⁻¹) is unstable at 100 Hz in our tests and stable at ≥ 400 Hz.
> 3. **Training:** is the KNODE training script (loss, ODE solver, data format) available, even in rough form? Alternatively, what data and trajectories did you use (28 s at 0.4 m / 0.4 m/s) and with which ROM parameters?
> 4. **Constants:** the code has `C_d_GT = 0.55` while the paper states 0.7 for the simulated plant — which one produced the sample checkpoint? Is the vehicle mass 0.03 kg also what the hardware controller used (the paper says 36 g)?
> 5. **Near-aligned stacks:** the frame normalisation and the residual jerk become singular when the horizontal offset goes to zero (exactly aligned: ≈ 10¹³ m/s³; 1/r₀ growth otherwise). How did you handle this on hardware?
> 6. **Yaw:** the wrapper zeroes the yaw angular-acceleration residual because of yaw drift and there is no yaw-angle feedback — how was yaw regulated in the experiments?
> 7. **Small points:** `FBLControl._position_commands` accumulates the integral with `dt = t − prev_t` starting at 0 (large first step if time does not start at 0), and `flatness_utils._vee_skew` returns −ω for S = hat(ω) (opposite sign of `omega_des`) — are these known?
> 8. **Release:** is a full repository release planned, and roughly when?
> Many thanks and best regards

## 11. Recommendation and decisions needed
1. **Ask the authors now** (low cost; items 2–4 decide the effort of option B). 2. Decide with the supervisor how Strategy 3 enters the thesis: (i) late study tier via option B (December+, if answers and time allow), (ii) simulation-only comparison + the authors' reported hardware numbers (option C), or (iii) documented as blocked/limited. 3. In any case keep the harness (`experiments/analysis/fbl_bundle_check/`) as the numerical reference for a later port. 4. Do not start firmware work before the study protocol is frozen and the shakedown is done.

## 12. Appendix — file by file
| File (lines) | Role | Key facts / risks |
|---|---|---|
| `LEARNED_FBL.md` (124) | guide | reading order, frames, 30-entry pair layout, ROM+NN only, no training code |
| `examples/learned_fbl_inference.py` (57) | single evaluation | `weights_only=True`; mass 0.03, J, rate gains 238/238/140; thrust 0.329 N |
| `KNODE/Models/sample_model.pth` | weights | 114 parameters, simulation 9-neuron force + torque nets, only `state_dict` |
| `KNODE/Training/nn_models_full_only_zr.py` (136) | `ODEFunc` (nets, training wrapper) + re-exports | training wrapper uses neighbour attitude, runtime does not; `C_d_GT` unused |
| `KNODE/Training/downwash_residual.py` (378) | runtime residual (acc, jerk, ang. acc) | frame +z (paper: −z); unsigned z; vz = 0; optional profiling/compile |
| `KNODE/Training/residual_derivatives.py` (260) | analytic derivatives | exact for tanh(2,9,3); 1/r0 term; exact-alignment blow-up |
| `KNODE/Training/nominal_dynamics.py` (269) | vehicle constants, nominal ODE, `point_line_distance` | 30 g constants duplicated; C_d 0.3 / S 0.0997 |
| `KNODE/Training/Velocity_Field/force_batch_fo.py` (243) | ROM force + partials | 10×10 trapezoid, float64, ≤ 0.55 % error; constants duplicated |
| `KNODE/Training/Velocity_Field/torque_batch_fo.py` (163) | ROM torque | disk radius/l differ from the force file; world-vs-flow frame mix negligible (≤ 0.5 %) |
| `KNODE/Training/flatness_utils.py` (541) | legacy offline flatness inversion / MPC warm start | `_vee_skew` sign flipped; ignores residual jerk; batch path sets r0 = 0 and zero angular residual; not used by the controller |
| `simulator_multi_python/Controllers/fbl_control.py` (317) | `compute_fbl_command` + `FBLControl` | Eqs. 14a/15/10/19 verified; mass/gains hard-coded; yaw residual zeroed; integral first-step `dt`; sim-only allocation |
| `simulator_multi_python/Controllers/acceleration_estimator.py` (400) | four estimators | α-β-γ on velocity only; poles fixed in discrete time → bandwidth ∝ call rate |

## 13. Verification statement
- Bundle: 12 files, SHA-256 of every file identical before and after all work; no `__pycache__`, no new files inside it; runtime tests only on a copy under the session scratchpad.
- Harness and results: `experiments/analysis/fbl_bundle_check/{fbl_checks_part1.py, fbl_checks_part2.py, out/*.json}`.
- Not done (by design): no training, no hardware, no firmware change, no e-mail sent, nothing committed.
