# DiffCoTune specification (frozen procedure, Stage 1a)

Written 11 Nov 2026. Source of decisions: `docs/plan_diffcotune_2026-11-11.md` (sections 1–3). Source of the method: Krishna et al. 2025, arXiv:2505.24068v1 (`docs/papers/krishna2025diffcotune.pdf`, 8 pages). Every paper statement below was checked against the PDF itself (page / equation / algorithm in brackets, "p." = PDF page); the summary `docs/papers/summaries/krishna2025diffcotune.md` was used only as a help. Where the paper or the plan does not say something, the text says **not stated** and the point is listed in section 6 or 7.

Nothing here has been implemented or run. Numbers that come from the record are marked with their file.

## 1. Notation

| Symbol | Meaning | Paper |
|---|---|---|
| x_t | state of the simulator at step t | Eq. 1, p. 2 |
| f | simulator, x_{t+1} = f(x_t, u_t; β), deterministic, x_0 known | Eq. 1, p. 2 |
| π | controller, u_t = π(x_t; θ) | Eq. 2, p. 2 |
| θ | controller parameters (gains) | p. 2 |
| β | simulator parameters that are fitted | p. 2 |
| x_t^sys | observable state of the real system ("directly measured or estimated") | p. 2 |
| x_t^* | desired state / reference | p. 4 |
| L | number of real tuning iterations; Alg. 1 loops l = 0 … L | Alg. 1, p. 3; L = 5, p. 4 |
| K | maximum number of epochs of one update; K = 100, K/2 for each inner loop | p. 4, Alg. 3 |

### 1.1 θ: gains per controller (names and defaults)

Names and start values are those of the simulator package (`simulator/frsim/controllers.py`); the plan does not change the gain sets, so they are taken as they are (see open choice 6.9).

| Controllers | Gains (default) |
|---|---|
| S0, S1, S2, S3 (10 gains, shared names) | Pxy 7.0, Pz 7.0, Dxy 4.0, Dz 4.0, Iz 1.5, KRxy 0.007, KRz 0.008, Kwxy 0.00115, Kwz 0.002, KIxy 0.03 |
| FBL (4 variants, 6 gains each) | kp_xy 14.0, kv_xy 9.5, ki_xy 12.0, kp_z 11.25, kv_z 5.4, ki_z 12.0 |
| FBL, never tuned | kr_xy 59.5, kr_z 35.0 (stock inner loop), Kpos_I_limit 2.0 (tree, `s0_torch.GAINS0`) |

Each of the 8 controllers has its own θ and starts from the defaults above (plan section 1: S2 / S3 do not reuse the S0 / S1 gains). Gains are clipped to stay positive after every Adam step; no other bound (plan section 2; paper clips Q ⪰ 0 for one experiment, p. 5). The clip floor is **not stated** (open choice 6.9).

### 1.2 β: fitted simulator values (start = frozen calibration)

Start values from `simulator/frsim/data/calibration.json` (calibration 2026-10-30, frozen 2026-11-10):

| β | Calibration key | Start value | Remark from reading the code |
|---|---|---|---|
| Thrust factor (max motor force) | `max_motor_force_N` | 0.2758040577 N | In the plant this is a **cap** on the real force, `min(0.1098 N · (command / (0.2 N · 0.618))², cap)` (`frsim/vehicle.py` `real_thrust`; PyTorch copies `_block`). It only acts at near full-scale thrust, see inventory item 1 and plan remark in section 7 |
| Motor lag | `motor_lag_ms` | 30.0 ms | continuous in both plants |
| RPM delay | `rpm_delay_ms` | 4.0501500631 ms | the firmware simulator truncates it to whole ms (`int(...)`, `frsim/sensors.py:54`); only S1 and S3 read the RPM |
| Downwash scale | `downwash_scale` | 1.3417475092 | multiplies the network force; only acts when the top vehicle is above |
| Plume amplitude scale | `plume_amplitude_scale` | 4.2565977626 (× 0.0537218 N = 0.22868 N) | multiplies the plume torque amplitude; plume width scale 1.0293, exponent 0.6014 stay fixed |

### 1.3 Fixed values

Mass 0.0427 kg; inertia J = diag(16.571710e-6, 16.655602e-6, 29.261652e-6) kg·m² (exact Omar C tree values, plant and controller, `frsim/config.py`); gravity 9.81 m/s²; thrust law constants, arm length, yaw torque constant, 1 ms physics step, 2 ms control period (S0–S3 and the PyTorch FBL copy), 4 ms command delay (`frsim/config.py`). Gyro noise 12.0755 deg/s and accelerometer noise 0.0840 g are fixed calibration values (not fitted).

## 2. Outer loop (paper Alg. 1, p. 3)

Paper Alg. 1: initialise β_0 = β̃, θ_0 = θ_best = θ̃; for l = 0 … L: roll out π(·; θ_l) from x_0 on the system to collect {x_t^sys}; (θ_{l+1}, β_{l+1}) ← UpdateStrategy(β_l, θ_l, {x_t^sys}); if J_task(θ_best, sys) ≥ J_task(θ_{l+1}, sys) then θ_best = θ_{l+1}; return θ* = θ_best.

As we run it, for each controller separately (plan sections 1, 2):

1. θ_0 = default gains, β_0 = calibration, θ_best = θ_0.
2. Rounds l = 0 … 5 (L = 5, so 6 rounds; p. 4 and Alg. 1).
3. Round l: fly θ_l once on each tuning scenario (A1, A8, circle, figure 8), uSD logging on. Real cost of θ_l (section 4).
4. Keep-best: after round l ≥ 1, θ_best = θ_l if the real cost of θ_l is not worse than the real cost of θ_best (Alg. 1 line 5 compares the real cost of the *new* gains θ_{l+1}, which is only known from the next real round). Round 0 sets the reference cost of the default gains.
5. UpdateStrategy = Split-Alternate (section 3) on the 4 flights of round l, for l = 0 … 4. The update after round 5 (θ_6) is never flown and not used (deviation 7 of the plan, section 5).
6. After round 5, θ_best is the best of the 6 flown gain sets and is the final gain set of the controller.

Checked against the paper: "single rollout in the target system per iteration" (p. 4); the paper reports "< 5" trials (p. 2) next to L = 5 with a loop over l = 0 … L (p. 3, p. 4) and figures with l up to 4 (Figs. 2, 6, 7); the count is not consistent in the paper, our reading is in plan deviation 7.

Detail that follows from Alg. 1 and Alg. 3 and is **not stated in the plan**: the update of round l starts from θ_l (the gains just flown), not from θ_best (Alg. 3 line 1 "initialize θ_0 = θ_l"; Alg. 1 line 4 passes θ_l). The spec follows the paper. The fitted β also continues from β_l (Alg. 3 line 1), never from the calibration after round 0.

## 3. Update strategy: Split-Alternate (paper Alg. 3, p. 4; Eq. 8, 9, p. 3)

Paper Eq. 8: β_{l+1} = argmin_β J_sysId(θ_l, β, {x_t^sys}_l) subject to the model dynamics under π(·; θ_l). Eq. 9: θ_{l+1} = argmin_θ J_task(θ, β_{l+1}) subject to the model dynamics with β_{l+1}. "While solving Eq. (8), θ is fixed and alternatively for Eq. (9), β is fixed" (p. 3). The paper's other strategy, Combined (Eq. 7, Alg. 2), is **not used**: the paper reports it diverges under model mismatch (p. 7).

Alg. 3 (p. 4): for k = 0 … K/2 while C is false: roll out π(·; θ_l) from x_0 in the model with β_k, compute ∇_β J_sysId, β_{k+1} = β_k − GradientUpdate(∇_β J_sysId). Then for i = 0 … K/2 while C is false: roll out π(·; θ_i) in the model with β (the result of the first loop), compute ∇_θ J_task, θ_{i+1} = θ_i − GradientUpdate(∇_θ J_task). Return (β, θ).

### 3.1 Fit β (θ = θ_l fixed)

J_sysId(β) = (1/4) Σ_{s ∈ {A1, A8, circle, figure 8}} (1/T_s) Σ_{t=1}^{T_s} ‖x_t^s(β) − x_t^{sys,s}‖²  (paper Eq. 3, p. 2, averaged over the 4 scenarios; deviation 5)

- Closed loop: the simulator flies the same controller, the same gains θ_l and the same reference as the real flight (paper: "our definition of J_sysId compares the closed-loop dynamics of the model with the system for a given controller", p. 2–3).
- State x = position (3), velocity (3), attitude, body rates (3): the observable states of the uSD logs (plan section 2). The sum of the individual squared errors of each part is unweighted ("for states that evolve in different geometric spaces (e.g. position in R³ and quaternions in S³) we compute an unweighted sum of the individual cumulative norm squared errors", p. 4). How the attitude error is formed (quaternion difference form) is open (6.3).
- Real data from the uSD logs only, never from radio CSVs (plan).

### 3.2 Tune θ (β = new β fixed)

J_task(θ) = (1/4) Σ_s (1/T_s) Σ_t ‖p_t^s(θ) − p_t^{*,s}‖²  with p = 3-D position

Paper Eq. 5 with c(x_t, x_t^*) = ‖x_t − x_t^*‖²₂ (p. 4; the paper's Eq. 4 / 5 allow c(x_t, u_t), the experiments use the state error only). We use the position part only (deviation 6). No crash penalty, no delay robustness, no selection-error weighting (plan section 1).

### 3.3 Optimiser, stop rule, gradients

- Adam for both inner loops (p. 4). K = 100 epochs in total, K/2 = 50 for each of the two loops (p. 4, Alg. 3). Learning rate: **not stated** in the paper (p. 4 says only that the split variant uses "identical design choices (learning rate and maximum epochs, K) as the combined variant"); Adam constants **not stated**; whether the Adam state is reset between loops **not stated**. All open (6.1).
- Stop rule C of each inner loop (p. 4): C := 1(|J_k − J_{k−1}| < 10⁻³) ∨ 1(J_k > J_{k−1} + 10⁻³), "to terminate under convergence or divergence due to unbounded updates". The thresholds are absolute. The returned iterate is the last one, also when the loop stopped on a rise (Alg. 3 line 12 returns β_k, θ_i); the paper does not say that the previous iterate is restored.
- Unit of J: **not stated** and decisive, see deviation 8 and open choice 6.2.
- Gradients by automatic differentiation along the rollout (paper: "Compute ∇ J along the rollout", Alg. 3 lines 4 and 9); the simulator and the controller must be differentiable (p. 2).
- The tuning simulator runs without sensor noise: the paper's model Eq. 1 is a deterministic function (p. 2); the plan states the same. Whether the paper's model includes any noise is not stated.

## 4. Real-flight cost for keep-best and reporting

Real cost of a gain set = mean over the 4 tuning scenarios of the mean squared 3-D position error of the real flight, computed from the uSD logs (never radio CSVs), over the same time window T as J_task and with the same reference. It is the quantity that replaces J_task(θ, sys) of Alg. 1 line 5, which the paper cannot compute without a rollout. Reported as 3-D RMS in cm (square root of the mean squared 3-D error, not the mean of per-scenario RMS values), plus the z error (RMS of the z component, cm). The time window T is **not fixed** by the plan (open choice 6.6).

The paper has no repeated flights; every curve is a single run (summary §4, checked: no seeds or error bars in Figs. 2–8). With one flight per scenario and round, the real cost carries flight-to-flight noise, and keep-best decides on it. This is a property of the method as run, not a deviation we introduce.

## 5. Deviations from the paper

No official DiffCoTune code exists: the paper links only a video (Fig. 1 caption, p. 1) and arXiv has one version (v1, 29 May 2025). "As in the paper" means Algorithms 1 and 3, Eqs. 1–9 and the stated settings.

Unavoidable (plan):

1. **Quadrotor**, not cart-pole, quadruped or biped (Table I, p. 4, has no quadrotor).
2. **PyTorch** differentiable copies of S0–S3 and FBL, not JAX / MuJoCo-XLA (paper, p. 4).
3. **Our β values** (thrust factor, motor lag, RPM delay, downwash scale, plume amplitude scale) instead of link masses, friction, gear ratio (Table I, Fig. 8); mass and inertia are fixed (measured).
4. **Learning rate** is not stated by the paper; we choose one and state it.

Decided (user, 11 Nov):

5. **Several tasks:** the paper uses one task from one start state per loop; we average the cost over 4 tuning scenarios, one flight each per round, because one gain set must serve all scenarios. This is the largest deviation.
6. **Task cost on position only.** The paper's c is ‖x − x*‖² of the desired state (p. 4); we use the 3-D position. Note: the paper's "desired state" is not the full state in every experiment (quadruped: heading velocity, yaw rate, height, p. 5; biped: base position commands, p. 5), so the paper itself uses task-specific subsets; the plan's wording "full desired state" is slightly too strong.

Our reading of unclear points:

7. **Keep-best after the last round:** Alg. 1 needs the real cost of θ_{l+1}; the last update is never flown. We fly l = 0 … 5 and pick θ_best among the 6 flown gain sets; θ_6 is not used.

Found in Stage 1 (checked against the PDF and the code):

8. **Stop-rule units (found in Stage 1).** The paper's absolute thresholds of 10⁻³ (p. 4) were set on losses of order 0.2–1 (axes of Figs. 6, 7). In SI units our J_task is a mean squared position error in m²: a 3-D RMS of 3 cm is 9·10⁻⁴ m², already below the threshold, so the rule |ΔJ| < 10⁻³ would stop every inner loop at its first epoch. The unit (or a normalisation) of J must be fixed before the dry run (6.2). Not a paper statement, a consequence of applying it to our scale.
9. **Start state and internal controller state (found in Stage 1).** The paper rolls the model out "from x_0" and assumes x_0 known (Eq. 1, p. 2), the same for model and system. Our real flights start on the ground (takeoff ramp, hold, scenario, landing ramp, 1 s ground phase in the simulator, `frsim/config.py`) with unknown values of the controller states (integrators, INDI filter states, estimator states) and slightly different start positions. The simulator start must be taken from the log or the whole flight including takeoff must be simulated (6.5).
10. **What x^sys is (found in Stage 1).** The paper allows states "directly measured or estimated" (p. 2). Ours are the onboard estimates (EKF position and velocity, attitude quaternion, gyro rates, filtered onboard by the gyro low-pass) with the mocap delay, while the simulator states are the true states. The measurement chain is not part of β (6.8).
11. **Mismatches outside β (found in Stage 1).** Known sim-to-real differences that no β covers: state delay (the variant `md10` is "measured on the drone", `frsim/variants.py`), feedforward scale ("hardware behaves like about 0.4 in A8", same file). The plan does not say whether the tuning simulator keeps them at zero / 1 (6.8).
12. **Return iterate of a stopped loop (found in Stage 1).** The paper returns the last iterate also after a divergence stop (Alg. 3 line 12); the plan does not say otherwise. Followed as in the paper; listed because it makes the stop rule less safe than "keep the best iterate".

## 6. Open choices, to be fixed in Stage 3 by the dry run (listed, not decided)

1. Adam learning rate (deviation 4), β1, β2, ε, state reset between inner loops and between rounds.
2. Unit / normalisation of J_sysId and J_task for the stop rule (deviation 8), and of the individual state parts of J_sysId (m, m/s, rad, rad/s have different magnitudes; the paper's "unweighted sum" fixes the form, not the units).
3. Time step and horizon of the rollouts: physics step 1 ms and control period 2 ms exist; the copies are run in chunks of 1.5 s with detached carry (truncated backpropagation) in the earlier tuning; whether the sysId and task rollouts cover the whole flight (≈ 20–30 s with takeoff and landing) with gradient through the chunks (`--carry_grad` with clips existed for FBL / S1) or a window.
4. Scaling and form of the attitude part of J_sysId (quaternion error form; the paper only says individual norm-squared errors are summed, p. 4).
5. Initial state and internal controller state of the simulator rollouts (deviation 9).
6. Time window T of both costs and of the real cost (scenario time ≥ some start, takeoff and landing included or not). The earlier scoring used errors from 2 s after the scenario start (`SCORE_AFTER_S`, `frsim/config.py`).
7. Parametrisation of β and θ for Adam: the five β differ by orders of magnitude (30 ms, 0.2758 N, 1.34, 4.26, 4 ms) and Adam's step is about the learning rate in each coordinate, so a relative (log) parametrisation or per-parameter scaling is needed (the earlier gradient tuning used log factors around the defaults, `fbl_plant_torch.gains_from`).
8. Which sim-to-real mismatches the tuning simulator carries (state delay, feedforward scale, noise off) (deviations 10, 11).
9. Gain sets θ per controller (the plan does not list them; here the existing 10 / 6), the clip floor, and whether S2 / S3 gains include the same 10 names.
10. Whether the 4 FBL variants use the same 6 gains; whether the net-only variant (FBL + learning-based part without the ROM) is defined as ROM off + network on (inventory item 2).
11. Treatment of a crashed real flight: plan says repeat, not used for the fit; the threshold of "crashed" in the log is not defined.

## 7. Remarks found while checking the plan against the paper and the code

- Plan table row "Fixed … sensor noise" and plan section 2 "tuning simulator runs without sensor noise" are consistent only if the noise *values* stay fixed (not fitted) and are switched off in the tuning rollouts; this is our reading.
- The plan's dry-run shift "thrust factor × 0.7" acts on the force cap only (item β, section 1.2): with the cap 0.2758 N × 0.7 = 0.193 N, and a hover force of 0.105 N per motor, it changes the flight only where a motor needs more than 0.193 N. The calibration's name for this parameter says "max motor force", the plan calls it "thrust factor". If a real thrust *gain* is meant, a new multiplicative parameter is needed (inventory gap list).
- Paper typo: p. 7 says "wtask = wsysId = 1 in Eq. (3)" where Eq. (7) is meant (the same statement on p. 4 says Eq. (7)). Irrelevant for Split-Alternate (no weights).

## 8. Changes after Stage 2 (S0 end to end, 11 Nov 2026, marked "Stage 2")

Decisions of the user after Stage 1 (plan section 5b) and what the S0 implementation (`experiments/diffcotune/`) added or showed:

1. **Stage 2: thrust β is the thrust-law gain** (`THRUST_LAW_GAIN_N`, 0.1098 N), not the force cap (section 1.2 row 1 is superseded). The cap `max_motor_force_N` (0.2758 N) is fixed. S0 β = {thrust-law gain, motor lag, downwash scale, plume amplitude scale}; there is no RPM delay in S0 (S1 / S3 only, later as a smooth fractional delay).
2. **Stage 2: parametrisation (open choices 6.7 and 6.9, decided for S0).** Gains θ and β are optimised as log-factors relative to the defaults / the frozen calibration (`phi`, `psi` = 0 at the start). Positivity is automatic, so the clipping rule of section 1.1 is not needed for S0; there is no bound.
3. **Stage 2: rollouts (open choice 6.3, for S0).** A whole flight (takeoff, hold, scenario, landing, 15 - 24 s, 1 ms physics, 2 ms control) is one differentiable rollout with `torch.compile`; no chunking or truncated backpropagation. Eager autograd stores about 1 MB per 2 ms block (10 GB per flight) and was not usable; compiled it is about 1 GB. The forward model agrees with frsim to 0.015 % in 3-D RMS (30 cells), see `experiments/diffcotune/checks/forward_check.log`.
4. **Stage 2: costs in SI units.** At the default gains J_task (m²) is 1.0e-2 (A1), 2.7e-3 (A8), 1.6e-3 (circle); the 1e-3 stop rule of the paper (deviation 8) is therefore inside the range of J_task as soon as the error is below about 3 cm, and still has to be rescaled (open choice 6.2, unchanged). J_sysId at the shifted plant of the gradient check is 3.6e-2 (A1), 0.46 (A8), 7.0e-3 (circle).
5. **Stage 2: identifiability seen in the gradients (new, relevant for the loop).** (a) The plume amplitude scale has a gradient only on flights with a top vehicle that moves sideways: A8 (2.0e-3 on J_task); on A1 it is zero to numerical noise (the four rotors see the same plume, the torque cancels), on the solo circle exactly zero. In the four tuning scenarios {A1, A8, circle, figure 8} only A8 informs it. (b) The downwash scale acts on every flight (also solo, through the ground effect of the network, which it scales). (c) The yaw gains KRz, Kwz have exactly zero gradient on A1 and A8 (no yaw excitation); the lateral gains and attitude gains have exactly zero gradient on A1 (no lateral motion without noise).
6. **Stage 2: gradient quality of A1.** At the default gains the noise-free A1 flight settles at a vertical equilibrium (z = 0.4147 m for a 0.5 m reference, integrator at its limit) that is dynamically marginal; the autograd gradient of the weakly acting parameters (Dz, motor lag, plume) there differs from central finite differences by 0.6 - 28 % (absolute 4e-6 against a largest gradient of 0.09) and eager and compiled autograd differ from each other by the same amount. The gradient check on A1 therefore does not meet the 1 % gate for those parameters; A8 and the circle meet it (worst 0.47 %). Finite differences need h = 1e-6 (in log-parameters); h = 1e-4 leaves a truncation error of up to 5 %. For Stage 3: weight or omit A1 for the parameters it cannot resolve, or add the sensor noise of the real flight to the fit rollouts; the spec's "tuning simulator without sensor noise" (section 3.3) is the reason the equilibrium is exactly noise-free.
7. **Stage 2: scenarios.** CIRCLE, FIG8, OVAL exist in frsim and in the model (parameters from the lab's `mode1` files, `simulator/docs/SCENARIOS.md`). They are faster than every A-scenario (peak speed 0.67 - 0.90 m/s against at most 0.44 m/s, peak acceleration 2.3 - 3.0 m/s² against at most 0.3 m/s²), and the oval (1.6 m x 4.4 m) exceeds the tape-measured flight volume (±2.0 m in y): plan section 1 "test scenarios: A4, oval" and Stage 4 flight files need a decision on the oval size.
8. **Stage 2: runtime of the S0 model (sizes Stage 3; `experiments/diffcotune/checks/runtime.json`, `runtime_batch.log`).** One whole-flight rollout, `torch.compile`, 4 CPU threads: forward 1.4 - 2.2 s, forward + backward 5.0 - 8.3 s (A1 2.0 / 7.9, A4 2.1 / 8.3, A8 2.1 / 8.2, circle 1.4 / 5.0, figure 8 1.6 / 7.0, oval 2.2 / 8.2 s), peak 1.7 GB; one frsim episode 1.8 - 3.5 s; the first compiled call takes about 30 s. GPU (float32, compiled): 7 - 8 s forward, 22 - 25 s forward + backward at B = 1 and 8.3 / 27.3 s at B = 16 (launch bound: extra rows are almost free, one row is slower than the CPU). Compiled CPU rollouts with B > 1 fail in torch 2.5.1 (inductor C++ codegen error); eager is about 25 s forward and 60 - 180 s forward + backward at 10 GB. One round without early stop = 2 x 50 epochs x 4 scenarios x 8 s = about 50 min on one CPU process; with the 4 scenarios in 4 parallel processes (gradient = sum) about 13 min, which is at the target of 15 min (plan section 4 Stage 3); the stop rule (spec 3.3) can only shorten it. The GPU path needs CUDA graphs or batching of the four scenarios (padded to the longest) to beat that.

## 9. Changes after Stage 3 (S0 dry run, simulator to simulator, 11 / 12 Nov 2026, marked "Stage 3")

Setup and results: `experiments/diffcotune/dryrun/` (`report_stage3.md`, plots, JSON in `results/`, diagnostics `diag_fit*.py`). "Real drone" = frsim, noise on; P1 = thrust-law gain x 0.8, motor lag 45 ms, downwash x 1.3, plume x 0.7; P2 = P1 + state delay 10 ms. Learning rate frozen at 0.01 by the pilot (lowest real cost after round 2: 30.8 cm against 44.3 cm at 0.03 and 67.2 cm at 0.1; all three worse than the default gains, 19.8 cm).

1. **Stage 3: choices made for the loop where the paper is silent (our choices, to be stated in the thesis).** Adam beta1 0.9, beta2 0.999, eps 1e-8, state reset at the start of every inner loop; one epoch = one Adam step on the mean cost of the 4 tuning scenarios; the stop rule is evaluated on the cost at the current iterate (before the step), the iterate that triggered it is returned (the paper: the last iterate, also after a rise); a non-finite cost returns the previous iterate; the update of round l starts from theta_l (paper); a crashed flight is repeated with another seed (never happened); the fit after round 5 is done only to report beta; the real cost of a gain set = mean over the 4 tuning scenarios of the mean squared 3-D error over the task window (scenario time > 2 s), from the true state trace.
2. **Stage 3: the stop rule normalised by the cost at the start of the loop (decision 4) is not sensible as specified, recommend change.** It fires on the size of ONE step, not on convergence: the beta fit stopped after 2 - 5 epochs in every round and both plants (P1: 3 x "converged" at |dJn| < 0.001 with Jn 0.993 - 1.000 and 3 x "rose" at Jn 1.003 - 1.010; P2: 4 and 2; a "rose" is a rise by about 1 % after one step of 0.01), so beta never moved (log-factors changed by at most 0.08 over all rounds). Cause: J_sysId contains a noise floor that does not depend on beta (below), so a normalised change per step is below 0.1 % for a small learning rate; the learning rate then decides whether the loop stops, not the cost. The theta loops end at the 50-epoch limit, after "converged" at 11 - 38 epochs, or after "rose" (3 - 43 epochs); a "rose" return gives the risen iterate (in the pilot at lr 0.1, round 1, the returned gains had model Jn 2.43: the model cost more than doubled). Evidence that the fit works without the rule (diagnostics, not the method): clean P1 flights, 100 epochs at lr 0.03, no stop: beta = (0.803, 1.51, 1.34, 0.696) against the truth (0.8, 1.5, 1.3, 0.7); noisy P1 flights of theta_0, same: (0.74, 1.47, 1.02, 0.76), i.e. thrust gain -7.5 %, lag -2 %, downwash scale -21 %, plume +9 % off. Options to decide: minimum number of epochs, a window (change over 5 - 10 epochs), the best instead of the last iterate.
3. **Stage 3: J_sysId (unweighted sum of the parts, spec 3.1) is dominated by the body-rate part under noise.** At theta_0 and the calibration, per scenario (noisy real flights): A1 J = 11.5 of which w 10.8 (94 %), A8 0.63 (w 0.62), circle 0.065 (w 0.062), figure 8 0.063 (w 0.061); the sum over the four scenarios is 94 % body rates. The noise-free model cannot reproduce the rate jitter excited by the gyro noise (and the A1 flight with default gains oscillates with noise: real 3-D RMS 37 cm, tilt up to 38 degrees in frsim), so this part is a floor; position and velocity, the parts that carry beta, are 5 % of J (almost all of it in A1). Also without noise the A8 cost is 97 % body rates (0.286 of 0.296) and its gradient at beta_0 does not point to the truth for three of four values (descent direction (+0.54, -1.1, -0.22, -2.25) against the direction to the truth (-0.22, +0.41, +0.26, -0.36), plume correct): the landscape is not convex in beta; only a long fit converges (clean data above). Open choice 6.2 (unit / weight of the parts) therefore matters and is no longer "unit only".
4. **Stage 3: results (P1; P2 as information).** DiffCoTune did not meet the plan's criteria for beta and not "(d) not worse than (b)": (a) beta after round 5 is essentially the start value (thrust gain +26.8 % / +21.1 % off in P1 / P2, lag -37.9 % / -35.2 %, downwash -25.9 % / -24.7 %, plume +32.1 % / +33.4 %, at the start +25 / -33 / -23 / +43 %), (b) final evaluation on the real drone, mean 3-D RMS over the 4 tuning cells and 5 seeds: default 15.3 cm (P1) / 15.0 (P2), tune only 7.4 / 6.9, sysId then tune 63.8 / 15.5, DiffCoTune 8.6 / 7.1; on the test cells (A4 x 2, oval): 9.0 / 9.0, 9.9 / 9.8, 62.9 / 19.5, 10.8 / 9.7, i.e. the tuned gains are better on the tuning scenarios and worse than the default on A4 in both plants (P1 13.7 / 15.5 cm against 8.8 / 13.2, P2 12.2 / 13.9 against 8.9 / 12.8; the oval is better, 3.2 against 5.2 cm). (c) Real cost per round of DiffCoTune: 19.8, 11.9, 30.8, 27.3, 42.0, 72.6 cm (P1), 19.3, 8.6, 37.0, 24.8, 52.3, 32.6 cm (P2): theta_best is round 1 of the loop in both plants (keep-best is what makes DiffCoTune better than the default; the later updates, each started from the gains just flown in a model that is not fitted, make the real flight worse). Tune only: 19.8, 9.4, 13.8, 28.0, 26.2, 23.0 cm (P1). No crash in any final evaluation (4 methods x 2 plants x 7 cells x 5 seeds) and none in the loops.
5. **Stage 3: Adam on the weak parameters.** Motor lag and plume scale move together by lr per step (Adam normalises the step; both log-factors reach -0.08 at most). The autograd gradient agrees with the central finite difference (h = 1e-6, checked at epoch 0 of rounds 0, 2, 4, both plants) in sign everywhere and to 0.0 - 3.6 % (one value 11 %); the magnitude varies strongly with theta_l: motor lag d J_sysId / d psi = +0.27 ... +0.34 in round 0, +25 / +16 (P1) and -31 / -27 (P2) in rounds 2 and 4 (real flights of tuned gains oscillate), so the sign of the lag step changes between rounds.
6. **Stage 3: wall time.** Per round (fit + tune, 4 scenarios in 4 parallel processes, 2 threads each): P1 10.9, 7.0, 9.4, 9.0, 2.8, 0.4 min, mean 6.5; P2 10.6, 7.8, 1.3, 3.8, 1.3, 0.4 min, mean 4.1 (the last value is the fit only). One epoch = about 10 s; without early stops a round would be 100 epochs = 17 - 19 min, above the 15 min target; the target is met only because the stop rule ends loops early (see 2). Four workers need about 7 GB; two plants could not run at once (12 GB free).
7. **Stage 3: for the other controllers (plan stage 5 onwards).** The three points above (stop rule, weight of the rate part, start of the update from the last flown gains) have to be decided before any hardware round; the A1 scenario with default gains and noise is an oscillating flight that no noise-free model reproduces, and A4 (lateral offset passes) was not represented in the tuning scenarios.

## 10. Changes after Stage 3b (S0 dry run, rerun with the decisions of 12 Nov 2026, marked "Stage 3b")

Setup and results: `experiments/diffcotune/dryrun/stage3b/` (`report_stage3b.md`, plots, `results/*.json`, `logs/`). Same "real drone" as Stage 3 (P1; noise on; model without noise). The Stage 3 results in `dryrun/results/` are unchanged and reproducible with the default switches (`--stop-rule normalised --states pvqw`). P2 was started after P1 and stopped by the user on 12 Nov (partial log `logs/full_P2.log`, marked not used).

1. **Stage 3b: deviation 8 (stop rule), replaces the normalised stop rule of Stage 3 (decision 4 of 11 Nov).** Every inner loop (fit and tune) runs the full 50 epochs and returns the iterate with the lowest model cost (best iterate, not the last one). One extra evaluation of the final iterate, so 51 evaluations per loop. The paper stops at |ΔJ| < 1e-3 or a rise > 1e-3 and returns the last iterate; K = 100 (2 x 50) is kept. Switch: `--stop-rule full50best` (default `normalised`).
2. **Stage 3b: deviation 9 (states of J_sysId).** Position, velocity and attitude (squared norm of the quaternion difference with sign alignment, as before), unweighted; the body rates are left out. The paper uses all observable states. Reason: under noise the body-rate part is 94-98 % of the Stage 3 J_sysId and a noise-free model cannot match it. Switch: `--states pva` (default `pvqw` = Stage 3). Step B1 decided between pva and pv.
3. **Stage 3b: B1 identifiability (P1, noisy flights of theta_0, 2 noise seeds, 50 epochs, best iterate).** Error of the fitted beta against the truth (thrust / lag / downwash / plume): pva at learning rate 0.01: -1.7 / -34.5 / +5.7 / +22.3 %; 0.03: +5.2 / -26.6 / +34.5 / +11.5 %; 0.1: +6.8 / +16.6 / +35.1 / -7.0 %. pv: -2.0 / -34.0 / +5.8 / +22.0 %, +4.7 / -22.4 / +37.7 / +8.2 %, +5.5 / +10.4 / +39.1 / -4.1 %. Chosen: pva (as good or better on the thrust gain at 0.01 and on downwash at 0.03 and 0.1; the difference between the two sets is small). Downwash is within 10 % only at 0.01, where the lag is still 34 % off: not all four within 10 % for any setting. In the full four-part cost the body rates stay about 95 % at start and end.
4. **Stage 3b: learning rate 0.01 (frozen).** Pilot, P1, rounds 0 / 1 / 2 real 3-D RMS: 0.01: 19.80 / 7.98 / 16.93 cm; 0.03: 19.80 / 43.81 / 56.25; 0.1: 19.80 / 21.45 / 23.48 (lowest after round 2: 0.01).
5. **Stage 3b: results P1 (final evaluation, mean 3-D RMS over 5 seeds, cm; crashed flights included).** Tuning cells / test cells: (a) default 15.29 / 9.04; (b) tune only 8.70 (4 crashed A1 flights; 3.45 over the 16 others) / 15.11; (c) sysId then tune 5.65 / 8.58; (d) DiffCoTune 7.08 / 7.11. Real cost per round of (d): 19.8 / 8.0 / 16.9 / 69.8 / 95.3 / 66.7 cm; theta_best = round 1. beta after round 5: thrust -47.1 %, lag -78.9 %, downwash -36.7 %, plume -78.5 % (after round 1: thrust -1.8 %). Every inner loop of the P1 (d) run had its best iterate at the last epoch (epoch 50, 51 evaluations). Wall time per round 17.1-19.1 min (mean 16.3 incl. the fit-only round 5).
6. **Stage 3b: pass criteria P1.** beta toward the truth (thrust within 5 % after round 5): no. (d) beats (a) on tuning and test cells: yes. (d) not worse than (b) and (c): no (tuning cells: (c) 5.65 < 7.08). No crash of theta_best: yes (tilt rule; flights that stay on the floor are not flagged by it). Wall time about 20 min per round or less: yes. P2: not run to the end (stopped by the user).
7. **Stage 3b: why (c) failed in Stage 3 (63.75 cm).** The stop rule ended every beta fit after 2-4 epochs, beta stayed at the calibration, 174 epochs of gain tuning followed in that unfitted model without a flight in between, the gains went to x3 to x4 on Dxy, Dz, Iz and the real drone lost height (z RMS 62.3 of 63.75 cm). With the Stage 3b rule the same procedure gives 5.65 cm.
8. **Stage 3b: switches added (defaults unchanged).** `adam_inner(..., stop, best)`, `run_loop(..., stop, best, skip_last_fit)`, `Pool(states=...)`, `costs.j_sysid(..., states=...)`; results of a run go to `--out`. `skip_last_fit` is Stage 3c (section 11).

## 11. Changes after Stage 3c (cause of the drift, speed-up; 12 Nov 2026, marked "Stage 3c")

Setup and results: `experiments/diffcotune/dryrun/stage3c/` (`report_stage3c.md`: Part B desk checks and Part C speed-up; scripts `partb.py`, `clean_flights.py`, `c1_memory.py`, `c2_threads.py`, `job_runner.py`, `gpu_probe.py`, `c3_gates.py`; `results/`, `logs/`). No change of the method, the costs or the gates.

1. **Stage 3c: the drift of (d) on P1 is not the fit alone.** Gain sets of rounds 3-5 fly without noise (A1 1.2 / 1.6 / 0.7 cm) but not with noise (A1 stays on the floor, 3-D RMS 50.0 cm, z mean 0.00 m; CIRCLE theta_3 2.7 cm without, 83.2 cm with noise). The model with the true beta agrees with noise-free frsim to a few percent (theta_3: 1.2 / 14.6 / 2.7 / 42.8 cm against 1.2 / 14.0 / 2.7 / 42.3). The tuning model has no sensor noise, so the tuning can move gains (Dxy x2.5, Iz x3, KIxy x0.3 in round 3) that only work noise-free. A beta fit started at the true beta stays within 10.4 % of it on the failed flights (cost nearly flat); one tune loop with the true beta from theta_0 gives 7.29 cm on the tuning cells (test 7.00), the same level as theta_best of (d). Open, would change the method: noise (or a noise-robust term) in the tuning model.
2. **Stage 3c: velocity part.** The trace holds the true plant state, not the noisy estimate; the large velocity part in rounds 0-2 comes from A1 (0.55 / 0.51 / 0.40 against 0.001-0.02 for the others), whose real flight oscillates under noise.
3. **Stage 3c: speed-up, measured.** `TORCHINDUCTOR_COMPILE_THREADS=1` (now set by `loop_lib.py`): 98 processes -> 6, PSS 7552 -> 6117 MB for a pool, numbers bit-identical. Torch threads per worker 2 / 4 / 8: no gain (about 10-11 s per epoch; 2 stays). A job runner with the limits (16 processes, load 22, 3 GB) exists and passed its self-test; two pools do not fit in 12 GB. Option `--skip-last-fit` (default off): saves the unused fit of the last round (8.5 min of about 96 min per loop).
4. **Stage 3c: GPU scenario batch is not faster.** `s0_batch.py` (rows = scenarios) in float64: 37.9 s per epoch against about 10 s on the CPU pool (about 64 min per round against 17-19); 24 rows cost the same (launch bound). Float32: forward passes (1e-3 %), gradient gate fails; float64: forward passes, J_task gradient passes, J_sysId gradient fails at the default point (marginal A1) and passes at the working point. `mode="reduce-overhead"`: no result in 36 min. The CPU path stays the default.


## 12. Changes after Stage 3d (noise in the tuning model, gain limit; 12 Nov 2026, marked "Stage 3d")

Status: **implemented, checked and run (13 Nov 2026):** forward check with noise passes the revised gate, gradient check with noise fails, variants A, B and A + B run and evaluated on P1. Details and numbers in `experiments/diffcotune/dryrun/stage3d/report_stage3d.md`.

10. **Deviation 10 (Stage 3d, variant A): sensor noise in the tuning step.** The paper's model rollouts are deterministic and noise-free. In variant A the task cost J_task of a tuning scenario is the mean over 2 fixed noise seeds per round, using frsim's own noise sequence (`s0_model.frsim_noise`: position std 0.0005 m, velocity std 0.005 m/s, gyro 12.0755 deg/s, white Gaussian, no filter, no bias; the same stream as `sensors.py` for the same seed). The noise is data, not a parameter. J_sysId and the beta fit stay noise-free.
11. **Deviation 11 (Stage 3d, variant B): per-round gain limit.** The paper has no bound besides positivity. In variant B, after each Adam step of the tuning loop every gain is projected to within a factor 1.25 of the flown gains theta_l (|phi - phi_l| <= log 1.25); the number of projected steps per gain is recorded per inner loop (`--gain-limit 0.25` in the code, default off).
12. **Floor rule (Stage 3d):** a flight whose mean z in the scored part is below 50 % of the mean reference z counts as failed, in addition to the tilt rule; recorded per flight, not used to repeat a flight.
13. **Forward check with noise (Stage 3d), gate = our choice (set by the user on 13 Nov, not from the paper):** 0.5 % relative 3-D RMS per cell, but only in cells where frsim itself moves < 0.5 % under a 1e-6 change of the thrust-law gain; in the other cells the model must show the failure too (floor rule failed, or 3-D RMS > 3x that cell's round-1 value). Cells: default, round-1 and round-3 gains of Stage 3b loop (d) x 4 tuning scenarios x seeds 1000, 1001 = 24. Result: **pass**; 20 of 24 cells stable and within 0.5 % (max 0.30 %); the 4 sensitive cells (all round 3: A8 s1001 5.7 %, CIRCLE s1001 8.6 %, FIG8 s1000 8.9 %, FIG8 s1001 11.8 % change under 1e-6) show the failure in the model. Per-cell table: `experiments/diffcotune/dryrun/stage3d/logs/noise_gate.log`. (The first gate, 0.5 % in every cell, failed in 3 cells before it was replaced.)
13b. **Gradient check with noise (Stage 3d):** combined cost, 4 scenarios, seeds 1000/1001, all 10 gains, default and round-1 points; gate same sign and relative <= 1 % or absolute <= 0.1 % of the largest component. Result: **fail**. h = 1e-5: both points fail; h = 1e-6: round-1 point passes, default point fails in KRxy only (1.68 % relative, 0.45 % of the largest). The mismatch comes from A1 (floor contact, finite difference depends on h). The runs went on as instructed.
13c. **Speed (Stage 3d):** 8 rollouts per epoch: CPU pool of 4 workers x 2 rows 12.5 s steady (about 11 s measured in the loops); GPU float64 batch of 8 37.9 s. CPU used; about 18-19 min per round. For compiled CPU batches with more than one row, Inductor's `try_loop_split` is replaced by the identity (workaround of an Inductor crash, loop structure only).
13d. **Results (Stage 3d, P1, one run each, 4 rounds):** real cost theta_0..theta_3 in cm: Stage 3b (d) 19.80 / 7.98 / 16.93 / 69.77; A 19.80 / 8.70 / 8.86 / 11.38; B 19.80 / 10.88 / 20.64 / 97.03; A + B 19.80 / 10.16 / 8.53 / 6.86. Final evaluation (5 seeds), tuning / test mean cm: default 15.29 / 9.04; A theta_best 7.66 / 6.40, theta_3 8.71 / 6.89; B theta_best 9.22 / 7.27, theta_3 94.06 / 99.86; A + B (theta_best = theta_3) 5.60 / 6.89. Pass criteria 1 / 2 / 3: A no / yes / yes; B no / no / no; A + B yes / yes / yes. The default gains already fail the floor rule on A1 in P1; criterion 2 is judged as "no flown set after theta_0 fails a scenario that theta_0 or theta_1 passed". Full tables: `experiments/diffcotune/dryrun/stage3d/report_stage3d.md`. Not measured: run-to-run spread.
14. **Code note (Stage 3d):** the keep-best variable of `run_loop` shadowed the flag `best`; renamed, no effect on the Stage 3b and 3c results (their records show the best-iterate path in every inner loop).

## 13. Stage 3e / 4a (13 Nov 2026, marked "Stage 3e / 4a")

14. **Method version (user decision 13 Nov):** DiffCoTune Split-Alternate with deviations 10 and 11 as written: sensor noise in the tuning step (2 fixed seeds per round), ±25 % per-round gain limit, 50 epochs with the best iterate, J_sysId = position + velocity + attitude, lr 0.01, last-round fit skipped.
15. **Gradient check with noise accepted (user decision):** 19 of 20 components pass at h = 1e-6. KRxy at the default point fails by 1.68 % (A1, floor contact). Noted, not blocking.
16. **Patch gate (Part A1):** compiled, patched batched rollout (B = 2 rows, noise seeds 5000/5001) against the eager float64 model, 4 scenarios x {default, θ₁, θ₃ of Stage 3d A + B} x P1. Max relative J difference 4.49e-15, max relative gradient component 2.18e-10 (gate: 1e-9 and 1e-6, our choice). PASS. `dryrun/stage3e/patch_check.json`.
17. **Full run P1 (6 rounds, new seeds):** real cost 19.99, 9.71, 8.30, 8.16, 8.13, 7.77 cm. Pass criteria a, b, c all yes. Final evaluation (5 seeds): tuning mean 15.36 -> 6.39 cm, test mean 9.02 -> 7.76 cm. `dryrun/stage3e/results/pass_P1.md`.
18. **Full run P2 (P1 + 10 ms state delay, information only):** real cost 19.28, 9.77, 7.62, 7.60, 7.70, 8.21 cm; keep-best round 3. Tuning mean 15.01 -> 6.29 cm, test mean 9.27 -> 7.65 cm (θ_best). `dryrun/stage3e/results/pass_P2.md`. The β error in that table is against the P1 truth; the 10 ms delay is not a β parameter.
19. **Real-flight ingest (Stage 4a, `experiments/diffcotune/usd_ingest.py`):** merged uSD files only. Frame (rotation + translation, A4 −90°) and time lag are estimated per flight and stored in the manifest. The lag (A4 40–58 ms, A8 82–120 ms) comes from `merge_usd_logs.py --meta`, which sets t = 0 by matching measured position to the commanded trajectory. It is fitted per flight and reported. A1 lag not observable (0).
20. **Quaternion turn by conjugation** (q_m = qz(θ) q qz(−θ), body rates w_m = R w) assumes symmetry of the airframe under 90° yaw. Jx 16.57e-6, Jy 16.66e-6 (0.5 %); accepted by the user.
21. **Logged top-drone path is the default peer input for A1 and A8** (`round.py --top auto`). 23 Oct A1: model 10.1 cm (planned path) -> 20.4–20.7 cm (logged) against real 20.5–22.1 cm.
22. **Flight files** (`experiments/diffcotune/flightfiles/`): degree-7 Hermite pieces ≤ 0.5 s, `%.12g`, matched to frsim ≤ 0.003 mm; figure 8 turned by 90°.
23. **Dry test of `round.py`** (4 frsim flights as fake merged files): θ₁ and β identical to `loop_lib` (max difference 0.0); yaml round trip 2e-16; float32 gain error 4e-8; round time 1163 s. The dry test sets `strict=False` because `loop_lib` fits the floor flight of θ₀ on A1; the lab default repeats such a flight.


## 14. Stage 4b (13 Nov 2026, marked "Stage 4b")

24. **Session kill 14:52:53:** two jobs with one worker pool each (`dry_test.py` round 1 and `dry_round1_strict.py`, started together by `dryrun/stage4b/start_strict.sh`) took MemAvailable to 1.2 GB; systemd-oomd killed the terminal scope. Rule since then: one worker pool at a time. `start_strict.sh` is marked "NOT USED"; `dry_round1_strict.py` no longer waits for `dry_test.py`; both run in one chain through `dryrun/stage3c/job_runner.py`. `dry_test.py --from-round 1` keeps `lab/dry/` and runs round 1 only.
25. **Dry test, round 1 (gate passed):** θ₂ and the round-1 β identical to `loop_P1.json` (max difference 0.0, gate 1e-12); real cost 9.713 cm (`loop_lib` 9.711; gate ±0.01); keep-best = round 1; `round_2/gains.yaml` round trip 8e-17; gain-limit hits identical. Round time 1232 s (fit 608, tune 621). Strict run (`strict=True`, copy): θ₂ identical (difference 0.0), 1174 s; no failed flight in this round, so the strict path with a repeat is not exercised. (`lab/results/dry_test.json`, `dry_round1_strict.json`).
26. **Gyro part of J_sysId is a dynamics gap, not an ingest error:** logged gyro against the derivative of the logged attitude: correlation 0.96–0.997; power above 20 Hz 0.00–0.02; S0 model body rate against frsim ≤ 7e-4 rad/s; real and model oscillate at about 1.0–1.4 Hz with unrelated phase (`lab/results/gyro_check*.json`).
27. **J_sysId per part, 23 Oct flights, β₀ and default gains:** J_p 4e-3 … 3e-2 m², J_v 2e-2 … 1.3 (m/s)², J_q 3e-3 … 1.2e-1, J_w 1.7 … 31 (not in the fit); all far above the noise floor; A1 the largest (table in `dryrun/stage4b/report_stage4b.md` §3). On A1 the attitude part has the same phase problem as the gyro: tilt rms 26.7 against 26.8°, correlation −0.17 … 0.24 at the best lag, 1.27 against 1.07 Hz (`lab/attitude_check.py`).
28. **Real A1 tilt and the abort rule (listed, rule not changed):** scored part, bottom drone: max pitch 38.6–40.5°, `gyro_y` max 487–512 °/s, longest continuous run above 25° 0.22–0.25 s, up to 0.68 s in a 1-s window. The rule "|roll| or |pitch| > 25° for 0.5 s" fires on A1 only under the cumulative reading (`lab/tilt_abort_check.py`; open point 10 of the protocol draft).
29. **P2 stall (reading, no new run):** on all four tuning steps of rounds 2 and 3 (P1 and P2) J_task rises at the first Adam step although the gradient predicts a fall; the gradient flips sign on 39–57 % of the gains per step; in P2 round 2 the gradient norm is four orders larger than in P1 (median 2.0e4 against 0.12, max 2.9e6) and sits in `Kwxy` / `KRxy`. No gain-limit hit. β of P2 rounds 2 and 3 is as close to the P1 truth as in P1 (max |error| 0.113 / 0.126 against 0.178 / 0.138). So the stall is a rough J_task surface near θ₂, not a worse β. Proposal in the report (1-D scan; smaller step or repeat on "best iterate = epoch 0"); method not changed.

## 15. Stage 4c (13 Nov 2026, marked "Stage 4c")

30. **Proposals for the open protocol points** (desk only, user decides): command line and `rotate_deg` per cell, how the gain file is applied, file names of the flight files, flights per scenario, battery rule, lab time. Section "Proposals for the open points (Stage 4c)" in `docs/diffcotune_lab_protocol_S0_DRAFT.md`; report `experiments/diffcotune/dryrun/stage4c/report_stage4c.md`.
31. **Commands reproduce the frsim cells** (`lab/cmd_check.py`, parser and scenario code of crazyswarm2 imported read-only): A1 0.000 mm, A8 0.30 / 0.60 mm (bottom / top − bottom), A4 cells 0.38 mm, flight files 0.003 mm. A8 tuning cell needs `--passes 2 --duration 8.0`; the 23 Oct `--passes 4` flies the same first 14.5 s. ingest `rotate_deg` = −(`--rotate`): A4 −90, FIG8 −90.
32. **(Corrected in Stage 4d, see section 16.)** The Stage 4c statement "OVAL does not fit at pad y = 0.3 m ... CIRCLE and FIG8 fit" took the file origin as the hover point and was wrong: CIRCLE (unturned) does not fit either.
33. **Gain application:** recommended = yaml merge per round (`lab/apply_gains.py merge`) with a firmware readback on a fresh connection (`readback`); alternatives and risks in the report. The 15 gains are not in the uSD log. `readback`, `set` and `lab/fly_file.py` (without `--check`) are not tested with ROS or on hardware.
34. **Lab time, 4 rounds, tuning flights only (estimate):** 16 flights = 23 min flying + 71 min desk = 94 min (1 flight per scenario); 64 flights = 92 + 71 = 163 min (4 per scenario). Rate 86.6 s per flight from the 23 Oct stamps; desk from the dry test.

## 16. Stage 4d (13 Nov 2026, marked "Stage 4d")

35. **Error in Stage 4a / 4c, corrected:** the pad windows and margins of the flight files took the file origin as the hover point. `simple_flight` hovers at pad + height and calls `startTrajectory(relative=True)` (crazyflie_py default); the firmware shifts the trajectory so that its first point sits at the current position (`planner.c:335–348`). The path about the hover point is file − file(0). Corrected windows (pad x / y): CIRCLE unturned [0.50, 1.00] / [−1.25, 1.25] m (does not fit at (0.0, 0.3)); CIRCLE turned +90° [−0.25, 0.25] / [−0.50, 2.00]; FIG8 [−0.54, 0.54] / [−1.08, 1.08]; OVAL [0.31, 1.00] / [−0.20, 0.20] (does not fit). Wrong places corrected: `fly_file.py`, `flightfiles/make_flightfiles.py` + `checks.json`, the flight files table and figure of the protocol draft (`lab/plot_protocol.py`), the Stage 4c texts (report banner, spec item 32).
36. **User decisions of 13 Nov:** gain file option A (yaml merge + fresh-connection readback), flight files through the `fly_file.py` wrapper, 1 flight per tuning scenario per round, order A1, A8 then CIRCLE, FIG8 (`cf_second` disabled), CIRCLE turned by 90°, OVAL dropped from the lab test set (lab test cells A4 cell 1 and 2).
37. **Turned circle** `flightfiles/circle_frsim_rot90_tuning_mode1.csv` (+90°, margin 0.25 m at the pad, 0.60 m to the cf_second pad; −90° would give 0.20 m): 0.0026 mm to the frsim reference, peak speed 0.667 m/s and acceleration 0.747 m/s² unchanged, 19 pieces. **The frame fit is degenerate for a circle** (rotation = time shift): `lab/test_frame_circle.py` finds −95.4° / −112 ms instead of −90° / +80 ms; with `rotate_deg = -90` given the result is exact. `flights.json` must give `rotate_deg: -90` for CIRCLE.
38. **A1, A8, A4 (run_formation):** anchored absolutely (takeoff to anchor + slot, goTo, then relative start); the first point of every curve is 0.0 mm, so the shift is zero and the pads do not enter. At the centred anchor: horizontal margins ≥ 0.94 m.

## 17. Stage 4e: deviations after the ablations (13 Nov 2026, marked "Stage 4e")

Report and data: `experiments/diffcotune/dryrun/stage4e/` (`report_stage4e.md`, `results/*.json`, `logs/`). New switches in `loop_lib.py` (defaults = old behaviour): `paper_stop`, `trace_fn`, `stop_after`.

39. **T0 (gyro in J_sysId, fit only, P1, theta_0 flights):** beta error thrust / lag / downwash / plume (%): pva -1.7 / -34.5 / +5.7 / +22.3 (= Stage 3b B1); body rates raw -11.4 / -37.9 / -16.1 / +39.4; Savitzky-Golay (5th order, zero phase) 110 ms -7.2 / -42.0 / -10.3 / +36.8, 70 ms -6.7 / -41.8 / -2.6 / +34.6, 210 ms -5.0 / -42.6 / -8.4 / +33.0. The body-rate part is 94-95 % of the four-part J_sysId at start and end in every variant and falls by 0.8-1.2 % in 50 epochs. Real data (23 Oct, 10 flights, beta_0): 95.7 % of the sum, unchanged by filtering (raw 10.985, 110 ms 10.979, 210 ms 10.930 rad^2/s^2). Setup notes: the frsim state trace holds the true body rate at 100 Hz, so the measured gyro is rebuilt as true rate + white noise of 12.0755 deg/s (fixed seeds); filter windows are given in ms (the prompt's "51 samples at 1 kHz" does not fit a 100 Hz trace).
40. **T1 (paper's literal stop rule on the raw SI cost, 5 rounds):** the prediction of Stage 1 item 8 (stop at the first epoch) did not hold in round 0 (fit 7 epochs, tune 5); later loops stop after 2-3 epochs. Real cost per round 19.99 / 17.45 / 17.89 / 16.09 / 16.69 cm (B1: 19.99 / 9.71 / 8.30 / 8.16 / 8.13). Final evaluation theta_best (round 3): tuning 13.00, test 8.56 cm against B1 theta_4 6.41 / 7.61; A1 fails in every round. Criteria 1-3 yes (formally), 4 no (+6.59 / +0.94 cm). Not passed.
41. **T2 (body rates in J_sysId, SG 210 ms = the variant with the best thrust gain in T0, 50 epochs, best iterate, 5 rounds):** real cost 19.99 / 9.54 / 7.46 / 25.72 / 91.81 cm; theta_3 fails A1, theta_4 fails all four tuning scenarios. Final evaluation theta_best (round 2): tuning 5.95, test 7.04 cm (B1 theta_4: 6.41 / 7.61). Criteria 1 no, 2 no, 3 yes, 4 yes. Not passed. One run each, spread not measured.
42. **Decision basis (user, 13 Nov evening):** stay as close to the paper as possible; every deviation is either needed for our setup or backed by a test. T1 and T2 do not pass: **the stop rule deviation and the J_sysId deviation stay, now backed by these tests.** Spec Stage 1 item 8 (prediction "stops at the first epoch") is corrected by item 40: round 0 ran 7 and 5 epochs, from round 1 on 2-3.
43. **Corrections to older text:** §5 item 7 says l = 0 ... 5 (6 flown sets); the lab and the dry runs of Stage 3e / 4e use 5 rounds, l = 0 ... 4 (user decision 13 Nov evening). The two numberings below are separated by labels.

### Deviations from the paper, with distinct labels

Labels: **S1-n** = item n of section 5 (Stage 1); **S3b-n** = deviation of Stage 3b (section 10; called "deviation 8 / 9" there); **S3d-n** = deviation of Stage 3d (section 12; called "deviation 10 / 11" there). Classes: **needed** (needed for our setup), **test** (backed by a test, which), **reading** (our reading of an unclear point), **given** (not a choice of ours).

| Label | Deviation | Class | Basis |
|---|---|---|---|
| S1-1 | quadrotor instead of cart-pole / quadruped / biped | given | Table I, p. 4 has no quadrotor |
| S1-2 | PyTorch instead of JAX / MuJoCo-XLA | given | p. 4 |
| S1-3 | our beta values (thrust gain, motor lag, downwash, plume) | needed | the plant of this study; mass and inertia fixed |
| S1-4 | learning rate 0.01 (not stated in the paper) | test | Stage 3b pilot: 0.01 / 0.03 / 0.1, rounds 0-2: 19.80 / 7.98 / 16.93 vs 43.81 / 56.25 vs 21.45 / 23.48 cm |
| S1-5 | cost averaged over 4 tuning scenarios, one flight each per round | needed | one gain set must serve several scenarios |
| S1-6 | task cost on the 3-D position only | reading | the paper's c uses the desired state; its own experiments use task-specific subsets (p. 5); decided by the user 11 Nov |
| S1-7 | keep-best over the flown sets; the last update is never flown | reading | Alg. 1 needs the real cost of theta_l+1; now 5 flown sets (l = 0 ... 4) |
| S1-8 | stop-rule units | superseded | replaced by S3b-1; the prediction of this item is corrected by item 40 |
| S1-9 | real flights start on the ground with unknown controller states | needed | lab setup (takeoff, hold, scenario, landing in the simulator) |
| S1-10 | x^sys = onboard estimates, simulator states = true states | needed | what the lab logs; the paper allows "measured or estimated" (p. 2) |
| S1-11 | mismatches outside beta (state delay, feedforward scale) | needed | not covered by any beta; P2 (10 ms delay) is information only |
| S1-12 | return iterate after a stop | superseded | followed as in the paper in Stage 3; replaced by S3b-1 |
| S3b-1 | 50 epochs per inner loop, best iterate, instead of the stop rule of p. 4 | test | Stage 3 (normalised rule: loop stopped after 2-4 epochs, theta_best 63.75 cm) and **Stage 4e T1** (literal rule: 16.1-17.9 cm in rounds 1-4, tuning mean 13.00 vs 6.41 cm, test mean 8.56 vs 7.61 cm) |
| S3b-2 | J_sysId = position + velocity + attitude, without body rates (paper: all observable states, unweighted, Eq. 3, p. 2, p. 4) | test | Stage 3b B1 (pva vs pv only); **Stage 4e T0 / real-data check / T2**: beta recovered worse with body rates, the rate part is 94-95 % of J and does not fall, T2 drifts (25.72 / 91.81 cm in rounds 3 / 4) |
| S3d-1 | sensor noise in the tuning step (2 fixed seeds per round) | test | Stage 3d variants A, B, A + B (not tested again: the paper's sim-to-sim tests have no sensor noise) |
| S3d-2 | +-25 % per-round gain limit | test, needed | Stage 3d (B alone fails, A + B passes); the paper projects theta into a feasible set for the biped (p. 5), not stated for the other systems (not tested again) |
| other | Adam state reset at the start of every inner loop | reading | the paper does not say |
| other | last-round fit skipped | reading | its beta is never used (Stage 3c) |
| other | floor rule (mean z < 50 % of the reference) as a second failure flag | needed | A1 stays on the floor without a tilt crash (Stage 3d) |

Not tested alone, listed: Adam reset per loop; the +-25 % limit and the noise are tested only as the combination A + B and as A and B (Stage 3d), not against the paper's literal setting; S3b-2 was tested with one body-rate variant in the full loop.

## 18. Stage 5: S1, S2, S3 and the S0 lab preparation (14 Nov 2026, marked "Stage 5")

Desk only. The method is frozen as for S0; nothing was tuned for S1–S3. Report: `experiments/diffcotune/dryrun/stage5/report_stage5.md`.

* **Models** (all on the frsim plant, forward gate ≤ 0.5 %): `s2_model.py` (S0 + NS2 every 5th block, held, clamp 8 m/s², measured position/velocity, 10-ms peer-velocity difference; max difference 0.37 %), `s1_model.py` (INDI mode 3 with the firmware Butterworth filters on a_rpm, a_imu, tau_rpm and angular acceleration; fractional rpm delay as fifth β `rpm_delay_ms`, start 4.0502 ms; max 0.156 %), `s3_model.py` (S1 + NS2). `dryrun/loop_lib.py`: `DIFFCOTUNE_CONTROLLER`, plant P1RD. frsim: option `rpm_delay_fractional` (off by default, `test_equivalence.py` 199/199 identical, `frsim_rd_check.py`).
* **Noise gate** (Stage 3d rule): PASS for S1, S2, S3. **Gradient check: FAIL on A1 for all three** (single A8, CIRCLE, FIG8 pass). S1/S3: autograd 1e20–1e24 against finite differences 1e-2…1e-4; S2: mismatch 1–14 %.
* **Diagnosis of the S1 explosion** (`jac1.py`, `jac0.py`, `eig1.py`, `iter1.py`, `tangent.py`, `revfwd.py`): one-block autograd equals finite differences to 3e-9 (S1) and 1.4e-9 (S2): no operation with a wrong gradient. Forward mode equals finite differences, reverse mode drifts from 1 s and fails from 3 s. The S1 one-block state Jacobian has an eigenvalue 1.006–1.012 per 2 ms (INDI filter states), S0/S2 1.003–1.0046 (body rate). Cause **partly located, not proven**. Options (none adopted; each a new deviation): drop A1 from the S1/S3 gradient; truncate or damp the backward pass; shorter horizon; finite differences.
* **Identifiability** (pva, 50 epochs, best iterate): S2 thrust +21.4, lag −30.1, downwash −16.8, plume +40.7 %; S1 +17.4 / −30.3 / −27.7 / +35.1 / rpm delay −46.9 %; S3 +24.6 / −32.8 / −21.7 / +48.4 / −49.9 %. RPM-delay verdict **withheld** (user, 14 Nov).
* **Dry runs** (5 rounds, same seeds for all): S2 15.71 → 7.96 cm (a, b, c met); S1 10.32 → 4.10 cm (met); S3 10.75 → 8.90 cm (met formally, flat to round 3). Tuning/test means of the final evaluation: S2 12.36→6.54 / 6.77→5.07, S1 9.04→3.06 / 5.73→1.35, S3 9.02→7.39 / 5.78→5.09.
* **Lab tools:** `round.py --controller`, `apply_gains.py` with the switches per strategy (read from `study_S1/S2/S3` via `git show`), `preflight.py --controller` (S0–S3 PASSED), round-0 dry tests (θ₁ difference 0.0 for S0–S3, S0 against `dry_test.json` and the Stage 3e loop). Not tested with ROS or hardware.
* **Lab-ready on the desk side:** S0 yes; S1 no; S2 no (open); S3 no.

## 19. Stage 5b: forward-mode gradient test (14 Nov 2026, marked "Stage 5b")

Desk only. Report: `experiments/diffcotune/dryrun/stage5b/report_stage5b.md`.

* **Gradient mode** (`loop_lib.GRAD_MODE`, environment variable `DIFFCOTUNE_GRAD_MODE`, `round.py --grad-mode`, default `reverse` for S0, S1, S2, S3): `forward` computes the same exact automatic-differentiation gradient with forward mode (`torch.func.jacfwd`, one tangent per parameter, eager). **This is not a deviation from the paper**: the paper uses the automatic-differentiation gradient and does not fix the differentiation mode; only the cost differs.
* **Result:** the forward-mode gradient passes the combined gradient gate for S0, S1, S2, S3. Without noise, reverse mode equals forward mode to 1e-14 on A8, CIRCLE, FIG8 and is wrong on A1 (S1 / S3 1e27 to 1e32 times too large, S2 0.2 relative, S0 4e-3); with noise reverse mode equals forward mode on A1 to ≤ 2e-11 relative. Cause (consistent with the growth rate, not separately proven): rounding in the reverse pass amplifies an unstable mode of the INDI filter states that no parameter excites (float64: 1e22, float32: NaN).
* **Cost:** forward mode 450–590 s per 4-worker call against 15–38 s for reverse (torch 2.5.1 cannot compile `torch.func.jvp`); a round would take about 14–17 h. Not adopted; the reruns of S1, S3, S2 were not started (45 min per round limit).
* **Lab-ready on the desk side:** S0 yes (reverse everywhere); S1, S2, S3 no.
* Also: `round.py --n-rounds` default 5; S0 last-round desk time 0.1 min (measured) in the protocol lab-time table.

## 20. Stage 5c: exact gradient at near-normal speed, reruns (15 Nov 2026, marked "Stage 5c" and "Stage 5c, RPM delay fixed (user, 14 Nov)")

Desk only. Reports: `experiments/diffcotune/dryrun/stage5c/report_stage5c.md`, `dryrun/stage5c_rdfix/report_rdfix.md`. Not a deviation from the frozen method: the gradient is the same automatic-differentiation gradient, computed in another way (stated per controller and loop).

* **Gradient per controller and loop** (`loop_lib.GRAD_MODE`, `round.py --grad-mode`, table `GRAD_MODE_DEFAULT`): S0 `reverse` everywhere (unchanged, bit-identical). S1, S2, S3 `mixed`: the noise-free β fit on A1 uses the exact forward sensitivity of one 2 ms block, traced once with `make_fx` into an aten graph and compiled (`fsens.py`; S2 / S3 two graphs for the NS2 hold block); the tuning step (noise) and all other scenarios use compiled reverse. Reason: reverse mode on A1 without noise is rounding driven (Stage 5b).
* **Checks:** reverse = forward with noise on A1 (gate ≤ 1e-6, passed); mixed = eager forward ≤ 6e-14 (S1, S2, S3); round test identical (θ₁ and β difference 0.0 for S1, S2, S3); adopted-mode gradient gate passed for S1, S3 and for S2 at finite-difference step 1e-8 (at 1e-7 the finite difference of S2 with noise is not converged). The truncated-reverse fallback (step 4) was not run.
* **Desk time per round** (round test): S1 22.5, S2 25.9, S3 28.8 min (target ≤ 30, hard 45).
* **Stage 5c, RPM delay fixed (user, 14 Nov):** the RPM delay of S1 and S3 is fixed at the calibrated 4.0502 ms (error after the round-0 fit S1 −44 %, S3 −63 %, above 25 %); `loop_lib.FIX_RD` / `PSI_FIT_IDX`, env `DIFFCOTUNE_FIX_RD`, `round.py --fix-rd` / `--fit-rd` (default S1, S3 fixed). Dry runs: S1 criteria a to c yes (real cost 10.32 → 2.93 cm); S3 criterion c no (θ_best = θ_0).
* **Lab-ready on the desk side:** S0 yes, S1 yes, S2 yes, S3 no. Open: cause of the S3 flat start and of the β drift (S1 downwash −74 %).
