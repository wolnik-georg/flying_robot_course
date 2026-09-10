# NA-INDI repository survey

**What it is:** the authors' own code for Cobo-Briesewitz, Wahba & Hönig (2026), *Learned
Incremental Nonlinear Dynamic Inversion for Quadrotors with and without Slung Payloads*
(L4DC, arXiv 2503.09441v2). Received directly from the first author. Paper summary:
[`papers/summaries/cobobriesewitz2026lindi.md`](papers/summaries/cobobriesewitz2026lindi.md).

**Location:** `~/Desktop/NA-INDI` · `github.com/Tupryk/NA-INDI` · HEAD `00302a2`
("updated data analysis scripts and added online learning") · 691 MB, 864 files.

**Visual companion:** `docs/indi_comparison.html` →
<https://claude.ai/code/artifact/45a5f215-c23a-4a5b-8d66-45855c403fe0>

**Headline:** their INDI and ours are **the same control law written two ways** (§2d). What
actually separates them is *conditioning* (§2e) — and our position-INDI path has none at all.

**Status of this survey:** read-only reconnaissance, 2026-09-10. Nothing in their repo was
modified. Their firmware branch was cloned separately, read-only, to `~/Desktop/NA-INDI-firmware`
— see §2b. **`~/Desktop/NA-INDI-firmware` is THEIRS; `~/Desktop/crazyflie-firmware` is the one
we build and flash. Never confuse the two.**

---

## 1. The headline finding — four "controllers" are a 2×2, not four implementations

`latex_results_generator.ipynb` selects the method by **two independent booleans**:

| Folder / method key | `using_nn` | `using_indi` | Paper label |
|---|---|---|---|
| `lee` | false | false | **Lee** (geometric baseline) |
| `indi` | false | true | **INDI** (classic, needs RPM) |
| `ilndi` | true | false | **IL-NDI** — learned residual, *no* INDI, *no* RPM at runtime |
| `naindi` | true | true | **NA-INDI** — learned residual **and** INDI on the leftover |

```python
if   method == "ilndi":  using_nn = True;  using_indi = False
elif method == "indi":   using_nn = False; using_indi = True
elif method == "naindi": using_nn = True;  using_indi = True
elif method == "lee":    using_nn = False; using_indi = False
```

So their contribution is a **clean 2×2 factorial ablation** over "learned residual on/off" ×
"INDI on/off" — not four separate control laws. That is architecturally the same shape as our
own strategy set (geometric / INDI / Geometric+NN / hybrid), and the same shape as our runtime
flags `indi_gains.ctrl_mode` × `rnn.en`. **This is the most useful single fact in the repo for
us**: it means their comparison and ours are structurally comparable, and their naming maps
onto our modes directly.

⚠️ **Naming trap:** the paper and this repo use **IL-NDI** (folder `ilndi`), *not* "L-INDI".
Our own cite key is `cobobriesewitz2026lindi` and the summary title says "LINDI / NA-INDI".
Keep the three apart: **LINDI/IL-NDI = NN only**; **INDI = RPM only**; **NA-INDI = both**.

---

## 2. What is present, and what is declared but missing

| Path | What it is |
|---|---|
| `LMCE/` | The whole Python side. Data prep, residual computation, MLP, trainers, model→C export, uSD log decode |
| `LMCE/c_utils/` | Hand-written C: `nn_utils.c/h`, `nn_main.c`, `online_nn_main.c`, `tree_utils.c/h`, `tree_main.c`, `nn_null.c/h` |
| `data_old/` | ~28 flight-data folders (the bulk of the 691 MB) — `circle_*`, `figure8_*`, `helix_*`, `payload_*`, `indi_no_payload`, `new_gains`, … |
| `*.ipynb` | `main`, `main_payload`, `main_tree`, `latex_results_generator` (+ a `copy`), `test` |
| `uav_trajectories` | Gitlink, commit `860e5b6` — **not checked out** |
| `create_null_net.py`, `data_standardization.py`, `pwm2thrust.py` | Standalone helpers |

**Not in this repo — but since located (2026-09-10):**

`.gitmodules` declares two submodules that **are not tracked in `HEAD` at all** (no gitlink
entry, directories absent):

```
[submodule "crazyflie-data-collection"] url = https://github.com/IMRCLab/crazyflie-data-collection.git
[submodule "crazyflie-firmware"]        url = ../crazyflie-firmware.git   branch = upstream_lee
```

The onboard controller C code is **not** in what we received. `git submodule update --init`
cannot recover it — the gitlinks were never committed.

**The declared URL is also a dead end.** `../crazyflie-firmware.git` is *relative*, and git
resolves it against the origin path — `github.com/Tupryk/NA-INDI.git` → **`github.com/Tupryk/
crazyflie-firmware.git`**. That fork exists but has only `master` and `dev-leePayload`; it has
**no `upstream_lee` branch at all**. So the submodule declaration cannot be satisfied as
written.

**Where the controller actually is — found by search, see §4.** Not on `upstream_lee` (that
branch has no NN code whatsoever), but on **`Tupryk/crazyflie-firmware`, branch
`dev-leePayload`** — the first author's own working branch, HEAD `ef2858fd` *("updated nn")*.
Cloned read-only to **`~/Desktop/NA-INDI-firmware`** (47 MB, deliberately named so it can
never be confused with our own `~/Desktop/crazyflie-firmware`).

`git submodule status` also errors (`no submodule mapping found ... for path
'uav_trajectories'`) — the repo's submodule config and tree disagree, so submodule commands
fail outright.

---

## 2b. The controller itself — `controller_lee.c` on `Tupryk/dev-leePayload`

**There is no new `ControllerType`.** The enum is stock (`PID / Mellinger / INDI /
Brescianini / Lee / Oot`). Everything lives **inside `controller_lee.c`**, selected by **two
`PARAM_UINT8` bitmasks**, both in the `ctrlLee` group:

| Param | Bit | Effect |
|---|---|---|
| `ctrlLee.use_nn` | `&1` | NN supplies **lateral** force residual → `a_nn.x`, `a_nn.y` |
| | `&2` | NN supplies **vertical** force residual → `a_nn.z` |
| | `&4` | NN supplies **torque** residual → `u_nn` (`nn_output[3..5]`) |
| `ctrlLee.indi` | `&1` | **force/position INDI** → `a_indi` |
| | `&2` | **moment/attitude INDI** → `tau_rpm` |
| | `&4` | **actuator force from PWM instead of RPM** — inverts a linear calibration, `rpm = (pwm_norm − rpm2pwmA)/rpm2pwmB`, then the usual `κ_f·rpm²`. An explicit no-RPM-deck path for INDI itself |

So the notebook's `using_nn` / `using_indi` booleans are a *simplification* for labelling
recorded flights — on the drone these are **finer-grained bitmasks**, and force vs torque and
lateral vs vertical can be enabled independently. The network has **6 outputs**: 3 force, 3
torque. Its input vector is ~19 wide and includes **motor PWM ratios** (`motorsGetRatio(0..3)`)
and gyro — so it is *not* RPM-free in the sense of using no actuator information; it uses
commanded PWM instead of measured RPM.

Files added on this branch: `src/modules/src/controller/nn.c` (generated weights) and
`nn_utils.c`. Also present, unrelated to this paper: `controller_lee_payload.c`,
`controller_rl.c`.

### Does their INDI need the RPM deck? Not necessarily — ours does

`indi & 4` computes the per-motor force from **commanded PWM** instead of measured RPM, by
inverting a linear `rpm ↔ pwm` calibration (`rpm2pwmA`, `rpm2pwmB`) and then applying the same
`κ_f · rpm²`. So their INDI has a documented fallback that does not require the deck.

Their **network** never uses RPM at all — `input_vec[15..18]` are `motorsGetRatio()`, i.e. PWM.
That is the paper's actual selling point: IL-NDI gives INDI-like residual rejection with no
RPM hardware in flight.

**Ours has no such path.** `rpm_get_all()` is the only source; without it `a_res` is exactly
zero and INDI silently degrades to geometric. Worth knowing as a contingency if the RPM deck
ever fails mid-campaign.

⚠️ Oddity: their PWM branch still sits inside `if (self->indi && rpm_deck_available)`, so it
is gated behind the very deck it is meant to replace. Either an oversight, or they always fly
with the deck fitted and use PWM mode only for like-for-like comparison.

### ⚠️ Their residual sign is the sign that crashed us

```c
struct vec F_d = vsub2(a_d, a_indi, a_nn);   // vsub2(a,b,c) = a - b - c   (math3d.h:303)
a_indi = vsub(self->a_imu_filtered, self->a_rpm_filtered);   // measured - model
```

So their **published, working** law is `F_d = a_d − a_res − a_nn` — i.e. **exactly our
`.sub(a_indi).sub(a_nn)`**, the derivation-correct sign that we parked at `res_sign=+1` after
it crashed on 2026-09-09. That is meaningful independent evidence that the *sign* is right and
our failure has a different cause.

**And here is the difference that most plausibly explains why it works for them:**

| | NA-INDI (`controller_lee.c`) | Ours (`lib.rs`) |
|---|---|---|
| Clamp before differencing | `vclampnorm(a_rpm, 10)` **and** `vclampnorm(a_imu, 10)` | none |
| Filter before differencing | 2nd-order Butterworth, **80 Hz, on both sides** | none |
| Resulting `a_res` | difference of two clamped, filtered signals | **raw instantaneous difference** |

Our `a_res` is `a_meas.sub(a_model)` computed fresh every tick with no filter and no bound,
and it feeds `f_d` → `thrust_vec` → `desired_rot()` → `Rd` → the attitude command. Theirs is
heavily conditioned first. **An unfiltered, unclamped residual driving the commanded attitude
is exactly the noise/lag path suspected on 09-09** — and this is the concrete, published
counter-design.

**Also: they structurally prevent double-counting.** When both are on, `a_nn` is folded into
the *model* side before the INDI difference is taken:

```c
self->a_rpm = vadd( ...model..., a_nn);      // NN prediction added to the model
a_indi = vsub(a_imu_filtered, a_rpm_filtered);  // INDI then sees only the leftover
```

So INDI corrects only what the network failed to predict — *that is what "NA-INDI" means*. Our
implementation applies both terms independently (`.sub(a_indi).sub(a_nn)`), and our own code
comment admits double-counting "is not prevented here, because preventing it would remove the
comparison". Their design is the alternative worth measuring against ours in C.3.

---

## 2c. How the two INDIs are written

> **Read §2d first if you have not.** The two forms are *algebraically equivalent* — this
> section is about how each is written, not about a difference in method.

Ours is **Tal & Karaman incremental** — it differences *angular accelerations* and multiplies
the result by J:

```rust
// ours, lib.rs
let alpha_err = alpha_ref_filt.sub(alpha_meas);        // kr=2400, kw=170  [1/s^2]
let delta_tau = J * alpha_err;
let tau = clamp_torque(tau_current.add(delta_tau));    // increment on tau_current
s.tau_prev = tau;                                      // fallback source only, see below
```

Theirs is the **disturbance-observer arrangement** — it differences *torques*, having applied
J to each side first, and subtracts the result from an unchanged geometric law:

```c
// theirs, controller_lee.c
indi_moments = vsub(tau_imu_filtered, tau_rpm_filtered);   // measured - model torque
self->u = vsub2(self->u, indi_moments, u_nn);              // geometric torque - residual
```

`self->u` is the *ordinary Lee geometric torque* from `KR = {0.007,0.007,0.01}` and
`Komega = {0.002,0.002,0.002}` — **in Nm, the same unit system as our `kr_geo`/`kw_geo`**.
Enabling their INDI does not change any attitude gain; it only subtracts an estimated
unmodelled torque.

**⚠️ Retracted claim.** An earlier draft of this section argued that ours "carries actuator
memory (`tau_prev`, `tau_act`) which can limit-cycle, while theirs has none", and offered that
as the explanation for the 5–8 Hz brushless shake. **That was wrong.** At our shipped defaults
— `act_tau = 0` with RPM present — `tau_current` comes **straight from RPM**, exactly as their
`tau_rpm` does. The `tau_prev`/`tau_act` path is a *fallback*, used only when RPM is
unavailable. We also already phase-match via `filt_tau = 1`. The actuator-memory story does
not explain the shake, and should not be repeated.

What genuinely differs is enumerated in §2e: where the subtraction happens, how each side is
filtered and clamped, and the gyro and `dt` sources.

Their signal conditioning, for reference:

| Stage | Clamp | Filter |
|---|---|---|
| `a_rpm`, `a_imu` (force) | `vclampnorm(·, 10)` m/s² | Butterworth-2 @ **80 Hz** |
| `tau_rpm`, `tau_imu` (moment) | `vclampnorm(·, 0.006)` Nm | Butterworth-2 @ **40 Hz** |

Angular acceleration is differenced from **`sensors->gyroNoLpf`** (raw gyro), then the
*torque* is filtered — i.e. they filter after converting to torque, not before differentiating.

**Correction (2026-09-10):** an earlier draft of this survey called the `i < 2` tau-filter
loop a bug. It is **not** — index 2 (yaw) is initialised immediately afterwards on its own,
at a deliberately much lower **`cutoff_z = 10 Hz`** against 40 Hz for roll/pitch. Yaw torque
is filtered ~4× harder on purpose. That is a design choice worth noting, not a defect.

---

## 2d. They are the same law — an equivalence analysis

> **This is the load-bearing section.** Everything in §2c and §2e is downstream of it.

Worth settling, because the instinct is "theirs is published with a PhD student and a
supervisor, so if they differ, ours must be wrong." **The algebra says both are correct.**
They are the same control law in two arrangements.

**Rigid body:** `J*omega_dot = tau_motors + tau_dist - omega x J*omega`

**Ours (incremental, Tal & Karaman):**

```
tau_cmd = tau_current + J*(alpha_ref - alpha_meas)
```

With `tau_current = tau_motors` (from RPM) and
`alpha_meas = J^-1 (tau_motors + tau_dist - omega x J*omega)`:

```
tau_cmd = tau_motors + J*alpha_ref - (tau_motors + tau_dist - omega x J*omega)
        = J*alpha_ref + omega x J*omega - tau_dist
```

**Theirs (subtractive / disturbance-observer):**

`tau_imu = J*alpha - (J*omega x omega) = J*alpha + omega x J*omega`, which by the rigid-body
equation **is** the total torque `tau_motors + tau_dist`. So `tau_res = tau_imu - tau_rpm =
tau_dist`, and with the Lee geometric law already containing its own gyroscopic term:

```
u = u_geo - tau_res = (J*alpha_ref + omega x J*omega) - tau_dist
```

**Identical.** `tau_cmd = J*alpha_ref + omega x J*omega - tau_dist` in both cases. Ours cancels
the gyroscopic term *implicitly* (alpha_meas already contains it); theirs handles it
*explicitly* on both sides. Neither is a different method — they are algebraic rearrangements
of one law.

This is the known INDI <-> disturbance-observer correspondence. Ours is faithful to Tal &
Karaman's `u = u_0 + G^-1 (nu - alpha_meas)`; theirs is the DOB arrangement of the same thing.

### Where they genuinely differ: filter placement, not formulation

The equivalence is exact only for *unfiltered* signals. Once filters are inserted the two
arrangements are no longer identical, and the difference is in **which quantities get matched
phase**:

| | Ours | Theirs |
|---|---|---|
| Difference taken between | `alpha_ref` and `alpha_meas` (angular accelerations) | `tau_imu` and `tau_rpm` (torques) |
| Both sides same filter? | yes — `fc_bw=60 Hz` on `alpha_ref`, `alpha_meas` **and** `tau_current` (`filt_tau=1`) | yes — same Butterworth, same cutoff, both sides |
| Yaw treated differently? | no | **yes — 10 Hz on yaw vs 40 Hz on roll/pitch** |
| Extra state | `tau_prev` / `tau_act`, used **only as fallback** when RPM is unavailable or `act_tau>0` | none |

So an earlier framing in this document — "ours has memory, theirs does not" — **overstated
it**. At the shipped defaults (`act_tau = 0`, RPM present) our `tau_current` comes straight
from RPM, exactly like their `tau_rpm`. The memory path is a fallback, not the normal path.
The honest remaining differences are:

1. **Where the subtraction happens** — in angular-acceleration space (ours) vs torque space
   (theirs). Equivalent on paper; differently sensitive to J errors, since ours multiplies the
   *difference* by J while theirs multiplies each side by J before differencing.
2. **Yaw filtering** — they deliberately filter yaw torque 4x harder. We do not distinguish
   axes at all. Our shake is a roll/pitch phenomenon so this is not the cause, but it is a
   considered choice we have not made.
3. **Clamping** — they bound every residual (`10 m/s^2` force, `0.006 Nm` torque) before it
   enters the law. We clamp only the final output.

**Conclusion: most likely both correct.** Ours is not wrong for being different — it is the
textbook incremental form, which is the method under study. Theirs is the same law in DOB form
with more conservative conditioning. The gap to close is **conditioning**, not formulation.

---

## 2e. Every difference, enumerated — and what to do about each

Seven, not three. Established by reading both signal chains end to end (2026-09-10).

| # | Difference | Ours | Theirs | Verdict |
|---|---|---|---|---|
| 1 | **Force residual conditioning** | **nothing at all** | clamp 10 m/s² + 80 Hz BW, both sides | **adopt** |
| 2 | Torque residual conditioning | 60 Hz BW both sides ✓ | 40 Hz BW + clamp 0.006 Nm | consider |
| 3 | Per-axis yaw filtering | uniform 60 Hz | 10 Hz yaw vs 40 Hz roll/pitch | consider |
| 4 | Gyro source | `sensors->gyro` (stock LPF applied) | `gyroNoLpf` (raw), filter downstream | investigate |
| 5 | `dt` resolution | tick counter — **1 ms quantised** | `usecTimestamp()` — µs | investigate |
| 6 | Subtraction space | angular acceleration | torque | **keep ours** |
| 7 | Gyroscopic term | implicit in `alpha_meas` | explicit both sides | **keep ours** |

### The observation that reframes the rest

**"Conditioning" is not one thing. Our ATTITUDE INDI is carefully conditioned; our POSITION
INDI is not conditioned at all.**

- **Torque side (ours):** `alpha_ref`, `alpha_meas` *and* `tau_current` all Butterworth-filtered
  at 60 Hz, phase-matched via `filt_tau=1`. Careful work.
- **Force side (ours):** `a_res = a_meas - a_model`, **raw**. No filter, no clamp, straight into
  `f_d -> thrust_vec -> desired_rot() -> Rd ->` the commanded attitude.

The 2026-09-09 crash was at `ctrl_mode=3`, which enables the **position** loop. The
unconditioned path is exactly the one that was live when it diverged.

### Recommendations, by value-per-risk

**1 — Condition `a_res`. ADOPT.** It is a difference of two noisy estimates (accelerometer;
RPM² model, quantised and actuator-lagged), and differencing amplifies both. Filter *both sides
identically then subtract*, so the filter lag cancels in the difference — filtering after the
subtraction cannot do that. Add a norm clamp so one bad sample never reaches the law. Their
constants (10 m/s², 80 Hz) are a starting point; our airframe differs.

**2 — `dt` resolution. INVESTIGATE.** `alpha_raw = d_omega/dt`, so `dt` error scales `alpha_meas`
directly. Ours is `(tick - last_tick) * 0.001`. If the scheduler is exact (tick always +2) this
is fine; any jitter quantises to 1 ms steps, i.e. **up to 50 % error** on that sample. Cheap
first step: log `dt` for one flight and look at the distribution. Precedent: *"2 kHz call with
ms tick"* was one of the five sim-fidelity bugs.

**3 — Gyro source. INVESTIGATE.** They take raw gyro and filter downstream; we take the
stock-LPF'd gyro and filter again, putting an unknown filter in series with ours. The 5–8 Hz
shake is a *phase* problem that four investigations have not closed, and a hidden series filter
is the kind of thing that survives four investigations.

**4 — Per-axis yaw filtering. CONSIDER.** Yaw torque comes from motor drag-torque differences
(`t2t ≈ 0.006`), an order of magnitude below roll/pitch arm torque and correspondingly noisier.
Lower priority — our shake is roll/pitch, so this is not the cause.

**5 — Their subtractive form. DO NOT ADOPT.** Algebraically identical (§2d), so it buys no
correctness, and it would change what "INDI" denotes in the comparison. Tal & Karaman
incremental INDI *is* the method under study. If ever wanted, add it as a **second** INDI
variant, honestly labelled — never as a replacement.

### Sequencing

**Nothing before the six controller-validation flights** (`C0_FLIGHT_CARD.md` rung −1) —
changing the controller first makes that result uninterpretable. Then: pull the uSD cards,
apply #1, re-test, and only then #2–#4 **one at a time**.

⚠️ **Caveat.** All of the above is reasoning from their code, not a controlled experiment on our
airframe. Their platform is a standard CF2.1 (J ≈ 16.6e-6, ~30 g), their disturbance is a slung
payload, their gains do not transfer. These are well-motivated hypotheses to test individually,
not a repair kit to apply wholesale.

**Visual companion:** `docs/indi_comparison.html` — published at
<https://claude.ai/code/artifact/45a5f215-c23a-4a5b-8d66-45855c403fe0>

---

## 2f. Investigation results, 2026-09-10 — what was applied and what was found

Acting on §2e. **Everything is behind runtime params defaulting to today's behaviour**, and
that was verified, not assumed: against the compiled SIL controller, the new build is
**0/100 samples different** from the previous one in all four `ctrl_mode`s. The
controller-validation flights are unaffected.

### ✅ #1 applied — `a_res` conditioning (default OFF)

| Param | Default | Effect |
|---|---|---|
| `indi_gains.res_fc` | `0` = off | Butterworth cutoff [Hz] on `a_meas` and `a_model` **separately, before the subtraction** |
| `indi_gains.res_clamp` | `0` = off | direction-preserving norm clamp [m/s²] on each side before differencing |

Filtering each side with identical coefficients means the filter lag **cancels in the
difference** — which a filter applied *after* the subtraction cannot do. Verified active when
set; `res_clamp` engages progressively as the bound drops past the residual magnitude
(inert at 10, 42/100 samples at 0.5, all samples at 0.1). Both correctly inert at
`ctrl_mode=0`, where `a_res` is unreachable.

### 🔴 #2 — a real bug found: the filters are designed for the wrong sample rate

`controllerOutOfTree` is called from `stabilizer.c`'s main loop **unconditionally at
`RATE_MAIN_LOOP` = 1000 Hz** ("the sensor should unlock at 1kHz"). There is **no**
`RATE_DO_EXECUTE(ATTITUDE_RATE)` gate — contrary to `firmware_app/CLAUDE.md`, which states
500 Hz. Our `dt` computation is correct (`(tick − last_tick) × 0.001` = 0.001), but every
Butterworth and notch was initialised with a hard-coded `DT = 0.002`.

| Asked for | Actually gets |
|---|---|
| `fc_bw = 60` Hz | **−3 dB at ~206 Hz** — ~3.4× less filtering than the parameter claims |
| `notch_f0 = 6.9` Hz | **centred at 13.8 Hz** — exactly 2× |

The notch result is the striking one: **the stage-2 notch aimed at the measured 6.9 Hz peak
was notching 13.8 Hz and never touched it.** That is a concrete explanation for "notch filter
tested + failed", and it plausibly explains why the `fc_bw` sweeps read as flat — the swept
range never reached the shake band.

This has been true for *every* flight including the successful July campaign, so **it is not
the crash cause** — the gains were tuned empirically around it. But `fc_bw` and `notch_f0` do
not mean what they say, which matters for all future filter work.

**New param `indi_gains.filt_dt_us`** — default `2000` (the flown 500 Hz assumption); set
`1000` to make the parameters honest; `0` = derive from the measured loop dt.

**New log var `indi.dt_us`** — the measured loop dt, so the 500-vs-1000 Hz question can be
settled from a real flight rather than from reading `stabilizer.c`.

> ⚠️ **Correcting this ADDS phase lag — do not treat it as a free fix.** A real 60 Hz filter
> lags more than a 206 Hz one: **−4.66° vs −2.33° at 6.9 Hz** (and −13.7° vs −6.8° at 20 Hz).
> Since the shake is a phase problem, correcting `filt_dt_us` could plausibly make it *worse*.
> Change it as a deliberate single-variable A/B, and expect to retune `fc_bw` afterwards —
> `fc_bw = 120` reproduces today's coefficients EXACTLY at the corrected rate.

### ⚪ #3 investigated — gyro source is NOT significant

They take raw `gyroNoLpf`; we take `sensors->gyro`, which the firmware has already passed
through a **2nd-order 80 Hz LPF** (`GYRO_LPF_CUTOFF_FREQ 80`, `lpf2pInit(..., 1000, 80)`), so
ours has two filters in series. Quantified at the shake frequencies:

| f | stock 80 Hz LPF | our BW (as shipped) | total |
|---|---|---|---|
| 5.0 Hz | −2.53° | −1.69° | −4.22° |
| **6.9 Hz** | −3.50° | −2.33° | **−5.83°** |
| 10.0 Hz | −5.07° | −3.38° | −8.45° |

**~6° of total phase lag at 6.9 Hz is too small to drive the oscillation.** Recorded as a
negative result — the series-filter hypothesis is not promising, and #2's *effective cutoff*
error is the far larger effect on the same path.

### Not done — #4 (per-axis yaw filtering)

Deferred. Our shake is roll/pitch, and #2 changes the whole filter picture; revisit only after
the sample-rate question is settled on hardware.

---

## 3. The residual they learn — identical formalism to ours

`LMCE/residual_calculation.py`:

```python
f_a = total_mass * acc_world - rowan.rotate(q, f_u)          # no payload
f_a = total_mass * acc_world - rowan.rotate(q, f_u) - T_p    # with payload (T_p = cable tension)
```

That is **exactly our `indi.a_res_*`**, in force rather than acceleration units:

$$ f_a = m\,a_{\text{meas}} - R\,f_{\text{thrust}} \qquad\Longleftrightarrow\qquad a_{\text{res}} = a_{\text{meas}} - a_{\text{model}} = f_a/m $$

**But the disturbance source is different.** Theirs is a **slung point-mass payload** (and a
no-payload case for the plain residual); ours is **another vehicle's downwash**. Same
formalism, different physics, different input features. Their no-payload numbers are the
comparable ones for us.

Notable methodological detail: residuals are computed on **offline-smoothed** data
(`SplineFitter`, `make_smooth`, `spline_segments`) — they fit splines to `omega` and
differentiate the spline rather than the raw signal. That is a training-data quality step we
do not currently do; our `a_res` is the raw onboard value.

---

## 4. The learned model

`LMCE/models.py`:

- **`MLP`** — `Linear → LeakyReLU` stack, default hidden `[24, 24, 24]`, all biases
  zero-initialised (comment: *"Improves Torque Z output"*).
- **`DTE`** — a decision-tree variant (`main_tree.ipynb`, `tree_utils.c`). A second model
  family, not just the MLP.

**Comparison with ours:** a plain MLP over a single vehicle's own state. Ours is a
**deep-sets** network (φ/ρ, permutation-invariant, 987 weights) over *neighbour* relative
states — necessarily different, because our residual depends on other vehicles and theirs does
not. Their architecture is not directly reusable for us; their *pipeline* is.

**`LMCE/model_to_c_conversion.py`** is the closest analogue to our
`tools/residual/` + `rnn.*` upload path, and is worth reading properly:
- `exportNet()` writes a generated `nn.c`
- `nn_c_model_test()` compiles it with `gcc` and **numerically compares C output against the
  PyTorch output** (`compare_outputs`, `err_thresh=1e-4`)
- same pattern for the tree (`exportTree`, `tree_c_model_test`)

That is the same verify-the-C-against-the-reference discipline as our
`host/test_residual_nn.py`, arrived at independently. Their route is *code generation*
(weights baked into generated C, reflash to change); ours is *runtime upload* (`rnn.wi/wv/wc`
over CRTP, no reflash). Ours is more flexible; theirs is simpler and has no upload protocol to
get wrong.

**`online_nn_main.c`** — `add_dataset_entry()` + `backprop()` running in C. HEAD's commit
message says "added online learning". This is **onboard/at-runtime training**, which neither
the paper's main claims nor our thesis currently covers. Worth understanding before assuming
the repo matches the published method exactly.

---

## 5. What this repo is useful for, concretely

1. **A flight-proven reference for conditioning the residual.** Their law uses our "fixed"
   sign, but clamps and filters the residual on both sides before differencing, and folds the
   NN into the model so INDI sees only the leftover. Since the two laws are *identical* (§2d),
   their conditioning is directly transferable without adopting their arrangement — see §2e.
2. **Their 2×2 is a template for our comparison table.** Same ablation structure, already
   validated in a published paper. Worth aligning our reporting to it where honest.
3. **`model_to_c_conversion.py`'s numerical C-vs-PyTorch check** — independent confirmation
   that our own equivalent test is the right discipline, and a second implementation to
   compare against if ours ever disagrees.
4. **The spline-smoothing step on residual training data** is a concrete technique we do not
   currently use and could evaluate for C.2.
5. **`data_old/` is real flight data from a published method** — usable as a sanity target for
   our own analysis tooling, though it is single-drone-with-payload, not formation.
6. **Their no-payload results are the directly comparable baseline** to our single-drone case.

**What it is NOT useful for:** the onboard controller is not in *this* repo — it is in the
separately cloned `~/Desktop/NA-INDI-firmware` (§2b). Their network is also not reusable for
us: it predicts a single vehicle's own payload/aero residual from its own state, while ours
must depend on neighbours.

---

## 6. Open questions before this repo can be used further

1. ~~Fetch the firmware to read the controller~~ — **DONE 2026-09-10.** Not `upstream_lee`
   (no NN code there at all); the controller is on `Tupryk/crazyflie-firmware`, branch
   `dev-leePayload`. Cloned read-only to `~/Desktop/NA-INDI-firmware`. See §2b.
2. ~~What do the flags map to on the drone~~ — **ANSWERED.** Two `PARAM_UINT8` bitmasks,
   `ctrlLee.use_nn` (bits: lateral / vertical / torque) and `ctrlLee.indi` (bits: force /
   moment), both inside stock `controller_lee.c`. No new `ControllerType`.
3. **What are the MLP's input features?** `input_size` is a constructor argument and the call
   sites are inside notebooks; not yet traced. This matters — it is the main design decision
   we would compare against.
4. **Is the online-learning code (`online_nn_main.c`) part of the paper's method or work after
   it?** HEAD post-dates the arXiv v2; treat as after unless shown otherwise.
5. **`data_old/` naming suggests a `data/` that is not present** — the results notebook reads
   `data/figure8_online`, which does not exist in the tree. The published results may not be
   reproducible from this snapshot alone.

---

## 7. Repository status for the thesis

NA-INDI is now a **reference repository** for this thesis — read-only, third-party, never
modified and never pushed to. It sits alongside:

| Repo | Ours? | Role |
|---|---|---|
| `flying_robot_course` | yes, pushed | thesis, firmware source, logs, docs |
| `crazyswarm2` | yes, pushed | flight scripts, configs |
| `crazyflie-firmware` | no — upstream, local mods only | bitcraze base, patch-preserved |
| **`NA-INDI`** | **no — third-party reference** | **authors' code for a paper we cite. Read-only** |

Same rule as `crazyflie-firmware`: **do not fork, do not push, do not modify.** If we ever
need changes to run it, they belong in our repo as a script or patch, not in their tree.
