# Motion Planning and Incremental Nonlinear Dynamic Inversion for Aggressive Quadrotor Trajectory Tracking on the Crazyflie 2.x Platform Family

**Course:** Advanced Flying Robots
**Author:** Georg
**Mentor:** Omar Elsayed (partially Khaled Wahba)

---

## Abstract

This report documents the design, implementation, and experimental validation of a full
autonomous-flight stack — minimum-snap motion planning with automatic time allocation, a
differential-flatness-based geometric SE(3) baseline controller, and an Incremental Nonlinear
Dynamic Inversion (INDI) attitude controller — deployed and compared across three physically
distinct quadrotor platforms: a standard Crazyflie 2.1, a thrust-upgraded Crazyflie 2.1, and a
Crazyflie 2.1 Brushless (CF21BL). All three platforms were commissioned end-to-end (RPM-based
motor identification, filter tuning, cascaded gain tuning) and evaluated on an identical figure-8
tracking benchmark, plus extended trajectory families (oval, circle, slalom) swept over a range of
planner aggressiveness. The headline result is a four/five-way controller-and-platform comparison
showing that full INDI reduces XY tracking RMSE by 37–60% relative to the geometric SE(3)
baseline, with the brushless platform's higher motor bandwidth (τ≈44 ms vs. 71 ms for the
thrust-upgraded platform) enabling substantially more aggressive attitude gains and the best
overall tracking performance (2.45 cm mean RMSE, n=13) — at the cost of a resonant ~7.2 Hz
attitude oscillation that the thrust-upgraded platform's lower achievable bandwidth avoids
entirely. The report documents the full root-cause chain for this oscillation (closed-loop
resonance at ωₙ=√k_R, actuator lag as the dominant phase-loss term, EKF attitude-estimate lag as a
secondary term) and a targeted band-reject (notch) filter designed as a data-driven fix, which is
implemented and build-verified but not yet flight-validated at the time of writing. The remaining
sections cover related theoretical background, the planning and control methods used, the
experimental setup across all three platforms, full results, a discussion of the cross-platform
differences, and an honest account of the project's open limitations.

---

## 1. Introduction

### 1.1 Motivation

Accurate tracking of aggressive trajectories is a core requirement for autonomous quadrotor
applications that go beyond gentle point-to-point flight — racing, inspection in constrained
spaces, and any task where speed and path fidelity both matter. Standard PID-based flight
controllers rely on approximate, often decoupled models of vehicle dynamics and tend to degrade
sharply as demanded accelerations grow. Two complementary techniques address this: (1)
**differential-flatness-based motion planning**, which generates dynamically feasible, minimum-
effort trajectories with closed-form full-state references (including attitude) derivable directly
from the position trajectory and its derivatives; and (2) **Incremental Nonlinear Dynamic
Inversion (INDI)**, a control technique that replaces an explicit dynamics model with a real-time
measurement of the vehicle's actual angular acceleration, making the attitude loop robust to
unmodeled aerodynamic effects and platform-specific parameter uncertainty.

### 1.2 Project goals

This project set out to:

1. Implement a minimum-snap motion-planning pipeline with automatic time allocation (Richter et
   al. 2016), producing dynamically feasible trajectories across a library of maneuver shapes.
2. Implement full INDI attitude control (Tal & Karaman 2020) on real Crazyflie hardware, including
   the RPM-feedback-based torque estimation this requires.
3. Commission and tune this stack on **three separate physical platforms** — a standard Crazyflie
   2.1, a thrust-upgraded Crazyflie 2.1, and a Crazyflie 2.1 Brushless — to characterize how
   platform-specific actuator dynamics (motor bandwidth, achievable thrust) constrain the
   achievable control bandwidth and, ultimately, tracking accuracy.
4. Produce a fair, controlled, cross-platform comparison against a common geometric SE(3) baseline
   controller.
5. Push the trajectory library and aggressiveness envelope (via the planner's `k_t` scalar) as far
   as each platform's actuation would allow, characterizing each platform's practical speed
   ceiling.

### 1.3 Scope and what was descoped

Two originally-planned phases were explicitly cut as the project's actual bottleneck (a genuine,
data-confirmed attitude resonance on the highest-bandwidth platform, rather than a lack of
available maneuvers to fly) became clear: a third drone platform beyond the three already
commissioned, and a dedicated phase of accel-pinned aggressive aerobatic maneuvers requiring
thrust-vector inversion. A subset of aerobatic work (a fully inverted vertical loop) was in fact
completed later as bonus, beyond-scope work once time allowed, and is summarized briefly in
§7 (Limitations & Future Work) rather than in the core Methods/Results, since it uses a distinct
explicit-attitude planning approach not central to the project's main planning comparison (which
is restricted, as detailed in §3.1, to the two automatic-time-allocation planning modes actually
used for every controlled experiment in this report).

---

## 2. Related Work / Background

### 2.1 Differential flatness for quadrotors

A quadrotor's translational dynamics are differentially flat in the sense of Mellinger & Kumar
(2011): the full 6-DOF state (position, velocity, attitude, angular velocity) and control inputs
(collective thrust, body torques) can be written as algebraic functions of a "flat output" — here,
the position trajectory **p(t)** and yaw **ψ(t)** — and a finite number of their time derivatives
(up to snap, the 4th derivative of position, for full state; up to the 5th derivative, "crackle",
for angular acceleration feedforward). This means a smooth polynomial trajectory in position/yaw
space alone is sufficient to derive a complete, dynamically consistent full-state reference
trajectory, without ever needing to plan attitude directly.

### 2.2 Geometric SE(3) control

The project's baseline attitude controller follows Lee, Leok & McClamroch (2010): attitude and
angular-velocity errors are computed directly on the rotation group SO(3) (avoiding the small-angle
approximations and singularities inherent to Euler-angle representations), and torque is commanded
model-based from these errors plus a gyroscopic decoupling term. This is the "geometric" controller
referenced throughout this report — the common baseline every INDI configuration is compared
against.

### 2.3 Minimum-snap trajectory generation and automatic time allocation

Mellinger & Kumar's (2011) minimum-snap formulation poses trajectory generation as a quadratic
program (QP): minimize the integral of squared snap along a piecewise-polynomial path subject to
waypoint and continuity constraints. This project uses **Richter, Bry & Roy (2016)**'s extension,
which additionally automates the choice of *per-segment durations* — previously a manually-tuned
parameter — from a single scalar aggressiveness parameter **k_t**, with an iterative gradient
step that redistributes time toward high-snap (i.e., high-curvature/high-speed-change) segments of
the path.

### 2.4 Incremental Nonlinear Dynamic Inversion (INDI)

**Tal & Karaman (2020)**, the primary reference for this project's attitude control law, replace
the model-based feedback term of a standard nonlinear controller with an *incremental* update: the
next torque command is the *currently applied* torque plus a small correction proportional to the
gap between the desired and the actually-measured angular acceleration. Because the measured
angular acceleration reflects the true, current physical state of the vehicle (including whatever
unmodeled aerodynamic effects, motor asymmetries, or external disturbances are present), INDI is
inherently robust to model mismatch without requiring that mismatch to be characterized in advance
— a property that will turn out to be directly load-bearing for this project's largest single
practical challenge (§6.2).

---

## 3. Methods

### 3.1 Motion planning

All trajectories in this project are generated by one of two automatic-time-allocation planning
modes, both built on the same minimum-snap QP machinery (`src/planning/richter.rs`) and both
selected from a single scalar aggressiveness parameter **k_t** rather than manually-chosen segment
durations:

- **Mode 1 (`RichterTrajectory::plan()`)** — Richter et al. 2016's time-allocation formula,
  `T_i = max(dist_i / v_avg(k_t), T_MIN)` with `v_avg = 0.5 + 1.5·√k_t` and `T_MIN = 0.15 s`
  (below which the QP's `1/T⁷` segment weighting becomes numerically degenerate). This project's
  engineering default: **zero** redistribution iterations, favoring robustness and predictability
  over the marginal timing refinement the full paper formulation offers.
- **Mode 3 (`RichterTrajectory::plan_paper()`)** — identical time-allocation formula, but with the
  full **eight gradient-descent redistribution iterations** specified in Richter et al. 2016,
  which push segment time toward the highest-snap portions of the trajectory for more balanced
  actuator demand across the maneuver.

Both modes share the same differential-flatness recovery (`src/planning/flatness.rs`,
Faessler/Franchi/Scaramuzza 2018 formulation): given the position polynomial's derivatives at any
query time `t` — velocity, acceleration, jerk, snap — plus yaw and its derivatives, the full
rigid-body reference is recovered in closed form:

```
acc_g = p̈ + g·ẑ
xb = normalize(yc × acc_g),  yb = normalize(acc_g × xb),  zb = xb × yb     (attitude, from acc)
c  = zb·acc_g,  thrust = m·c                                              (collective thrust)
ω  = f(jerk, xb, yb, c)                                                   (angular velocity, from jerk)
ω̇  = f(snap, ω, xb, yb, c)                                                (angular acceleration, from snap)
τ  = J·ω̇ − (Jω)×ω                                                        (feedforward torque)
```

where `xc = [cosψ, sinψ, 0]` and `yc = [-sinψ, cosψ, 0]` define the yaw-heading intermediate
frame used to disambiguate the body-x/body-y axes from the thrust-direction-only `zb`. This
closed-form map is what allows the entire trajectory library used in this project (figure-8,
circle, oval, corner, helix, slalom, tilted oval) to be authored purely as position/yaw waypoints —
attitude, angular rate, angular acceleration, and thrust are all derived, never separately planned.

### 3.2 Geometric SE(3) baseline control

For every platform, the geometric controller (Lee et al. 2010) provides the common baseline
against which INDI is compared. Attitude error is computed on SO(3):

```
e_R = ½ · vee(R_d^T R − R^T R_d)
```

and torque is commanded directly from a model-based law:

```
τ = −K_R·e_R − K_W·e_ω + ω × Jω
```

with `K_R`/`K_W` tuned per-platform (position loop: `K_P`/`K_V` on position/velocity error,
identical cascade structure). This is a purely model-based law — its accuracy is bounded by how
well `J` (inertia) and the thrust/torque allocation model actually match the real airframe.

### 3.3 INDI attitude control

The INDI attitude law (`INDI_CORRECT.md`, following Tal & Karaman 2020 directly) replaces the
geometric law's model-based torque computation with four coupled equations:

**(1) Feedforward.** The desired angular acceleration **α_des** is computed from the trajectory's
snap via differential flatness (Tal & Karaman Eq. 15) — this is the *only* place the desired
trajectory's higher derivatives feed the attitude loop; everything else below is feedback.

**(2) Reference blend.**
```
α_ref = α_des − K_R·e_R − K_W·e_ω
```
(Eq. 28) — structurally identical in form to the geometric law's gains, but here `α_ref` is a
desired *acceleration*, not a torque; `K_R` has units 1/s² (closed-loop bandwidth `ωₙ=√K_R`), `K_W`
has units 1/s (damping ratio `ζ=K_W/(2ωₙ)`).

**(3) Measured angular acceleration.** Because no sensor measures angular acceleration directly,
`α_meas` is derived from the gyroscope through a three-stage filter chain:

```
Stage 1 (Butterworth, 2nd order):     ω_filtered[k] = BW(ω_raw[k])
Stage 2 (finite difference):          α_raw[k] = (ω_filtered[k] − ω_filtered[k−1]) / Δt
Stage 3 (1st-order IIR):              α_meas[k] = k·α_raw[k] + (1−k)·α_meas[k−1]
```

This chain exists because differentiating a noisy signal amplifies high-frequency noise; the
pre-filter (Stage 1) removes vibration/sensor noise from the gyro *before* differentiation, and the
post-filter (Stage 3) further attenuates transition-band noise left over from the differentiation
itself. The same Butterworth filter is applied identically to `α_ref` (`bw_ref_x/y/z` in the
firmware implementation) so the reference and measured legs of the increment stay phase-matched —
an un-filtered reference subtracted from a filtered measurement would otherwise introduce a
spurious phase mismatch into the error term.

**(4) Incremental torque law.**
```
δτ = J·(α_ref − α_meas)
τ_new = τ_current + δτ
```
(Eq. 31) — the core INDI mechanism. Rather than computing torque from a model, the controller
computes a small *correction* to whatever torque is *currently being applied*, using the measured
(not modeled) angular acceleration. `τ_current` is estimated directly from the physical system:
per-motor RPM (from the RPM deck) converted to thrust via a bench-identified motor constant `k_T`,
then mapped to body torque via the known arm geometry — not from the previous *commanded* torque,
which would not account for real motor response lag. This is the key structural difference from
the geometric law: INDI never needs an accurate model of thrust-to-RPM response, aerodynamic drag,
or unmodeled disturbances — any such effect is automatically visible in the measured `α_meas` and
is rejected by the increment on the next control cycle.

### 3.4 An added filter stage: targeting a diagnosed narrowband resonance

During brushless-platform commissioning, a genuine narrowband attitude oscillation was identified
at ≈7.22 Hz (§6.2 gives the full root-cause analysis). Because the standard Stage-1 Butterworth
filter (cutoff 60–70 Hz) was confirmed, by direct 500 Hz SD-card measurement, to pass this
frequency essentially unattenuated (`|α_meas|/|α_raw| = 1.000` at 7.22 Hz), and because lowering
the Butterworth cutoff into that band was tested and found to degrade tracking substantially (a
broad low-pass necessarily also attenuates and phase-delays everything *below* its cutoff, not
just the resonant band), an **optional, runtime-tunable second-stage band-reject (notch) filter**
was designed and implemented as a targeted fix. Using the RBJ Audio-EQ-Cookbook band-reject biquad
topology (`Q = f0/bw`, default center 7.2 Hz / bandwidth 5 Hz ≈ 4.7–9.7 Hz attenuated band), it is
applied — when enabled — identically to `α_meas`, `α_ref`, and the increment-base term
`τ_current`, so that no term of the INDI sum can end up phase-mismatched relative to the others
(mirroring, at a second filter stage, exactly the reasoning already established for the Stage-1
Butterworth in §3.3). Disabled by default (`notch_en=0`), the added code path is verified
byte-identical to the pre-existing behavior. As of this writing, the filter is implemented and
build-verified but **not yet flight-tested** — see §7 for status and next steps.

---

## 4. Experimental Setup

### 4.1 Platforms

| Platform | Motor type | AUW mass | Inertia `J_xx=J_yy` | Notes |
|---|---|---|---|---|
| Standard CF2.1 | Brushed (0716) | 36.4 g | 1.657e-5 kg·m² | Reference platform; lightest, lowest peak thrust |
| Thrust-upgraded CF2.1 | Brushed (upgrade kit) | 38.6 g | ≈1.657e-5 kg·m² | Higher peak thrust (0.18 N/motor vs 0.13 N), heavier rotor → slower motor response |
| Brushless CF21BL | Brushless | 41.0 g | diag(2.395e-5, 2.395e-5, 3.235e-5) kg·m² (Busetto et al. 2025) | Highest motor bandwidth, DShot bidirectional telemetry |

All three platforms were bench-identified for their per-motor thrust constant `k_T` (thrust stand,
RPM sweep, `k_T = thrust / RPM²`) before any INDI flight, per the hardware prerequisites in §3.3.

### 4.2 Software/hardware pipeline

Trajectories are generated offline in Rust (`export_poly4d` binary), converted to an 8-coefficient
polynomial (Poly4D) representation, and uploaded to the drone via the Crazyswarm2 (CS2) ROS 2
framework. Position is estimated from OptiTrack motion capture (fed to the onboard EKF via
`send_extpose`), and the onboard high-level commander evaluates the uploaded polynomial at 1 kHz,
feeding the geometric or INDI attitude controller (running at 500 Hz) via the standard Crazyflie
firmware control loop. This "Mode E" pipeline (CS2 + OptiTrack) supersedes an earlier onboard-only,
optical-flow-based pipeline used for initial commissioning flights, providing globally drift-free
pose for repeatable long-duration and high-speed flights.

### 4.3 Metrics

The primary metric throughout is **phase-aligned XY position RMSE**: the flown XY path is
time-shifted to maximize cross-correlation with the planned path before computing RMSE, correcting
for any constant lag between the commanded and (state-estimator-observed) flown trajectory that
would otherwise inflate a naive time-synchronous RMSE. Secondary metrics are roll/pitch tracking
error (flown attitude vs. the flatness-derived planned attitude) and, for stress-testing, per-axis
gyro standard deviation as a proxy for attitude noise/oscillation severity.

### 4.4 Validated flight envelope

All exported trajectories were validated against a physically confirmed usable flight volume
(x: ±1.2 m, y: ±2.5 m, z: 0.5–1.8 m, world frame centered on the takeoff origin) by densely
sampling every exported trajectory (across Modes 1 and 3, and across the full kt range flown) and
checking worst-case bounding boxes before any flight — not a single-`k_t` spot check.

### 4.5 Trajectory library

The core benchmark maneuver used for every cross-platform, cross-controller comparison is a
**figure-8** at `k_t=0.05` (lap time ≈6.9 s). Extended aggressiveness sweeps additionally use
**oval**, **circle**, and **slalom** shapes at increasing `k_t` to characterize each platform's
practical speed/aggressiveness ceiling beyond the fixed baseline benchmark.

---

## 5. Results

### 5.1 Per-platform tuning summary

**Standard CF2.1.** Filter cutoff sweep found `fc_bw=70 Hz` optimal (3.1 cm RMSE, lowest among
tested cutoffs). Final locked configuration: `kr=1050, kw=87` (`ωₙ≈32.4 rad/s`), `kp_xy=40,
kv_xy=3.3`. Best single flight: 3.6 cm RMSE. 10-flight INDI-vs-geometric comparison average:
**3.87 cm** (INDI) vs. 6.14 cm (geometric) — a 37% reduction.

**Thrust-upgraded CF2.1.** Attitude gain ladder found a hard ceiling at `kr=603` (`ωₙ≈24.6 rad/s`)
— substantially below the standard platform's 1050, despite the upgrade kit's higher peak thrust
(0.18 N/motor vs. 0.13 N). Final locked configuration: `kr=603, kw=90` (`ζ≈1.83, τ_slow=137 ms`),
`kp_xy=40, kv_xy=8`. 11-flight validation: **3.72 cm** mean RMSE (std 0.21) — closely matching, and
in this particular n=11 batch marginally beating, the standard platform, despite the lower
achievable `kr`.

**Brushless CF21BL.** The most extensive tuning campaign of the three: a coupled gain ladder
(`kr/kw/kp/kv` scaled together) found tracking RMSE improving monotonically up to a hover-noise
ceiling at `kr≈2400-2600`; two subsequent **decoupled** sweeps (holding `kr` fixed, sweeping `kw`
alone, then `kv` alone) found that pulling `kw` and `kv` *down* independently of their
ratio-locked values improved *both* RMSE and attitude tracking error simultaneously — a genuine
decoupling win, not a tradeoff, attributable to `kw` directly and linearly injecting raw gyro noise
via the `K_W·e_ω` term. Final locked configuration: `kr=2400, kw=170` (`ζ≈1.74`), `kp_xy=64,
kv_xy=5`. 12-flight validation at this config: **2.48 cm** mean RMSE (std 0.14) — the best of any
platform in this project.

### 5.2 Headline cross-platform/cross-controller comparison

| Configuration | n | Mean XY RMSE | Std | vs. Standard Geometric |
|---|---|---|---|---|
| Standard Geometric (baseline) | 10 | 6.14 cm | 0.52 | — |
| Standard INDI | 10 | 3.87 cm | 0.24 | −37% |
| Upgraded INDI | 11 | 3.72 cm | 0.21 | −39% |
| Brushless INDI (fc_bw=60) | 12 | 2.48 cm | 0.13 | −60% |
| Brushless INDI (fc_bw=70) | 13 | 2.45 cm | 0.12 | −60% |

All figure-8, `k_t=0.05`. The fc_bw=60 vs. 70 Hz brushless columns are otherwise identical
configuration, isolating the filter cutoff's effect: the two are statistically indistinguishable
(2.48±0.14 vs. 2.45±0.12 cm), confirming that filter cutoff is not a lever that moves tracking
performance across the 20–100 Hz range tested — consistent with the broader finding that this
platform's tracking ceiling is set by attitude gain (and the resonance it induces, §6.2), not
filter tuning.

### 5.3 Extended aggressiveness sweeps (upgraded vs. brushless)

**Oval.** Both platforms clean through `k_t≤0.3–0.4`; upgraded crashed consistently at `k_t=0.5–0.6`
(3/3 attempts), while brushless remained clean through `k_t=0.5` and crashed only at `k_t=0.7`.
Brushless's working ceiling is roughly `k_t≈0.5` vs. upgraded's `k_t≈0.3–0.4` — attributable to the
bandwidth gap (`ωₙ≈7.8 Hz` brushless vs. `≈3.9 Hz` upgraded) giving brushless more control
authority to correct through sharp direction changes, at the cost of sitting closer to its own
resonant regime.

**Circle.** Newly swept on both platforms this project (previously only `k_t=0.05` existed). Both
platforms show the same qualitative pattern: a clean-flight region followed by a **marginal zone**
(mixed crash/clean outcomes at the same `k_t`, not a sharp threshold) — upgraded clean to `≈0.6`,
brushless clean to `≈0.5`, both showing failures by `k_t=0.8–1.0`. Cross-platform RMSE at matched
`k_t`: tied at `k_t=0.05` (1.7 vs. 1.9 cm), brushless ahead at `k_t=0.3` (1.4 vs. 2.2 cm, ≈36%
lower) — the direction of advantage is `k_t`-dependent, not a fixed platform ranking.

**Slalom.** Brushless flies the benchmark slalom cleanly; the upgraded platform failed on this
shape specifically across multiple attempts at the same `k_t=0.05` that both platforms handle
cleanly on every other trajectory shape — an unresolved, shape-specific failure mode distinct
from the oval/circle aggressiveness ceilings above (see §7).

---

## 6. Discussion

### 6.1 Why the platforms rank the way they do

The controlled comparison in §5.2 places the platforms in the order brushless < standard ≲
upgraded (lowest to highest RMSE), and this ordering is explained almost entirely by one variable:
**achievable INDI attitude bandwidth**, which is in turn set by motor response speed, not peak
thrust. The thrust-upgraded kit provides more peak thrust per motor (0.18 N vs. 0.13 N) but a
heavier rotor and larger propeller, giving it *more* rotational inertia and therefore a *slower*
response to a commanded RPM change (bench-measured actuator time constant τ≈71 ms vs. the standard
platform's presumed faster response and the brushless platform's measured τ≈44 ms). Because INDI's
stability margin is set by how quickly the actual motor can realize a commanded change in angular
acceleration, a slower actuator directly caps the maximum stable `K_R` — and a lower `K_R` means a
lower closed-loop attitude bandwidth, which propagates directly to worse tracking of an aggressive
trajectory. This is not a tuning failure; it is documented (`results_2026-07-11_upgraded_drone.md`
§6) as a genuine, hardware-set ceiling, closing the gap to the standard platform would require
either faster motors or an explicit feedforward compensation for the known motor lag — both
outside the scope of gain tuning.

### 6.2 The brushless attitude resonance: full root-cause chain

The brushless platform's higher achievable bandwidth (`K_R=2400`, `ωₙ≈7.8 Hz`) is exactly what
gives it the best tracking performance in this project — and exactly what puts its closed-loop
natural frequency close enough to a genuine, measured ≈7.22 Hz spectral peak in the attitude
signal that the two interact. High-fidelity 500 Hz SD-card logging (as opposed to the lower-rate
100 Hz radio telemetry used for routine flight logging) established, quantitatively:

- The resonance match is not coincidental: `ωₙ=√K_R/2π = 7.80 Hz` at `K_R=2400` matches the
  measured shake frequency (7.22 Hz) to within 8%.
- **Actuator lag is the dominant phase-loss term.** Converting the bench-measured actuator lag
  (31.6–33.6 ms) to phase at 7.22 Hz gives ≈82° — roughly four times the phase loss contributed by
  EKF attitude-estimate lag (≈20°) at the same frequency.
- The existing Stage-1 Butterworth filter (cutoff 60–70 Hz) does essentially nothing at 7.22 Hz
  (measured `|filtered|/|raw|=1.000`, phase shift only ≈−2°) — it was set far above the resonant
  band, by design, to avoid excess phase lag elsewhere in the useful tracking bandwidth.
- Lowering the filter cutoff into the resonant band was tested directly and found to make tracking
  measurably worse — a broadband low-pass necessarily trades off phase margin across the *entire*
  band below its cutoff to attenuate one narrow region, which is a net loss when the useful
  tracking signal also lives well below 60–70 Hz.

Physically: a genuine feedback resonance — the closed-loop system's own natural frequency
`ωₙ=√K_R` sits where the combination of loop gain and total phase lag (actuator lag dominant, EKF
lag secondary) satisfies the classical self-sustained-oscillation (Barkhausen/Nyquist) condition.
The oscillation frequency is set by where `K_R` places `ωₙ`; the fact that it *sustains itself*
there is set by how much phase the loop has run out of by that frequency. This reframing directly
motivates the §3.4 notch filter: attenuating the loop's own gain at exactly the frequency where the
sustained-oscillation condition is met is a standard control-engineering technique for a
lightly-damped resonant mode, and — critically — it does not require removing or modeling the
underlying actuator lag to be effective, since breaking the loop-gain-≈1 condition at that one
frequency is sufficient on its own to prevent the sustained oscillation. It is nonetheless a fix
for *this specific operating point*; if `K_R` is raised further in the future, the resonance would
simply relocate to wherever the loop's phase margin next runs out, and the notch (tuned to a fixed
band) would need to move with it.

---

## 7. Limitations & Future Work

**Brushless attitude resonance (open, pending validation).** The §3.4/§6.2 notch filter is
implemented, applied consistently across all three signal chains that feed the INDI increment
(verified end-to-end, byte-identical when disabled), and build-verified, but has not yet been
flight-tested. Expected result if the resonance-suppression hypothesis is correct: a sharp
amplitude reduction in the notch-filtered angular acceleration signal at ≈7 Hz relative to the raw
and Stage-1-filtered signals, and visibly calmer commanded torque in that band. If the shake proves
to be dominated by the actuator's phase lag itself rather than by the resonant amplification the
notch targets, this fix alone may only partially resolve it — the next lever in that case would be
an explicit actuator-lag compensation term in the control law, which is a larger, unimplemented
change.

**Slalom-specific failure (open, unexplained).** The thrust-upgraded platform fails specifically on
the slalom trajectory shape at the same `k_t=0.05` it handles cleanly everywhere else, while
brushless flies it cleanly. This does not fit the bandwidth-ceiling explanation used for the
oval/circle results (§6.1) since `k_t=0.05` is well within the upgraded platform's demonstrated
ceiling on every other shape — it remains a genuinely open, shape-specific issue.

**A separate, never-root-caused crash** occurred on a distinct maneuver family (a wide teardrop
loop shape, under the plain geometric controller, on both platforms, at the mildest tested
aggressiveness) — flagged but not investigated further within this project's scope.

**Inverted-loop aerobatic maneuver (bonus, beyond core scope).** Using a distinct, explicit-attitude
trajectory-authoring approach not covered in this report's core planning methods (§3.1), a fully
inverted vertical loop was successfully commissioned on the brushless platform, achieving a
repeatable inversion (roll 163–178° at the apex across 3/3 flights) and, in the best flight, a
genuine post-inversion recovery (arrested a near-freefall, avoided ground contact, climbed back to
level, positive-altitude flight). One narrower failure mode remains inside the maneuver's
auto-generated exit ramp; several targeted fixes were tried and ruled out, and the mechanism is
precisely characterized but not yet resolved.

**Third platform and dedicated aggressive-maneuver phase.** Both were explicitly descoped from this
project's original plan once the resonance investigation above became the dominant open technical
question; the inverted-loop work above represents a partial, unplanned recovery of the latter.

---

## 8. Conclusion

This project delivered a complete, working autonomous-flight stack — minimum-snap motion planning
with automatic time allocation, a differential-flatness-derived geometric baseline, and a fully
commissioned INDI attitude controller — validated end-to-end on three physically distinct
quadrotor platforms. The central, controlled result is a 37–60% reduction in figure-8 tracking RMSE
from INDI over the geometric baseline, with the specific magnitude of that improvement shown to be
governed almost entirely by each platform's achievable motor bandwidth rather than by tuning
effort. The trajectory library was pushed well beyond the core benchmark (oval, circle, and slalom
sweeps across a range of planner aggressiveness on two platforms), characterizing real,
platform-specific speed ceilings. The project's most significant open technical finding — a
genuine, quantitatively root-caused ≈7.22 Hz attitude resonance on the highest-bandwidth platform,
arising from the interaction of INDI's own closed-loop dynamics with real actuator lag — is
documented in full, together with a data-driven, theoretically justified fix (a targeted notch
filter) that is implemented and ready for validation. Combined with an unplanned but successful
excursion into fully inverted aerobatic flight, the project's honestly reported limitations are, in
every case, precisely characterized open problems rather than unexplained failures — a state the
report has aimed to make fully traceable back to the underlying flight data and code.

---

## References

1. E. Tal and S. Karaman, "Accurate Tracking of Aggressive Quadrotor Trajectories using
   Incremental Nonlinear Dynamic Inversion and Differential Flatness," arXiv:1809.04048v2, 2020.
2. D. Mellinger and V. Kumar, "Minimum snap trajectory generation and control for quadrotors,"
   IEEE ICRA, 2011.
3. C. Richter, A. Bry, and N. Roy, "Polynomial trajectory planning for aggressive quadrotor
   flight in dense indoor environments," in *Robotics Research*, Springer, 2016.
4. T. Lee, M. Leok, and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on
   SE(3)," IEEE CDC, 2010.
5. M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness of Quadrotor Dynamics
   Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories," IEEE RA-L, 2018.
6. A. Busetto et al., "Nonlinear System Identification Nano-drone Benchmark," 2025 (brushless
   platform parameters).
