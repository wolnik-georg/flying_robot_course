# Problem Statement and Research Questions

**Thesis:** *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*
**Track:** Writing (theoretical). Runs in parallel with the lab track — see
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md).
**Status:** first draft, 23 August 2026. **No experimental result is stated in this document**;
the hypotheses below are stated so that they can be falsified, not because they are believed.

---

## 1. Problem statement

### 1.1 Setting

A team of \(N\) multirotors flies in close proximity, closer than the separation at which their
airflows can be treated as independent. Each vehicle \(i\) obeys

$$
\dot{p}_i = v_i,\qquad
m\,\dot{v}_i = m g e_3 + R_i f_i + f_{\mathrm{res},i},\qquad
\dot{R}_i = R_i\hat{\omega}_i,\qquad
J\dot{\omega}_i = -\omega_i\times J\omega_i + \tau_i + \tau_{\mathrm{res},i},
$$

following the notation of [`04_Unified_Residual_Wrench_Model.md`](04_Unified_Residual_Wrench_Model.md).
The terms \(f_{\mathrm{res},i}\) and \(\tau_{\mathrm{res},i}\) collect every force and moment the
nominal model omits. In free flight they are small. In tight formation they are dominated by the
**aerodynamic interaction between vehicles** — principally the downwash of vehicles above.

The residual is a function of the *relative* configuration of the team, not of the vehicle's own
state alone:

$$
f_{\mathrm{res},i} \;\approx\; \mathcal{F}\!\left(\{\,p_j - p_i,\; v_j - v_i \,\}_{j\neq i}\right).
$$

Two properties of \(\mathcal{F}\) matter for control. It is **strongly nonlinear** — the force
grows sharply as separation closes and decays quickly with lateral offset — and it is
**asymmetric**: the vehicle below is affected by an order of magnitude more than the one above.

### 1.2 The control problem

A controller must produce a wrench that holds each vehicle on its commanded trajectory while
\(f_{\mathrm{res},i}\) acts. Two fundamentally different sources of information about
\(f_{\mathrm{res},i}\) are available, and the choice between them defines the families of methods
this thesis compares:

- **Measurement.** \(f_{\mathrm{res},i}\) can be *estimated after the fact* by comparing measured
  acceleration against the acceleration the nominal model predicts for the commanded actuation.
  This requires no prior knowledge of the interaction, but the estimate is necessarily delayed by
  the sensing and filtering chain, and it exists only once the disturbance has already perturbed
  the vehicle.
- **Prediction.** \(f_{\mathrm{res},i}\) can be *predicted before it acts* from the relative states
  of the neighbours, using a model learned offline. This can act ahead of the disturbance, but it
  is only as good as the model, and the model is only valid over the region of relative
  configurations its training data covered.

These are complementary rather than competing in principle: one is reactive and general, the other
is predictive and specific. Whether combining them helps in practice — and under what conditions
each is preferable — is not established for multi-robot flight.

### 1.3 What is missing

The literature contains strong instances of both families (see
[`02_Literature_Matrix.md`](02_Literature_Matrix.md)), but they are reported under different
hardware, different trajectories, different separations and different metrics. Reported
improvements are therefore not comparable across papers: a method evaluated on a stacked hover at
30 cm and a method evaluated on an aggressive trajectory at 24 cm are not measuring the same
difficulty. A hybrid of the two families has been demonstrated on a **single** vehicle, with and
without a slung payload, where its advantage over reactive compensation alone was **small and not
consistent across conditions**; the multi-robot case — where the residual is produced by *other
vehicles* rather than by the vehicle itself, and is a function of a measurable relative state —
has not been studied.

**In one sentence:** it is currently not possible to say, on evidence, *which family of
interaction-force compensation is preferable for tight multirotor formation flight, under which
conditions, and at what cost* — because no study has evaluated them against each other under
identical conditions.

### 1.4 Scope

| In scope | Out of scope |
|---|---|
| Inter-vehicle aerodynamic interaction (principally downwash) in tight formation | Wind, gusts, and other external disturbances |
| 2- and 3-vehicle teams | Large swarms |
| Homogeneous vehicles (Crazyflie brushless) | Heterogeneous teams |
| Trajectory tracking of *pre-planned*, collision-free formations | Interaction-aware motion planning; collision avoidance |
| Onboard inference, offline training | Onboard/online learning |
| Force residual \(f_{\mathrm{res}}\) | Moment residual \(\tau_{\mathrm{res}}\), measured but not separately compensated |

The scope is deliberately narrow. The contribution is a *controlled comparison*, and every degree
of freedom left open in the experiment is one more explanation for an observed difference.

---

## 2. Research questions and hypotheses

Four questions. Each is answerable by the planned experiments, and each hypothesis is stated so
that a negative result is a result.

---

### RQ1 — Comparison under identical conditions

> **Under identical hardware, trajectories, formations and separations, how do reactive,
> predictive, and hybrid interaction-force compensation strategies compare in trajectory-tracking
> accuracy and residual-force rejection?**

This is the thesis's primary question and the one the whole experimental design serves. The
strategies (Section [`01`](01_Thesis_Project_Snapshot.md) §2) span the three families; the
formation library ([`10`](10_Formation_Library.md)) is frozen so that every strategy meets exactly
the same excitation.

**H1.** All compensating strategies reduce tracking error relative to an uncompensated geometric
baseline, and the difference *between families* is smaller than the difference between any of them
and the baseline.

*Why it is worth testing even though the first half seems obvious:* the second half is not. If the
families separate strongly, the choice of family matters and the thesis is a selection guide. If
they do not, the practical conclusion is that *any* compensation suffices and the engineering
question becomes one of cost, which is RQ4.

**Measured by:** position RMSE (total and per axis); maximum deviation from the commanded
separation; the magnitude of the measured residual \(\|f_{\mathrm{res}}\|\) that remains
uncompensated. The coplanar control scenarios (C1–C3), where no interaction should exist, bound
the measurement noise floor and make a positive result interpretable.

---

### RQ2 — Where does each family win?

> **Does the relative advantage of reactive versus predictive compensation depend on the
> interaction regime — separation, relative velocity, and whether the disturbance is quasi-static
> or transient?**

This is the question with the most engineering value, because it turns a ranking into a rule.

**H2.** Predictive compensation is favoured where the residual is *well determined by the relative
state* and varies slowly — static and slowly-varying formations at separations inside the training
distribution. Reactive compensation is favoured where the residual is *poorly predicted* — fast
transients, and relative configurations outside the training data. If both hold, a crossover exists
and can be located in separation/relative-velocity space.

**Measured by:** the same metrics as RQ1, resolved by scenario class. The formation library
separates these regimes by construction: static offsets (A1, A2), quasi-static vertical stacks in
motion (A3), lateral traverses (A4), and fast relative motion / crossings (A7). A model trained on
one class and evaluated on another isolates the "outside the training distribution" case directly.

---

### RQ3 — How much does our 2-robot residual model lose on 3 robots?

> **How much prediction accuracy does a deep-sets residual model trained only on two-vehicle data
> lose when applied to three-vehicle formations, and is that loss small enough for the summed
> per-neighbour architecture to remain the right engineering choice?**

> ⚠️ **Deliberately narrower than "do interactions superpose?"** That question is already answered
> in the literature, and the answer is no: Shi et al. (2020) report that with more than two
> vehicles *"the aerodynamic effect is not a simple superposition of each pair"*, and Gielis et al.
> (2023) exists specifically because single-vehicle models are *"unlikely to be sufficient for
> predicting aggregate forces"*. Asking it as an open question would be claiming to discover
> something already published. See [`20_Verified_Claims.md`](20_Verified_Claims.md).

What is genuinely open is the **magnitude** of the penalty on our platform, with our architecture,
at our separations — and whether it is small enough to matter. A deep-sets model sums a
per-neighbour embedding and is structurally committed to superposition; the literature says that is
imperfect, but a controller does not need an exact model, only one good enough that the residual it
leaves is smaller than the one it removes.

**H3.** A model trained on 2-vehicle data transfers to 3-vehicle formations with measurably
degraded accuracy, and the degradation is concentrated where two upstream wakes overlap. It remains
useful — i.e. the compensated residual is still smaller than the uncompensated one.

**Measured by:** predicted versus measured residual on 3-robot scenarios (B1–B3), from a model
trained on 2-robot data only. Available on **every** flight, because the prediction is logged
whether or not it is used.

*Answerable even if the 3-robot control campaign is descoped*, since it needs only logged data.

---

### RQ4 — What does each strategy cost?

> **What are the sensing, computational and data requirements of each strategy, and is the added
> complexity of the hybrid justified in the multi-robot case?**

A method that wins by a small margin while requiring rotor-speed feedback, an offline training
campaign and a larger compute budget may still be the wrong engineering choice.

**H4.** The hybrid's advantage is larger in the multi-robot case than the single-vehicle case.
Cobo-Briesewitz et al. report the hybrid best without a payload but *"with a small margin"*, and
*"similar in performance"* to INDI with one — on a **single** vehicle, where the residual is
self-induced (blade flapping, drag, payload swing) and predicted from the vehicle's *own* state.
The multi-robot residual is different in kind: larger, and a function of a *measurable relative*
state, which is more information for the predictive half to exploit.

*This hypothesis may well be false, and that would be worth reporting.* If the hybrid's margin
stays small when the residual is caused by neighbours, the practical conclusion is that reactive
compensation alone is sufficient and the training campaign is not worth its cost.

**Measured by:** onboard computation per control step; sensing prerequisites (in particular
whether rotor-speed measurement is required at all); quantity of training data needed to reach a
given residual-prediction accuracy; control effort.

---

## 3. How the questions map onto the experiments

| RQ | Needs | Provided by |
|---|---|---|
| RQ1 | Every strategy, same library, same conditions | C.4 Systematic Comparison |
| RQ2 | Metrics resolved by scenario class; a model evaluated outside its training set | C.4, plus a deliberate train/test split across scenario classes in C.2 |
| RQ3 | 3-robot logs, and a model trained on 2-robot data only | C.1 (3-robot logging) + C.2 |
| RQ4 | Compute/sensing accounting, and a training-set-size sweep | C.2 and C.4 |

The lab track (C.0 → C.4) is defined in
[`07_Thesis_Progress_Checklist.md`](07_Thesis_Progress_Checklist.md).

---

## 4. Threats to validity — to be addressed, not hidden

Stated now so the experimental design can protect against them, and so the discussion chapter can
report honestly on those it cannot.

| Threat | Mitigation |
|---|---|
| **Tuning confound** — a strategy looks better because it was tuned more | Gains are frozen before any data collection (C.0 stage 2) and shared across strategies where the architecture permits. Where a strategy needs its own gain, this is stated |
| **Training/test leakage** — the residual model is evaluated on flights it was trained on | Held-out scenarios, and a split by contiguous flight segment rather than by sample: at 500 Hz consecutive samples are near-identical and a random split would leak almost the entire test set |
| **Measurement floor** — reported differences are smaller than the noise | The coplanar control scenarios C1–C3 exist to measure that floor under the same pipeline |
| **A single airframe** — conclusions may not generalise beyond the Crazyflie brushless | Stated as a limitation. Reported quantities are normalised where a normalisation is defensible |
| **Selection of the "representative" method per family** | Each implemented method is identified with the specific paper it follows, and deviations are documented |
| **Simulation-to-hardware gap** | No claim rests on simulation. Simulation results are used to *disprove* implementations, never to validate a method |

---

## 5. Open questions for the supervisor

- Should RQ3 (2→3 robot transfer) be promoted to a primary question? It is cheap — it needs logged
  data, not a working 3-robot controller — and it directly tests an assumption the predictive
  literature relies on.
- Is a training-set-size sweep (RQ4) worth the flight time, or is a qualitative statement enough?
- ~~How much of the strategy set must be flown for the comparison to be defensible?~~ **Resolved
  2026-09-14**: two tiers, not three — Minimum = Methods 1–4 (Pure INDI, Geometric+NN, FBL+NN,
  Hybrid/NA-INDI), Advanced = Methods 5–7 (the RL-based strategies). See `01` §2.
