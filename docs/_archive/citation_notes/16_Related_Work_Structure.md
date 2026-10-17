# Related Work — Chapter Structure

**Thesis:** *Comparison of Control Strategies for Interaction-Force Aware Multirotor Teams*
**Track:** Writing (theoretical).
**Status:** structural proposal, 23 August 2026. This is the **skeleton and argument** of Chapter 2,
not the prose. Papers and their attributes are in
[`02_Literature_Matrix.md`](02_Literature_Matrix.md); reading status in
[`06_References_Overview.md`](06_References_Overview.md).

---

## The argument the chapter has to make

A related-work chapter that lists papers by topic is a bibliography with headings. This one is
organised so that reading it end to end *produces* the gap statement rather than asserting it at
the end. The line of argument is:

> The interaction force is real, large, and structured (§2.2) → it can be **measured** online and
> rejected reactively (§2.3) → or **predicted** offline from relative state and cancelled ahead of
> time (§2.4) → prediction only helps once it is correctly injected into a controller, and how that
> is done differs by controller family (§2.5) → the compensation can also be learned end to end
> rather than modelled (§2.6) → the two information sources are complementary, so hybrids have been
> proposed (§2.7) → **but these families have never been evaluated against one another under
> identical conditions, and the evaluation practice of the field makes cross-paper comparison
> invalid** (§2.8).

§2.8 is the load-bearing section. It is what makes this thesis a contribution rather than a
re-implementation, and it is the section most easily written badly — as a complaint about other
people's papers rather than as a methodological observation.

---

## Proposed structure

### 2.1 Multirotor dynamics and baseline control

Establishes notation and the uncompensated baseline that every strategy is built on.

- Rigid-body multirotor model; the residual wrench \(f_{\mathrm{res}}, \tau_{\mathrm{res}}\) as the
  term collecting unmodelled effects — consistent with
  [`04_Unified_Residual_Wrench_Model.md`](04_Unified_Residual_Wrench_Model.md)
- Geometric control on SE(3) (Lee et al.) — the baseline, and the controller three strategies build on
- Differential flatness and the flatness-based feedforward that INDI and FBL both rely on

*Role in the argument:* defines \(f_{\mathrm{res}}\) precisely, so the rest of the chapter is about
one clearly named quantity.

---

### 2.2 Aerodynamic interaction in multirotor teams — the physics

What the disturbance actually is, before any control.

- Downwash structure; the sharp radial decay and the strong asymmetry between the vehicle above
  and the vehicle below
- Experimental characterisation — force/moment measurement and flow visualisation
  (Kiran et al., RSS 2025)
- Why first-principles modelling is hard: near-field turbulence, ground and wall effects,
  vehicle-specific geometry

**Key references:** Kiran et al. 2025 (characterisation, PIV); the downwash-modelling half of
Gielis et al. 2023.

*Role in the argument:* justifies *learning* the residual rather than deriving it, and gives the
physical intuition (asymmetry, sharp gradients) that later explains why permutation-invariant
architectures and distance guards are used.

---

### 2.3 Reactive compensation — measuring the residual online

The first family: no prior model, estimate the disturbance from its effect.

- Incremental Nonlinear Dynamic Inversion: the incremental formulation and what it assumes
  (Smeur et al. 2018, cascaded INDI)
- INDI for accelerations and aggressive flight (Tal & Karaman 2018/21) — the implementation this
  thesis follows
- The role of rotor-speed feedback in reconstructing the applied wrench, and what degrades without it
- Limits: the estimate is delayed by sensing and filtering, and exists only after the disturbance
  has already acted

**Key references:** Smeur et al. 2018; Tal & Karaman 2018/21.

*Role in the argument:* establishes the reactive family as general and cheap, and identifies delay
as its structural limitation — the thing prediction exists to remove.

---

### 2.4 Predictive compensation — learning the residual from relative state

The second family: model the interaction offline, evaluate it online.

- Neural-Swarm (Shi et al. 2020): learned residual as feedforward; permutation-invariant network
- Neural-Swarm2 (Shi, Hönig et al. 2022): heterogeneous deep sets; residual used in planning as
  well as control
- **Architectural choices and what they buy**, treated as a comparison rather than a list:
  - *Deep sets* — permutation invariance and neighbour-count independence
  - *Equivariance* — SO(2)-equivariant models (Smith et al. 2023) exploit rotational symmetry to
    reduce data requirements
  - *Learned aggregation* (Gielis et al. 2023) — Deep Sets (sum-pool) against a linear-summation
    baseline, measured on a load stand; "graph" is the abstract's language, not the implementation.
    For when pairwise superposition is not exact
- The superposition assumption, which is where §2.4 connects to **RQ3**

**Key references:** Shi et al. 2020; Shi, Hönig et al. 2022; Smith et al. 2023; Gielis et al. 2023.

*Role in the argument:* establishes the predictive family, and surfaces its structural
assumption — that the aggregate interaction is a sum of per-neighbour terms — as something this
thesis can test rather than inherit.

---

### 2.5 Injecting the prediction — the controller matters

A predicted residual is useless until it enters a control law, and *how* it enters differs by
family. This section exists because it is where the thesis's Strategies 2, 3 and 6 differ from one
another despite sharing a residual model.

- **Geometric + NN** — feedforward into the desired force of an SE(3) controller. The simple case
- **FBL + NN** — feedback linearisation with a *flatness-preserving* residual, so the flatness
  structure the feedforward relies on is not destroyed by the correction (Hsieh et al. 2026)
- **Learning-based MPC** — the residual as part of the prediction model inside the horizon rather
  than as a correction at the current instant (Chee et al.; Hsieh et al. 2025, L1 KNODE-DW MPC);
  the compute cost this implies

**Key references:** Hsieh et al. 2026; Chee et al. 2024; Hsieh et al. 2025.

*Role in the argument:* shows that "predictive compensation" is not one method, and that comparing
a single predictive instance against a single reactive instance would be comparing two arbitrary
points rather than two families.

---

### 2.6 Learning the compensation directly — residual reinforcement learning

The third stance: do not model the force at all, learn the corrective action.

- ProxFly (Zhang et al. 2024/25): residual RL on top of a cascaded controller; residual thrust and
  body rates
- Simulation-to-reality transfer and domain randomisation
- What is gained (no explicit force model, no residual labels) and what is lost (no interpretable
  force estimate, harder to attribute failures, sim-to-real dependence)

**Key references:** Zhang et al. 2024/25.

*Role in the argument:* completes the space of approaches, and explains why these are the two
strategies deliberately scheduled last in this thesis — they answer a different question and carry
a different risk profile.

---

### 2.7 Hybrid reactive–predictive compensation

The direct motivation for the thesis's central hypothesis.

- Cobo-Briesewitz, Wahba & Hönig (L4DC 2026). Two methods: **LINDI**, where a network replaces
  INDI's residual estimate entirely, and **NA-INDI**, the hybrid, where the network predicts the
  bulk and INDI corrects the remainder
- **The paper's headline result is sensor elimination, not the hybrid**: LINDI matches INDI's
  tracking performance *without requiring rotor-speed measurement*. Compensation of any kind
  reduced tracking error by roughly 40 % relative to the geometric baseline
- The hybrid's benefit is **real but small and inconsistent**: best without a payload, "although
  with a small margin" (4.14 cm vs 4.28 cm for INDI, figure-8); with a payload it "remains similar
  in performance" to INDI
- Why this does not settle the multi-robot case: the residual there is **self-induced** (blade
  flapping, drag, payload swing) and predicted from the vehicle's *own* state. The multi-robot
  residual is caused by *other vehicles* and predicted from *relative* state — a different input,
  a different magnitude, and a different failure mode when a neighbour moves unexpectedly
- The complementarity argument: prediction is fast but only valid inside its training
  distribution; measurement is general but delayed

**Key reference:** Cobo-Briesewitz et al. 2026.

*Role in the argument:* this is where the gap becomes specific rather than generic — a concrete
published result whose scope the thesis extends into the regime where it should matter most.

---

### 2.8 How these methods are evaluated — and why the results do not compose

**The section that establishes the gap.** Methodological, not critical.

- A tabulation of the evaluation conditions in the literature: platform, team size, minimum
  separation, trajectory class, and reported metric. [`02_Literature_Matrix.md`](02_Literature_Matrix.md)
  already holds these columns; the chapter presents them as evidence
- The observation: reported improvements are relative to *each paper's own baseline*, measured on
  different hardware, at different separations, on different trajectories. A 36 % improvement on a
  stacked hover and a 56 % improvement on an aggressive trajectory are not comparable quantities,
  and neither can be subtracted from the other to rank two methods
- Why this is not a criticism of any individual paper — each is internally valid — but is
  nonetheless a limitation *of the field*, and one that only a dedicated comparison can remove
- What a fair comparison requires: identical airframe, identical trajectories and formations,
  frozen gains across strategies, a shared residual definition, and a measurement floor established
  by a null case

*Role in the argument:* converts "nobody has compared these" from an assertion into a demonstrated
consequence of how the field currently reports results.

---

### 2.9 Summary and gap

Half a page. Restates, in the terms built up above:

- The two information sources (measurement, prediction) and the third stance (learn the action)
- That representative methods exist for each and are individually validated
- That no study evaluates them against one another under identical conditions, and that the field's
  reporting practice makes such a comparison impossible to assemble post hoc
- That the hybrid case specifically is untested where its premise is strongest — multi-robot
- Forward reference to the research questions in
  [`15_Problem_Statement_and_Research_Questions.md`](15_Problem_Statement_and_Research_Questions.md)

---

## Mapping: sections → strategies → research questions

| Section | Strategies it grounds | RQ it feeds |
|---|---|---|
| 2.1 | Baseline (uncompensated geometric) | RQ1 |
| 2.2 | — (motivates learning the residual) | RQ2 |
| 2.3 | 1 Pure INDI | RQ1, RQ2, RQ4 |
| 2.4 | 2, 3, 4, 6 (shared residual model) | RQ2, **RQ3** |
| 2.5 | 2 Geometric+NN · 3 FBL+NN · 6 Learning-based MPC | RQ1, RQ4 |
| 2.6 | 5, 7 Residual RL | RQ1 (if flown) |
| 2.7 | 4 Hybrid neural-augmented INDI | **RQ4 / H4** |
| 2.8 | — | The gap itself |

Every section earns its place by grounding a strategy or a research question. A section that
grounds neither should be cut.

---

## Writing notes

- **Length target:** ~10–14 pages. §2.4 and §2.8 carry the argument and deserve the space; §2.6 can
  be short, since those strategies are scheduled last.
- **Do not restate the literature matrix as prose.** The matrix is an appendix or a figure; the
  chapter's job is the argument between the entries.
- **Name the specific paper each implemented strategy follows**, and note deviations. RQ1 depends on
  each strategy being a defensible representative of its family, and that claim is made here.
- **§2.8 must stay methodological.** The point is that the field's reporting conventions prevent
  composition, not that the papers are deficient.
- Reading status and priorities: [`06_References_Overview.md`](06_References_Overview.md).

---

## Open

- **Placement of §2.2 (physics).** Alternative: fold it into the introduction and start Chapter 2
  at the control methods. Kept separate here because the asymmetry and sharp gradients are needed
  to motivate the architectural choices in §2.4.
- **Is §2.6 (residual RL) needed** if strategies 5 and 7 are not flown? Recommendation: keep it,
  substantially shortened — the space of approaches is incomplete without it, and the thesis should
  say why those methods were scoped out rather than pass over them.
- Whether the evaluation-conditions tabulation in §2.8 belongs in the chapter body or an appendix.
