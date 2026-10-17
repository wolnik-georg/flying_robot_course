# Contribution Statement (draft)

**Track:** Writing (W.4). **Status:** draft, 23 August 2026.
Supersedes the contribution section of [`03_Gap_and_Contribution_Statement.md`](03_Gap_and_Contribution_Statement.md),
which was written before the research questions existed and before the source ledger caught a
misattribution in its gap statement.

> **Drafting rule applied throughout:** claim the comparison, not a method. This thesis proposes no
> new control law. Its contribution is evidential and methodological, and overclaiming would be
> both dishonest and easy for an examiner to puncture.

---

## 1. The gap, stated precisely

Representative methods exist for reactive compensation (INDI), predictive compensation (learned
residual models injected into geometric, feedback-linearising or MPC controllers), and their
hybrid. Each is internally validated. **They have never been evaluated against one another under
identical conditions.**

This is not an oversight by any individual author; it is a structural property of how the field
reports. Improvements are stated relative to each paper's own baseline, on different airframes, at
different separations, on different trajectories, under different metrics. Those numbers cannot be
assembled into a ranking after the fact — an improvement measured on a stacked hover and one
measured on an aggressive trajectory are not the same quantity.

Two more specific gaps follow:

- **The hybrid is untested where its premise is strongest.** Cobo-Briesewitz et al. (L4DC 2026)
  demonstrated a hybrid on a single vehicle. Its advantage there was **real but small** — best
  without a payload "although with a small margin", and "similar in performance" to INDI with one.
  In that setting the residual is self-induced and predicted from the vehicle's *own* state. In
  multi-robot flight it is caused by other vehicles and is a function of *measurable relative
  state*, which gives the predictive half more to work with. Whether that turns a small margin into
  a meaningful one is exactly what is untested.
- **The cost of assuming superposition is unquantified for a deployed model.** The literature
  already establishes that pairwise interactions do *not* compose exactly (Shi et al. 2020;
  Gielis et al. 2023). Yet deep-sets architectures, which sum a per-neighbour embedding, remain
  standard — because a controller needs a model that is *useful*, not exact. How much accuracy that
  simplification actually costs when a 2-robot-trained model meets a 3-robot formation is not
  reported.

---

## 2. What this thesis claims

### C1 — A controlled comparison *(primary)*

To the best of our knowledge, the first systematic evaluation of reactive, predictive and hybrid interaction-force compensation
**on the same airframe, the same trajectories, the same formations and the same frozen gains**,
with a shared definition of the residual and a measured noise floor.

**2026-09-14: dropped the "shared control-law substrate" framing that used to sit here** (grouping
strategies into those claimed to share one control law vs. those called "structurally different,"
as a basis for a stronger vs. weaker evidentiary claim). Operator's explicit correction: each of
the seven strategies is its own controller, evaluated as such — the comparison's rigor comes from
the shared *experimental* conditions below, applied uniformly to all seven, not from asserting
that some subset shares an implementation substrate. Where two strategies genuinely share
something worth noting for the reader's understanding (e.g. the hybrid extends INDI's residual
term with a predicted component, or geometric and INDI share the same outer-loop position
controller) — that is a clarifying, one-line remark in the strategy's own description (`18`), not
a formal comparability category.

This thesis provides, uniformly across all seven strategies:

- a **frozen formation library** of 16 scenarios, spec-checked and safety-gated, so every strategy
  meets identical excitation;
- **gains frozen before data collection** and shared across strategies wherever the architecture
  permits, so a difference cannot be a tuning artefact;
- **one residual definition**, \(f_{\mathrm{res}} = m(a_{\mathrm{meas}} - a_{\mathrm{model}})\),
  logged identically in every controller mode — including modes that do not use it;
- **null control cases** (coplanar formations, where no interaction should exist) that bound the
  measurement floor, so a positive result is separable from noise.

### C2 — The first multi-robot evaluation of the hybrid

**2026-09-14: this is now a guaranteed contribution of the Minimum-scope thesis, not a
stretch item** — the hybrid (Strategy 4) moved from the Advanced tier into Minimum on the
strength of its numerically-verified Rust port, so C2 no longer depends on reaching beyond the
baseline four strategies.

An empirical answer to whether combining a measured and a predicted residual helps when the
residual is produced by neighbouring vehicles. The single-vehicle result it extends showed only a
small margin for the hybrid — so a finding in **either** direction is informative: a larger margin
supports the complementarity argument, and a margin that stays small says reactive compensation
alone is sufficient and the training campaign is not worth its cost.

### C3 — Quantifying the cost of the superposition assumption

**How much** a 2-robot-trained deep-sets model loses on 3-robot formations.

**This is a measurement, not a discovery.** That pairwise interactions do not superpose is already
established — Shi et al. (2020) state it outright and Gielis et al. (2023) model aggregation
because of it. The thesis must not, anywhere, present non-superposition as its own finding. What is
unreported is the *magnitude* of the penalty for a model actually deployed on a vehicle, and
whether it is small enough that the summed architecture remains the right engineering choice.

Requires only logged data, not a working 3-robot controller, so it survives descoping of the
3-robot control campaign.

> Scoped down on 2026-08-23 after reading the sources. The earlier phrasing — "a direct test of the
> superposition assumption" — would have claimed to discover something Shi et al. (2020) already
> state outright.

### C4 — An engineering account of cost

Sensing prerequisites (in particular whether rotor-speed measurement is needed at all), onboard
computation, and training-data requirements, reported alongside performance so that the comparison
supports a decision rather than only a ranking.

### C5 — A reusable experimental apparatus *(supporting, not a scientific contribution)*

> Demoted 2026-08-23 to match Chapter 1, which lists four numbered contributions and describes the
> apparatus in a following paragraph. A thesis whose contribution is its tooling has not answered a
> question; this is listed so the engineering effort is visible, not to pad the claim.

An open, documented pipeline: the formation library, the residual measurement path, an onboard
deep-sets network with a weight-upload protocol, an offline training pipeline verified end-to-end
against the compiled controller, and a simulation environment running the same controller source
that flies. Secondary to C1–C4, but the reason C1 is feasible at all.

---

## 3. What this thesis does **not** claim

Stated explicitly, because a contribution is only credible next to its limits.

| Not claimed | Why |
|---|---|
| **A new control method** | Every strategy follows published work. The contribution is the comparison |
| **A better residual model than the literature's** | Our network is a deep-sets model sized for our platform; it is not benchmarked against published architectures |
| **That results generalise beyond this airframe** | One platform (Crazyflie brushless). Quantities are normalised where defensible; generalisation is discussed, not demonstrated |
| **That results generalise beyond downwash** | Wind, gusts and ground effect are out of scope |
| **That any strategy is best in general** | The useful outcome is *conditional*: which method under which interaction regime (RQ2) |
| **Anything from simulation** | Simulation is used to disprove implementations, never to validate a method. No claim rests on it |
| **Interaction-aware planning** | Trajectories are pre-planned and collision-free. Only control is compared |
| **Online learning** | Inference is onboard; training is offline |

### An outcome that would still be a contribution

If the strategies do not separate — if reactive, predictive and hybrid all perform comparably —
that is a **publishable negative result**, and arguably the more useful one: it would say the
choice of compensation family matters less than whether compensation is present at all, and shift
the engineering question to cost (C4). The experimental design is deliberately built so this
outcome is distinguishable from a failed experiment, which is what the null control cases and the
frozen gains are for.

---

## 4. Honest positioning

Three things are worth saying plainly, and are easy to get wrong in a contribution statement:

1. **"First systematic comparison" is a claim about the literature, and must be checked.** It is
   currently based on our own survey ([`02`](02_Literature_Matrix.md)), which the source ledger
   rates as the *least verified* internal document. Before this sentence goes in the thesis, the
   survey must be re-audited against the actual papers. If a comparable study exists, the claim
   becomes "the first to include the hybrid family" or similar — narrower, and still worth making.

2. **The apparatus is not the contribution.** A lot of engineering exists here — a formation
   library, an onboard network, a verified training pipeline, a simulator running the flight
   controller. It is C5 and it is secondary. A thesis whose contribution is its tooling has not
   answered a question.

3. **Nothing has flown yet.** As of this draft, every strategy is either implemented-and-unflown or
   not implemented. The contribution above is what the thesis *will* claim if the experiments
   succeed; it is not a description of results, and no part of this document should be read as one.

---

## 5. One-paragraph version (for the abstract)

> Tight multirotor formation flight is limited by aerodynamic interaction between vehicles,
> principally downwash. Two families of methods address it: reactive approaches that estimate the
> residual force online from inertial measurements, and predictive approaches that learn it offline
> from relative states and apply it as feedforward. Both are individually validated, and a hybrid
> has been demonstrated on a single vehicle — but they have never been compared under identical
> conditions, and the field's reporting conventions make such a comparison impossible to assemble
> post hoc. This thesis evaluates representative methods from each family on the same airframe,
> trajectories and formations with frozen gains and a shared residual definition, quantifies the
> conditions under which each is preferable, tests whether a two-robot residual model transfers to
> three, and reports the sensing and computational cost of each approach.

---

## 6. Open

- [ ] Verify the "first systematic comparison" claim against a re-audited survey (§4.1)
- [ ] Decide with the supervisor whether C3 (superposition) is promoted to a primary contribution —
      it is cheap and tests a widely inherited assumption
- [x] Fix the minimum defensible strategy set — **resolved 2026-09-14**: the hybrid (4) is now
      part of the Minimum set (1–4), not a stretch goal, on the strength of its
      numerically-verified Rust port (`stabilizer.controller=8`, not `7` — see
      `docs/strategy_controller_map.html`). C2 (below) is a guaranteed contribution of the
      Minimum draft, not contingent on reaching Advanced scope. **2026-09-16: `controller=8`
      crashes in CS2 closed-loop sim** (not flyable yet, root cause partly fixed) — does not
      change the writing-track claim (numerical verification against the reference stands), but
      flight-derived results for Strategy 4 remain blocked on that being resolved.
