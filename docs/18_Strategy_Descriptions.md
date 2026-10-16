# Strategy Descriptions — the seven methods compared

**Track:** Writing (W.3). **Status:** first draft, 23 August 2026.
Feeds Chapter 4 (Control Architectures).

> ## ⚠️ How to read this document
>
> Every strategy is described in **two strictly separated parts**:
>
> - **What we implement** — verifiable against our own source (`firmware_app/src/lib.rs`,
>   `residual_nn.rs`). High confidence. This is what will actually fly.
> - **Reference method** — the published work the strategy follows, with a
>   **verification status** from [`17_Source_Ledger_and_Citation_Discipline.md`](17_Source_Ledger_and_Citation_Discipline.md)
>   and the deviations stated explicitly.
>
> **These are never merged into one sentence.** Our implementation is not the paper's method, and
> claiming otherwise would misrepresent both. Where the reference is 🔴 MATRIX ONLY, the description
> deliberately says very little — that is a gap to close by reading, not by writing more
> confidently.

---

## 0. What separates the strategies

All seven track the same trajectories with the same airframe. They differ in **three questions**
about the interaction force \(f_{\mathrm{res}}\) (notation: [`04`](04_Unified_Residual_Wrench_Model.md)):

1. **What does the controller know about \(f_{\mathrm{res}}\)?** Nothing, a measurement, a
   prediction, or both.
2. **When does it know it?** After the disturbance has acted (measurement), or before (prediction).
3. **Where does the knowledge enter?** The desired force, the linearising law, the MPC prediction
   model, or directly as an actuation correction.

| # | Strategy | Knows | When | Enters at | Needs rotor speed | Needs training data |
|---|---|---|---|---|---|---|
| 0 | *Geometric baseline* | nothing | — | — | no | no |
| 1 | Pure INDI | measurement | after | desired force / torque | **yes** | no |
| 2 | Geometric + NN | prediction | before | desired force | no | **yes** |
| 3 | FBL + NN | prediction | before | linearising law | no | **yes** |
| 4 | Hybrid NA-INDI¹ | both | both | desired force | **yes** | **yes** |
| 5 | Residual RL | neither² | — | actuation correction | no | **yes (policy)** |
| 6 | Learning-based MPC | prediction | before, over a horizon | prediction model | no | **yes** |
| 7 | Geometric + Residual RL | neither² | — | actuation correction | no | **yes (policy)** |

¹ Strategy 4 subtracts the prediction and then the *difference* between measurement and
prediction — not both terms independently. See its section below.

² Strategies 5 and 7 never form an explicit force estimate. They learn the corrective *action*
directly, which is why they have no entry in the "knows" column and no interpretable residual to
log — a real cost when attributing failures.

**Strategy 0 is not in the seven** but is flown as the uncompensated reference. Without it, "how
much did compensation help" has no denominator.

---

## Strategy 1 — Pure INDI *(reactive)*

### What we implement
Incremental nonlinear dynamic inversion in both loops, selected by `indi_gains.ctrl_mode`
(1 = position INDI, 2 = attitude INDI, 3 = both). The residual is reconstructed by comparing
measured acceleration against the acceleration the nominal model predicts for the actuation
actually applied:

$$
a_{\mathrm{res}} = a_{\mathrm{meas}} - a_{\mathrm{model}}, \qquad
a_{\mathrm{model}} = \frac{f_{\mathrm{total}}}{m}\,b_3 - g e_3 ,
$$

with \(f_{\mathrm{total}}\) obtained from measured per-motor rotor speeds via \(k_t \omega^2\).
It enters the position loop as \(-a_{\mathrm{res}}\) in the desired-acceleration vector, and the
attitude loop as an incremental torque about the current command. Logged as `indi.a_res_*` in
**every** mode, including geometric.

**Known properties of our implementation.** It requires a rotor-speed source; without one the
reconstruction falls back to the previous torque command and the method degrades silently. The
estimate is delayed by the filter chain. It needs no training data and no prior knowledge of the
interaction, which makes it the natural baseline for *unmodelled* disturbances.

### Reference method
**Primary: Tal & Karaman (2018)** — 🟢 verified. The incremental formulation and the
flatness-derived feedforward follow that work. **Secondary: Smeur et al. (2017)** — 🟢 verified —
for the cascaded arrangement of position and attitude INDI.

**Deviations to state in Chapter 4:** our gains, filter cutoffs and the airframe differ; our
implementation runs out-of-tree on a Crazyflie brushless at 500 Hz. Any claim that our INDI
reproduces published INDI performance would need a like-for-like comparison we are not running.

### 2026-09-15: a second controller option for this strategy — `stabilizer.controller=7`
A faithful Rust port of Cobo-Briesewitz's own `controller_lee.c` (Strategy 4's reference paper,
see below), numerically verified to ~1e-9 against their compiled C
(`firmware_app/host/test_naindi_reference.py`). Their file's `use_nn` flag — the part that would
make it NA-INDI rather than plain INDI — is dead code in every configuration either project has
flown, so as it exists today `controller=7` runs their **plain INDI**. Their own paper's
comparison table treats INDI as one of its four baseline methods (Lee / INDI / LINDI / NA-INDI),
so this is a legitimate second implementation of Strategy 1, not the hybrid.

**2026-09-16: crashes in CS2 closed-loop sim** on a plain single-drone hover (root cause partly
fixed, see Strategy 4 below) — not flyable as-is, do not treat as a fallback for `controller=6`.

---

## Strategy 2 — Geometric + NN residual *(predictive)*

### What we implement
Geometric SE(3) control with a learned residual added as feedforward. The network
(`residual_nn.rs`) is now a byte-faithful port of Neural-Swarm2's own published architecture,
not a custom-sized approximation:

$$
\mathrm{Fa}_z = \rho_S\!\Big(\phi_G(\text{ground term}) + \textstyle\sum_j \phi_{S}(\,p_j - p_i,\; v_j - v_i\,)\Big),
$$

with `phi_Net: input_dim -> 25 -> 40 -> 40 -> H=20` (ReLU/ReLU/ReLU/linear) and
`rho_Net: 20 -> 40 -> 40 -> 40 -> 1` (same pattern), ported directly from the authors' own
reference code (`crazyswarm2/crazyflie_sim/crazyflie_sim/backend/neuralswarm.py`), evaluated
onboard each control tick. Five weight sets exist (`phi_S`/`rho_S`, `phi_L`/`rho_L`, `phi_G`) —
our fleet is Crazyflies only, so `phi_L`/`rho_L` are present but never selected. The prediction
is only a **Z-axis** force (the reference's own `Fa = (0,0,faz)`, not a 3D residual), converted
from an equivalent-grams unit to an acceleration via this project's own mass before being
subtracted from the desired-acceleration vector, gated by `rnn.en`.

> **STATUS 2026-09-14 (end of day): architecture port DONE, RAM RESOLVED, test suites
> REWRITTEN AND PASSING, training from logs STILL BLOCKED.**
>
> The exact architecture is 19297 weights (~77 KB), up from the old network's 987 (~4 KB).
> **Operator's explicit, standing instruction: the network must not be modified — it stays
> exactly as ported, "out of the box."** That instruction was honoured: the RAM overflow was
> fixed by a `residual_nn` Cargo feature (default **off**) that gates only the `static mut RNN`
> allocation, leaving `residual_nn.rs` byte-identical. All three platforms link clean with the
> feature off; host builds that need to evaluate the network pass `--features residual_nn`.
>
> | Piece | State |
> |---|---|
> | `residual_nn.rs` | ✅ ported, untouched since |
> | RAM | ✅ resolved via the feature gate, network not modified |
> | `host/test_residual_nn.py` | ✅ rewritten, **19 checks passing** |
> | `tools/residual/model.py` (the firmware contract) | ✅ rewritten, agrees with the compiled firmware to ~1e-6 m/s² |
> | `tools/residual/test_pipeline.py` | ✅ rewritten, **13 checks passing** |
> | `tools/residual/dataset.py` | ⛔ stale, **raises on import** |
> | `tools/residual/train.py` | ⛔ blocked behind `dataset.py` |
>
> **Consequence: weights that train will behave on the drone exactly as in training — but
> nothing can be trained from flight logs yet.** `dataset.py` needs four decisions (see
> `docs/13` §6), the two load-bearing ones being that the architecture predicts **vertical
> residual only**, and that `--z-floor` now contradicts a network that models ground effect
> explicitly. Not on the critical path until C.1 produces logs; blocks C.2.

**Known properties.** No rotor-speed sensing required. Acts *before* the disturbance perturbs the
vehicle. Valid only over the relative configurations the training data covered — the basis of
**RQ2**. The prediction is computed and logged whether or not it is used, so predicted-versus-
measured residual is available on every flight.

> **Scoping consequence that belongs in the results, not in a footnote.** `compute_Fa` returns a
> scalar: `Fa = (0, 0, faz)`. **Strategy 2 can only ever predict and compensate the *vertical*
> residual.** Strategy 1 (INDI) measures all three axes. So the comparison between them is not
> like-for-like on x/y by construction, and any lateral residual is, for this strategy,
> uncompensated by design rather than by failure. A second property in the same vein: ground
> effect is *part of the model* (`phi_G`, evaluated unconditionally), so Strategy 2 is
> compensating downwash **and** ground effect together, which the other strategies are not.

### Reference method
Neural-Swarm / Neural-Swarm2 (Shi et al.; Shi, Hönig et al.) — 🟢 **read at claim level**
(`docs/papers/summaries/shi2022neuralswarm2.md`; citation verified in
`docs/thesis/CITATION_BASELINE.md`).

> ✅ **RESOLVED 2026-09-14**: this section previously described the network as "inspired by"
> the deep-sets formulation, with our own sizes. That is no longer accurate — `residual_nn.rs`
> is now a direct, exact port of Neural-Swarm2's own `phi_Net`/`rho_Net` from their vendored
> reference source (`neuralswarm.py`, itself copied from the authors' `aerorobotics/
> neural-swarm` repo), not an approximation "inspired by" it. One correction to earlier notes:
> there is no spectral normalization visible anywhere in the vendored inference-only file —
> that claim was carried over from an earlier, unverified summary and should not be repeated;
> if the original training used it, it is a training-time detail invisible at inference (a
> spectral-normalized `nn.Linear` is just a regular `nn.Linear` once weights are loaded). Their
> pretrained weights are not directly usable regardless (different airframe/data) — training on
> our own flight data, with their exact architecture, is still required and not yet done.

---

## Strategy 3 — FBL + NN residual *(predictive)*

### What we implement
**Nothing yet.** The feedback-linearising controller code was requested via the professor and is
still with the authors. This strategy is not blocking the others.

### Reference method
Hsieh et al., *Flatness-Preserving Residual* (2026) — 🔴 **matrix only**.

The distinguishing idea, as far as our current notes support: the residual correction is
constructed so that the differential-flatness structure the feedforward relies on is **preserved**
rather than destroyed by the correction. The contrast with Strategy 2 is where the prediction
enters — the linearising law rather than the desired force.

> ⚠️ Do not confuse this with Hsieh et al. 2025 (*L1 KNODE-DW MPC*), which is a different paper
> underpinning Strategy 6. Always cite with year **and** short title.

---

## Strategy 4 — Hybrid Neural-Augmented INDI *(hybrid)*

### Form adopted — decided 2026-08-23
Follows the **reference arrangement**: the network predicts the bulk of the residual and the
incremental term corrects only the remainder,

```
a_des = … − â_res − (a_res − â_res)
```

**not** `− a_res − â_res`. Subtracting both independently double-counts whatever the two capture in
common, so a network that predicted *well* would over-compensate — the strategy would measure an
artefact of the arrangement rather than the value of combining the two sources. The adopted form
degrades gracefully at both limits: perfect prediction → reduces to Strategy 2; no prediction →
reduces exactly to Strategy 1.

**The cost, stated:** the incremental term absorbs whatever the network misses, so a poor
prediction is partly masked and the hybrid's tracking performance alone cannot reveal model
quality. That is assessed separately, from `rnn.pred_*` against `indi.a_res_*` on a flight where
`rnn.en = 0`.

### What we implement
2026-09-15 clarification: `stabilizer.controller=7` (see Strategy 1) is a faithful port of the
reference file this strategy is named for, but its `use_nn` flag — the part that makes it NA-INDI
rather than plain INDI — is dead code there, with no trained residual model in the reference's
convention behind it. Genuine NA-INDI behaviour (network predicts the bulk of the residual, INDI
corrects only the remainder, per the differencing form above) needs `use_nn` enabled and their
real trained network wired in, in a separate, isolated module — `controller=8`.

**2026-09-16: `controller=8` is now built and numerically verified vs the reference's own
compiled C to ~1e-9 — but crashes in CS2 closed-loop sim** on a plain single-drone hover, along
with `controller=7`. One shared root cause fixed (hardcoded reference-airframe constants); a
second (reference attitude gains vs this project's real inertia) remains open. **Not flyable, not
a fallback, not ready for C.1 data collection with this strategy.** See
`docs/strategy_controller_map.html`.

### Reference method
Cobo-Briesewitz, Wahba & Hönig, *Learned Incremental Nonlinear Dynamic Inversion for Quadrotors
with and without Slung Payloads*, L4DC 2026 (arXiv:2503.09441) — 🟢 **verified against full text**,
`docs/papers/cobo_briesewitz_learned_indi_2025.pdf`.

- Two methods: **LINDI**, where a network replaces INDI's residual estimate entirely, and
  **NA-INDI**, the hybrid, where the network predicts the bulk of the residual and INDI corrects
  the remainder.
- Evaluated on a single Crazyflie 2.1, with and without a slung payload; figure-8 and circle.
- **The paper's headline claim is sensor elimination**: LINDI matches INDI *without rotor-speed
  measurement*. Compensation of any kind reduced tracking error by ~40 % vs the geometric baseline.
- **The hybrid's margin is small.** Without payload (figure-8): Lee 7.07, INDI 4.28, LINDI 4.41,
  NA-INDI 4.14 cm — the paper writes "best performance, although with a small margin". With
  payload: NA-INDI "remains similar in performance" to INDI (5.39 vs 5.38 cm figure-8).

**The difference that motivates our RQ4.** There, the residual is **self-induced** — blade
flapping, drag, payload swing — and predicted from the vehicle's *own* state. Here, it is caused
by **other vehicles** and predicted from *relative* state. Different input, different magnitude,
and a different failure mode when a neighbour moves unexpectedly. That is why the multi-robot case
is an open question rather than a foregone conclusion.

> ⚠️ **Two corrections on record for this one paper.** The original `03` said "benefits were
> limited"; a first correction (from our own summary) overshot to "achieved the lowest tracking
> error"; the full text shows the truth is in between — best in one condition by a small margin,
> similar in another, with sensor elimination as the actual headline. See
> [`17`](17_Source_Ledger_and_Citation_Discipline.md) §1–2. Our `paper_summaries.md` entry is
> **unreliable for this paper** and is annotated accordingly.

---

## Strategy 5 — Residual RL *(learned action, deferred)*

### What we implement
**Nothing, deliberately.** Scheduled last by operator decision.

### Reference method
ProxFly (Zhang et al.) — 🔴 **matrix only**. Residual reinforcement learning on top of a cascaded
controller, producing corrections to thrust and body rates.

**Why it is scheduled last, stated honestly:** it answers a different question. Strategies 1–4 and
6 all reason about an explicit force; a residual RL policy learns a corrective action and never
forms a force estimate. That removes the interpretability the rest of the comparison depends on —
there is no predicted residual to compare against the measured one — and it introduces a
sim-to-real transfer dependence the other strategies do not have.

---

## Strategy 6 — Learning-based MPC *(predictive, over a horizon)*

### What we implement
**Nothing yet.** Feasibility on the Crazyflie brushless is an open question: MPC is the only
strategy here whose compute cost may not fit onboard, which makes it directly relevant to **RQ4**.

### Reference method
Chee et al., *KNODE-DW MPC* (2024) — 🔴 matrix only — and Hsieh et al., *L1 KNODE-DW MPC* (2025) —
🔴 matrix only.

The distinguishing idea: the learned residual enters the **prediction model inside the MPC
horizon**, rather than as a correction applied at the current instant. In principle this lets the
controller plan against a disturbance it expects to encounter, rather than only cancelling the one
it expects right now.

---

## Strategy 7 — Geometric + Residual RL *(learned action, deferred)*

### What we implement
**Nothing, deliberately.** Scheduled last with Strategy 5.

### Reference method
Combines the geometric baseline of Strategy 2 with the residual-RL stance of Strategy 5. No single
paper; the pairing is ours, and Chapter 4 must say so rather than imply a source.

---

## Summary — where each strategy's knowledge enters the control law

Schematically, with the desired-acceleration vector of the position loop:

$$
a_{\mathrm{des}} = a_{\mathrm{ff}} + K_p e_p + K_v e_v + g e_3
\;\underbrace{-\; a_{\mathrm{res}}}_{\text{S1, S4: measured}}
\;\underbrace{-\; \hat{a}_{\mathrm{res}}}_{\text{S2, S4: predicted}}
$$

- **S3** injects at the linearising law instead, not this vector.
- **S6** injects into the MPC's prediction model; there is no single instant at which it "enters".
- **S5, S7** bypass this level entirely and correct the actuation.

This is the sense in which the comparison is fair: strategies 1, 2 and 4 differ **only** in which
of the two terms above is active, on the same controller, the same airframe and the same frozen
gains.

---

## Open items for W.3

- [ ] Promote every 🔴 to 🟢 by reading at claim level (see [`17`](17_Source_Ledger_and_Citation_Discipline.md))
- [x] ~~Decide the Strategy 4 form~~ — the differencing form (INDI correcting only the network's
      mismatch) was already decided 2026-08-23, see above. What's still open (2026-09-15): this
      requires a NEW `controller=8` (flag + trained model inside the `controller=7` module), not
      yet built or scoped. Affects C.3, not C.0/C.1
- [ ] Establish whether Strategy 6 is computationally feasible onboard, or must be descoped
- [ ] Confirm the geometric baseline (Strategy 0) is included in the campaign as a flown condition
