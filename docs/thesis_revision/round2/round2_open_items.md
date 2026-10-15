# Round 2 — open items

Line numbers refer to `round2/revised/`. Decisions resolved by the author for round 2 (D-01, D-02, D-04, D-06, D-11)
are applied and their comments removed. Still open: D-03 and D-05 (as stated), D-07–D-10 (carried over from round 1,
unchanged), and two new rows found in this round, **D-12** (the hybrid) and **D-13** (two vehicles versus the
three-vehicle content).

---

## A. Decisions

### D-12 (new) — How the measured and the predicted term combine in Strategy 3  ⚠️ read first

**Finding.** The prompt asked Chapter 4 to say that "the measured residual falls as the prediction takes effect, because
the measurement is of the remaining acceleration". For this controller that is not what the code does. The measured
residual is formed from the **measured** rotor speeds (`controller_omar_indi.c` L179–196, L250–262), so it equals the
residual force at the current state whatever the controller commands (Ch3 §3.3.1, fourth paragraph; Shi et al. 2020
§III-C). A prediction that raises the thrust raises $a_{\mathrm{rot}}$ and $a_{\mathrm{meas}}$ alike. The two terms
therefore **add**. In increment form (Ch4 eq. ctrl-s3-increment), once the thrust has settled,
$a_{\mathrm{meas},f} = a_{\mathrm{des}} - \hat a_{\mathrm{res}}$: for the INDI loop the prediction is an offset of its
acceleration reference. The position loop answers that offset with a position error, and the shared vertical integral
removes it at steady state. Strategy 3 can differ from Strategy 1 mainly where the prediction leads the filtered
measurement in time. Chapter 4 §ctrl-s3 states this derivation, as a modelling statement, with a `% DECISION D-12`
comment at L505.

**Options.**
- (a) Fly the hybrid as implemented and keep the Chapter 4 text, which describes the code. The comparison then tests
  "INDI with a prediction-shaped reference offset".
- (b) Change the controller before tier 1 so that the measured term reasons about the remaining mismatch, as in NA-INDI
  ("the INDI control law only needs to reason about the remaining mismatch", Cobo-Briesewitz et al. PDF §4.4). One
  formulation keeps the prediction's lead but removes the steady double counting: subtract only the part of the
  prediction that the filtered measurement has not yet seen,
  $a_{\mathrm{cmd}} = a_d - a_{\mathrm{res},f} - (\hat a_{\mathrm{res}} - \hat a_{\mathrm{res},f})$, with
  $\hat a_{\mathrm{res},f}$ the prediction passed through the same 30 Hz filter. This is a firmware change and needs the
  lab; the Ch4 text would then be rewritten (derivation: steady state $\hat a - \hat a_f \to 0$).
- (c) Drop Strategy 3 from the comparison again.

**Recommendation:** decide with the supervisor **before** the tree validation flights. Option (a) is honest but tests
something narrower than "a hybrid"; (b) is closer to the published hybrid. The text needs no change for (a); for (b)
the paste text is the formula above plus one paragraph.

### D-13 (new) — Two vehicles in the protocol versus three-vehicle content in the thesis

**Finding.** The protocol of the final study (draft v0.2, §1, and the 2026-10-14 banner) flies **two vehicles only**
and excludes B1–B3, C1–C3, C5, A6 and C4. The thesis still relies on three-vehicle flights in several places:
- the null scenarios C1–C3, which set the measurement floor (Ch3 Table 3.1; Ch5 §5.3.1, Table 5.2, §5.5.5, §5.8;
  Ch6 "negative answer" paragraphs);
- RQ3 and contribution 2 (Ch1), and the multi-robot results chapter (Ch7);
- the scope table, "two and three vehicles" (Ch1 Table 1.1);
- the 16-scenario library table (Ch5 Table 5.1).

`% DECISION D-13` comments: Ch1 L205, L300; Ch5 L181, L412, L601; Ch7 L30.

**Options.**
- (a) Narrow the thesis to two vehicles: mark B and C scenarios as defined but not flown; define a two-vehicle null
  condition, for example the protocol's baseline flights or a two-vehicle offset beyond the network gate; keep RQ3 only
  as the documented limitation of Ch7 §7.4. Removing RQ3 entirely would change the research questions (H7) and needs the
  supervisor.
- (b) Keep the three-vehicle content as a later tier, if a third vehicle and time allow.
- (c) Leave as is. Not recommended: Ch5 would describe a floor the study does not measure.

**Recommendation:** (a), with the supervisor, because RQ3 is affected.

### D-03 — Gap statement (open; proposal rewritten for the hybrid in scope)

The text is unchanged at Ch1 L102, L117, L230, L274 and Ch2 L631, L683, L714, L751. The scope sentences next to the gap
("the hybrid is outside its scope") were changed under author decision 2. The new recommended wording is:

> To the best of our knowledge, no study has compared rotor-speed-based reactive compensation, learned per-neighbour
> predictive compensation and their combination as switch settings of one onboard controller on one airframe, across a
> fixed library of interaction regimes, with one definition of the residual for every flight. The closest study, Hsieh
> et al. (2025), compares an online $\mathcal L_1$ estimate, a learned downwash model and their combination inside one
> off-board model predictive controller on three vehicles in two stacked formations; it finds the combination best and
> the online estimate alone ahead of the learned model alone for the middle vehicle.

Add the two-to-three-vehicle clause only if D-13 keeps three-vehicle flights. Contribution 1 (Ch1 L230):
`To the best of our knowledge, the first evaluation of rotor-speed-based reactive, learned per-neighbour predictive and
hybrid compensation as switch settings of one onboard controller, on the same airframe, trajectories and formations …`.
The rest of the paste-ready text in `../author_decisions.md` (D-03) still applies.

### D-05 — Campaign-status passages (open; wording conservative)

**Places:**
- Ch4 L46: the Table 4.1 "Implemented" column (neutral).
- Ch4 L534: the status of Strategy 4 (neutral; the "requested from the authors" sentence is gone because S4 is now
  described by the author's decision 1).
- Ch5 L37: the campaign-status paragraph. It describes flights of the **retired** controllers. Nicknames, file names and
  internal labels were removed under author decision 3; the content is unchanged. Recommendation: delete it, since
  nothing has been flown with the final controllers.
- Ch5 L302: "to be confirmed against real logs on the first two-vehicle flight".
- Ch6 L39: the §6.1 paragraphs; names removed, content unchanged.
- Ch6 L228: the Strategy 4 row.
- Ch7 L33: the "roster" sentence.
- Ch8 L169.
- Ch9 L73 and L77: the future-work list; names removed.

**Applied rather than left:** the Ch4 summary sentence "At the time of writing, two strategies fly (the baseline and
pure INDI) …" was removed, because after renumbering it had become false. Nothing has been flown with the final
controllers, and "pure INDI" named the retired implementation. If a status sentence is wanted, use
`Strategies~0--3 are implemented in the flight software; Strategy~4 is not.`

### D-07 – D-10 (carried over from round 1, unchanged)
- D-07: H1 removals, confirm.
- D-08: Abro paragraph (no PDF).
- D-09: hypotheses and statistics. The `\todo`s in Ch5 §5.5.4/§5.5.5 remain; the number of repetitions is now fixed by
  the protocol, 5 clean flights per cell.
- D-10: bibliography corrections (`../followup_bib_corrections.md`).

---

## B. [VERIFY] items

| # | Place | What to verify |
|---|---|---|
| V1 | Ch2 L106; Ch4 L312 | Lee et al. CDC 2010: the 90°/180° stability regions and the proposition numbering (CDC PDF not held) |
| V2 | Ch2 L669–670 | the Abro et al. paragraph (no PDF) |
| V10 (new) | Ch4 L300 (Table 4.2 inertia; S0 deviation) | The inertia in the controller comes from an identification of the **Crazyflie 2.0** (code comment: BA thesis, J. Förster, ETH Zürich; `controller_omar_indi.c` L59–62), not of the Crazyflie 2.1 Brushless flown here. The text states this as a deviation. Decide whether a brushless identification (e.g. Busetto et al., arXiv:2512.14450, named in the retired code at `lib.rs:230–231`, V9) should replace it in the controller; that is a firmware change. |
| V11 (new) | Ch3 L179; Ch4 L447 | Rotor-speed source of the **offline** labels: the optical sensor, as switch A uses, or the motor controllers' telemetry. `docs/state_2026-10-14_night.md` lists it as open. |
| V12 (new) | Ch4 Table 4.2 (mass) vs Ch5 §5.1 | The controller's mass parameter is 42.7 g (build configuration), while Ch5 states a vehicle mass of about 41 g (configured mass of the retired controller, "measured with prop guards"). Confirm the take-off mass of the vehicles as flown. A thrust-model mismatch appears as a constant residual (Ch3 §3.3.1), and the shared vertical integral absorbs it in flight. |
| V7 | Ch5 L525 | Supervisor: whether the simulator's interaction-model weights are those of the Neural-Swarm2 paper |

---

## C. `% CONFIRM-AFTER-LAB` places

| Tag | Places | Conservative sentence in the text | What the lab must settle |
|---|---|---|---|
| (a) take-off and landing of the INDI strategies | Ch4 L248 (runtime selection); Ch5 L327 (hardware gate) | "How take-off and landing are flown for the strategies that use switches A and B is fixed at the hardware gate; where it differs from the flight phase compared, the analysis windows exclude it." | Whether S1/S3 take off with switches A and B set (the current yaml, `tree_S1/S3.patch`, pins the controller with `indi 3` for the whole flight) or with the geometric law and the switches set at altitude. **Note:** the residual filters run only while switch A or B is set (`controller_omar_indi.c` L231–262, L365–410). Setting the switches in flight therefore starts the filters from their last state, which gives a transient. |
| (b) training-flight counts and retrained model | Ch4 L441; Ch5 L359 | Training flights are described by design only (grid plus randomised waypoint flights); no counts. | The number of flights flown and every figure about the retrained model, after the training flights. |
| (c) execution time of the learned model | Ch4 L422; Ch5 L336 | "Its execution time on the vehicle is measured at the hardware gate." | The measured time per evaluation (recorded by the flight software). |

---

## D. Fact-sheet corrections and clarifications (the code wins)

| # | Fact sheet | Code | Effect on the text |
|---|---|---|---|
| F1 | "the measured residual falls as the prediction takes effect" (WHAT TO DO 2) | the measured residual is independent of the commanded thrust; the terms add (see D-12) | Ch4 §ctrl-s3 states the code's behaviour; D-12 |
| F2 | $a_{imu}$ is "the estimator's world-frame acceleration (gravity removed)" | it is the accelerometer reading rotated into the world frame with the estimator's attitude, minus gravity (`kalman_core.c:790–797`), not a filtered estimator state | Ch4 S1 wording states exactly that |
| F3 | attitude integral "ACTIVE" | active, **without a limit**; reset only on the ground (`controllerOmarIndiReset`) | stated in the code comment; not in the text |
| F4 | vertical integral "with a limit" | its state is clamped to ±2 m·s (`Kpos_I_limit` default 2); it is reset on the ground and outside position mode | "its state is limited" in Ch4 |
| F5 | NN evaluated "every 10th main-loop tick" | yes: `tick % rnn.div` with the main-loop tick, called from the 500 Hz body, so every fifth controller step | Ch3/Ch4 state both |
| F6 | inertia values | from a Crazyflie 2.0 identification (V10) | stated as a deviation in Ch4 §S0 |
| F7 | logging: "96 variables" | still 96; the residual and attitude-error channels of the retired controller (`indi.*`) are in the configuration but not computed under the final controller | Ch5 recorded list rewritten; open: drop or keep those channels in the logging configuration |
| F8 | counts "4 strategies × 12 settings = 48 cells, 240" | consistent with the protocol banner and `study_shakedown_card.md`. Protocol §3 body still says 12 × 3 = 36 (pre-banner) | Ch5 uses 48/240; update protocol §3 when re-freezing |
| F9 | top vehicle always S0 | yes (`tree_S*.patch`: top `indi 0`, `rnn.en 0`) | Ch4 runtime selection |
| F10 | — | the rotor-speed deck dropping single motors has been a past hardware issue (project notes); with the deck as the only control source, a dropped reading corrupts switches A/B and the labels | Ch3 assumption-table row "a rotor reading missing"; check every session |
