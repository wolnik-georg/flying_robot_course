# Stage 3 — Revision plan (no edits; every row APPLY or DECISION)

**Classification rule (from the run prompt).** APPLY = an error the code or a paper unambiguously settles; a missing
definition, derivation, example, structure, signposting or consistency fix; a reference-context fix. DECISION =
the residual-sign wording, which integral terms the thesis describes, what to do with lab-status passages, whether to
add or drop a related-work item, anything that changes scope or an argument. Code-settled design facts (network
architecture, size, gate, output, rate; logged-variable count; loop rate) are APPLY with `% CHANGED: code, <file:fn>`.

**Two additional rules adopted for this run (stated so the author can reverse them):**
1. **H1 statements** (project simulation or lab findings) cannot stay in the compiled text without failing H1. They
   are APPLY: the printed sentence is replaced by a design-level statement and the **original is kept verbatim as a
   LaTeX comment** (`% REMOVED (H1): …`) so nothing the author wrote is lost.
2. **H9 statements** (lab status) are DECISION as the prompt requires: text unchanged, `% DECISION D-05` comment with
   the three options.

Severity S1/S2/S3 as in Stage 0. Risk: **L** cannot change a claim or an equation; **M** changes wording of a claim;
**H** changes an equation, a claim about our implementation, or the argument.

---

## 1. Target outline (changes marked ✚ new, ✎ changed, ✂ removed)

**Ch1 Introduction** — 1.1 Motivation ✎ (sources for nonlinearity/asymmetry) · 1.2 Problem ✎ (wording) · 1.3 Gap ✎
(factual correction on the hybrid; gap sentence DECISION D-03) · 1.4 Compared strategies ✎ · 1.5 RQs (unchanged) ·
1.6 Contributions ✎ (apparatus wording) · 1.7 Scope (table referenced ✎) · 1.8 Outline ✎ (prose names for Ch6–9).

**Ch2 Background and related work** — 2.1 Dynamics and baseline control ✎ (+ ✚ "Rotations, SE(3) and the attitude
error" box; + flatness flat outputs) · 2.2 Aerodynamic interaction ✎ (+ sources, + magnitudes) · 2.3 Reactive ✎
(+ ✚ "How INDI obtains the residual" intuition; + limitations evidence) · 2.4 Predictive ✎ (context fixes) · 2.4¶
Superposition ✎ · 2.5 Injecting ✎ (+ Hsieh 2026 SE(3) result; Hsieh 2025 corrected) · 2.6 RL (unchanged) · 2.7 Hybrid ✎
(team instance added) · 2.8 Evaluation ✎ (+ ✚ "How the field evaluates" paragraph; Tab 2.1 rows fixed) · 2.9 Summary
and gap ✎ (second gap corrected; first gap DECISION D-03).

**Ch3 System modelling** — 3.1 Notation ✎ (+ symbols, units, hat accent conflict resolved) · 3.2 Rigid body ✎ · 3.3
Measuring ✎ (+ ✚ worked 1-D example; filters; bias) · 3.3.1 Requirements/failures ✎ · 3.3.2 Sign ✎ (+ derivation step;
sign wording DECISION D-01) · 3.4 Predicting ✎ (input set incl. own height/velocity) · 3.4.1 Deep sets ✎ (**network
corrected to the code**; + ✚ toy example) · 3.4.2 Superposition ✎ · 3.4.3 Input conditioning ✎ (**gate corrected**;
ground branch; output bound) · 3.4.4 Normalisation ✎ (two first layers) · 3.5 Assumptions ✎ (table rows) · 3.6 Summary ✎.

**Ch4 Control architectures** — intro + Tab 4.1 ✎ (status column DECISION D-05; rotor-speed column footnote) · 4.1
Position and attitude loop ✎ (+ ✚ frame mapping to Lee; $R_d$ construction; velocity-error conditioning; integral terms
DECISION D-02; sign DECISION D-01) · 4.1.3 Runtime selection ✎ · 4.2 S0 ✎ ($e_\omega$ deviation) · 4.3 S1 ✎ (+ ✚
INDI derivation; reference-side filter; deviations from Tal & Karaman) · 4.4 S2 ✎ (**implemented network corrected**;
deviations rewritten; rationale D-04) · 4.5 S3 ✎ (Hsieh 2026 corrected; status D-05) · 4.6 Fairness ✎ (+ what differs
between strategies by design) · 4.7 Summary ✎ (status D-05).

**Ch5 Experimental setup** — status ¶ (D-05) · 5.1 Platform ✎ (**1 kHz**; rotor-speed sources) · 5.2 Volume (unchanged;
todo kept) · 5.3 Library ✎ (+ default parameters column; regimes corrected; Tab referenced) · 5.3.1 Null ✎ · 5.3.2 Regimes
✎ (dry-run ¶ ✂ H1) · 5.4 Logging ✎ (**96 variables**; recorded list; radio numbers ✂ H1) · 5.4.3 Alignment ✎ (status
D-05) · 5.5 Protocol ✎ (rationale D-04) · 5.5.4 Metrics ✎ (+ ✚ mathematical definitions; $e_R$ is logged) · ✚ 5.5.5
Statistical treatment (skeleton + todo; D-09) · 5.6 Simulation ✎ (findings ✂ H1; model cited) · 5.7 Threats ✎ (internal
vocabulary) · 5.8 Summary ✎.

**Ch6–9 skeletons** — Ch6: 6.1 Campaign overview ✎ (D-05 for status) · 6.2 Tracking and attitude ✎ · 6.3 Effort ✎ ·
6.4 Predictive strategy ✎ (prediction-off vs on) · ✚ 6.5 Residual-sign and loop-attribution checks (promised by Ch3/Ch4)
· ✂ old 6.5 Hybrid · 6.6 Summary. Ch7: 7.1 ✎ · 7.2 ✎ (penalty metric) · 7.3 ✎ (future-work framing) · 7.4 ✎ · 7.5.
Ch8: 8.1–8.4 ✎ (what a negative answer looks like) · ✚ 8.5 Assumptions revisited (Tab 3.1 row by row) · ✚ 8.6 Where a
frozen gain may have disadvantaged a strategy · ✚ 8.7 The uncompensated moment residual · 8.8 Contributions revisited ·
8.9 Threats revisited. Ch9: 9.1–9.5 ✎ (RQ-mapped; future work D-05 for internal vocabulary).

---

## 2. Change list per chapter

### Chapter 3 (rewrite first)

| ID | Location | Category | Reason / evidence | Sev | Risk | Class | Dep. |
|---|---|---|---|---|---|---|---|
| P3-01 | §3.4.1 ¶"What is borrowed" + ¶"concrete network" (L217–229) | error (design fact) | Code is the NS2 architecture: $\phi_S$ 6→25→40→40→20, $\phi_G$ 4→25→40→40→20, $\rho_S$ 20→40→40→40→1, ReLU, linear outputs; 19 297 stored weights (11 461 on the evaluated path); not "our own sizing", not 987 (`residual_nn.rs:12–30, 65, 91–125`); NS2 §VII-C (R02) | S1 | H | APPLY (code) | — |
| P3-02 | eq. (model-deepsets) + ¶ after it (L204–215) | error (design fact) | output is scalar vertical force (gram units → N → ÷m), clamped; ground term $\phi_G(-z,-v)$ always added; sum over gated neighbours only (`residual_nn.rs:262–309`) | S1 | H | APPLY (code) | P3-01 |
| P3-03 | eq. (model-F) (L189–198) | error / missing | the predictor also uses own height and velocity (ground branch); state as $\mathcal F(\{\cdot\}_{j}, z_i, v_i)$ | S2 | H | APPLY (code) | P3-02 |
| P3-04 | §3.4.3 guards (L265–277) | error (design fact) | no 2 m cut-off, no 4 cm clamp; gate $\lvert\Delta x\rvert,\lvert\Delta y\rvert<0.2$ m ∧ $\lvert\Delta v_x\rvert<1.5$ m/s from the authors' public code (`residual_nn.rs:36–44, 139–140, 281–286`; `crazyswarm2/.../neuralswarm.py:78`); NS2 "cuboid" interaction volume (R02 §III-B); consequence: laterally offset neighbours (>0.2 m) contribute nothing whatever their vertical separation | S1 | H | APPLY (code) | P3-02 |
| P3-05 | §3.4.3 output bound (L279–289) | consistency | bound applies to the scalar vertical output; non-finite output → 0 and flag (`residual_nn.rs:298–307`); Gu citation context correct (R18) | S3 | L | APPLY | P3-02 |
| P3-06 | §3.4.4 normalisation (L297–310) | error (design fact) | folded into the first layers of $\phi_S$ and $\phi_G$ with separate statistics (`tools/residual/model.py:224–241`) | S2 | M | APPLY (code) | P3-01 |
| P3-07 | ¶"These equations are the code" (L231–239) | consistency | keep; footnote adds `traj_iface.c` (rate, defaults) | S3 | L | APPLY | P3-01..06 |
| P3-08 | §3.4.1 "evaluated onboard within the control period" (L227) | error (design fact) | evaluated every 10th control step (≈100 Hz) and held (`traj_iface.c:132`; `lib.rs:884–903`); weights flash-resident in the study build (`Cargo.toml` features; `lib.rs:922–926`) | S1 | M | APPLY (code) | — |
| P3-09 | §3.4 neighbour states | missing | neighbour positions come from broadcast motion-capture poses; relative velocity is finite-differenced onboard (`lib.rs:839–873`); at most three neighbours (`residual_nn.rs:129`) | S2 | L | APPLY (code) | — |
| P3-10 | §3.3 accelerometer sentence (L119–123) | missing definition / clarity | "specific force (… not due to gravity)" muddled; correct statement: accelerometer measures $R^\top(\dot v + ge_3)$ in g units; code rotates, scales and subtracts $ge_3$ (`lib.rs:1790`); accelerometer Butterworth-prefiltered (`lib.rs:545, 2374–2382`); rotor speeds pass a sanity filter (`traj_iface.c:726`); per-motor $k_{t,k}$ in N/RPM² (`traj_iface.c:285–288`) | S2 | M | APPLY (code) | — |
| P3-11 | §3.3 "evaluates at every control step and logs it" (L125) | error (design fact) | computed at every control step (1 kHz) whenever rotor speeds are available; recorded at 500 Hz | S2 | L | APPLY (code) | — |
| P3-12 | §3.3.1 "It is delayed" (L146–149) | missing derivation | matched-filter requirement (Tal & Karaman §III-B; Smeur §2–§3) and the code's mismatch (accelerometer filtered, thrust not, unless `res_fc` set) as a stated property; illustrative group delay of a 2nd-order Butterworth ($\sqrt2/(2\pi f_c)$) labelled textbook | S2 | M | APPLY (code + R13/R14) | — |
| P3-13 | §3.3.1 "It absorbs model error" (L151–155) | missing failure mode | accelerometer bias (Smeur §4.6, R14) | S2 | L | APPLY | — |
| P3-14 | §3.3.1 "It requires rotor-speed sensing" (L137–144) | evidence | Cobo-Briesewitz Table 1 (R07): INDI with PWM-derived thrust worse than uncompensated Lee | S3 | L | APPLY | — |
| P3-15 | §3.3 ✚ worked example | missing example | 1-D illustrative example: hover, thrust model 5 % low ⇒ constant $a_{\mathrm{res}}$ of 0.49 m/s²; with a 35 % thrust loss below a neighbour (Kiran static rig, R11) ⇒ ≈3.4 m/s²; label as illustrative/rig values | S2 | L | APPLY | — |
| P3-16 | §3.3.2 sign derivation (L160–181) | missing derivation step / sign | add the one-line substitution and the analogy with NS/NS2/Tal/Cobo/Hsieh (all subtract, R01, R02, R05, R07, R13, R18); **sign wording itself unchanged** | S2 | M | APPLY (derivation detail) | — |
| P3-17 | §3.3.2 "The residual is subtracted … approximately twice" | **DECISION D-01** | code: `+res_sign·a_res`, default +1, study S1 = +1, S2 = −1 (`lib.rs:1857–1861, 1930–1931`; `traj_iface.c:442`; `crazyflies.yaml:227`; `study/ours.patch:13–14`); the "twice" consequence assumes no integral action on that axis (S0/S2 run a vertical integral) | S1 | H | DECISION | D-02 |
| P3-18 | §3.1 notation | missing definitions / conflict | define vee map; $\varpi_k$ in RPM; $k_t$ as the set $\{k_{t,k}\}$; resolve hat conflict: keep $\hat a$ (predicted) and write the hat map as $[\cdot]_\times$? — **no**: renaming changes symbols across chapters; instead add one sentence "the hat on an acceleration denotes a prediction; on an angular velocity it denotes the hat map" | S2 | L | APPLY | — |
| P3-19 | §3.4.1 ✚ toy example | missing example | two-neighbour order invariance with a scalar $\phi$ and $\rho$ (illustrative) | S3 | L | APPLY | — |
| P3-20 | §3.4.2 superposition | context | add Hsieh 2025's use of pairwise summation citing Gielis (both readings exist) | S3 | M | APPLY (ref context) | — |
| P3-21 | Tab 3.1 | missing rows | relative velocity from finite differences (latency/noise); neighbour gate; accelerometer bias | S2 | L | APPLY | P3-04, P3-09, P3-13 |
| P3-22 | §3.2 "coplanar control scenarios" | consistency | C2 is coplanar but moving; wording "coplanar controls" fine | S3 | L | APPLY | — |
| P3-23 | §3.6 summary | consistency | "Both are expressed as accelerations in the world frame" → the prediction is the vertical component only | S2 | M | APPLY (code) | P3-02 |
| P3-24 | §3.0 intro | consistency | "Strategies 1 and 2 … one number" → $a_{\mathrm{res}}$ is logged in 3-D; the network predicts its vertical component | S3 | L | APPLY | P3-02 |

### Chapter 4

| ID | Location | Category | Reason / evidence | Sev | Risk | Class | Dep. |
|---|---|---|---|---|---|---|---|
| P4-01 | eq. (ctrl-ades) integral term | **DECISION D-02** | vector integral disabled (`lib.rs:525, 1865–1869`); vertical-only integral enabled with per-strategy gain (`lib.rs:535, 1545–1577, 1880`; study: 16 for S0/S2, 0 for S1) | S1 | H | DECISION | — |
| P4-02 | eq. (ctrl-ades) residual signs | **DECISION D-01** | as P3-17 | S1 | H | DECISION | D-01 |
| P4-03 | eq. (ctrl-tau-geo) attitude integral | **DECISION D-02** | `ENABLE_ATTITUDE_INTEGRAL = false` (`lib.rs:540, 2225`) | S1 | H | DECISION | — |
| P4-04 | §4.1.1 after eq. | missing (code fact) | XY velocity error clamped to ±0.6 m/s and low-passed at 10 Hz before $K_v$ (`lib.rs:564–574, 1504–1521`); thrust = projection on current body $z$; $R_{\mathrm{des}}$ construction stated (`lib.rs:1463–1473`) | S2 | L | APPLY (code) | — |
| P4-05 | §4.1 ✚ frame mapping to Lee | missing | Lee uses $e_3$ down, thrust $-fRe_3$, $e_x = x - x_d$ (R15 eqs. 3, 17) | S2 | L | APPLY | — |
| P4-06 | §4.1.2 / §4.2 $e_\omega$ | deviation (code) | $e_\omega = \omega - \omega_d$, no transport $R^\top R_d$ (`lib.rs:2221`); $\omega_d$ from flatness (jerk) or setpoint | S2 | M | APPLY (code) | — |
| P4-07 | §4.1.2 gains units | missing | $K_R^{\mathrm{geo}}$ [N·m/rad], $K_\omega^{\mathrm{geo}}$ [N·m·s/rad] (`traj_iface.c:266–280`) | S3 | L | APPLY (code) | — |
| P4-08 | §4.1.3 runtime selection | consistency | selection also sets per-strategy gains, residual sign, vertical integral gain and network enable; computation of the prediction requires loaded weights (`lib.rs:1752–1756`) | S2 | M | APPLY (code) | D-01, D-02 |
| P4-09 | §4.3 ✚ INDI derivation | missing derivation | from $J\dot\omega = \tau + \tau_{\mathrm{ext}} - \omega\times J\omega$ via estimated $\tau_{\mathrm{ext}}$ (Tal & Karaman eqs. 29–31, R13); 1-D toy | S2 | L | APPLY | — |
| P4-10 | §4.3 eq. (ctrl-indi-tau) + text | deviation (code) | the same Butterworth filters $\alpha_{\mathrm{ref}}$ (`lib.rs:2076–2080`); $\tau_{\mathrm{cur}}$ from rotor speeds via mixer inverse (arm, drag ratio; `lib.rs:1481–1493`), unfiltered by default (`traj_iface.c:342`); effectiveness scale (default 1); optional torque clamp; Tal & Karaman filter $\mu_f$ instead — state deviation | S2 | M | APPLY (code) | — |
| P4-11 | §4.3 filter values | **DECISION D-06** | nominal `fc_bw` vs realised cutoff (design sample time `filt_dt_us` = 2000 µs at a 1 kHz loop, non-pre-warped by default; `traj_iface.c:444–478`); which values the study uses is a configuration fact | S2 | M | DECISION | — |
| P4-12 | §4.3 Deviations ¶ (L237–241) | error / missing | "platform's own rotor-speed measurement" → DShot ESC telemetry (default) or optical deck, with a sanity filter (`traj_iface.c:508–520, 701–730`); position INDI: accelerometer filtered, thrust not (deviation from Tal & Karaman's identical-LPF rule); Tal & Karaman need no integral — relevant to D-02 | S2 | M | APPLY (code + R13) | — |
| P4-13 | §4.4 Implemented ¶ (L252–257) | error (design fact) | ≈100 Hz evaluation; peers from broadcast poses, velocity finite-differenced; weights flash-resident | S1 | M | APPLY (code) | P3-08, P3-09 |
| P4-14 | §4.4 ¶ prediction logged whether enabled or not (L259–264) | **DECISION D-04** (rationale) | "the measured residual is the one that survived compensation" contradicts R01 §III-C / R02 §IV-D for this residual definition | S2 | H | DECISION | — |
| P4-15 | §4.4 Deviations ¶ (L270–278) | error (design fact) | rewrite: NS2 reference architecture port; no spectral normalisation in our training; small-vehicle branch only; vertical output converted to acceleration; 8 m/s² bound; ≤3 neighbours; ≈100 Hz; trained on two-vehicle data under S0 (design); Huber loss on $a_{\mathrm{res},z}$ (`tools/residual/train.py:17, 118`); NS2 attitude law and delay compensation not adopted | S1 | H | APPLY (code) | P3-01 |
| P4-16 | §4.4 / §4.5 Requirements; Tab 4.1 rotor-speed column | error / missing | S2/S3 need rotor speeds to produce training labels (`lib.rs:1781`), not at runtime (cf. LINDI, R07) | S2 | M | APPLY | — |
| P4-17 | §4.2 Role ¶ (L186–192) | **DECISION D-04** | same rationale error | S2 | H | DECISION | — |
| P4-18 | §4.5 Hsieh 2026 ¶ (L300–308) | error (reference context) | C4.10 wrong: hardware vs published; simulation vs NMPC head-to-head; "under 30 s" and "5 ms" abstract-level (R05) | S1 | M | APPLY | — |
| P4-19 | §4.4 NS2 SN sentence (L272–275) | reference context | C4.08 hedge restored (R02) | S2 | M | APPLY | — |
| P4-20 | Tab 4.1 Status column; caption; §4.5 status; §4.7 last ¶ | **DECISION D-05** | H9 | S1 | M | DECISION | — |
| P4-21 | §4.6 fairness ✚ | missing | list what differs by design between strategies (control law, gains, vertical integral gain, residual sign — pointer to D-01/D-02); that the comparison is between implementations | S2 | M | APPLY (structure) — sign/integral content stays behind D-01/D-02 comments | D-01, D-02 |
| P4-22 | header comment (L21–30) | stale scope | comment mentions Strategies 4–7 | S3 | L | APPLY (comment edit; not printed) | — |

### Chapter 5

| ID | Location | Category | Reason / evidence | Sev | Risk | Class | Dep. |
|---|---|---|---|---|---|---|---|
| P5-01 | §5.1 "control loop runs at 500 Hz" | error (design fact) | 1 kHz (`stabilizer.c:303, 350`; `stabilizer_types.h:383`; `lib.rs:2350–2351`) | S1 | L | APPLY (code) | — |
| P5-02 | §5.4.1, §5.8 "35 variables" | error (design fact) | 96 (`usd_study_config.txt`) | S1 | L | APPLY (code) | — |
| P5-03 | §5.4.2 recorded list | error / missing | add quaternion, commanded velocity/acceleration, rotor speeds (telemetry, deck, filtered), $e_R$, battery voltage, external pose input, controller-internal diagnostics | S2 | L | APPLY (code) | — |
| P5-04 | §5.5.4 "$e_R$ not logged; will be added" | error (stale design) | logged (`usd_study_config.txt` lines 56–59; `lib.rs:1975–1976`) | S1 | L | APPLY (code) | — |
| P5-05 | §5.3.2 dry-run ¶ (L174–177) | **H1** | project simulation finding | S1 | M | APPLY (H1 rule 1: comment-out + design statement) | — |
| P5-06 | §5.6 findings (L311–317) | **H1** | "It established … exposed two defects" | S1 | M | APPLY (H1 rule 1) | — |
| P5-07 | §5.4.1 radio numbers (L185–188) | **H1-suspect** | project-measured numbers without source | S2 | M | APPLY (H1 rule 1; keep reasoning; numbers commented + `[VERIFY]`) | — |
| P5-08 | status ¶ (L28–39) | **DECISION D-05** | H9 | S1 | M | DECISION | — |
| P5-09 | §5.4.3 "to be confirmed … on the first two-vehicle flight" | **DECISION D-05** | H9 | S2 | L | DECISION | — |
| P5-10 | §5.5.2 rationale (L255–258) | **DECISION D-04** | as P4-14 | S2 | H | DECISION | — |
| P5-11 | §5.5.4 metrics ✚ definitions | missing | define $e_p$ RMSE per axis over window $W$; max deviation; separation error $e_{\mathrm{sep}}(t) = (p_{\mathrm{top}} - p_{\mathrm{bot}})_z - \Delta z_{\mathrm{cmd}}(t)$, mean and std; attitude $\lVert e_R\rVert$ RMS and peak; remaining residual RMS of $a_{\mathrm{res},z}$; prediction error RMSE of $\hat a_{\mathrm{res},z} - a_{\mathrm{res},z}$ on prediction-off flights; transfer penalty (RQ3) as the difference in prediction RMSE between three- and two-vehicle scenarios for the same model; effort $\frac1{\lvert W\rvert}\int\lVert\tau\rVert^2$; windows defined from the **commanded** relative geometry (from the library, `scenarios.py: Scenario.relative`); `\todo` for the author to confirm windows and the crossing detector | S1 | M | APPLY (definitions; protocol choices flagged by todo) | — |
| P5-12 | ✚ §5.5.5 statistical treatment | missing | repeats $n$, paired within-session comparison, report mean ± spread and effect size; `\todo` with no number; hypotheses link to D-09 | S2 | M | APPLY (skeleton) + D-09 | — |
| P5-13 | Tab 5.1 | missing / reproducibility | add default parameters per scenario and team size (`scenarios.py:134–465`); reference the table in the text | S2 | L | APPLY (code) | — |
| P5-14 | §5.3.2 regimes "C1–C3 are static" | error | C2 moves (0.4 m/s, `scenarios.py:402–413`) | S2 | L | APPLY (code) | — |
| P5-15 | §5.3.2 lateral-excitation box | consistency | still true for $\phi_S$ inputs; add: the 0.2 m lateral gate means coplanar C1 (0.30 m) and wide parts of A5 contribute only the ground term | S2 | M | APPLY (code) | P3-04 |
| P5-16 | §5.6 "published downwash model" | missing citation | Neural-Swarm2 model (R02); weights `[VERIFY]` | S2 | L | APPLY | — |
| P5-17 | Tab 5.2 internal vocabulary (`ctrltarget.*`, "merge tool aborts above 15 cm RMS") | internal vocabulary | rephrase generically; keep 15 cm with `% VERIFY` | S2 | L | APPLY | — |
| P5-18 | §5.1 "9 cm rotor-to-rotor" for the Brushless | `[VERIFY]` | code arm 0.050 m ⇒ 0.10 m diagonal (`lib.rs:261`) | S2 | M | VERIFY comment only (text unchanged) | — |
| P5-19 | §5.4.3 alignment method (cross-correlation against a radio trace) | `[VERIFY]` | the merge procedure is not in the listed code; the author's notes say alignment uses each vehicle's commanded trajectory | S2 | M | VERIFY comment only | — |
| P5-20 | §5.1 rotor-speed source | consistency | DShot telemetry is the default source; deck optional (`traj_iface.c:508–520`) | S3 | L | APPLY (code) | — |
| P5-21 | §5.8 summary | consistency | 1 kHz / 96 / 500 Hz | S2 | L | APPLY | P5-01, P5-02 |
| P5-22 | §5.0 intro + todos | consistency | todo L374 reworded without "C.4" | S3 | L | APPLY (todo text) | — |
| P5-23 | §5.5.3 "Each condition is repeated" | missing | $n$ not specified → `\todo` | S2 | L | APPLY | P5-12 |

### Chapter 2

| ID | Location | Category | Reason / evidence | Sev | Risk | Class |
|---|---|---|---|---|---|---|
| P2-01 | §2.2 ¶1 (C2.06) | missing citation | Kiran (65 %/90 %, lateral recovery), Gielis (concentrated wake) | S2 | M | APPLY |
| P2-02 | §2.2 Kiran/Gielis ¶ (C2.07, C2.08, C2.09) | reference context | PIV plane; Gielis 700 g sufferer; Gielis validates its own models | S2 | M | APPLY |
| P2-03 | §2.3 Smeur (C2.14) | reference context | 4 Hz within the wind-tunnel run | S2 | M | APPLY |
| P2-04 | §2.3 ✚ intuition paragraph | missing explanation | one paragraph: how INDI forms the residual and why matched filters | S2 | L | APPLY |
| P2-05 | §2.3 limitations ✚ | evidence | NS2's online-estimation remark (with caveat), Shankar observer convergence, Gielis "without first observing", Cobo INDI-PWM | S2 | M | APPLY |
| P2-06 | §2.3 terminology ✚ footnote | terminology | Chee's "reactive" usage; $\mathcal L_1$ as reactive estimator | S3 | L | APPLY |
| P2-07 | §2.4 Shi 2020 (C2.20) | reference context | attitude loop open in design; experiments use Lee's | S2 | M | APPLY |
| P2-08 | §2.4 NS2 (C2.29, C2.30) | error / context | 24 cm not in abstract; 15→10 cm small vehicle with upgraded motors | S1 | M | APPLY |
| P2-09 | §2.4 Smith (C2.32, C2.34, C2.36) | reference context | 56 % abstract/contributions only; 55 %/49 % position/velocity; leader at NED $z=-2.5$ m | S2 | M | APPLY |
| P2-10 | §2.4¶ Gielis (C2.41) | overstated | "explore", K = 3 is four in total | S2 | M | APPLY |
| P2-11 | §2.5 FF ¶ ✚ | related work (cited paper) | Hsieh 2026 learned residual in SE(3): −43.7 % RMSE (R05) | S2 | M | APPLY |
| P2-12 | §2.5 Hsieh 2026 ¶ | reference context | intro "identical hardware setup" vs body "reported" | S3 | L | APPLY |
| P2-13 | §2.5 MPC ¶ Hsieh 2025 (C2.55), C2.56 | reference context | it is a measured+predicted hybrid on three vehicles with separate arms; "none runs its optimiser onboard a Crazyflie-class vehicle" | S1 | M | APPLY |
| P2-14 | §2.5 docking (C2.60) | reference context | failure attribution | S3 | L | APPLY |
| P2-15 | §2.7 hybrid ¶3 (C2.70) "the swing of a payload" | error | not named as a residual (R07; ledger D14) | S2 | M | APPLY |
| P2-16 | §2.7 ¶1 "has been done for a single vehicle" + ¶3 "untested on teams" | error | Hsieh 2025 combines measurement and prediction on three vehicles (MPC) | S1 | M | APPLY (minimal factual correction) |
| P2-17 | §2.8 ✚ evaluation-practice paragraph | missing (methodological) | Stage 2 §5 points 1–8 | S2 | M | APPLY |
| P2-18 | §2.8 "Comparisons that do exist" — add Hsieh 2025, ProxFly, Cobo | related work (all cited) | same-rig comparisons; Hsieh 2025 spans the families inside MPC | S1 | H | **DECISION D-03** (the gap consequence) — description of Hsieh 2025 itself applied in P2-13 |
| P2-19 | §2.8/§2.9 gap sentences + provenance todo | argument | D-03 | S1 | H | DECISION |
| P2-20 | §2.9 second gap (L560–564) | error | as P2-16 | S1 | M | APPLY (minimal) + D-03 for restructuring |
| P2-21 | §2.8 misplaced `% UNVERIFIED` comment (L500) | consistency | move to Abro ¶; Gu ¶ verified (R18) | S3 | L | APPLY |
| P2-22 | §2.8 Abro ¶ | `[VERIFY]` | no PDF; `% VERIFY` comment | S2 | L | VERIFY |
| P2-23 | Tab 2.1 rows | context | Hsieh 2025 row; Gielis 700 g; Smith eval; Chee "off-board, Crazyflie"; caption claim check | S2 | M | APPLY |
| P2-24 | §2.1 ✚ box "Rotations, SE(3), attitude error" | background | definitions + Ψ + small example | S2 | L | APPLY |
| P2-25 | §2.1 flatness | background | flat outputs $[p,\psi]$; what fixes thrust/attitude/rates (Hsieh 2026 eqs. 3–11; Tal & Karaman eqs. 9–15) | S2 | L | APPLY |
| P2-26 | §2.4 NS2 SN | background | one-sentence definition of Lipschitz/SN (Shi 2020 eq. 7) | S3 | L | APPLY |
| P2-27 | §2.2 Bauersfeld demo | related work (cited paper) | jet-model feedforward compensation 3–5× at 1–2 m (R12) | S3 | L | APPLY |
| P2-28 | §2.8 recap numbers | consistency | Smith 56 % "abstract" | S3 | L | APPLY |

### Chapter 1

| ID | Location | Category | Reason / evidence | Sev | Risk | Class |
|---|---|---|---|---|---|---|
| P1-01 | §1.1 ¶1, ¶3 | missing citation | destabilisation/nonlinearity/asymmetry → Kiran, Gielis, Shi 2020 (−9 g at 30 cm) | S2 | M | APPLY |
| P1-02 | §1.1 ¶4 numbers | context | 56 % abstract; NS2 heterogeneous | S2 | M | APPLY |
| P1-03 | §1.2 ¶1 "exactly two ways … organises the entire field" | overstated | soften; prediction may be learned or physics-based | S3 | M | APPLY |
| P1-04 | §1.2 ¶3 "unconstrained" | inconsistency | Ch3 bounds the output | S2 | M | APPLY |
| P1-05 | §1.2 ¶5 hybrid "has been built, on a single vehicle" | error | add the team instance inside MPC (Hsieh 2025) | S1 | M | APPLY (minimal) |
| P1-06 | §1.2 ¶6 comparisons list | related work | add Hsieh 2025 | S1 | H | **DECISION D-03** |
| P1-07 | §1.3 italic gap | argument | D-03 | S1 | H | DECISION |
| P1-08 | §1.3 ¶3 narrower gaps (C1.10) | error | "only where self-induced … single vehicle"; "ground forces" | S1 | M | APPLY (minimal) + D-03 |
| P1-09 | §1.4 S2, S3 | consistency | S2 predicts the vertical component; S3 sentence on status → D-05 | S2 | M | APPLY (S2) / DECISION (S3) |
| P1-10 | §1.6 apparatus | error (stale) | flash-resident weights; "verified" → "checked numerically" | S2 | M | APPLY (code) |
| P1-11 | §1.7 Tab 1.1 | style | reference the table | S3 | L | APPLY |
| P1-12 | §1.8 outline \ref to Ch6–9 | H6 | prose names | S3 | L | APPLY |
| P1-13 | §1.5 RQ2/RQ4 | consistency | RQ4 "training data" — link to the data-size question; no change of RQ | S3 | L | — (no change; noted for D-09) |

### Chapters 6–9 (skeletons, last)

| ID | Location | Category | Reason | Class |
|---|---|---|---|---|
| P6-01 | Ch6 §6.1 cf5/cf_second, OptiTrack four-camera, C.1/C.4, "indicative flights exist" | H9 + internal vocabulary | **DECISION D-05** (text unchanged) |
| P6-02 | Ch6 §6.2–6.4 placeholders | structure | RQ-mapped table/figure specs, statistic, what a negative answer looks like; S0–S3 → "Strategy 0–3"; file names removed from todos | APPLY |
| P6-03 | Ch6 §6.5 Hybrid (Strategy 4) | stale scope (strategy > 3) | remove; keep commented | APPLY |
| P6-04 | Ch6 ✚ §6.5 sign and loop attribution | promised by Ch3/Ch4 | APPLY |
| P6-05 | Ch6 ✚ prediction-off vs prediction-on slot | promised by Ch4 | APPLY |
| P6-06 | Ch6 L86–89 "any reference INDI flights included for context" | H5 risk | APPLY: reword to "Strategy 1 only" |
| P7-01 | Ch7 §7.1 "third brushless vehicle in the roster" | H9 | DECISION D-05 |
| P7-02 | Ch7 §7.2 penalty metric; negative answer | structure | APPLY |
| P7-03 | Ch7 §7.4 "simulation sanity checks" as evidence | H1 risk | APPLY: remove from evidence chain |
| P8-01 | Ch8 ✚ sections (assumptions, frozen gains, moment residual) | promised | APPLY |
| P8-02 | Ch8 internal vocabulary (SIL backend, C.1, upload vs flash) | internal vocabulary | APPLY (generic wording) |
| P9-01 | Ch9 §9.4 future-work bullets with internal vocabulary/status | H9 | DECISION D-05 (text unchanged) |
| P9-02 | Ch9 §9.2 RQ placeholders | structure | APPLY |

---

## 3. Additions list

| Addition | Ch | Length | Sources |
|---|---|---|---|
| Box: rotations, SE(3), hat/vee, Ψ, quaternion remark | 2 §2.1 | ½ p | R15 (eqs. 6–9); code `lib.rs:177–183` |
| Flatness: what fixes thrust, attitude, rates | 2 §2.1 | ⅓ p | R05 eqs. 3–11; R13 eqs. 9–15 |
| INDI intuition | 2 §2.3 | ⅓ p | R13, R14 |
| How the field evaluates | 2 §2.8 | ½ p | Stage 2 §5 |
| Worked 1-D residual example (illustrative) | 3 §3.3 | ⅓ p | textbook arithmetic; Kiran rig value labelled |
| Sign derivation step | 3 §3.3.2 | 4 lines | R01 eq. 9; R13 eq. 20 |
| Deep-sets toy example | 3 §3.4.1 | ¼ p | illustrative |
| Network description (code) | 3 §3.4.1–3.4.4 | 1 p (replaces ¾ p) | code; R02 §VII-C |
| Frame mapping to Lee | 4 §4.1 | 4 lines | R15 |
| INDI derivation + toy | 4 §4.3 | ½ p | R13 eqs. 29–31 |
| Metric definitions | 5 §5.5.4 | ¾ p | design |
| Statistical treatment skeleton | 5 §5.5.5 | ¼ p | — |
| Scenario parameter column | 5 Tab 5.1 | — | `scenarios.py` |
| Ch6–9 RQ-mapped skeleton sections | 6–9 | 2 p total | — |

---

## 4. Reference actions

Output files: `proposed_references.md` (UNVERIFIED additions and suspected `references.bib` errors). No edit to
`references.bib`.

| Action | Entry | Evidence |
|---|---|---|
| correct year / type | `shi2022neuralswarm2` (year 2020 → T-RO 2022; `@article`) | R02; key/yearnote |
| correct year | `cobobriesewitz2026lindi` (2025 → 2026, L4DC/PMLR) | R07 PDF header |
| version note | `tal2018indi` (held = extended version; T-CST 2021 per R07 refs) | R13 |
| version note | `lee2010geometricse3` (held = v4; note says v1) | R15 |
| stale note | `gu2025comparative` ("full text not held" — it is held) | R18 |
| year check | `bauersfeld2024airflow` (RA-L vol. 10, 2025?) | R12, R05/R11 refs |
| venue additions (optional) | `yang2025` CDC 2025; `li2023` CDC 2023; `smeur2017` CEP 2018; `shi2020` ICRA 2020; `shankar2023` ISER; `gielis2023` ISER | held reference lists |
| re-attach context | Smith 56 %; NS2 24 cm; Smeur 4 Hz; Hsieh 2026 Ch4; Hsieh 2025 everywhere; Gielis "fourth vehicle"; Cobo "swing"/"ground forces"; Shankar attribution | R02, R04, R05, R07, R10, R14, R17 |
| proposed new (UNVERIFIED) | Zaheer 2017; Mellinger & Kumar 2011; Sun et al. 2022; Jain et al. 2019; Preiss et al. 2017 (IROS); Hönig et al. 2018; Smeur et al. 2016; Fliess et al. 1995; Faessler et al. 2018; Shi et al. 2019 (Neural Lander); Slotine & Li 1991; Mahony et al. 2012; Busetto et al. (arXiv:2512.14450); Crazyswarm 2017 | Stage 2 §2 |

---

## 5. Decisions needed from the author

| ID | Decision | Options | Recommendation |
|---|---|---|---|
| **D-01** | **Residual-sign wording** (Ch3 §3.3.2; Ch4 eq. ctrl-ades and §4.1.1, §4.3, §4.6) | (a) keep "subtracted" as the derived law and add that the study configuration of Strategy 1 uses the opposite sign (+1), with the consequence the chapter itself predicts; (b) write the law with an explicit sign parameter $s\in\{-1,+1\}$, give the derivation that $s=-1$ cancels the residual, and state each strategy's configured value; (c) change the Strategy 1 configuration to −1 before the campaign (lab action) and keep the text. | **(c) if feasible, otherwise (b).** The thesis' own derivation (and Shi 2020 eq. 9, NS2 eq. 18, Tal & Karaman eq. 20, Cobo eq. 8a, Hsieh 2026 eq. 14a, Gu eq. 15) says the residual must be subtracted; a Strategy 1 that adds it is, by Ch3's own argument, not a representative of the reactive family. The text must not describe a sign the study does not fly. |
| **D-02** | **Integral terms** in Ch4 eqs. (ctrl-ades), (ctrl-tau-geo) | (a) rewrite to the code: remove $K_i\int e_p$ and $k_{i,\mathrm{att}}\int e_R$, add $k_{i,z}\int e_z\,e_3$ with per-strategy gain (S0/S2: on; S1: off); (b) keep the general form and state which terms are zero in the study; (c) leave as is. | **(a).** Equations claimed to be "the code" must be the code. The per-strategy difference (vertical integral on for S0/S2, off for S1) must be named in §4.6 as part of what "each tuned for itself" means; it also qualifies Ch3's "approximately twice" prediction (an integrator removes a constant offset whatever the sign). |
| **D-03** | **Gap statement and Hsieh et al. 2025** (Ch1 §1.2 ¶6, §1.3; Ch2 §2.8 comparisons, §2.9 first gap; both provenance todos) | (a) narrow the gap to the recommended wording (Stage 2 §3) and list Hsieh 2025 among existing comparisons; (b) keep the broad gap and describe Hsieh 2025 as adjacent (MPC-internal); (c) recast contribution 1 around onboard controllers + multi-regime library + RQ3. | **(a) with elements of (c).** (b) would leave a claim an examiner can refute with a paper the thesis already cites. |
| **D-04** | **Rationale for collecting training data only under the baseline** (Ch4 §4.2 Role, §4.4 ¶2; Ch5 §5.5.2) | (a) replace with reasons that hold: the prediction-vs-measurement comparison on prediction-off flights is not influenced by the model's own action; the visited state distribution does not depend on the model being trained; the collection controller is identical for every model; the residual itself is unchanged by compensation (Shi 2020 §III-C) — so compensated flights *could* add data; (b) keep the current argument (not recommended); (c) adopt a curriculum like Shi et al./Smith et al. | **(a).** The current argument contradicts the thesis' own residual definition (thrust from *measured* rotor speeds). |
| **D-05** | **Lab-status passages (H9)**: Ch4 Tab 4.1 Status + caption + §4.5 first sentence + §4.7 last ¶; Ch5 status ¶ + §5.4.3 last sentence; Ch6 §6.1; Ch7 §7.1 "roster"; Ch9 §9.4 bullets; Ch1 §1.4 S3 sentence | (i) delete; (ii) replace by a future-tense design statement; (iii) neutral implementation status ("implemented in the flight code" / "not implemented"). | Tab 4.1 → (iii) column "Implemented: yes/yes/yes/no"; Ch5 status ¶ → (i); §4.7 → (iii); §4.5 → (iii) "not implemented; the reference method is described"; §5.4.3 → (ii); Ch6 §6.1 → (ii); Ch7 roster → (ii); Ch9 bullets → (ii) without internal names; Ch1 S3 → (iii). |
| **D-06** | **Filter values to state** (Ch4 §4.3; Ch3 §3.3.1) | (a) state the configured nominal cutoff and the realised −3 dB frequency given the design sample time and discretisation; (b) state only that a 2nd-order Butterworth is used and that its realised cutoff is set by configuration; (c) reconfigure (`filt_dt_us`, `filt_prewarp`) so nominal = realised before the campaign. | **(c) then (a).** An examiner who reads "fc = 60 Hz" will assume 60 Hz. |
| **D-07** | **H1 removals** (Ch5 dry-run ¶, §5.6 findings, radio numbers) — applied by rule 1 | keep removed (as now); move the original statements to an appendix of development notes after the campaign; or restore with explicit "development observation" framing (violates H1 in this round). | keep removed in the thesis body; an appendix "Software verification" may report them later as verification, not as results. |
| **D-08** | **Abro et al. paragraph** (no PDF held) | verify every clause against the full text; or reduce to "Abro et al. compare fractional-order and integral state-feedback regulators for close-proximity flight" | verify; otherwise reduce. |
| **D-09** | **Hypotheses and statistics** (Ch1/Ch5) | (a) add the falsifiable hypotheses of `docs/15` (H1–H3; H4 rewritten without the hybrid) to Ch5 §5.5 as expected outcomes with their falsifiers, and fix $n$ and the paired analysis; (b) leave RQs only. | **(a)** — RQ1–RQ3 then each have a stated falsifier; Ch6–9 skeletons already reference "a negative answer". |
| **D-10** | **Bib corrections** (years, types, versions) | apply the `proposed_references.md` list | apply after checking each primary record. |

---

## 6. Rejected ideas (not done, with reason)

| Idea | Why not |
|---|---|
| Rename the hat-map symbol to avoid the $\hat a$ conflict | changes symbols across five chapters (H4/H6); a one-sentence convention suffices |
| Add the new UNVERIFIED references as `\cite` | breaks the build and violates H2/H6; listed in `proposed_references.md` only |
| Report the sign A/B, integral A/B or any flight outcome to justify D-01/D-02 | H1 |
| Describe which of the project's INDI implementations the study will use | H5 |
| Add the hybrid (NA-INDI) back as a compared strategy | supervisor decision; H7 |
| Write metric values, repeat counts or effect sizes | H1 |
| Fix the code (sign, filters, integral flags) | read-only rule |
| Rewrite RQs to include Hsieh 2025's framing | H7 (no new RQs) |
| Delete lab-status passages unilaterally | H9 says DECISION |
| Use `docs/*.md` notes (15, 18, 19) as sources for any statement | H3; they are stale (hybrid in scope, seven strategies, shared gains) |

---

## 7. Order of work and expected size

Ch3 (≈2 850 → ≈3 600 words) → Ch4 (≈2 530 → ≈3 400) → Ch5 (≈2 910 → ≈3 500) → Ch2 (≈5 410 → ≈6 300) → Ch1 (≈2 090 →
≈2 200) → Ch6–9 (≈2 600 → ≈3 000). After each chapter, the citation audit (pass D) is run before moving on.
