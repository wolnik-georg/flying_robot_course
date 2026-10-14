# Stage 1B — Reconciliation of the independent reading (Stage 1) with the existing ledger

Ledger: `docs/_archive/citation_notes/20_Verified_Claims.md` (created 23 Aug 2026, corrections 26–27 Aug). Read in full
for this stage. Records R01–R20 are in `stage1_sources.md`. Where the two disagree, the PDF decides; where the PDF
cannot decide, the row is marked for the author.

Legend: **A** agreement · **D** disagreement · **L-only** in the ledger, not found by me in the PDF · **S-only** found by
me, absent from the ledger.

---

## (a) Agreements

| Paper | Points on which ledger and Stage 1 agree |
|---|---|
| Neural-Swarm | 60 cm cited from planning work for CF2.0 (9 cm); separate networks per team size; 2.4 cm is the 4-vehicle-trained net on the 2-vehicle task, 2.7 cm the 2-trained; "factor two or better, up to four"; 0.20–0.25 m with 25 cm explicit for three vehicles; directional generalisation; $f_{a,z}$ only; STM32 onboard; "not a simple superposition of each pair". |
| Neural-Swarm2 | heterogeneous deep sets; SN with abstract-level guarantees vs body Lipschitz/bounded-error assumption; planning + control; threefold is abstract-only; 16 robots / 24 cm and 15 → 10 cm (≈1.5×) must not be merged; trained on ≤3 vehicles / ≤2 neighbours; "relatively less accurate". |
| Gielis | load-stand sufferer at hover thrust, neighbours fly freely; 4 in total = 1 + ≤3; deep-sets sum-pool vs linear summation, "graph" is abstract language; three regimes in the paper's labels; validates its own predictors on the stand; "natural ground truth" is the thesis' reading. |
| Smith | footnote reading (abstract 3-D/vertical vs contributions lateral); 0.154 → 0.098 m with 55 %/49 %; 5 vs 15 min is a validation-RMSE result; LQR; two identical vehicles; not a Crazyflie (0.26 m, 0.7 kg, RPi 50/45 Hz); 0.5 m is the closest training stage, evaluation 0.6–0.8 m. |
| Cobo-Briesewitz | sensor elimination as headline; ≈40 % without payload, ≈19–24 % with; NA-INDI "small margin" without payload, "similar" with; LINDI circle OOD (no payload only); RPM still needed for labels and for NA-INDI's INDI branch; "remaining mismatch"; single vehicle only; payload net 28-D incl. payload states; swing is not named as a residual. |
| ProxFly | residual RL on cascaded controller; ego state + commands only, no communication; heterogeneous pair; 50 cm; docking from 5 cm. |
| Hsieh 2026 | 31 % abstract-only; body 24/30 % and 29/43 %; 20× is simulation (Table 4); hardware match with Chee is against published figures; sim FBL z_max can be worse than NMPC; 28 s HW / 18 s sim; 5 ms abstract-level and off-board (i5, Crazyradio 400 Hz); forces-only on hardware, torques modelled in sim; hardware Δz = 0.3/0.4 m; two CF2.1 with thrust upgrade. |
| Yang 2025 | generic residuals can destroy flatness; lower-triangular preserves flatness and flat outputs; simulation only on a 2-D quadrotor. |
| Chee 2024 | learned component inside NMPC prediction constraint; two CF2.1, bottom with thrust upgrade, 10 cm, ≈34 g; off-board i5, Crazyradio ≈400 Hz, Vicon ≈120 Hz; 40.1 %/57.5 %; 46 s = 18 s sim + 28 s HW; commanded 0.2–0.4 m plus 0.08 m, average 0.12 m; 20× is not in Chee. |
| Hsieh 2025 | three CF2.1; V-stack and I-stack; hardware-only; off-board NUC (200 Hz) and i5 (400 Hz); only the $\mathcal L_1$ estimate adapts online; KNODE-DW reuses Chee's (two-vehicle) training data; $z_1$ = 0.2 m, $z_2 \in$ {0.2, 0.3, 0.4} m, $r$ = 0.1 m; 20× not in this paper. |
| Kiran | forces, torques and PIV; single and pair. |
| Bauersfeld | jet beyond 2.5 drone-diameters (abstract) / motor distances (body); 16 h; hover-only; lightweight vs CFD. |
| Tal & Karaman | fourth-order tracking; flatness FF of rate and acceleration; snap ⇒ torque control ⇒ encoders; no prior aerodynamic model. |
| Smeur | cascaded attitude + position INDI; 10 m/s wind tunnel; infrequent position updates / GPS. |
| Gu | four estimators; modular framework; lumped wrench by inverting discrete Newton–Euler (analogous, not identical, to $a_{\mathrm{res}}$); BO for the parametric models only; trade-offs not a winner; "hardware experiments" = Jetson in the loop with a simulator, real flight future work; RBFNN crash (periodic wind) and ELM instability (ground effect), "may be attributed", "excessively large prediction values". |
| Li 2023 | neural downwash predictor inside NMPC; spectral normalisation; prediction from neighbours' states. |
| Shankar | learned downwash inside an optimal feedback controller; vertical docking within 1–2 BL; follower takes the interference; "precondition" is the thesis' reading. |
| Lee | CDC 2010 primary, companion arXiv for equations; $e_R$ eq. (8). |

---

## (b) Disagreements

| # | Paper | Ledger says | Stage 1 says (evidence) | Recommendation |
|---|---|---|---|---|
| D1 | **Hsieh 2025** | An MPC-family paper: "\(\mathcal L_1\) residual-acceleration estimate inside the KNODE-DW prediction model"; listed under "inside a predictive horizon" only. The W.10 audit "found no study comparing reactive, predictive and hybrid interaction-force compensation against one another under common conditions". | The $\mathcal L_1$ module **estimates the residual acceleration online** from predicted-vs-measured velocity (§2.2) — by the thesis' own definition (Ch1 §1.2: "measure … comparing the acceleration the vehicle actually experienced against the acceleration its nominal model predicts") this is the **reactive** family. The paper flies **estimation-only ($\mathcal L_1$ MPC), prediction-only (DW/KNODE-DW MPC) and hybrid ($\mathcal L_1$ KNODE-DW MPC)** on the **same three Crazyflies and formations**, five runs each, and reports that estimation alone beats prediction alone for the middle vehicle (−28.5 % RMSE) while the hybrid is best (§3). | **Stage 1 is right.** Consequences: (i) Ch1 §1.3 and Ch2 §2.9 "hybrid demonstrated only on a single vehicle" are false (APPLY minimal correction); (ii) the main gap statement must be narrowed (DECISION D-03); (iii) the two provenance todos and the ledger's audit conclusion are outdated. |
| D2 | **Gu** | "the only paper we hold that is *itself* a systematic comparison". | Hsieh 2025 (six MPC variants, three families), ProxFly (residual RL vs model-based aerodynamic compensation vs basic, same hardware), Chee 2024 (four MPC models), Hsieh 2026 (FBL and SE(3) × three model fidelities, 60 runs), Cobo-Briesewitz (Lee/INDI/LINDI/NA-INDI) are all same-rig multi-method comparisons. | **Stage 1 is right.** Gu remains the closest *benchmark-style* comparison of estimators; the thesis should not call it the only comparison. |
| D3 | **Smith** | "≈36 % 3D and 56 % vertical … follows the abstract and the translation/lemniscate results body". | The **body** gives 55 % (translation, vertical position) and ≈51 % (lemniscate, "vertical"); **56 % appears only in the abstract and the contributions list**. | **Stage 1 is right.** Keep 56 % only as "the abstract's" figure, or use 55 % from the body. |
| D4 | **Hsieh 2026** | "The introduction separately says 7×". | The held v2 (17 Aug 2026) introduction says "**20×** less compute" and "We directly compare our approach with this baseline **in an identical hardware setup**" (§1.1.1); no "7×" occurs. | The ledger probably read a different version (v1). **Use v2 wording.** The introduction's "identical hardware setup" contradicts the body's "match the reported tracking performance" — the thesis should cite the body and note the introduction's contrary wording. |
| D5 | **Neural-Swarm** | "Geometric SE(3) is an optional attitude inner loop only". | The paper lists Lee et al. [24] as one possible attitude controller, and the **baseline and experiments use** "a nonlinear attitude tracking controller [24]" (§V-C). | Both partly right. Recommended: "the attitude loop is left open in the design; the experiments use the nonlinear (geometric) attitude tracking controller of Lee et al." |
| D6 | **Neural-Swarm2** | Corrections box "never put 16 robots, 24 cm and 3× in one sentence"; silent on where 24 cm appears. | 24 cm is in the Fig. 1 caption and the contribution list (§II end), **not** the abstract; the draft (Ch2 L204–205) says "The two figures are reported in the abstract". | **Stage 1 is right**; correct the sentence (APPLY). |
| D7 | **Gielis — fourth vehicle** | "Generalisation tested by adding a fourth vehicle … Abstract-level. Body tables stop at **K = 3** … Do not claim a successful held-out four-neighbour result". | The abstract says "we explore how our proposed models generalize when a fourth vehicle is added to the formation"; K = 3 neighbours plus the sufferer **is** four vehicles, and the conclusion speaks of "formations of up to 4 multirotors in total". Whether "a fourth vehicle" means K = 3 (four in total) or K = 4 is ambiguous in the paper. | **Mark for the author** (low stakes): use neutral wording — "the abstract announces an exploration of generalisation to a fourth vehicle; the largest reported case is three neighbours around the stand, and the conclusion cautions that the model 'may be incapable of generalising to cases where K > 3'". |
| D8 | **Lee** | "Our torque law is theirs — eq. (11) matches the geometric torque in `lib.rs`". | The code omits the feedforward bracket and uses $e_\omega = \omega - \omega_d$ without the transport $R^\top R_d$ (`lib.rs:2221`); the thesis itself calls the law "reduced". Also: the held companion is **v4**, while the bib `note` speaks of v1. | **Stage 1 is right**; the thesis' "reduced form" wording stands; add the $e_\omega$ difference; record the held version (v4). |
| D9 | **Cobo-Briesewitz — residual sources** | "The introduction names blade flapping, drag, ground forces" as the residual. | The introduction lists these as generic sources of residual forces with citations to other work (§1); the experiments do not isolate any of them. | Agree on the facts; the thesis (Ch1 L108–111) should not present the list as the conditions demonstrated (APPLY). |
| D10 | **Li 2023**, **Shankar 2023** | "⚠️ Abstract-level only. Claim nothing about their experiments or results." (while also saying "Read: full text … confirmed in the body") | Both read in full (R16, R17); the experimental details the thesis uses (TX2 NX onboard, mocap; 0.27 m/0.7 kg, 0.36 m dock, 8/10, curriculum, RPi) are confirmed. | **Stage 1 supersedes** the ledger's restriction; details are now usable with the citations in R16/R17. |
| D11 | **Smeur** | "Tolerates infrequent position updates … using a standard GPS module". | The 4 Hz position feed is part of the **wind-tunnel** run (§5); the GPS take-off is separate (§6). The draft says "two *separate* experiments … indoors with motion capture deliberately downsampled to 4 Hz". | **Stage 1 is right**; reword (APPLY). |
| D12 | **Shankar — moving-leader failure** | (not addressed) | Draft: "attributed to gripper latency"; paper §3.3: "usually attributed to Bravo's oscillatory behavior that prevents the servo actuation from clasping", §4: physical actuation limits. | Reword (APPLY). |
| D13 | **Gielis — C2.09 wording** | "The paper validates its own linear and Deep-Sets predictors on the stand; it does not claim that role." | Same facts; the **draft** sentence "neither validates a learned model of ours or anyone else's" reads as if Gielis validated no learned model. | Ledger and Stage 1 agree on facts; draft wording must change (APPLY). |
| D14 | **Cobo-Briesewitz — "swing of a payload"** | Corrected 26 Aug: "Swing is not named as a residual". | Agree. But the **draft still says** "the swing of a payload" (Ch2 L401) — the ledger's claim that "every one has been applied to the chapters" is not true for this item. | APPLY. |

---

## (c) Claims in the ledger that could not be found in the paper

| Ledger claim | Status |
|---|---|
| Hsieh 2026 "The introduction separately says 7×" | **Not in the held v2** (D4). |
| Abro et al.: "§II does write a linear $f_a$ model"; "$n$ = 2, Z-axis only"; "CoDrone platform"; "0.5 / 0.85 / 1.1 m tables"; "They do not fly Crazyflies"; "did not demonstrate large swarms" | **Cannot be checked — no PDF held.** Under H2 nothing of this may enter the thesis; the draft's current Abro paragraph (Ch2 L510–518) must be re-verified by the author against the full text (`[VERIFY]`). |
| Lee: "metadata verified via Crossref" (CDC pages 5420–5425, DOI) | Not checkable from held material; plausible; `[VERIFY]` optional. |
| Gielis ❌ "Usable onboard REFUTED" | Consistent with the PDF (no onboard discussion) — a negative claim, fine. |
| Smith "They have similar estimation and control stacks onboard" | Found (§II) ✓ — not a disagreement, listed for completeness. |

---

## (d) Claims found in Stage 1 that the ledger lacks and the thesis should consider

| # | Source (record) | Claim and location | Where in the thesis | Use |
|---|---|---|---|---|
| N1 | Shi 2020 (R01) §III-C; Shi 2022 (R02) §IV-D | "data collection and learning are independent of the controller used and independent of the $f_a$ compensation … will only affect $f_u$ … and not the observed $f_a$" | Ch4 §4.2 Role; Ch4 §4.4; Ch5 §5.5.2 | **contradicts** the thesis' rationale for baseline-only collection (DECISION D-04) |
| N2 | NS2 (R02) §III-B | neighbours defined by an interaction volume, "a cuboid based on observed interactions" | Ch3 §3.4.3 (gate) | source for the gate concept (APPLY) |
| N3 | NS2 (R02) §VII-C | split into 50 ≈20 s pieces, 80/20 by piece | Ch5 §5.7 | precedent for split-by-segment (APPLY) |
| N4 | NS2 (R02) §VII-E | online estimate-and-compensate "would quickly crash the drone" (no method given) | Ch2 §2.3 | evidence on reactive limitations, with caveat (APPLY) |
| N5 | Cobo-Briesewitz (R07) Table 1 | INDI with PWM-derived thrust worse than uncompensated Lee on a CF2.1 | Ch3 §3.3.1; Ch2 §2.3 | evidence for the rotor-speed prerequisite (APPLY) |
| N6 | Tal & Karaman (R13) §III-B | "Identical LPFs must be used …"; "eliminates the need for integral action" | Ch4 §4.3 deviations; D-02 | deviation statement (APPLY, code) and decision input |
| N7 | Smeur (R14) §4.6 | outer-loop INDI sensitive to accelerometer bias | Ch3 §3.3.1 "absorbs model error" | missing failure mode (APPLY) |
| N8 | Kiran (R11) §III-A | lower Crazyflie thrust 65 % of hover at Δz = 13 cm; upper 90 %; recovery Δx/l ≈ 3, Δz/l ≈ 19 | Ch1 §1.1; Ch2 §2.2 (C2.06); Ch3 magnitude example | source for nonlinearity/asymmetry and an illustrative magnitude (APPLY) |
| N9 | Hsieh 2026 (R05) §5.2 | learned residual as feedforward in an **SE(3)** controller: −43.7 % RMSE on hardware | Ch2 §2.5 FF paragraph | closest published analogue of Strategy 2 (APPLY) |
| N10 | Hsieh 2025 (R10) §3–§4 | estimation-only beats prediction-only for the centre vehicle; hybrid best; $\mathcal L_1$ slower entering/exiting the wake | Ch1 §1.3; Ch2 §2.5, §2.7, §2.8, §2.9; RQ2 hypothesis | gap narrowing (D-03) and prior evidence for RQ2 |
| N11 | Shankar (R17) §1 | disturbance observers have "a finite (non-zero) convergence time" under the leader | Ch2 §2.3 | reactive limitation (APPLY) |
| N12 | Li (R16) §IV | "should be compared with non-prediction methods" | Ch2 §2.8 | the field's own call for the comparison (APPLY) |
| N13 | Li (R16) §III-B | shuffled sample split 0.75/0.25 | Ch5 §5.7 | methodological contrast (optional) |
| N14 | Li (R16) §III-A | DShot rotor-speed telemetry used to form the nominal force | Ch3 §3.3 | precedent (optional) |
| N15 | Chee (R09) §I | calls learned-feedforward controllers "reactive" (current time step) | Ch1 §1.2 / Ch2 §2.3 terminology footnote | prevents a terminology objection (APPLY) |
| N16 | Bauersfeld (R12) §VII | jet-model feedforward compensation: 3–5× lower height RMSE at 1–2 m | Ch2 §2.2 or §2.5 | first-principles predictive instance (APPLY, short) |
| N17 | Tal & Karaman (R13) | held version is the extended journal version (T-CST 2021 per R07 refs) | `references.bib` | bib correction (DECISION, bib) |
| N18 | Gu (R18) | open-access full text held — bib note stale | `references.bib` | bib note fix (bib output) |
| N19 | Hsieh 2026 (R05) §1.1.1 vs §5.2 | introduction "identical hardware setup" vs body "reported" | Ch2 §2.5 | precision (APPLY) |
| N20 | Gielis (R03) §1 | "reactive controllers are not able to reject them without first observing the disturbance" | Ch1 §1.2 / Ch2 §2.3 | quotable support for the reactive limitation (APPLY) |

---

## Summary

- 18 papers reconciled; 14 disagreements (D1–D14). **Stage 1 prevails in 12**, two are wording-level or ambiguous
  (D7 marked for the author; D9 facts agree).
- The **most consequential** item is D1 (Hsieh 2025 as a three-family comparison on three vehicles), which the ledger
  and both gap audits classified as adjacent MPC work. It changes the gap statement (DECISION D-03).
- The second most consequential is N1: the two Neural-Swarm papers state that compensation does not change the observed
  residual, contradicting the thesis' stated reason for collecting data only under the baseline (DECISION D-04).
- The ledger's statement that all 26–27 Aug corrections are applied to the chapters is not true for D14 and D6.
