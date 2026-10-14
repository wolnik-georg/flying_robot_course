# Stage 4 — Chapter 4 (Control architectures): passes A–F

Revised file: `revised/ch4_control_architectures.tex`. Combined build (revised Ch3–Ch4) compiles with no errors, no
undefined references or citations.

## Verdict (5 lines)
1. The chapter's structure (implemented / reference / deviations / requirements per strategy) is right and kept.
2. Strategy 2's implementation and deviations described a network that no longer exists; now corrected to the code.
3. The INDI section gains the derivation, the code's filtering choices and the deviations from Tal & Karaman; the derivation also shows that position INDI *is* the subtracted-residual law, which sharpens D-01.
4. The two central equations still carry integral terms and residual signs the study configuration does not fly; they are unchanged by rule and marked D-01/D-02 — this is the chapter's main open risk.
5. The Hsieh 2026 paragraph contradicted Chapter 2 and is corrected; fairness now lists what differs by design.

## Rubric (0–5), before → after

| Criterion | Before | After | Note |
|---|---|---|---|
| 1 Correctness | 2 | 3 | network, rates, Hsieh fixed; eqs. still D-01/D-02 → 4–5 once decided |
| 2 Completeness | 3 | 4 | velocity-error conditioning, $R_d$, filters, rotor source, label requirement |
| 3 Depth | 3 | 4 | INDI derivation + toy, Ψ definition, frame mapping |
| 4 Academic standard | 4 | 4 | |
| 5 Explainability | 3 | 4 | |
| 6 Related work / referencing | 3 | 4 | Hsieh 2026 SE(3) analogue added |
| 7 Cross-chapter consistency | 2 | 4 | consistent with revised Ch2/Ch3 except D-rows |
| 8 Reference correctness | 3 | 5 | C4.08, C4.10 corrected |

## A. Claim re-check

### A1. Literature claims

| Location | Claim | Key | Record | Verdict |
|---|---|---|---|---|
| Tab 4.1 fn | learned method needs RPM for labels | cobo2026 | R07 §5.4.1 | supported |
| §4.1.1 | $R_d$ construction (position + heading) | lee2010se3 | R15 eqs. 22–23, 38 | supported (equivalent projection form) |
| §4.1.2 | reduced form of Lee's law; symbol mapping | lee2010tracking | R15 (companion) | supported in v4; CDC `[VERIFY]` |
| §4.2 | Lee: position + heading, four outputs; $e_R$ | lee2010tracking | R15 | as above |
| §4.2 | Lee: $e_3$ down, $m\dot v = mge_3 - fRe_3$, $e_x = x - x_d$ | lee2010se3 | R15 eqs. 3, 17 | supported |
| §4.2 | eq. (ctrl-tau-lee), $e_\Omega$ | lee2010tracking | R15 eqs. 9, 11, 20 | supported (v4) |
| §4.2 | Ψ regions | lee2010tracking | R15 Props. 3–4 | supported (v4); CDC `[VERIFY]` |
| §4.2 | Ψ = ½ tr(I − R_dᵀR) = 1 − cos θ | lee2010se3 | R15 eq. 6 (+ identity) | supported |
| §4.3 | derivation, assumptions (slow δ, alike filtering) | tal2018 | R13 §III-D, §III-B | supported |
| §4.3 | closed-loop α follows reference with motor dynamics; J error affects transient | tal2018 | R13 §IV-A, §IV-B | supported |
| §4.3 | position term = linear-acceleration increment | tal2018, smeur2017 | R13 eq. 20; R14 eq. 19/28 | supported (algebra shown) |
| §4.3 Reference | flatness FF from Tal; cascade from Smeur | tal2018, smeur2017 | R13, R14 | supported |
| §4.3 Deviations | Tal encoders; identical LPFs; filter μ_f; no integral needed | tal2018 | R13 abstract, §III-B, §III-D | supported |
| §4.4 Reference | NS line | shi2020, shi2022 | R01, R02 | supported |
| §4.4 Reference | Hsieh 2026 learned residual as FF in SE(3) (secondary configuration, hardware) | hsieh2026 | R05 §5.2 | supported |
| §4.4 Deviations | NS2 SN, abstract guarantees, bounded-error assumption | shi2022 | R02 abstract, §IV-C | supported |
| §4.4 Deviations | NS2 planning + control | shi2022 | R02 §V–VI | supported |
| §4.4 Deviations | NS2 own attitude law + motor-delay compensation | shi2022 | R02 eqs. 19–21 | supported |
| §4.5 | Hsieh builds on Yang; lower-triangular; planar sim | hsieh2026, yang2025 | R05 §1.1.2; R06 | supported |
| §4.5 | residual depends only on positions and velocities | hsieh2026 | R05 §3.2 eq. 13 | supported |
| §4.5 | 31 % abstract; 24 %/29 % HW; ≈5 cm matching reported Chee; simulation head-to-head ≈1/20 compute; <30 s and 5 ms abstract; off-board | hsieh2026 | R05 abstract, §5.1, §5.2, Table 4 | supported |

### A2. Implementation statements vs code

| Statement | Code | Verdict |
|---|---|---|
| eq. (ctrl-ades) | `lib.rs:1876–1931` | **D-01/D-02** (integral, sign) |
| $R_d$ columns | `lib.rs:1463–1473` | ✓ |
| thrust = projection, clipped at 0 | `lib.rs:1949–1950` | ✓ |
| XY velocity error ±0.6 m/s, 10 Hz, z untouched | `lib.rs:564–574, 1504–1521` | ✓ |
| eq. (ctrl-tau-geo) | `lib.rs:2219–2235` | ✓ except $k_{i,\mathrm{att}} = 0$ (D-02) |
| geometric gains in N·m (per rad), N·m·s | `traj_iface.c:266–280` | ✓ |
| runtime bits; rnn.en; weights-loaded gate; per-strategy gains, sign, ki_z | `traj_iface.c:203–214, 522–582`; `lib.rs:1752–1756, 1837, 1851, 2175` | ✓ (single file `[VERIFY]`) |
| $e_\omega = \omega - \omega_d$ no transport | `lib.rs:2221` | ✓ |
| INDI gains 1/s², 1/s | `traj_iface.c:257–260` comments | ✓ |
| α_ref filtered by same Butterworth | `lib.rs:2075–2080` | ✓ |
| τ_cur from mixer inverse (arm, drag ratio) | `lib.rs:1481–1493, 2108–2109` | ✓ |
| effectiveness factor default 1; torque limits optional | `traj_iface.c:350, 370–372`; `lib.rs:1651–1664` | ✓ |
| no gyroscopic term in INDI branch | `lib.rs:2175–2218` | ✓ |
| four configurations by two bits | `lib.rs:240–244, 1837, 2175` | ✓ |
| DShot or deck + plausibility filter | `traj_iface.c:508–520, 701–730` | ✓ |
| accelerometer filtered, thrust not by default | `lib.rs:545, 1801–1828`; `traj_iface.c:493` | ✓ |
| τ_cur unfiltered by default | `traj_iface.c:342`; `lib.rs:2156–2157` | ✓ |
| S2: every 10th step, ≈100 Hz; broadcast positions; differenced velocity; vertical only; flash image | `traj_iface.c:132`; `lib.rs:839–873, 922–926`; `residual_nn.rs:302` | ✓ |
| S2: ≤3 neighbours; 8 m/s² | `residual_nn.rs:129, 134` | ✓ |
| labels require rotor speeds | `lib.rs:1781` | ✓ |

## B. Defect log (new)

| # | Defect | Sev | Handling |
|---|---|---|---|
| B4-1 | "nothing else about the position loop changes between them" false for the study configuration (sign, vertical integral gain) | S1 | D-01/D-02 comment + §4.6 "What differs by design" |
| B4-2 | Tab 4.1 "Rotor speed: no" for S2/S3 omits label requirement (RQ4 cost) | S2 | APPLY footnote |
| B4-3 | INDI gains' units given only qualitatively | S3 | APPLY |
| B4-4 | Ψ used without definition | S3 | APPLY |
| B4-5 | header comment described an obsolete seven-strategy numbering | S3 | APPLY (comment) |

## C. Rewrite
`revised/ch4_control_architectures.tex` (complete). Equations (ctrl-ades) and (ctrl-tau-geo) unchanged (D-01/D-02).

## D. Citation audit

| `\cite` | Exists | Claim | Location | Context |
|---|---|---|---|---|
| cobobriesewitz2026lindi | ✓ | labels need RPM | §5.4.1 | ✓ |
| lee2010geometricse3 (×4) | ✓ | $R_d$ construction; frame; Ψ | eqs. 3, 6, 17, 22–23, 38 | ✓ |
| lee2010geometrictracking (×4) | ✓ | reduced law; reference; source law; regions | (no PDF; companion) | `[VERIFY]` CDC |
| tal2018indi (×6) | ✓ | derivation, assumptions, motor dynamics, J error, increment, encoders, LPF, μ_f, no integral | §III-B, §III-D, §IV | ✓ |
| smeur2017cascadedindi (×2) | ✓ | cascade; increment | §3 eq. 19 | ✓ |
| shi2020neuralswarm, shi2022neuralswarm2 | ✓ | NS line; SN; planning; attitude/delay | R01, R02 | ✓ |
| hsieh2026flatness (×5) | ✓ | SE(3) FF config; flatness structure; numbers; off-board | §3.2, §5.1–5.2 | ✓ |
| yang2025flatnessresiduals (×2) | ✓ | lower-triangular; planar sim | R06 | ✓ |

No failed row (CDC entries carry `[VERIFY]` as in the draft).

## E. Examiner simulation

| # | Question | Where answered |
|---|---|---|
| 1 | Eq. (ctrl-ades): does your code have a full position integral? | **D-02** (comment; to be answered in text) |
| 2 | Which sign does Strategy 1 fly, and is it then INDI? | §4.3 increment equivalence + **D-01** |
| 3 | Eq. (ctrl-tau-geo): is it Lee's law? what's missing? | §4.2 reduced form; $e_\omega$ deviation |
| 4 | Eq. (ctrl-alpharef)/(ctrl-indi-tau): derive; assumptions | §4.3 "Why the increment rejects disturbances" |
| 5 | Are your filters matched as Tal & Karaman require? | §4.3 Deviations (+ D-06 values) |
| 6 | Lee uses z-down — did you transcribe signs correctly? | §4.2 frame mapping |
| 7 | Fairness: what differs between strategies besides interaction information? | §4.6 "What differs by design" |
| 8 | Fairness: one tuning per strategy — who tuned, how much effort? | §4.6 (RQ4); Ch5 gate |
| 9 | Is Strategy 2 Neural-Swarm2? | §4.4 Deviations (four points) |
| 10 | Strategy 3 not implemented — is the comparison still meaningful? | §4.5 status (D-05) + scope |
| 11 | Does the network see lateral residuals? | §4.4 "The prediction is a vertical acceleration" |
| 12 | Do learned strategies avoid rotor-speed sensors? | Tab 4.1 footnote; §4.4/4.5 Requirements |

## F. Student-reader test

| Stumbling point | Resolution |
|---|---|
| "usual flatness construction" | explicit three columns |
| gains in different units | units given for both gain sets |
| why INDI works | derivation + one-axis numbers |
| why position INDI and "subtract the residual" are the same | the substitution shown |
| Ψ | definition + 90°/180° values |
| frame differences with the source | frame mapping sentence |

## Change log
P4-01 → D-02 comment; P4-02 → D-01 comment; P4-03 → D-02 comment; P4-04 ✓; P4-05 ✓; P4-06 ✓; P4-07 ✓; P4-08 ✓
(+VERIFY single file); P4-09 ✓; P4-10 ✓ (+D-06); P4-11 → D-06; P4-12 ✓; P4-13 ✓; P4-14 → D-04 comment; P4-15 ✓; P4-16 ✓;
P4-17 → D-04 comment; P4-18 ✓; P4-19 ✓; P4-20 → D-05 comments; P4-21 ✓ (values D-01/D-02); P4-22 ✓.

## DECISION rows left open
D-01 (eq. ctrl-ades, §4.1.1, §4.3 ×2, §4.4, §4.6), D-02 (eq. ctrl-ades, eq. ctrl-tau-geo, §4.2, §4.3, §4.6), D-04 (§4.2
Role, §4.4), D-05 (Tab 4.1, §4.5, §4.7), D-06 (§4.3).

## [VERIFY]
§4.1.3 "single configuration file" vs per-strategy study variants; CDC 2010 stability regions (companion v4 only).
