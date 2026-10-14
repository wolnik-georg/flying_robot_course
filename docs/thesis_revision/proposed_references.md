# Proposed references and suspected bibliography errors

**Status of everything in §A: UNVERIFIED — the author must check each primary source before adding it.** None of these
works is among the 18 held PDFs, none is cited in the revised chapters, and `docs/references.bib` was not touched.
Bibliographic details come from the reference lists of held papers (named "via R0x", records in `stage1_sources.md`) or,
where marked *memory*, from general knowledge. A reference list is secondary evidence: it shows the work exists as
cited by others, not that it says what the thesis would use it for.

## A. Proposed additions (UNVERIFIED)

| # | Work (as given in the secondary source) | Via | Where it would go | What it would support | Priority |
|---|---|---|---|---|---|
| A1 | M. Zaheer, S. Kottur, S. Ravanbakhsh, B. Póczos, R. Salakhutdinov, A. Smola, "Deep sets", NeurIPS 2017, pp. 3391–3401 | R01, R02, R03 | Ch3 §3.4.1 (eq. model-deepsets); Ch2 §2.4 | the sum-decomposition representation of permutation-invariant functions (until then: Shi et al. Lemma 1 / NS2 Thm 2, held) | high |
| A2 | D. Mellinger, V. Kumar, "Minimum snap trajectory generation and control for quadrotors", ICRA 2011, pp. 2520–2525 | R05, R06, R16, R18 | Ch2 §2.1 flatness ¶; Ch4 §4.1.1 $R_d$ | flat outputs $[p,\psi]$; desired-attitude construction | high |
| A3 | S. Sun, A. Romero, P. Foehn, E. Kaufmann, D. Scaramuzza, "A comparative study of nonlinear MPC and differential-flatness-based control for quadrotor agile flight", IEEE T-RO 38(6):3357–3373, 2022 | R06, R07, R16 | Ch2 §2.8 "Comparisons that do exist" | a same-hardware controller comparison (single vehicle, no interaction force) whose methodology is close to this thesis | high |
| A4 | K. P. Jain, T. Fortmuller, J. Byun, S. A. Mäkiharju, M. W. Mueller, "Modeling of aerodynamic disturbances for proximity flight of multirotors", ICUAS 2019, pp. 1261–1269 | R03, R05, R08, R09, R11 | Ch2 §2.2 | the physics model underlying Chee/Hsieh's reduced-order model and ProxFly's model-based compensator | high |
| A5 | J. A. Preiss, W. Hönig, N. Ayanian, G. S. Sukhatme, "Downwash-aware trajectory planning for large quadrotor teams", IROS 2017, pp. 250–257 | R11 | Ch1 §1.1, Ch2 intro | origin of the ellipsoidal separation (60 cm vertical, 24 cm horizontal per Kiran et al.) | medium |
| A6 | W. Hönig, J. A. Preiss, T. K. S. Kumar, G. S. Sukhatme, N. Ayanian, "Trajectory planning for quadrotor swarms", IEEE T-RO 34(4):856–869, 2018 | R01, R02, R16 | Ch1 §1.1, Ch2 intro | the planning work from which Shi et al. take the 60 cm figure (cite as "cited by Shi et al." until checked) | medium |
| A7 | J. A. Preiss, W. Hönig, G. S. Sukhatme, N. Ayanian, "Crazyswarm: a large nano-quadcopter swarm", ICRA 2017, pp. 3299–3304 | R01, R02, R07, R08 | Ch5 §5.1 | the software stack the trajectory interface belongs to | medium |
| A8 | E. J. J. Smeur, Q. Chu, G. C. H. E. de Croon, "Adaptive incremental nonlinear dynamic inversion for attitude control of micro air vehicles", J. Guid. Control Dyn. 39(3):450–461, 2016 | R07, R13, R14 | Ch2 §2.3 | attitude INDI on micro air vehicles | medium |
| A9 | S. Sieberling, Q. P. Chu, J. A. Mulder, "Robust flight control using incremental nonlinear dynamic inversion and angular acceleration prediction", J. Guid. Control Dyn. 33(6):1732–1742, 2010 | R13 | Ch2 §2.3 | origin of INDI | medium |
| A10 | M. Fliess, J. Lévine, P. Martin, P. Rouchon, "Flatness and defect of non-linear systems: introductory theory and examples", Int. J. Control 61(6):1327–1361, 1995 | R05, R06, R13 | Ch2 §2.1 | definition of differential flatness | medium |
| A11 | M. Faessler, A. Franchi, D. Scaramuzza, "Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories", RA-L 3(2):620–626, 2018 | R05, R06, R07, R13 | Ch4 §4.1.1 | the frame construction the flight code names after this paper | medium |
| A12 | G. Shi, X. Shi, M. O'Connell, R. Yu, K. Azizzadenesheli, A. Anandkumar, Y. Yue, S.-J. Chung, "Neural Lander: stable drone landing control using learned dynamics", ICRA 2019, pp. 9784–9790 | R01, R02, R05, R06, R08, R16, R18 | Ch2 §2.4 | origin of residual learning with spectral normalisation; ground effect | medium |
| A13 | M. O'Connell et al., "Neural-Fly enables rapid learning for agile flight in strong winds", Science Robotics 7(66), 2022 | R16, R18 | Ch2 §2.7 | learned + adaptive combination on one vehicle | medium |
| A14 | L. Bauersfeld, E. Kaufmann, P. Foehn, S. Sun, D. Scaramuzza, "NeuroBEM: hybrid aerodynamic quadrotor model", RSS 2021 | R07, R12, R18 | Ch2 §2.4 | physics + learned single-vehicle residual model | low |
| A15 | A. Spitzer, N. Michael, "Feedback linearization for quadrotors with a learned acceleration error model", ICRA 2021, pp. 6042–6048 | R05 | Ch2 §2.5 FBL ¶; Ch4 §4.5 | FBL with a learned residual, single vehicle | medium |
| A16 | K. Y. Chee, T. Z. Jiahao, M. A. Hsieh, "KNODE-MPC: a knowledge-based data-driven predictive control framework for aerial robots", RA-L 7(2):2819–2826, 2022 | R05, R06, R09, R10 | Ch2 §2.5 MPC ¶ | the KNODE method used by Chee et al. 2024 and Hsieh et al. | low |
| A17 | G. Torrente, E. Kaufmann, P. Föhn, D. Scaramuzza, "Data-driven MPC for quadrotors", RA-L 6(2):3769–3776, 2021 | R06 | Ch2 §2.5 MPC ¶ | learned residual inside MPC, single vehicle | low |
| A18 | Z. Wu et al., "L1 Quad: L1 adaptive augmentation of geometric control for agile quadrotors with performance guarantees", arXiv:2302.07208 | R10 | Ch2 §2.3 footnote; §2.7 | $\mathcal L_1$ online estimate on a geometric controller (the reactive family outside INDI) | medium |
| A19 | D. Hanover, P. Foehn, S. Sun, E. Kaufmann, D. Scaramuzza, "Performance, precision, and payloads: adaptive nonlinear MPC for quadrotors", RA-L 7(2):690–697 | R10 | Ch2 §2.5 | adaptive MPC | low |
| A20 | P. Bartlett, D. J. Foster, M. Telgarsky, "Spectrally-normalized margin bounds for neural networks", NeurIPS 2017 | R01, R02 | Ch2 §2.4 SN sentence | Lipschitz/SN generalisation bounds | low |
| A21 | T. Miyato, T. Kataoka, M. Koyama, Y. Yoshida, "Spectral normalization for generative adversarial networks", ICLR 2018 | R16 | Ch2 §2.4 SN sentence | the SN technique | low |
| A22 | I. C. Cheeseman, W. E. Bennett, "The effect of ground on a helicopter rotor in forward flight", ARC R&M, 1955 | R01, R02 | Ch5 §5.2 (floor) / C5 | classical ground-effect model | low |
| A23 | X. Kan et al., "Analysis of ground effect for small-scale UAVs in forward flight", RA-L 4(4):3860–3867, 2019 | R01, R02 | Ch5 §5.2 / C5 | ground effect on small vehicles | low |
| A24 | D. W. Carter, R. Bouchard, D. B. Quinn, "Influence of the ground, ceiling, and sidewall on micro-quadrotors", AIAA J. 59(4):1398–1405, 2021 | R03, R11, R16 | Ch5 §5.2 ("within 10 cm of a wall") | wall and ceiling effects | medium |
| A25 | R. Mahony, V. Kumar, P. Corke, "Multirotor aerial vehicles: modeling, estimation, and control of quadrotor", IEEE RAM 19(3):20–32, 2012 | R12 | Ch2 §2.1 | tutorial background for the student reader | low |
| A26 | J.-J. E. Slotine, W. Li, *Applied Nonlinear Control*, Prentice Hall, 1991 | R13 | Ch2 §2.3 / Ch4 §4.5 | NDI and feedback linearisation | low |
| A27 | H. K. Khalil, *Nonlinear Systems*, 3rd ed., Prentice Hall, 2002 | R01 | Ch2 §2.1 / Ch4 §4.2 | exponential stability, attractiveness | low |
| A28 | Busetto et al., arXiv:2512.14450 (*memory/code comment only*: named in `lib.rs:230` as the source of the brushless inertia constants) | code comment | Ch3 §3.1 / Ch5 §5.1 | inertia of the Crazyflie 2.1 Brushless — title, authors and content all `[VERIFY]` | medium |
| A29 | A. Kiran, … N. Ayanian, K. Breuer, "Downwash dynamics: impact of quadrotor separation on forces, moments, and velocities for dense formation flight", AIAA Aviation 2024, p. 4145 | R05, R09 | Ch2 §2.2 | earlier rig study by the same group | low |
| A30 | Martini et al., "A benchmark framework for testing, evaluation, and comparison of quadrotor linear and nonlinear controllers", ICUAS 2023, pp. 471–478 | R18 | Ch2 §2.8 | controller-benchmark methodology | low |
| A31 | Bitcraze, Crazyflie 2.1 Brushless data sheet (*memory*) | — | Ch5 §5.1 (9 cm, 41 g) and the radio packet-rate statement (§5.4.1) | platform dimensions and radio specification | high (resolves two `[VERIFY]`s) |

## B. Suspected errors in `docs/references.bib` (not edited)

| Key | Issue | Evidence | Suggested correction (check the primary record) |
|---|---|---|---|
| `shi2022neuralswarm2` | `@misc`, `year = 2020` (arXiv v1) while the key, the `yearnote` and the cited content are the T-RO version; `plain` prints 2020 | held PDF is arXiv v2 (2021) in T-RO layout (R02 §2); R05's reference list gives T-RO 38(2):1063–1079 | `@article`, IEEE T-RO 38(2):1063–1079, 2022 (R05 gives "2021" — verify) |
| `cobobriesewitz2026lindi` | `year = 2025` (arXiv v1); key and `yearnote` say L4DC 2026 | held PDF header: PMLR, 8th L4DC, 2026 (R07 §2) | `@inproceedings`, L4DC 2026, PMLR |
| `tal2018indi` | `year = 2018` points to the CDC 2018 paper; the content the thesis cites (response analysis, quaternion form, 12.9 m/s) is from the extended version held | held arXiv v2 states "A preliminary version … CDC 2018" (R13 §2); R07 gives IEEE T-CST 29(3):1203–1218, 2021 | cite the T-CST 2021 version (new key or corrected entry); keep the CDC entry only if the CDC paper is also cited |
| `lee2010geometricse3` | the `note` describes checks against arXiv v1 and reserves v4-only content; the held PDF is v4 | R15 §2 | update the note to "v4 held; equation checks against v4" |
| `lee2010geometrictracking` | `note` cites "Prop. 2 / Prop. 3" for the stability regions; v4 numbers them Propositions 3 and 4; CDC not held | R15 §2, R19 | verify numbering against the CDC paper |
| `gu2025comparative` | `note` says "full text not held (paywalled)"; the open-access PDF is held | R18 §2 | remove the note |
| `bauersfeld2024airflow` | `year = 2024`; RA-L volume 10 is a 2025 volume | R12 §2; R05 gives 10(2):1241–1248 with 2024, R11 with 2025 | verify year against IEEE Xplore |
| `yang2025flatnessresiduals` | `@misc` arXiv; R05 gives IEEE CDC 2025, pp. 3035–3042 | R06 §2 | verify and change to `@inproceedings` if the CDC version is meant |
| `li2023ndpnmpc` | arXiv only; R08 gives IEEE CDC 2023, pp. 2122–2128 | R16 §2 | optional venue |
| `shankar2023docking` | arXiv only; R09 gives ISER 2023, Springer, pp. 427–437 | R17 §2 | optional venue |
| `shi2020neuralswarm` | arXiv only; published ICRA 2020 (*memory*) | R01 §2 | optional venue |
