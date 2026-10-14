# Stage 5 — Cross-chapter consistency and citation re-audit

Scope: the nine revised chapters in `revised/` read as one document, built with `revised/thesis_draft.tex` and the
attached preamble. **Build: clean** — no LaTeX errors, no undefined references or citations, no multiply defined labels,
no BibTeX warnings, 60 pages.

## 1. Facts that must agree across chapters

| Fact | Ch1 | Ch2 | Ch3 | Ch4 | Ch5 | Ch6–9 | Status |
|---|---|---|---|---|---|---|---|
| Control loop rate 1 kHz | — | — | §3.3 (1000 Hz) | §4.3/4.4 | §5.1, §5.8 | — | ✓ |
| Dataset 500 Hz, 96 variables | — | — | §3.3 (500 Hz) | — | §5.1, §5.4.1, §5.8 | Ch6 §6.1 (500 Hz, inside D-05 ¶) | ✓ |
| Network evaluated ≈100 Hz, output held | — | — | §3.4 | §4.4 (×2) | §5.1 | — | ✓ |
| Network: NS2 widths, H = 20, scalar vertical output, 19 297 stored / 11 461 evaluated | §1.4 S2 "vertical component" | — | §3.4.3 | §4.4 | — | — | ✓ |
| Input gate 0.2 m lateral, output bound | §1.2 "can bound the output" | — | §3.4.4 | §4.4 | §5.3.2 | Ch6 §6.4 (gate in figure spec) | ✓ |
| Weights compiled into flight image; upload path exists | §1.6 | — | §3.4.3 | — | — | Ch8 RQ4 (generic); Ch9 §9.4 (D-05) | ✓ |
| Residual definition $a_{\mathrm{res}} = a_{\mathrm{meas}} - a_{\mathrm{model}}$, thrust from measured rotor speeds | §1.2 | §2.3 mechanism ¶ | eq. model-ares | §4.3 | §5.1 rotor-speed source | — | ✓ |
| Rotor-speed source: DShot telemetry default, optical deck alternative | — | §2.3 (Cobo PWM evidence) | §3.3 | §4.3 | §5.1 | Ch8 RQ4 | ✓ |
| Residual **subtracted** (derivation) vs configured sign | — | §2.3 (INDI increment) | §3.3.2 + **D-01** | eq. ctrl-ades + **D-01** | §5.5.1 gate + **D-01** | Ch6 §6.5 + **D-01** | **open (D-01)** — every place that depends on the sign carries the comment |
| Integral terms (vertical only in code; full in equations) | — | — | — | eqs. + **D-02** | — | — | **open (D-02)** |
| Frame: world $z$ up, thrust $+fRe_3$; Lee's $z$-down mapped | — | eq. rw-trans | eq. model-trans | §4.2 frame mapping | — | — | ✓ |
| $\Psi = \tfrac12\mathrm{tr}(I-R_d^\top R) = 1-\cos\theta$; regions 90°/180° | — | §2.1 ¶Rotations | — | §4.2 | — | — | ✓ (identical formula; CDC `[VERIFY]` in both) |
| Force residual compensated, moment left to attitude loop | Tab 1.1 | §2.5 (Hsieh 2026 same choice) | §3.2 | §4.1 | — | Ch8 new §disc-moment | ✓ |
| Compared set S0–S3; S3 not implemented; no strategy > 3 | §1.4 + **D-05** | §2.1, §2.9 | — | §4.5 (status D-05) | — | Ch6 hybrid section removed; Ch6 Tab 6.3 S3 row D-05; Ch7 table S0–S2; Ch9 §9.1 "specified but not implemented" | ✓ with **D-05** open (Ch1 "Four … are compared", Ch6 row) |
| Only Strategy 1 reported as INDI; diagnostic configurations are not strategies | — | — | — | §4.3 | — | Ch6 §6.2, §6.5 | ✓ |
| Fairness: each strategy tuned for itself, then frozen | §1.6 contribution 1 | §2.8 (Gu) | — | §4.6 | §5.5.1–5.5.3 | Ch8 §disc-frozen | ✓ |
| Prediction-off / prediction-on flights | — | — | §3.1 (prediction checkable) | §4.4 | §5.5.3 | Ch6 §6.4, Ch7 §7.2 | ✓ |
| Transfer penalty definition (RQ3) | §1.5 RQ3 | §2.9 third gap | §3.4.2 | — | §5.5.4 | Ch7 §7.2 | ✓ |
| Data-collection rationale ("residual that survived compensation") | — | — | §3.3.1 (contradicts it, correctly) | §4.2 Role, §4.4 + **D-04** | §5.5.2 + **D-04**; §5.0 status ¶ (inside D-05) | — | **open (D-04)** |
| Hybrid on teams: single vehicle with INDI; team only inside off-board MPC | §1.2 ¶4, §1.3 ¶3 | §2.5, §2.7, §2.9 | — | — | — | — | ✓ (minimal corrections applied in both chapters) |
| Main gap statement | italic gap + **D-03** | §2.8, §2.9 + **D-03** | — | — | — | — | **open (D-03)** — Ch1 and Ch2 carry the same unchanged text and the same comment, so they stay consistent with each other whichever option is chosen |
| Smith numbers: 36 % 3-D; 56 % vertical from the abstract | §1.1 "as one abstract states them" | §2.4 + footnote, §2.8 | — | — | — | — | ✓ |
| NS2: threefold (abstract) and 16 robots/24 cm (intro, Fig. 1) separate | §1.1 (16, 24 cm only) | §2.4, Tab 2.1 † | — | §4.4 | — | — | ✓ |
| Crazyflie 2.0 9 cm (Shi) vs Brushless 9 cm | §1.1 (CF 2.0) | §2.0 (CF 2.0) | — | — | §5.1 **[VERIFY]** | — | ✓ (Brushless value flagged) |
| Ch6–9 referred to by name in prose (H6) | §1.8 | §2.4 | ✓ | ✓ | ✓ | Ch8, Ch9 | ✓ (remaining `\ref{ch:results…}` only inside `%` comments) |

## 2. Terminology

| Term | Use | Status |
|---|---|---|
| reactive | measured residual (INDI, and an $\mathcal L_1$ online estimate); Chee et al.'s different usage footnoted in §2.3 | ✓ |
| predictive | learned or physics-based prediction from relative state (Ch1 softened; Ch2 Bauersfeld FF demo) | ✓ |
| hybrid | measurement + prediction; never a compared strategy | ✓ |
| Strategy 0–3 | used consistently in prose; the "S0–S3" abbreviations in the Ch6 tables replaced by "Strategy 0–3" | ✓ |
| Hsieh et al. | two papers by overlapping authors (2025 MPC/$\mathcal L_1$; 2026 flatness/FBL). Every occurrence in Ch2–Ch4 was checked and carries the right key. **Suggestion (style, not applied):** add the year in prose, "Hsieh et al.\ (2025)", where both appear in one section (Ch2 §2.5, §2.9; Ch3 §3.4.2). | ✓ / style |
| Neural-Swarm vs Neural-Swarm2 | kept apart in every changed sentence (Ch2 §2.4, Ch3 §3.4, Ch4 §4.4) | ✓ |

## 3. Citation re-audit (whole document)

- **Keys:** 20 keys cited, 20 entries in `references.bib`; every key is used, none is missing. Citations per chapter: Ch1 18,
  Ch2 79, Ch3 27, Ch4 27, Ch5 3, Ch6–9 0.
- **No forbidden sources:** no `\cite` of, or prose reference to, `docs/*.md` or `papers/summaries/` as evidence. (File
  names that remain in the text are only inside D-05 status paragraphs, which the author is asked to delete or rewrite.)
- **Primary sources:** Lee CDC 2010 cited as primary (`lee2010geometrictracking`) with the companion (`lee2010geometricse3`,
  v4 held) for equations; Tal & Karaman as primary INDI, Smeur for the cascade.
- **Every new or changed prior-work sentence** (Ch1 ×5, Ch2 ×31, Ch3 and Ch4 as in their pass files, Ch5 ×3) is
  traced to a record in `stage1_sources.md` in the chapter pass files; no row failed.
- **Unverifiable (no PDF):** `abro2025fractional` (Ch1 §1.2, Ch2 §2.8, `% VERIFY` present), `lee2010geometrictracking`
  (stability regions, numbering; `% VERIFY` present).
- **Suspected bibliography errors** (not edited; listed in `proposed_references.md` §B): shi2022neuralswarm2 year,
  cobobriesewitz2026lindi year, tal2018indi version, lee2010geometricse3 version note, gu2025comparative stale note,
  bauersfeld2024airflow year/volume.

## 4. H1 scan (results statements)

Search over all revised chapters for flight/lab/simulation/replay findings (phrases such as "have been executed",
"indicative", "established in simulation", "SIL", "replay", "we found"). Remaining hits:

| Location | Text | Handling |
|---|---|---|
| Ch5 campaign-status ¶ | "Two-robot flights … have been executed …", "Indicative Strategy 0 vs 1 comparisons exist" | **D-05** (unchanged by rule; recommended: delete) |
| Ch6 §6.1 | "Indicative single flights and partial C.1 collection exist" | **D-05** |
| Ch9 §9.4 | "SIL and desk parity do not substitute for `rnn.en=1` flight logs …" | **D-05** |
| Ch4 L473 | "That theory is established in simulation on a planar quadrotor" | literature (Yang et al.), not a project result ✓ |

All other project findings found in Phase 0 were removed (`% REMOVED (H1)` comments keep the original text).

## 5. Labels and references

No label of the original draft was removed. `sec:res2-hybrid` is kept as an anchor after the removed hybrid section
(no reference to it exists). New labels: `sec:setup-stats`, `sec:res2-attribution`, `sec:disc-assumptions`,
`sec:disc-moment`, `sec:disc-frozen` (plus those listed in the Ch3/Ch4 pass files). All open `\todo`s of the draft are
kept; the two provenance todos carry D-03 comments.

## 6. Open inconsistencies (all owned by a DECISION row)

1. **D-01** — derivation says subtract; Strategy 1 is configured to add. Affects Ch3 §3.3.2, Ch4 eq. ctrl-ades and §4.3,
   Ch5 §5.5.1, Ch6 §6.5.
2. **D-02** — equations show full position and attitude integrals; the code has a vertical integral only (S0/S2 on,
   S1 off). Affects Ch4 eqs. ctrl-ades, ctrl-tau-geo, §4.6.
3. **D-03** — main gap and contribution 1 vs Hsieh et al. 2025. Affects Ch1 §1.2, §1.3, §1.6; Ch2 §2.8, §2.9; both todos.
4. **D-04** — "residual that survived compensation" in Ch4 and Ch5 contradicts Ch3 §3.3.1 and Shi et al.
5. **D-05** — lab-status passages and Strategy 3 status (Ch1, Ch4, Ch5, Ch6, Ch7, Ch8, Ch9).
6. **D-06** — filter realised cutoff (Ch3, Ch4).
7. **D-09** — hypotheses/falsifiers and the RQ4 data-size measurement (Ch5, Ch6).
