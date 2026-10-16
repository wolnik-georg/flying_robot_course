# Thesis change log — review feedback, draft-quality fixes only (final, 2026-10-05)

The thesis was reset to the last committed version, then only fixes that come from the draft text itself were re-applied.
Nothing from the last weeks' events or results is in the thesis (no status table, no Z-integral, no rates, no new sections,
no hypotheses/statistics text, no figures, no front matter). Meeting doc untouched. Build: 51 pages, no undefined references.
(Edited-but-dropped version kept outside the repo in the session scratchpad.)

## Applied (draft-quality)

| Finding | Change |
|---|---|
| P-002 / S-004 | Ch.1 gap statement split into short sentences; says the thesis compares the first two (hybrid out of scope). |
| P-003 | RQ4 and contribution 3 include tuning effort. |
| P-004 | 36 % and 31 % cited in Ch.1 (31 % "in the abstract"); 9 cm marked "vendor data sheet". |
| P-005 | Stale "Nothing has flown" comment in Ch.1 updated. |
| P-006 | Ch.2: unverifiable "sevenfold" replaced by "abstract and introduction state an order of magnitude" (PDF checked). |
| P-007 | Ch.2: Li et al. run onboard (TX2 NX); table entry corrected. |
| P-008 / S-008 | Table 2.1 re-laid out; no overflow. |
| P-010 | Comment marker: Abro et al. unverifiable (no PDF). |
| P-011 / S-013 | Ch.4 acceleration renamed `a_cmd`; Ch.3 `a_des` defined and linked. |
| P-012 / S-014 | `a_meas` comes from a filtered accelerometer signal, not "differentiated". |
| P-014 | "approximately twice" made conditional. |
| P-015 | 2 m cut-off: "chosen conservatively". |
| P-018 / S-021 | `omega_des` defined; `K_R`, `K_omega` split geo/INDI with units; symbol map to Lee. |
| P-020 | One sentence: each family has one implementation of our own. |
| P-031 | C2 caption/purpose corrected. |
| P-032 / S-031 | Six network inputs defined; four lateral components. |
| P-034 / P-040 (part) | Volume todo and script/docs names removed from Ch.5, 6, 7, 9 text. |
| P-036 | Ch.7 fallback no longer claims it works without a third vehicle. |
| P-037 / S-035 | `Chapter~\ref{sec:…}` → `Section~\ref{…}` in Ch.8, 9. |
| P-038 / S-034 | Ch.9: Strategies 0–2 implemented, Strategy 3 specified only. |
| S-002 / S-003 / S-006 / S-007 | INDI/FBL/NN expanded; short Ch.1 section on the four strategies; downwash, blade flapping, slung payload glossed; outline names Ch.6–9. |
| S-010 | ROM-FBL, FBL, NMPC expanded in Ch.2; Hsieh paragraph split into short paragraphs (S-009 partly). |
| S-012 | "Differencing" reworded to comparison with a no-neighbour residual. |
| S-015 / S-017 / S-019 / S-020 | Plain-words sentences for deep sets, INDI law, flatness step; STM32, specific force glossed. |
| S-018 (part) / S-023 / S-025 / S-026 | Table 4.1 columns explained; "Implementation status"; runtime bits mapped to strategies; "uncompensated" defined. |
| S-029 / S-032 / S-033 (part) | mocap, uSD, DShot defined; "trajectory ladder" reworded; "separation error, not the residual"; script name removed. |
| S-036 (part) | "Strategy 5/7" numbering removed from Ch.9. |

## Not applied (open) — would need last weeks' results/repo state, or a decision

P-001/S-001 (hypotheses: Ch.1 sentence reworded to promise none; no hypotheses written), P-016 (training target), P-017 (INDI variant), P-019 (Strategy 2 status),
P-021 (gain freeze vs Z-integral), P-023 (statistics plan), P-024, P-025/S-027 (campaign status), P-026, P-028, P-035 (hybrid slot
in Ch.6, kept as before), P-042, P-013 (moment residual "logged"), P-022/P-033 (time-alignment text), P-027, P-029, P-030/S-030
(scenario parameters), P-043/S-039 (figures), S-038 (abbreviation list), S-040 (title page/abstract), S-022, S-028, S-037 (part).
Skipped as nice-to-have: P-009, S-005, S-011, S-016, S-024; P-039.
