# 2026-10-15 tree validation — coverage and first numbers (cf5 card only; cf_second card and merges pending)

Strategy of every flight is read from the log itself (`experiments/analysis/tree_log_check.py`: INDI bit from `a_rpm_f`, NS2 from the thrust residual vs `rnn.pred`). No `--study` gate was possible (readback returned NaN for all 83 parameters, see next_session_plan.md), so the strategy is NOT in the radio meta.

| uSD | radio log | what | INDI (bit 0) | NS2 | note |
|---|---|---|---|---|---|
| thesis00 | hover 17:16 | S0 solo hover | off | off | z err −0.2 ± 0.7 cm |
| thesis01 | (none) | S0 solo figure-8 | off | off | xy rms 10 cm |
| thesis02 | A8 17:31 | S0 A8 | off | off | crossing dip −15.9 cm |
| thesis03 | A8 17:35 | S0 A8 | off | off | −15.4 cm; cf_second radio log only 0.7 s (radio logging, not uSD) |
| thesis04 | A1 17:37 | **A1, strategy unclear, FAILED** | off | unclear | pitch oscillation ±30–40° (period ≈ 3 s), z stuck 0.2–0.36 m, never reached the 0.5 m target |
| thesis05 | hover 17:41 | S1 solo hover | on | off | clean INDI takeoff, z +1.1 ± 1.7 cm |
| thesis06–08 | A8 17:43/44/46 | S1 A8 ×3 | on | off | dips −11.1 / −11.6 / −9.2 cm |
| thesis09 | A1 17:48 | S1 A1 | on | off | z −0.8 ± 2.0 cm |
| (17:53 A8 aborted, no uSD) | | | | | |
| thesis10, 11 | A8 17:54/17:56 | S3 A8 ×2 | on | **on** (coefficient +0.95 / +0.96…+1.00) | dips −1.7 / −2.1 cm, upward peak up to +5.7 cm |
| thesis12 | A1 17:57 | S3 A1 | on | **on** (+1.00) | z +3.5 ± 4.7 cm |

**Coverage: S0 geometric: hover, figure-8, A8 ×2, A1 MISSING. S1: complete. S3: complete. S2 (geometric + NS2): NO valid flight** (the two A8 flights after the S2 yaml push classify as NS2 off; the A1 is the failed flight above).

Preliminary crossing dips (z error minimum within ±0.8 s of the crossing, relative to the steady offset outside crossings): S0 −15.7 cm (n = 2), S1 −10.6 cm (n = 3), S3 −1.9 cm (n = 2). S3 offset +0.3/+0.4 cm against +2.2…+2.7 cm for S0/S1.
Sign check of the C implementation on hardware: with NS2 on the thrust residual follows −rnn.pred with gain +1.00 (S3), 0 with it off.
