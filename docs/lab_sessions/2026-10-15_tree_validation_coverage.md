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

## Merged two-vehicle analysis (both cards, 2026-10-15 evening)
Pairing by `usd.runTag` (all 10 flights matched), alignment against each vehicle's own commanded trajectory (bottom RMS 3.7–6.1 cm, top 1.7–3.1 cm; the failed A1: bottom 22 cm, merged with `--tag-quality-only`). Reproduce: `experiments/analysis/tree_validation_merge.py`, numbers: `experiments/analysis/tree_validation_analysis.py`.

**The top vehicle ran S0 (INDI off, NS2 off) in every one of the 10 flights** (`tree_log_check.py` on all cf_second logs), z std 0.3–0.9 cm.

| Strategy | A8 flights | crossing dip (cm) | worst dip | upward peak mean / max | steady offset | z std (A8) | separation error at crossing |
|---|---|---|---|---|---|---|---|
| S0 geometric | 2 | **−15.7** | −20.0 | −0.5 / +1.7 | +2.3 cm | 5.2 cm | −0.4 cm |
| S1 INDI | 3 | **−10.4** | −12.5 | −0.7 / +0.3 | +2.4 cm | 3.6 cm | −0.5 cm |
| S3 INDI + NS2 | 2 | **−1.9** | −5.2 | +1.6 / **+5.7** | +0.4 cm | 1.7 cm | −0.9 cm |
| S2 geometric + NS2 | — | not flown validly | | | | | |

A1 (stack hold, dz 0.5): S1 z error −0.8 ± 2.0 cm; S3 **+3.2 ± 4.5 cm**, separation error −3.3 ± 4.8 cm (S1 +0.8 ± 2.3): the plain-sum hybrid sits higher and bobs more in the stack (n = 1 each). The failed A1 (strategy unclear): z −17.5 cm, separation +17 cm, pitch up to 41°.
uSD write gaps over all 18 two-vehicle logs: mean 1.3 % of samples, worst 3.8 %, longest 194 ms (10-13: 0.03–0.19 %); not blocking for these metrics, but worse than before (cards are full of archives; reformat after backup is an option).

**What is still missing:** S2 A1 + A8 ×2, S0 A1 (and an explanation of the failed A1 at 17:37:45).

## Update after the S2 block and both cards (2026-10-15 night)
**S2 (geometric + NS2) flown: A8 ×3 (18:09, 18:14, 18:16) and A1 ×1 (18:18).** NS2 is active in all of them (thrust residual follows the prediction with gain +0.85…+0.97).

**Outliers excluded from the strategy comparison (battery / logging, from the radio logs):**
| flight | why |
|---|---|
| A8 17:53:07 (S3 attempt) | aborted after 7.5 s, cf5 z up to 1.74 m (runaway), no uSD |
| A8 18:09:04 (S2) | cf_second battery 2.63 V, motors cut in the air at z = 1.0 m after 30 s; its uSD file is 0 bytes; cf5 file exists (6.7 cm dip) but the separation is invalid after the cut |
| A8 18:11:24 (S2) | cf5 battery sagged to 2.69 V; cf5 uSD file 0 bytes (lost), radio log complete |
| A8 18:12:47 (S2) | cf5 stopped after 1.2 s (z 0.18 m, no liftoff), cf_second flew alone for 43 s; cf5 uSD 0 bytes |
Radio-only dropouts (uSD fine, not outliers): A8 17:35:53 and 18:14:47 (cf_second radio log truncated), A8 17:56:18 (cf5 radio 0 rows).

**Valid two-vehicle flights (both uSD logs, merged): S0 A8 ×2, S1 A8 ×3 + A1, S2 A8 ×2 + A1, S3 A8 ×2 + A1.**
| Strategy | A8 dip (cm) | worst | upward peak mean/max | steady offset | z std | A1 (z error, sep error) |
|---|---|---|---|---|---|---|
| S0 geometric | −15.7 (n=2) | −20.0 | −0.5 / +1.7 | +2.3 | 5.2 | **fails** (17:37:45, see below) |
| S1 INDI | −10.4 (n=3) | −12.5 | −0.7 / +0.3 | +2.4 | 3.6 | −0.8 ± 2.0 / +0.8 ± 2.3 |
| S2 geometric + NS2 | −5.5 (n=2) (+ cf5-only: −6.7) | −8.0 | +0.6 / +1.5 | +0.3 | 1.9 | **fails** (18:18:32): −13.5 ± 3.7 / +13.7 |
| S3 INDI + NS2 | −1.9 (n=2) | −5.2 | +1.6 / +5.7 | +0.4 | 1.7 | +3.2 ± 4.5 / −3.3 ± 4.8 |

**A1 finding:** both geometric strategies lose the stack hold at dz 0.5: pitch oscillates ±30–40° (period ≈ 3 s), z stays 0.2–0.4 m, never reaching 0.5; the INDI strategies hold it. The 17:37:45 A1 failure has the same signature as the S2 A1 (18:18:32) and is most likely the S0 A1 (the S0 block was A8, A8, A1; NS2 off in the log signature) — to be confirmed by the user. With the shared soft gains (Kp 7, Kv 4) the geometric controller does not hold a vehicle in the wash of an aligned neighbour at 0.5 m; NS2 (vertical force only) does not fix it. Design consequence for the study: A1 cannot be flown with S0/S2 at these shared gains; options in the lab plan.
