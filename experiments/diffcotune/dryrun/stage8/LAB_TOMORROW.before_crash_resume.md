# LAB_TOMORROW (Stage 8b, written 16 Nov 2026 00:09 from the result files; the user picks and commits)

Lab variant (decided by the user, 15 Nov evening): **"ours", frozen, the same for S0–S3** = DiffCoTune Split-Alternate loop (Krishna et al. 2025) with DiffTune's quadrotor settings (Cheng et al. 2024); 5 tuning scenarios (A1, A8, A4 cell 1, CIRCLE, FIG8), test A4 cell 2 (held out) and A4 cell 1 (in-sample), 6 rounds, keep-best, position cost, J_sysId position + velocity + attitude, 50 + 50 epochs best iterate, lr 0.01, ±25 % per round, noise 2 seeds, forward gradient, RPM delay fixed (S1 / S3). The paper variant is shown for comparison only (not adopted).

## S0

| | "ours" (lab variant) | paper (not adopted) |
|---|---|---|
| real 3-D cost per round (cm, mean of 5 flights) | 18.24 → 9.69  (2 of 6 rounds) | 18.24 → 17.24 → 22.09 → 24.33 → 24.63 → 30.81 |
| criteria a / b / c | not yet / yes / yes | no / no / no |
| crashes per round | 0 0 | 0 0 0 0 0 0 |
| floor flights per round (A1 is a floor flight at the default gains in the dry-run plant) | 1 0 | 1 1 1 1 1 3 |
| largest gain change to the next flown set |  | 14 %, 46 %, 6 %, 3 %, 4 % |
| means (cm, evaluation seeds 2000–2004) | tuning 14.04 → 7.39; held-out A4 cell 2 13.05 → 10.80; in-sample A4 cell 1 8.80 → 7.27 (θ_0 → θ_best, round None) | tuning 14.04 → 14.11; held-out A4 cell 2 13.05 → 13.75; in-sample A4 cell 1 8.80 → 10.06 (θ_0 → θ_best, round 1) |
| desk minutes per round (round 0 … 4) | 40 38 | 10 10 3 2 3 |
| round test (round.py round 0 against the dry run) | θ₁ difference 0.0e+00, β difference 0.0e+00, 46.3 min | |
| gradient check at the fitted β of every round | not yet | |
| safety preview of each round's θ (replay) | round 1 PASSED (real 9.7 cm); round 2 FAILED (real 9.1 cm); round 3 PASSED (real 16.0 cm) | |

**S0 lab-ready ("ours"): not yet**

## S2

| | "ours" (lab variant) | paper (not adopted) |
|---|---|---|
| real 3-D cost per round (cm, mean of 5 flights) | 14.38 → 8.09  (2 of 6 rounds) | 14.38 → 14.52 → 15.03 → 17.54 → 20.54 → 19.11 |
| criteria a / b / c | not yet / yes / not yet | no / yes / no |
| crashes per round | 0 0 | 0 0 0 0 0 0 |
| floor flights per round (A1 is a floor flight at the default gains in the dry-run plant) | 1 0 | 1 1 1 1 1 1 |
| largest gain change to the next flown set |  | 2 %, 7 %, 34 %, 6 %, 1 % |
| means (cm, evaluation seeds 2000–2004) | not yet | tuning 11.25 → 11.25; held-out A4 cell 2 8.71 → 8.71; in-sample A4 cell 1 6.81 → 6.81 (θ_0 → θ_best, round 0) |
| desk minutes per round (round 0 … 4) | 32 27 | 7 5 9 2 2 |
| round test (round.py round 0 against the dry run) | not yet | |
| gradient check at the fitted β of every round | not yet | |

**S2 lab-ready ("ours"): not yet**

## S1

| | "ours" (lab variant) | paper (not adopted) |
|---|---|---|
| real 3-D cost per round (cm, mean of 5 flights) | not yet | 9.54 → 6.49 → 7.87 → 8.14 → 8.18 → 8.29 |
| criteria a / b / c | not yet | no / yes / yes |
| crashes per round | not yet | 0 0 0 0 0 0 |
| floor flights per round (A1 is a floor flight at the default gains in the dry-run plant) | not yet | 0 0 0 0 0 0 |
| largest gain change to the next flown set | not yet | 57 %, 30 %, 6 %, 1 %, 1 % |
| means (cm, evaluation seeds 2000–2004) | not yet | tuning 8.30 → 5.48; held-out A4 cell 2 7.38 → 4.65; in-sample A4 cell 1 5.33 → 3.62 (θ_0 → θ_best, round 1) |
| desk minutes per round (round 0 … 4) | not yet | 36 20 3 2 2 |
| round test (round.py round 0 against the dry run) | not yet | |
| gradient check at the fitted β of every round | not yet | |

**S1 lab-ready ("ours"): not yet**

## S3

| | "ours" (lab variant) | paper (not adopted) |
|---|---|---|
| real 3-D cost per round (cm, mean of 5 flights) | not yet | 10.07  (1 of 6 rounds, halted: a round took more than 45 min) |
| criteria a / b / c | not yet | not yet / not yet / no |
| crashes per round | not yet | 0 |
| floor flights per round (A1 is a floor flight at the default gains in the dry-run plant) | not yet | 0 |
| largest gain change to the next flown set | not yet |  |
| means (cm, evaluation seeds 2000–2004) | not yet | tuning 8.38 → 8.38; held-out A4 cell 2 7.02 → 7.02; in-sample A4 cell 1 5.82 → 5.82 (θ_0 → θ_best, round 0) |
| desk minutes per round (round 0 … 4) | not yet | 46 |
| round test (round.py round 0 against the dry run) | not yet | |
| gradient check at the fitted β of every round | not yet | |

**S3 lab-ready ("ours"): not yet**

## Session 1 (S0 + S2 interleaved)

Cards: `lab/INTERLEAVED_LAB_CARD.md` (+ `lab/S0_LAB_CARD.md` items per flight). Command per round: `python3 lab/round.py --controller <C> --round <l> --logs <folder> --method ours` (default method is ours), then read `lab/<C>/round_<l+1>/PREVIEW.txt`; on `PREVIEW FAILED` do not fly the set without asking; default: fly the best flown set again. Preflight: `python3 lab/preflight.py --controller <C> --yaml <lab crazyflies.yaml>` must print PREFLIGHT PASSED for S0 and S2 (results: see `logs/pre_S0.log`, `logs/pre_S2.log`). Estimated hours (Stage 6 timeline model, now 6 rounds × 5 flights): see `lab/results/session_timeline.md` (not yet rerun with 6 rounds if missing). uSD cards: empty both before, archive once after round 2 of both controllers, never ≥ 85 files; check on the first flight that it is one file per flight.

## Open items (who decides)

* Which PC runs `round.py` in the lab (user); whether two desk steps at once are acceptable there (the desk PC runs out of memory with two "ours" pools: a pool worker grows to about 2.6 GB over two hours; one "ours" desk step at a time is safe).
* Flights per battery pack, uSD files per flight and card (assumed 1), the strategy-switch overhead (measured in the first block): not stated.
* The preview rule (a check that never changes the gains): the abort rules are the protocol's; the "NOTE worse" thresholds (1.5 × and 2 cm) are a proposal. Replay results: `dryrun/stage8/report_stage8.md` §6.
* Supervisor agreement: the user.
