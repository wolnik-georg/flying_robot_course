# Stage 4b report (S0, DiffCoTune; 13 Nov 2026)

Task of this session: finish Stage 4b after the session kill of 14:52:53. Nothing flown, nothing flashed, no commit. Times are from `date` on 13 Nov 2026 (CET).

## 1. The session kill and what was rerun

- **What happened (from the previous prompt, checked in `dryrun/stage4b/logs/chain.log`):** at 14:50:17 two jobs started at the same time, each with its own worker pool: `lab/dry_test.py` (round 1 of the dry test, `strict=False`) and `lab/dry_round1_strict.py` (the same round with `strict=True`, started by `dryrun/stage4b/start_strict.sh`). One minute later MemAvailable was 1.2 GB (`chain.log`, 14:51:17). At 14:52:53 systemd-oomd killed the terminal scope (167 processes), including the agent.
- **Cause in the files:** `start_strict.sh` waited only for `lab/dry/round1_logs/flights.json` and then started the second pool while the first was still fitting; `dry_round1_strict.py` also waited in a loop for `dry_test.py` to write its result, i.e. it was built to run alongside.
- **Changed:** `start_strict.sh` now says "NOT USED" and ends with `exit 1`. `dry_round1_strict.py` no longer waits; it reads `results/dry_test.json` and must be started after `dry_test.py` has finished. `dry_test.py` has `--from-round 1` (keeps `lab/dry/`, takes round 0 from `lab/dry/S0/round_0/report.json`, runs only round 1 on the existing `round1_logs/` and `round_1/state.json`).
- **Rerun in this session (one chain, one pool at a time, through `dryrun/stage3c/job_runner.py`, `jobs_round1.json`):** (1) `dry_test.py --from-round 1`, then (2) `dry_round1_strict.py`. Round 0 of the dry test was not redone (it matches B1: gate of Stage 4a, identical θ₁ and β, max difference 0.0).
- **Resources:** MemAvailable 6.5 GB one minute after the start (limit 3 GB), load 2.3 (limit 22). Between 5.3 and 7.1 GB MemAvailable during the run while the desk jobs of items 3 and 4 ran in single processes.

## 2. Dry test, round 1

Gate (all from `lab/results/dry_test.json`, run 15:23–15:44, `strict=False`, the same way as Stage 3e):

| Check | Result | Required | Gate |
|---|---|---|---|
| θ₂ (φ of round 2) against `loop_P1.json` round 2 | max abs difference 0.0 | ≤ 1e-12 | yes |
| β after the round-1 fit against `loop_P1.json` | max abs difference 0.0 | ≤ 1e-12 | yes |
| real cost of round 1 | 9.713 cm (`loop_lib`: 9.711 cm) | 9.71 ± 0.01 cm | yes |
| keep-best state after round 1 | round 1, 9.713 cm (round 0: 19.988 cm); `round_2/state.json` carries keep = round 1 and its θ₁ | round 1 | yes |
| gain-limit hits | `[0,0,0,0,9,0,0,0,0,0]` = same as `loop_lib` | same | yes |
| yaml round trip `round_2/gains.yaml` | 8e-17 (round 1: 2e-16; round 0: 0) | ≤ 1e-12 | yes |
| float32 relative error of the gains | 4.0e-8 | not stated | listed |

Round 0 (reused): real 19.988 cm (`loop_lib` 19.988), θ₁ and β identical.

Desk time round 1: 1232 s (fit 608 s, tune 621 s, ingest 0.5 s); round 0 (earlier run) 1209 s. The flights of round 1 were not flown again (existing files).

Strict run (`strict=True`, copy `lab/dry_strict/`): the strict run (15:43:59–16:03:32, 1174 s: fit 585 s, tune 586 s) gives the same result: θ₂ identical to the strict=False run (max abs difference 0.0) and to `loop_P1.json` (0.0), real cost 9.713 cm, keep-best round 1, same gain-limit hits. In this round no flight is crashed or on the floor, so strict and non-strict use the same four flights; the strict path with a failed flight (repeat) is not exercised by this test.

## 3. Gyro and attitude

**Cause of the large gyro part (Stage 4b before the kill, `lab/results/gyro_check*.json`; unchanged here):** the log and the model are consistent in sign, axes and units (correlation of the logged gyro with the derivative of the logged attitude 0.96–0.997; power above 20 Hz 0.00–0.02). Real and model both oscillate at about 1.0–1.4 Hz with large body rates, and the phases are not related. So it is a dynamics gap, not an ingest error. J_w is not used in the fit (deviation 9).

**J_sysId per part on the 23 Oct flights, β₀ (default β), default gains** (`lab/results/gyro_check.json`, key `d_J_parts`; peer path = logged for A1 and A8, planned for A4, as `round.py --top auto`; the noise floor is an upper bound from the 0.1 s moving average in `ingest_oct23.json`):

| flight | scenario | J_p [m²] | J_v [(m/s)²] | J_q | J_w (not in fit) | floor p / v / q |
|---|---|---|---|---|---|---|
| 19-47-31 | A1 | 2.6e-02 | 1.2e+00 | 1.1e-01 | 29.8 | 7e-06 / 7e-04 / 2e-04 |
| 19-48-59 | A1 | 3.0e-02 | 1.3e+00 | 1.2e-01 | 30.9 | 8e-06 / 7e-04 / 2e-04 |
| 19-50-23 | A1 | 2.8e-02 | 1.2e+00 | 1.1e-01 | 28.8 | 7e-06 / 8e-04 / 2e-04 |
| 19-37-24 | A4 | 1.7e-02 | 1.8e-01 | 1.5e-02 | 5.3 | 2e-06 / 1e-04 / 2e-05 |
| 19-38-38 | A4 | 1.5e-02 | 2.1e-01 | 1.6e-02 | 5.5 | 2e-06 / 9e-05 / 2e-05 |
| 19-40-01 | A4 | 1.7e-02 | 1.5e-01 | 7.9e-03 | 2.2 | 8e-07 / 3e-05 / 7e-06 |
| 19-41-36 | A4 | 1.6e-02 | 1.1e-01 | 6.5e-03 | 1.8 | 5e-07 / 2e-05 / 5e-06 |
| 19-43-07 | A8 | 4.4e-03 | 2.9e-02 | 3.6e-03 | 2.1 | 7e-07 / 4e-05 / 1e-05 |
| 19-44-24 | A8 | 4.2e-03 | 2.0e-02 | 2.7e-03 | 1.7 | 6e-07 / 4e-05 / 1e-05 |
| 19-45-56 | A8 | 4.1e-03 | 2.7e-02 | 3.4e-03 | 1.8 | 5e-07 / 3e-05 / 9e-06 |

All three parts are two to four orders above the floor on every flight, A1 the most (J_v 1.2).

**Does the attitude part on A1 have the same phase problem?** Yes (`lab/attitude_check.py`, `lab/results/attitude_check_auto.json`; roll / pitch from est_q of the log and of the model over the scored part 0 < t ≤ 15 s of the model grid, best lag searched within ±0.5 s):

| A1 flight | tilt angle rms real / model | pitch rms real / model | roll rms real / model | pitch corr (0 lag / best) | roll corr (0 lag / best) | f dominant real / model |
|---|---|---|---|---|---|---|
| 19-47-31 | 26.7° / 26.8° | 25.6° / 18.4° | 8.1° / 19.7° | −0.02 / 0.05 | −0.01 / 0.05 | 1.27 / 1.07 Hz |
| 19-48-59 | 26.9° / 27.4° | 24.5° / 18.5° | 11.7° / 20.4° | 0.01 / 0.24 | −0.17 / 0.17 | 1.27 / 1.07 Hz |
| 19-50-23 | 27.4° / 27.5° | 24.7° / 18.9° | 12.3° / 20.2° | 0.04 / 0.07 | 0.07 / 0.19 | 1.27 / 1.07 Hz |

- The amplitude of the tilt oscillation is the same (26.7 against 26.8°); the correlation is ≈ 0 on both axes even with the best lag; the frequencies differ by 0.2 Hz, so the phase drifts through a full turn in about 5 s of the 13 s window. So J_q on A1 is not an offset or a lag that a fit can remove; it is the phase of a limit cycle.
- The axis split differs: real is mostly pitch (pitch 25° rms, roll 8–12°), the model tilts in both (pitch 18°, roll 20°). Not investigated further.
- With the planned top path the model does not oscillate at all on A1 (tilt rms 0.00°, `recon_tilt.py`), so J_q 0.054 there is the real oscillation alone.
- A4 and A8 (same file, other rows) show weak and mixed correlation (pitch 0 lag −0.09 to 0.39, best 0.23–0.59 at lags of 0.1–0.4 s).
- **Error found and fixed in my own script during the session:** the first version of `attitude_check.py` assumed the quaternion order (w, x, y, z); the traces and the model use (x, y, z, w). It was found when the tilt rms of the model came out 18.9° instead of the 26.8° of `gyro_check.py`. After the fix the tilt rms agrees with `gyro_check.py` (26.7 / 26.8°). The tilt / abort check (next paragraph) reads the logged `roll_deg` / `pitch_deg` columns and was not affected.

**Real A1 tilt and the abort rule** (`lab/tilt_abort_check.py`, `lab/results/tilt_abort_check.json`; scored part only, 2–15 s for A1 and A4, 2–14 s for A8; bottom vehicle `cf5_unseen`):

| flight | max roll | max pitch | pitch p99 | time with tilt > 25° | longest continuous run > 25° | most time > 25° in a 1-s window | gyro_y max |
|---|---|---|---|---|---|---|---|
| A1 19-47-31 | 21.9° | 40.5° | 39.0° | 51 % | 0.25 s | 0.65 s | 512 °/s |
| A1 19-48-59 | 25.5° | 38.6° | 37.9° | 48 % | 0.22 s | 0.64 s | 511 °/s |
| A1 19-50-23 | 24.4° | 39.8° | 38.6° | 48 % | 0.24 s | 0.68 s | 487 °/s |
| A4 19-37-24 | 22.6° | 31.5° | 25.4° | 1 % | 0.14 s | 0.14 s | 299 °/s |
| A4 19-38-38 | 22.2° | 30.2° | 25.8° | 2 % | 0.14 s | 0.18 s | 316 °/s |
| A4 19-40-01 | 10.7° | 24.2° | 18.5° | 0 % | 0.00 s | 0.00 s | 213 °/s |
| A4 19-41-36 | 9.5° | 26.0° | 19.7° | 0 % | 0.05 s | 0.05 s | 268 °/s |
| A8 19-43-07 | 27.9° | 12.7° | 12.0° | 1 % | 0.08 s | 0.08 s | 162 °/s |
| A8 19-44-24 | 26.6° | 13.5° | 11.9° | 1 % | 0.04 s | 0.04 s | 251 °/s |
| A8 19-45-56 | 21.8° | 10.0° | 8.6° | 0 % | 0.00 s | 0.00 s | 156 °/s |

- **Confirmed per A1 flight:** the bottom vehicle pitches to 38.6–40.5° (p99 37.9–39.0°) and `gyro_y` reaches 487–512 °/s (the prompt said about 400 °/s; my maximum over the scored part is 487–512). The top vehicle `cf_second_unseen` stays at 1.0–1.5° (max over roll and pitch; the prompt said under 1°, the 1° bound is exceeded by up to 0.5° on A1; on A4 and A8 the top vehicle reaches 2.8 and 3.5°).
- **Abort rule (`docs/comparative_study_protocol.md` §4: |roll| or |pitch| > 25° for 0.5 s):** read as 0.5 s *continuous*, it would **not** have fired on any of the 10 flights (the oscillation at 1.27 Hz stays above 25° for 0.22–0.25 s at a time). The wording does not say continuous or cumulative (not stated). Read as 0.5 s *in total within 1 s*, it would have fired on all three A1 flights (0.64–0.68 s); on any reading it would not fire on A4 or A8. I listed this in the open points of the protocol draft (item 10); the rule is not changed.

## 4. P2 stall

Source: `dryrun/stage4b/results_p2_stall_raw.txt` and the `tune.log` lists in `dryrun/stage3e/results/loop_P1.json` / `loop_P2.json` (per epoch: J, gradient g, position x). Normalised cost J/J₀ and gradient norm |g| of the tuning step (Adam, lr 0.01, 50 epochs, best iterate kept):

| epoch | P2 round 2 J/J₀ | |g| | P2 round 3 J/J₀ | |g| | P1 round 2 J/J₀ | |g| | P1 round 3 J/J₀ | |g| |
|---|---|---|---|---|---|---|---|---|
| 0 | 1.000 | 693 | 1.000 | 1.8 | 1.000 | 3.57 | 1.000 | 1.18 |
| 1 | 1.207 | 442 | 1.305 | 28.9 | 1.233 | 0.274 | 1.021 | 0.115 |
| 2 | 1.005 | 59.9 | 1.011 | 0.601 | 1.314 | 11 | 1.120 | 1.5 |
| 3 | 1.286 | 3.76e4 | 1.102 | 2.58 | 0.918 | 0.0881 | 1.113 | 0.736 |
| 4 | 1.661 | 1.01e6 | 1.136 | 0.374 | 1.267 | 35 | 1.005 | 0.548 |
| 5 | 1.665 | 2.96e3 | 1.346 | 0.624 | 0.969 | 1.81 | 1.130 | 3.54 |
| 10 | 2.020 | 2.59e4 | 1.444 | 0.142 | 0.915 | 0.106 | 0.956 | 0.0365 |
| 20 | 2.308 | 3.76e4 | 1.347 | 3.83 | 1.010 | 0.0712 | 0.987 | 0.08 |
| 30 | 2.020 | 1.24e5 | 1.918 | 10.9 | 1.005 | 0.0869 | 0.953 | 0.0259 |
| 40 | 1.673 | 2.26e5 | 1.918 | 57.2 | 1.007 | 0.27 | 0.939 | 0.0303 |
| 50 | 1.534 | 1.98e3 | 2.122 | 131 | 1.000 | 0.0342 | 0.932 | 0.0393 |
| best iterate | epoch 0 | | epoch 0 | | epoch 14 (0.847) | | epoch 50 (0.932) | |

Measured from the per-epoch lists (all four tuning steps of P1 and P2 rounds 1–4 were looked at; rounds 2 and 3 are the stalled ones):

| tuning step | |g| median | |g| max | max / min | first step J/J₀ | predicted by the gradient (g·Δx / J₀) | steps where J rose although g·Δx < 0 (of 50) | sign flips of g per step |
|---|---|---|---|---|---|---|
| P1 round 2 | 0.12 | 44 | 1.5e3 | 1.23 | −8.9 | 12 | 46 % |
| P1 round 3 | 0.051 | 3.5 | 6.3e2 | 1.02 | −3.6 | 6 | 39 % |
| P2 round 2 | **2.0e4** | **2.9e6** | 4.9e4 | 1.21 | **−1.3e3** | 20 | 53 % |
| P2 round 3 | 2.6 | 149 | 1.1e3 | 1.31 | −5.5 | 20 | 57 % |

**Reading (from these numbers, not tested by a new run):**
1. **J_task is rough on the scale of one Adam step in all four cases, and much rougher in P2 round 2.** Over the first step (0.01 in log units per gain) the gradient predicts a change of J/J₀ of −3.6 to −1300 (more than the whole cost; the linearisation is not valid at that step), and J rises by 2–31 % instead. The gradient direction flips sign on about half of the gains in every step (39–57 %). In P2 round 2 the gradient norm is also four orders larger than in P1 round 2 (median 2.0e4 against 0.12) and jumps by up to 1e6 between neighbouring epochs. That is the signature of a nonsmooth or nearly singular surface (an edge: the closed loop of the model close to its stability limit), not of a slowly curved surface.
2. **Which gains:** the largest gradient component is the attitude and rate gains: `Kwxy` (index 7) in 40 of 51 epochs in P2 round 2, 31–36 in the other three steps; `KRxy` (index 5) in most of the rest (`GAIN_NAMES` order of `loop_lib.py`). Same two gains in P1 and P2; the stiff direction is the same, only its size differs. This agrees with the earlier finding of a delay knife-edge in the attitude loop (INDI oscillation investigation, 2026-10-04), but I did not test that link here.
3. **Not the gain limit and not the floor rule:** the projection never hit (hits 0 in rounds 2 and 3 of both runs); the largest move from θ₂ is 0.16 log units against a limit of 0.22; the floor rule is not part of the tuning step (it is applied to flights).
4. **Not obviously the step size alone:** P1 round 2 also rises at the first step (1.23) and then returns below 1 (best iterate epoch 14, 0.85); P2 never returns (minimum 1.000 at epoch 0). The first step is the same size in both (Adam moves every gain by lr = 0.01 at epoch 0). Whether a smaller step would have found a descent in P2 is not known; it needs a run (see the proposal).
5. **β:** error of the fitted β against the P1 truth (log units; truth thrust −0.223, lag +0.405, downwash +0.262, plume −0.357; lag is a log factor against 30 ms), max |error| of the four after the fit of the same round:

   | round | 0 | 1 | 2 | 3 | 4 |
   |---|---|---|---|---|---|
   | P1 | 0.446 | 0.329 | 0.178 | 0.138 | 0.126 |
   | P2 | 0.393 | 0.367 | **0.113** | **0.126** | 0.165 |

   In rounds 2 and 3 the P2 fit is as close to the truth as the P1 fit (P2 slightly closer in round 2), and lag error +0.074 / +0.047 (P2) against −0.031 / +0.048 (P1): the 10 ms state delay is only partly absorbed into the motor lag (P2 lag is about 11 % above the P1 value in round 2). So the 10 ms does **not** explain the gap through a worse β; the stall comes with an equally good β. Which implies the difference is in how the model with that β and the θ₂ of P2 reacts to a gain change (point 1), not in the model's accuracy on the flown data (J_sysId after the fit: 0.1232 / 0.1216 in P2, 0.1267 / 0.1174 in P1).
6. **What the stall cost:** θ₃ = θ₂ and θ₄ = θ₃ in P2 (no change of gains), so 2 of the 6 rounds produced no new gains; the final keep-best is round 3 (7.60 cm) against P1's 6.39 / 7.77.

**Proposal for the user (no method change made):**
- (a) Confirm the roughness with a 1-D scan of J_task along the Adam direction through θ₂ of P2 round 2 (steps 1e-4 … 1e-2, 2 noise seeds, one process, a few minutes) and in P1 for comparison. If J has jumps at 1e-3 or below, it is an edge and the gradient is not usable there.
- (b) If so, two options for the user to decide: a smaller learning rate from round 2 on (for example 0.003), or a rule that treats "best iterate = epoch 0" as "step failed" and repeats the tuning step with half the learning rate before flying the same gains again. Both change the method (deviations 8 and 11 would get a new line); not done.
- (c) In the lab the real plant has the 10 ms (P2 adds the measured state delay of 10 ms, `loop_lib.py`), so P2 is the more realistic dry run. A stalled round costs a battery set of flights without new gains.

## 5. Open lab points (for the user; nothing decided here)

1. Scenario order inside a block, flights per scenario, flights per final evaluation (protocol draft open 1).
2. `rotate_deg` and the command lines for CIRCLE, FIG8 and the test cells (open 2); how the yaml fragment is applied (open 3); gain readback by hand (open 4); file names and `--kt` label for `simple_flight.py` (open 5).
3. Battery rule for repeats and the time limit per battery (open 9).
4. Number of lab rounds (3–4 instead of 6).
5. **Abort rule on A1:** the rule "|roll| or |pitch| > 25° for 0.5 s" does not fire on the 23 Oct A1 flights if it means continuous time, and would fire on all three if it means 0.5 s in total within one second (new open point 10 of the draft).
6. A1 attitude, velocity and position parts of J_sysId cannot be matched in phase (correlation ≈ 0, 1.27 against 1.07 Hz); open whether the fit on A1 should use J_q and J_v at all (open 7).
7. P2 stall: run the 1-D scan, and decide on the proposal (b).
8. A1 round 0 at the default gains fails on the floor in the sim; the lab rule repeats it (open 8).
