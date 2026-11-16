# DiffCoTune on our S0 controller: lab plan (prepared 2026-11-02, NOT to be flown without the user's go)

Source: Krishna et al. 2025 (`papers/summaries/krishna2025diffcotune.md`). Method in one line: fly the current controller a few times, refit the simulator so that it reproduces those flights, retune the controller in the refitted simulator, repeat. Split-Alternate is the variant to use (Combined diverged under mismatch in the paper, Fig. 7). Nothing here has been flown; every number below that is not a design choice comes from the desk work listed at the end.

## 1. What one iteration is for us
| Step | What | Where it runs | Uses |
|---|---|---|---|
| 1 | Fly L real S0 flights with the current gains θ_l on one scenario (bottom drone tuned, top drone scripted) | lab | `--study`-free S0 flight, uSD logging (500 Hz) |
| 2 | Refit the plant parameters β = (motor lag, thrust-law amplitude, tilt gain, downwash scale, plume amplitude) so that the closed-loop replay of those flights matches the logs | desk, CPU | `diff/replay_fit_beta.py` (written and tested on existing S0 logs) |
| 3 | Retune θ in the refitted plant with the same gradient code (S0, matched plant distribution centred on the refitted β), new tag **C** | desk, GPU | `train_scen.py` with a plant file override (to be added, additive) |
| 4 | Keep θ_{l+1} only if the replay says it is not worse than θ_l in the refitted plant **and** it passes the hardware check below | desk | `rank_final.py` machinery |

## 2. Numbers (proposal)
| Item | Value | Reason |
|---|---|---|
| Real flights per iteration L | 4 per scenario (A4, A8, A1) | paper: one real rollout per iteration; ours has noise (battery, tracker), so four give a median; 3-5 range in the task |
| Iterations | 2 (θ_0 = best tuned set, θ_1 after the first refit, θ_2 after the second) | paper stops at 5 iterations on much simpler loops; we have little lab time |
| Flights in total | 3 scenarios x 4 flights x 3 rounds (θ_0, θ_1, θ_2) = 36 | plus the default-gain S0 flights already held (10 unseen-block flights, 48 pilot flights, mixed strategies) as the baseline |
| Gains flown in round 0 | the winner of the final S0 ranking per scenario, chosen **before** the flight by the frozen rule (not by the lab result) | avoid selecting on flown data |
| Stop rule | stop after round 2, or earlier if the median study-cell score of round l+1 is not at least 10 % better than round l, or on the first crash/abort | paper's termination: no further improvement |
| Cells | tuning cells for the refit; the study cells are flown **only** for the final evaluation of the last round, never used for the refit or the selection | keeps the study protocol clean |

## 3. Safety (the DiffTune hardware lesson)
DiffTune Table VI: gains tuned on a simulator crashed on the real drone. So:
- Every flown set stays inside the tuning box (`train_s0.LO/HI`), and in round 0 only sets that were crash-free in the stress cells (md40, md60, lag45) of the plant ranking are eligible.
- Hardware check before any new θ: one hover and one slow A4 pass with a tethered or low-altitude setup, abort limits as in the study protocol; gains with a very low horizontal damping or a horizontal integral at its box limit (the pattern of the best-in-plant BO sets, see `tuning_procedure_frozen_2026-11-01.md` notes) are **not** flown in round 0.
- Battery: fresh batteries, `vbat` logged; uSD file count checked (archive at >= 85, never reach 100 files).
- The yaml readback of the gains is logged for every flight (the uSD meta files do not record the Omar-tree gains, see `tuning_usd_log_check_2026-11-01.md`).

## 4. What the desk work already showed (inputs to the decision)
| Finding | File |
|---|---|
| The existing uSD logs are 500 Hz and contain everything the closed-loop replay needs except the integrator states (reconstructed) and the strategy label (known for the S0 unseen block `bank_unseen_S0_2026-10-23`: 10 flights, A1 x3, A4 x4, A8 x3) | `tuning_usd_log_check_2026-11-01.md` |
| The S0 copy reproduces the logged thrust command to within 0.1 % and the logged torques to 4-11 % (rms, correlation >= 0.99) on a real flight, with only two constant integrator offsets fitted | `diff/realgrad_difftune.py` |
| Fitted β versus the calibrated plant | `out/difftune_real_vs_sim/beta_fit.json` (see the report) |

## 5. Not done / open
- The plant-file override for `train_scen.py` (tag C) is not written; it is a small additive change once the first refit exists.
- Combined vs Split-Alternate: only Split-Alternate planned; no Combined run.
- L1 adaptation (DiffTune) is not used: our controllers are the objects under test, and adding L1 would change them.
