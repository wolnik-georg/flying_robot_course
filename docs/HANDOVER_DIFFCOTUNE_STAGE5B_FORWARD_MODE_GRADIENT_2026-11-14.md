# DiffCoTune, Stage 5b: forward-mode gradient for S1, S2, S3, then rerun their checks and dry runs (desk only, 2026-11-14)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 5 report was checked against the files:
- **Part A:** preflight PASSED for S0–S3; mock test; lab card. Confirmed.
- **S0 regression:** θ₁ difference 0.0. Confirmed.
- **Dry runs:** the S2, S1 and S3 tables, criteria and round-0 tests match the report. Confirmed.
- **Driver:** finished, no job running. Confirmed.

## The key finding is in your own C1c, item 2

- **Forward mode works:** the forward-mode tangent (`torch.func.jvp`) of the A1 position with respect to the thrust gain equals the finite difference to 1e-6 over the whole 5 s.
- **Reverse mode fails** on the same quantity at the same horizon: 808 against 0.153 at 4.95 s.

In exact arithmetic the two are the same number (vᵀ J u). So the derivative of the forward map is fine, and the reverse pass loses precision.

**Working hypothesis (to test, not a fact):** the unstable mode of the one-block Jacobian sits in the INDI filter states and `a_prev`, and it is not seen in the position.
- **Forward:** the parameter directions barely excite it, so the forward tangent stays correct.
- **Reverse:** the adjoint of a position cost has no exact component along it, but rounding puts one in, and that component grows about ×400 per second (S1/S3).

This also fits S0 / S2 (about ×10 per second): their small A1 mismatches (S0's accepted A1 row, S2's 1–14 %), and the eager vs compiled differences.

**Consequence:** computing the gradient in **forward mode** gives the exact autodiff gradient. **It is not a deviation from the paper**, since the paper uses the automatic-differentiation gradient and does not fix the mode. Only the cost changes: one tangent per parameter, i.e. 5 for the β fit (4 for S2) and 10 for the gain tuning.

## User decisions (14 Nov)

1. **Gradient for S1, S2, S3:** test forward mode. If it passes the gradient gate and a round stays within about 45 min, use it for S1–S3 and rerun their identifiability and dry runs. **Do not adopt** truncation, damping, dropping A1, a shorter horizon or finite differences. Those are new deviations; report them only as a fallback if forward mode fails or is too slow.
2. **S0 stays as it is** (reverse mode, lab-ready). Run its forward-mode gradient check for information only; do not rerun the S0 loop.
3. **RPM-delay β (S1, S3):** judged on the rerun. Proposal for the user: keep it if its error after the round-0 fit is under 25 %, otherwise fix it at the calibrated 4.0502 ms. Report the numbers against this proposal; **do not drop it yourself.**
4. **S3 flat start:** judged on the rerun.
5. **`round.py`:** the default becomes `--n-rounds 5`.
6. **S0 lab-time table:** correct the last-round desk time to the measured value (about 0.1 min).
7. **INDI thrust-arrival risk (64–72 %):** no model change. Lab order S0, S2, then S1, S3; the first real S1 round shows whether the gap matters. Write this into the protocol section S1–S3 as a known risk.
8. FBL later; the supervisor's agreement is still open.

## Do (one pool job at a time, in this order)

### 1. Test the hypothesis (S1, A1, short, eager is fine)

- **Precision:** reverse-mode J_task gradient on the 5-s A1 window in float32 and in float64.
  - If the gap to the finite difference grows by orders of magnitude in float32, the failure is rounding driven. Report both.
- **Observability of the unstable mode:**
  - at 3 blocks, take the left and right eigenvectors of the largest |λ| of the one-block state Jacobian;
  - report how large the projection of ∂J/∂state (position part) onto the left eigenvector is, and of ∂(block)/∂θ, ∂(block)/∂β onto the right one, each relative to the largest projection over all modes.
- These two results are diagnosis only. The gate in step 2 decides.

### 2. Forward-mode gradient

- In `loop_lib.py` (and where the models compute the gradient), add a switch `grad_mode = 'reverse' | 'forward'`, **default `'reverse'`**, so all earlier results stay reproducible.
- `'forward'`: the gradient of the scalar cost with respect to the parameters that the inner loop updates, by forward mode (`torch.func.jvp`, batched over the tangent directions with `torch.func.vmap`, or `torch.func.jacfwd`), so that:
  - the β fit uses one tangent per β;
  - the tuning uses one tangent per gain.
- Use `torch.compile` only if it works with forward mode and gives the same numbers; otherwise eager. State which one.
- **Correctness checks:**
  - (a) on A8, CIRCLE and FIG8, where reverse mode passes, forward and reverse gradients agree to ≤ 1e-6 relative, for S0, S1, S2, S3;
  - (b) the full gradient check (`checks/gradient_check.py` gate, all θ and β, J_task and J_sysId, A1 / A8 / CIRCLE, combined) in forward mode for S1, S2, S3, and for S0 as information;
  - (c) with noise (the Stage 3d noisy gradient check) for S1, S2, S3.
- **Timing:** seconds per epoch for the fit and for the tuning with the 4-worker CPU pool, forward against reverse. Estimate the minutes per round.
- **Stop rule:**
  - if (b) fails for a controller, stop that controller, report and go on with the next;
  - if a round would take more than 45 min, report it with the numbers and do not start that controller's dry run.

### 3. Rerun S1, S3, S2 with `grad_mode='forward'` (order: S1, S3, S2)

Everything else is the frozen method version and the Stage 5 setup:
- plant: P1-rd for S1 / S3, P1 for S2;
- seeds: flight 100 + 10·l + i, noise 6000 + 2·l (+1), evaluation 2000–2004;
- 5 rounds;
- tuning A1, A8, CIRCLE, FIG8; test A4 (2 cells), OVAL.

For each controller:
- **Identifiability** (as Stage 5 B2): β errors; for S1 / S3, the RPM delay against the 25 % proposal.
- **Dry run** (as Stage 5 B3):
  - real cost per round, criteria a, b, c, final evaluation, β per round, gain-limit hits;
  - compared with the reverse-mode Stage 5 run of the same controller (same seeds).
  - For S3: whether the flat start (rounds 0–3) is gone; if not, what the tables show (best epoch 0 in the fit, gain-limit hits, A1 cost).
- **Round-0 dry test** with `round.py --controller <c>` and the forward mode (θ₁ identical to `loop_lib`, as before). `round.py` gets the same switch, with the default per controller written in its settings: S0 `reverse`, S1 / S2 / S3 `forward` if adopted.
- **Each dry run within 3 h:** if the estimate after round 1 is longer, stop after that round and report.

### 4. Small fixes

- **`round.py`:**
  - default `--n-rounds 5`;
  - show with a cheap check that the S0 round-0 dry test is unchanged (θ₁ difference 0.0 against `lab/results/dry_test.json`, or the same proof without the 20-min run);
  - update the lab card and `preflight.py` (the reminder line becomes a check that the default is 5).
- **Protocol draft:**
  - S0 lab-time table: last-round desk time as measured;
  - section S1–S3: gradient mode per controller, the thrust-arrival risk and the lab order S0, S2, S1, S3 (user, 14 Nov);
  - the lab time per controller with the new round times.

## Report back (short): `dryrun/stage5b/report_stage5b.md`

1. Hypothesis test: float32 vs float64, and the eigenvector projections.
2. Forward mode:
   - correctness checks (a), (b), (c) per controller;
   - timing forward vs reverse;
   - compiled or eager.
3. Per controller (S1, S3, S2):
   - identifiability;
   - dry run against the Stage 5 run, criteria a–c;
   - for S1 / S3, the RPM delay against the 25 % proposal;
   - round-0 test;
   - minutes per round;
   - **"lab-ready on the desk side: yes / no"**, with the reason.
4. Open points for the user, with the decision each needs.

Also update `lab/README.md`, `experiments/diffcotune/README.md`, and `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md` marked "Stage 5b". If forward mode is adopted, add it to the spec's method description and state it as **not** a deviation (same gradient, other differentiation mode).

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (about 6 GB each); every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit is broken, stop that job (exact PID) and report;
  - at most 16 CPU processes; one GPU job at a time;
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - jobs strictly one after another; forward mode may need more memory per worker, so check MemAvailable one minute after the first forward-mode pool starts.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage5b/` and `lab/`); new switches default to the current behaviour;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 5b");
  - the protocol draft (the S0 lab-time table, section S1–S3; the decided S0 items stay as they are).
- **Do not touch:**
  - `simulator/` (the Stage 5 RPM-delay option stays as it is), `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md`, everything FBL;
  - Stage 3b–5 results (write new results under `stage5b/`).
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push in any repository;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. The hypothesis above is a hypothesis; report what the tests show.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
