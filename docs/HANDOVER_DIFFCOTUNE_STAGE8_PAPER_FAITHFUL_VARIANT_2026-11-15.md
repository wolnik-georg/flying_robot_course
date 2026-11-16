# DiffCoTune, Stage 8: a paper-faithful variant (user decisions 15 Nov), body-rate cause, learning rate, RPM delay from real data (desk only, 2026-11-15)

Paste everything below the line into the tuning (main) agent.

---

**Order:**
- Let the running Stage 7 diagnosis jobs (chain1, chain2) finish.
- **Do not start chain3** (the reruns with the old method version). It is queued as PID 3028319 (`./chain3.sh`, waiting for chain2). Check it with `ps -o pid,ppid,args -p 3028319` and stop it by that exact PID. Its jobs are replaced by this stage.
- Write the Stage 7 diagnosis results into `report_stage7.md` as planned. Then start this stage.

## User decisions (15 Nov, 16:00)

The method moves as close to the paper as possible (Krishna et al. 2025, `docs/papers/summaries/krishna2025diffcotune.md`). Per earlier deviation (spec §17 numbering in the supervisor summary):

| # | Decision |
|---|---|
| 1 | **Keep:** cost averaged over the tuning scenarios, one gain set per controller (needed for the study). Tuning set as decided: A1, A8, A4 cell 1, CIRCLE, FIG8; test A4 cell 2. |
| 2 | **Like the paper:** task cost c = ‖x_t − x_t*‖², the unweighted full state against the reference state, not position only. |
| 3 | **Like the paper:** inner-loop stop when the change is < 1e-3 **or** the objective rises by more than 1e-3, with K/2 = 50 epochs as the cap. |
| 4 | **Find out why** the full state (with body rates) works for the paper but not for us in J_sysId. Goal: the same as the paper if at all possible. |
| 5 | **Like the paper:** no sensor noise in the tuning step. The outer loop (refit on new flight data) is what corrects sim-to-real. |
| 6 | **Like the paper:** no ±25 % trust region. Only a validity projection like the paper's (gains stay positive, as its Q ⪰ 0 clip). |
| 7 | **Learning rate:** choose the most sensible value, with best practice and references (step 1c), measured, not guessed. |
| 8 | **Keep-best, and fly the last update:** Alg. 1 loops l = 0 … L, i.e. L + 1 = 6 real rollouts. A 6th round of 5 tuning flights flies θ_5, so the last update is judged on real flights and can be kept. No fit or tuning after it. |
| RPM delay | Back the fixed value with **real data** (step 1d), not a guess. |
| Gradient | Compiled forward sensitivity everywhere (Stage 7 decision, unchanged). |

## Do

### 1. Look-ups and investigations (desk, no pool unless stated)

**a. Paper settings, verbatim:** list every setting the paper states, with the page, from the summary and `docs/papers/krishna2025diffcotune.pdf` (read only): costs, stop rule, K, L, Adam, projection, noise (state "not mentioned" if absent).

Also list what the sister paper DiffTune (`docs/papers/summaries/cheng2024difftune.md`; the same group, quadrotor hardware) did. Facts only:
- the ±10 % trust region per trial on hardware;
- sensor noise in its simulations;
- loss = position + attitude on hardware;
- learning rates 0.1 / 0.01 / 0.001 compared.

Mark it clearly as **another paper**, not DiffCoTune.

**b. Point 4, why body rates break J_sysId for us:**
- **Error floor per state component (sim-to-sim, P1, at the true β):** the noise-free truth against the model, the noisy truth against the noise-free model, and two noise seeds against each other. Report the share of p, v, q and w in J_sysId for each, per scenario.
- **Phase decorrelation:** the cross-correlation of body rates between two noise realisations at the true β on A1, A8 and A4. Is the A1 rate oscillation (1.0–1.4 Hz, Stage 4b) noise-driven?
- **Units:** the scale of each component (m², (m/s)², rad², (rad/s)²) in the paper's unweighted sum, against ours.
- **Paper-exact J_sysId (all states, unweighted):** run identifiability on S0 (one pool, 50 epochs, with the paper stop rule and without it). Compare the β errors with the pvq version (Stage 4e numbers).
- **Conclusion:** the cause in plain words. Is there a form that is **exactly the paper's** and works? If not, say what the smallest change is and why, as a proposal for the user. Do not adopt it.

**c. Learning rate:**
- What the paper states (not stated, per the summary).
- Adam's default 1e-3 (Kingma & Ba 2015; Optax default, which the paper used).
- DiffTune's comparison (note that DiffTune used plain gradient descent in raw gain units, so its values do not transfer directly).
- Our parameterisation: log ratio to the default gains, so lr 0.01 ≈ at most about 1 % gain change per Adam step.

**Measure:** a round-0 + round-1 probe of the paper-faithful variant (step 2) on S0 at lr 1e-3, 3e-3 and 1e-2, one pool at a time. Report the real cost after round 1, the epochs used by the stop rule, the largest gain change per round, and crashes or floor flights.

**Choose:** the lr with the best real cost after round 1 and no crash or floor flight, with the reason. If two are within 0.3 cm, take the smaller one (the safer step).

**d. RPM delay from real data:**
- Where the calibrated 4.0502 ms comes from (trace it in the repo: simulator calibration, `frsim/sensors.py`, docs).
- The measured lags in `docs/43_RPM_Source_Quality.md` and `experiments/analysis/out/rpm_source_quality/` (DShot lags the deck by 0–2 ms median over 32 flights, 0–5 ms typical; 2-ms grid at 500 Hz).
- Which source the firmware uses (`indi_gains.rpm_source=1`, DShot) and its filtering.

**Report:** whether 4.05 ms is consistent with the data. What is measured is only the delay **relative to the deck**; the deck's own delay is not measured. Do not invent it. Propose the value to fix (with its basis), or "4.05 ms stays, consistent with the data". The user decides.

### 2. The paper-faithful variant (switches, defaults unchanged until the user adopts)

`loop_lib.py` / `round.py` get a method switch, `DIFFCOTUNE_METHOD=paper|ours` (default `ours`, so all earlier results stay reproducible). `paper` means:
- **task cost (2):** unweighted ‖x − x*‖² over the full state. The reference state comes from the reference trajectory (position, velocity, attitude and body rates from differential flatness, as `reference.py` gives them; state where a component is not available). Use the same state vector as J_sysId, so the two costs use the same components;
- **stop rule (3):** change < 1e-3 or rise > 1e-3, cap 50 epochs; return the last iterate before the stop, as the paper describes (state exactly what is returned);
- **noise (5):** off in the tuning step;
- **trust region (6):** none; only gains > 0 (projection);
- **J_sysId (4):** the paper form (all states, unweighted) by default in `paper`, unless step 1b proposes otherwise; then both are run and the user decides;
- **lr (7):** from step 1c;
- **6 rounds (8):** l = 0 … 5, 6 × 5 tuning flights; keep-best over the 6 flown sets; `--skip-last-fit` after round 5. This also applies to `ours` (user decision 8);
- **RPM delay:** fixed for S1 / S3 at the value from step 1d (stays 4.0502 ms until the user decides);
- **gradient:** compiled forward sensitivity everywhere.

**Checks:**
- the gradient gate with the paper task cost (A1, A8, CIRCLE, A4 cell 1);
- `METHOD=ours` with `TUNING_SET=4` and 5 rounds reproduces the Stage 5c S1 rdfix θ₁ (bit-identical, or explain);
- the lab tooling (`flights_template.py`, `session.py`, `preflight.py`, the mock test) handles 6 rounds.

### 3. Dry runs (one at a time; S0 first, plant P1, same seed rules, 6 rounds)

1. **S0 `paper`** (lr from step 1c).
2. **S0 `ours`** (current frozen method with 5 scenarios, forward gradient, 6 rounds), as the reference.

**Report for both:**
- real cost per round;
- criteria a–c;
- crashes, floor and tilt aborts per round, with the abort rule as in the protocol;
- **the largest gain change per round** (safety: without a trust region, how far do the gains jump between real flights?);
- tuning mean, held-out A4 cell 2 mean, in-sample A4 cell 1;
- desk minutes per round.

**Stop and report** if a round takes more than 45 min or the dry run would take more than 4 h.

**Then stop.** S1–S3 are rerun only after the user picks the variant (or a mix) from these results.

### 4. Safety note (facts only, for the user)

From the S0 `paper` run: in which rounds a real (dry-run) flight crashed, hit the floor or tilted past the abort rule. Also how many such flights a lab session would see, at most 2 repeats per flight. Propose (not adopt) a pre-flight frsim preview of every new θ on the 5 tuning scenarios: if it crashes in frsim, do not fly it and ask the user. That would be a safety gate, not a change of the gains.

### 5. Docs

- Spec: a new section "Stage 8, paper-faithful variant (user decisions 15 Nov)" with the decision table above, each item marked "decided", "under investigation" (4, 7, RPM delay) or "proposal".
- Inventory and READMEs: marked "Stage 8".
- **Do not change** the protocol draft or the lab cards yet, except the 6-round count (decided).

## Report back (short): `dryrun/stage8/report_stage8.md`

1. Paper settings (verbatim, with pages) and the DiffTune facts, clearly separated.
2. Point 4: the error floors, the phase test, the paper-exact identifiability, the cause in plain words, the closest working form.
3. Learning rate: references, the probe table, the chosen value and why.
4. RPM delay: the source of 4.05 ms, the measured lags, the proposal.
5. S0 `paper` against S0 `ours`: real cost per round, criteria a–c, crashes or floor flights, the largest gain jumps, means, minutes per round.
6. The safety note.
7. Decisions for the user, each with the evidence: per item 2, 3, 4, 5, 6, keep the paper form or not; lr; RPM delay.

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (5 workers); every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit breaks, stop that job (exact PID) and report;
  - at most 16 CPU processes; no GPU (driver mismatch, do not touch it);
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - jobs strictly one after another.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs, no torch upgrade.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage8/` and `lab/`); new switches default to the current behaviour, except the 6 rounds (decided);
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 8").
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine; the summaries are never edited), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md`, everything FBL;
  - Stage 3b–7 results (write new results under `stage8/`).
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push in any repository (the commit comes later, after the user picks the variant);
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. Paper settings are cited with pages; another paper's settings are never presented as DiffCoTune's.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
