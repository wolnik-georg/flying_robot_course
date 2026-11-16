# DiffCoTune, Stage 5c: exact gradient at near-normal speed for S1, S2, S3, then the reruns (desk only, 2026-11-14)

Paste everything below the line into the tuning (main) agent.

---

**First finish Stage 5b as planned:** the h-scans, the S0 identity proof, `report_stage5b.md`, the protocol section and the README / spec / inventory entries. Then start this stage.

## What your Stage 5b results show (checked in `dryrun/stage5b/results/fwd_checks_S*.json`)

- **(b) passes for all four controllers:** the forward-mode gradient is exact.
- **Without noise, reverse mode fails on A1 only.** On A8, CIRCLE and FIG8 it equals forward mode to 1e-14.
- **With noise, reverse mode is correct on A1 too.** In the noisy check (c), the A1 gradient of reverse mode equals forward mode to ≤ 2e-11 relative (S0 2e-12, S1 2e-11, S2 2e-13, S3 6e-12). The "FAIL" of (c) is the finite difference (h-scans running), not the gradient.
- **The cost:** uncompiled forward mode with 15 tangents takes 450–590 s per 4-worker call, against 15–33 s for compiled reverse.

## Consequences (the plan of this stage)

1. **The tuning step runs with noise** (2 seeds, deviation 10), so it **can keep compiled reverse mode**, as fast as now, if this holds across gain sets (step 1).
2. **Only the noise-free β fit (J_sysId) needs a different gradient, and only on A1**, with only 5 tangents (S1 / S3) or 4 (S2).
3. **The slowness comes from forward mode running uncompiled.** The exact forward-mode gradient can be computed inside the compiled model (step 3).

None of this is a deviation from the paper: it is the same exact gradient, only computed differently per scenario.

## User decisions (14 Nov evening)

1. Order: verify reverse with noise (step 1), forward mode only for the A1 fit (step 2), make that fast (step 3); then the reruns (step 5).
2. **Target:** a round of S1 / S2 / S3 within about 30 min (S0: about 22 min). Hard limit as before: 45 min per round.
3. **Fallback only if step 3 fails the target:** a truncated reverse pass for the A1 fit only (step 4). It is a **new deviation**: measure and report it, but **do not adopt it**; the user decides.
4. S0 unchanged (reverse everywhere, lab-ready). The Stage 5b decisions stay: the RPM-delay proposal (keep if its error after the round-0 fit is < 25 %), the S3 flat start judged on the rerun, lab order S0, S2, S1, S3.

## Do (one pool job at a time, in this order)

### 1. Reverse mode with noise across gain sets

- **What:** compare the noisy J_task gradient (2 noise seeds as in the loop), reverse against forward, per scenario.
- **Where:** S1, S3 and S2, on all 4 tuning scenarios, at θ₀, at θ₁ and at θ₃ of the Stage 5 dry run of that controller (`dryrun/stage5/results/loop_S*.json`), with the β of that round. Use 2 noise-seed pairs: the loop's pair of that round, and 7000/7001.
- **Gate:** max relative difference ≤ 1e-6 per scenario.
  - **If it passes everywhere,** the tuning step keeps compiled reverse mode for S1–S3.
  - **If it fails anywhere,** report where. That scenario's tuning gradient then also needs the step-2/3 method.
- **Timing:** forward is slow, so run only the A1 rows in forward mode. The other scenarios were already shown equal noise-free; give them 1 point each.

### 2. Mixed gradient for the β fit (exact, per scenario)

- **New switch:** `grad_mode = 'mixed'` in `loop_lib.py` / `round.py` / the models (default stays `'reverse'`). In the β fit, A1 uses the forward-mode gradient and A8 / CIRCLE / FIG8 use compiled reverse. The summed cost's gradient is the sum, so it stays exact. The tuning step is as decided by step 1.
- **Split the work:** spread the A1 tangents (5 or 4 β) over the 4 workers, e.g. 2 / 1 / 1 / 1 tangents, with A1 in the workers that have spare time. Use one rollout per tangent set, or whatever splits best. State the split.
- **Check:** the mixed gradient equals the pure forward gradient to ≤ 1e-8 relative (S1, S3, S2; θ₀, β₀).
- **Timing:** seconds per fit epoch, and minutes per round.

### 3. Exact forward mode at compiled speed (main speed-up)

**Goal:** the A1 fit gradient within a few seconds per epoch, exact (≤ 1e-8 relative to `torch.func.jvp`).

Try these in order and keep the first that meets the goal and passes the check:

- **(a) Hand-written forward sensitivity inside the compiled block:**
  - carry the tangent states dx/dβ (state size × 5) next to the state;
  - propagate them each block with the block's Jacobian-vector product, written with ops that `torch.compile` accepts. For example, evaluate the block on a batch of (state, tangent) pairs through a linearisation computed inside the compiled graph (`torch.autograd.functional.jvp` is not compilable either, so write the tangent update explicitly or with the double-vjp trick, if it compiles);
  - accumulate dJ/dβ at the cost rows.

  Check against `torch.func.jvp` on 6-s and full A1 flights.
- **(b) The double-vjp trick** (jvp = vjp of a vjp) on the compiled reverse block, if torch 2.5.1 compiles double backward for this graph.
- **(c) Forward mode in eager on the GPU, vmapped over the tangents (float64):** one GPU job at a time; the earlier GPU result was launch-bound, so measure it, do not assume it.
- **(d) Anything else that is exact,** stated with its check.

For each tried variant report the correctness against `torch.func.jvp`, the seconds per A1 gradient, and why it was kept or dropped.

### 4. Fallback, measure only: truncated reverse for the A1 fit

**Only if no variant of step 3 reaches a round of ≤ 45 min.**
- **Method:** reverse mode with the backward pass cut every W seconds of A1 (stop-gradient on the carried state).
- **Choose W:** the largest W at which the truncated gradient agrees with the forward-mode gradient within the gradient-check gate.
- **Report:** W, the gate rows, the seconds per epoch.
- **Do not use it in any run.** It is a new deviation for the user to decide.

### 5. Reruns (only with an exact method from step 2/3 within 45 min per round)

**Order:** S1, S3, S2. Use the Stage 5 setup unchanged:
- plants P1-rd / P1;
- seeds: flight 100 + 10·l + i, noise 6000 + 2·l (+1), evaluation 2000–2004;
- 5 rounds; frozen method version;
- tuning A1, A8, CIRCLE, FIG8; test A4 (2 cells), OVAL.

For each controller:
- **Identifiability** (Stage 5 B2): β errors; for S1 / S3, the RPM delay against the 25 % proposal.
- **Dry run:**
  - real cost per round, criteria a–c, final evaluation, β per round, gain-limit hits;
  - compared with the Stage 5 reverse-mode run with the same seeds;
  - for S3, whether the flat start is gone.
- **Round-0 dry test** with `round.py --controller <c>` and the new gradient (θ₁ identical to `loop_lib`). The per-controller default in `round.py` settings: S0 `reverse`; S1 / S2 / S3 the adopted mode.
- **Gradient check** (Stage 5 gate, combined, A1 / A8 / CIRCLE) with the adopted mode: must pass.
- **Each dry run within 3 h:** if the estimate after round 1 is longer, stop after that round and report.

## Report back (short): `dryrun/stage5c/report_stage5c.md`, updated after each step

1. Step 1: the reverse = forward table with noise (controller × gain set × scenario × seed pair); the decision for the tuning step.
2. Steps 2–3: the mixed gradient check, each step-3 variant (correctness, seconds), the kept variant, minutes per round.
3. Step 4 (only if run): W, gate, seconds; marked "not adopted, user decision".
4. Step 5, per controller:
   - identifiability (RPM delay against 25 %);
   - dry run against Stage 5, criteria a–c;
   - S3 flat start;
   - round-0 test;
   - gradient check;
   - minutes per round;
   - **"lab-ready on the desk side: yes / no"**, with the reason.
5. Open points for the user, with the decision each needs.

Also update `lab/README.md`, `experiments/diffcotune/README.md`, and `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md` marked "Stage 5c". In the spec, state the gradient per controller and loop (fit / tuning) and that it is **not** a deviation (same exact gradient, other differentiation mode).

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (about 6 GB each; check MemAvailable one minute after the first pool of each new gradient variant, since tangent states need more memory); every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit is broken, stop that job (exact PID) and report;
  - at most 16 CPU processes; one GPU job at a time, never together with a CPU pool;
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - jobs strictly one after another.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs, no torch upgrade.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage5c/` and `lab/`); new switches default to the current behaviour so earlier results stay reproducible;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 5c");
  - the protocol draft (section S1–S3: gradient mode and round times; the decided S0 items stay as they are).
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md`, everything FBL;
  - Stage 3b–5b results (write new results under `stage5c/`).
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push in any repository;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
