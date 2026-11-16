# DiffCoTune, Stage 6: fastest lab session (desk speed, two controllers interleaved, session tooling) and a scenario decision sheet (desk only, 2026-11-14)

Paste everything below the line into the tuning (main) agent.

---

**Order:**
- First finish Stage 5c as planned: chain4, the S1 identifiability rerun (it crashed: `logs/ident_S1.rc` = 1, "Cannot access data pointer of Tensor that doesn't have storage"), and `report_stage5c.md`.
- **No pool job of this stage before chain4 prints `ALL DONE`.** Writing, reading and tooling without a pool may start now.

## Why

A lab session per controller is about 2.1–2.5 h:
- 29 min flying;
- 81–105 min of desk time in which the drones wait;
- 17 min of test flights.

All four controllers take about 9.3 h (protocol draft, "Lab time"). About 75 % is waiting for the PC. The method stays frozen (50 + 50 epochs, 5 rounds, same costs, same seeds), so the time can only come from:
1. computing the same round faster;
2. flying a second controller while the PC computes the first.

## User decisions (14 Nov, 21:00)

1. **Make the lab session as fast as possible without changing the method.** Every speed-up must give the same numbers: the next gains θ_{l+1} and β of the round-0 dry test equal to the reference within 1e-10 relative. Summation order may differ; otherwise it must be bit-identical.
2. **Interleaving two controllers is the proposed lab mode.** While the PC computes round l of controller A, the drones fly round l of controller B.
   - Pairs follow the lab order S0, S2, S1, S3: session 1 is S0 + S2, session 2 is S1 + S3.
   - S1–S3 only once Stage 5c says "lab-ready on the desk side".
3. **Two worker pools at the same time are allowed for this stage** (one per controller), replacing "one pool at a time":
   - load ≤ 22;
   - MemAvailable ≥ 3 GB;
   - at most 16 CPU processes in total.

   Everything else in the resource rules stays.
4. **Scenarios are not changed in this stage.** Prepare the decision sheet (step 5); the user decides. No new dry run for other scenario sets until the user has picked one.

## Do

### 1. Desk speed of one round (measure; S0 first, then S2)

**Benchmark:** the round-0 dry test (`lab/round.py`, the reference is `lab/results/dry_test.json` for S0; for S2 a reference run with the current settings first). Report per lever:
- desk seconds, split into ingest, fit and tuning, and the compile share;
- peak memory and load;
- the θ₁ / β₁ difference to the reference.

**Levers:**
- **(a) Tuning step with 8 workers:** one per scenario × noise seed, instead of 4 workers × 2 rows.
- **(b) Threads per worker:** 2 against 4 (fit and tuning separately). Only within 16 processes and load ≤ 22.
- **(c) Pre-warm:** `round.py --warmup` (new flag) starts the worker pool and compiles all graphs of the controller while the round's flights are flown. The desk step then starts warm. Measure the compile time saved per round (the first calls were 35–79 s per worker and graph in Stage 5c).
- **(d) Ingest of real files:** time on the 23 Oct uSD files, including the lag search. If the ingest exceeds 30 s, also per-flight ingest as each file arrives, so only the last flight's ingest is on the critical path.
- **(e) GPU:** Stage 3c found the GPU batch slower (37.9 s against about 10 s per epoch, launch bound), and `reduce-overhead` gave no result in 36 min.
  - Run one new probe **only if** `torch.cuda.is_available()` works without touching the driver (NVML reports a driver/library mismatch; do not fix it).
  - Probe: the 8 noisy tuning rollouts plus the compiled forward-sensitivity graph as one batch, at most 30 min.
  - Otherwise write "GPU not used" with the reason.
  - The GPU job never runs together with a CPU pool.
- **(f) Anything else that keeps the numbers identical,** stated with its check.

**Adopt** the fastest combination that passes the 1e-10 check. Defaults go into `round.py` per controller. Old behaviour stays reachable by flags.

### 2. Two controllers at once on this PC (measure)

- Run the round-0 dry tests of S0 and S2 at the same time, each with the setup from step 1, but sized so that both together keep the limits (for example 2 × 4 workers × 2 threads).
- Report:
  - desk minutes per round of each, against alone;
  - MemAvailable and load, before, at 1 min, at the peak;
  - θ₁ identical to running alone.
- If a limit breaks, stop the second job (exact PID) and report the largest setup that fits.
- Also report, as "not stated" if unknown, which machine runs `round.py` in the lab (this PC: 22 cores, 30 GB) and whether the lab PC could run the second loop.

### 3. Session timeline (both modes, with the measured numbers)

`lab/session_timeline.py` writes a figure and a table for:
1. sequential: one controller after the other;
2. interleaved pairs: S0 + S2, S1 + S3.

**Inputs:**
- 86.6 s per flight;
- 4 tuning flights per round;
- 12 test flights per controller;
- measured desk minutes per round (alone and two at once, from steps 1–2);
- strategy-switch overhead (CS2 stop, yaml merge, `set_tree_strategy.py`, relaunch, readback) as a parameter of 3 / 5 / 10 min, since it is not measured;
- the extra `--solo` relaunch per round;
- test flights of controller A flown while the PC computes controller B, where the timeline allows.

**Output:**
- hours per session and for all four controllers;
- the drones' idle time;
- flights per hour.

Battery packs per hour are "not stated" (flights per pack are not stated); report flights per hour so the user can judge.

### 4. Session tooling for the interleaved mode (no hardware)

- **`lab/session.py`:** reads the state dirs of both controllers (`lab/S0`, `lab/S2`, …) and prints the next action. Examples: "fly S2 round 1: A1, A8, then --solo CIRCLE, FIG8", "S0 round 1 desk ready at hh:mm", "switch: set_tree_strategy S0 + apply_gains merge + readback". It calls `round.py` in the background for the controller whose flights are complete.
- **Strategy switch:** the exact command sequence from the existing tools (`flying_drone_stack/tools/set_tree_strategy.py`, `lab_check_strategy.py`, `lab/apply_gains.py merge/readback`). State what S2 / S3 need beyond S0 (NS2 on, neighbour enabled for A1 / A8). Copy only from existing protocol text and tools; mark anything missing "not stated".
- **File handling:** both controllers' flights land on the same uSD cards. The file-to-round mapping goes through `flights.json` per controller and round, matched explicitly by scenario and time (memory rule: never assume files that arrive together are the same flight). Count the card files: an interleaved session doubles the files per session. Check against the ≥ 85 archive rule and state when to archive.
- **Mock test:** a whole two-controller session with the Stage 5 dry-run flights as fake uSD inputs, the switch steps printed, both state dirs correct at the end, θ_best identical to the sequential run.
- **Lab card:** `lab/INTERLEAVED_LAB_CARD.md`, a draft built from `S0_LAB_CARD.md`, with every row sourced; new rows marked "Stage 6, proposed".

### 5. Scenario decision sheet (facts only, no runs)

Write `dryrun/stage6/scenario_decision.md`, plots first and little text. Per candidate (A1, A8, A4 cells 1 / 2, CIRCLE, FIG8, OVAL, and the tier-1 study settings of `docs/comparative_study_protocol.md` §3):
- flight time;
- one or two drones;
- whether it is a study scenario, and which settings;
- the tuning setting against the study settings (e.g. A1 dz 0.375 in tuning against 0.25 / 0.5 / 0.75 in the study; A8 `--duration 8.0` against the study speeds);
- which β it can identify (downwash and plume need two drones). Use the identifiability results of Stages 3–5c, not new runs;
- flight-space fit (Stage 4d windows);
- the desk-time effect (fit and tuning time scale with the number and length of the scenarios; use measured per-scenario rollout times).

Then the options, each with flights per round, the estimated desk minutes per round, and what is in-sample or held out for the study:
- **A (current, dry-run tested):** tuning A1, A8, CIRCLE, FIG8; test A4 cells 1 / 2.
- **B:** tuning only on non-study flights: what is lost (e.g. downwash β without two-drone flights) and what would need a new dry run.
- **C:** tuning on the study settings themselves (e.g. A1 / A8 at the study settings): what that means for the study's held-out claim.
- **D:** anything else the facts suggest, stated as a proposal.

No recommendation is binding. Give one with its reason, and list the dry run each non-A option would need.

### 6. Docs

- **Protocol draft:** a new section "Fast lab session (Stage 6, proposed, not decided)" with the adopted desk settings, the timeline table and figure, the switch sequence, and the file and battery points. The decided S0 items stay unchanged.
- **Elsewhere:** `lab/README.md`, `experiments/diffcotune/README.md`, and `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md`, marked "Stage 6". In the spec: the speed-ups are **not** deviations (same numbers).

## Report back (short): `dryrun/stage6/report_stage6.md`

1. Step 1: a table of levers (seconds, memory, θ₁ difference); what was adopted; desk minutes per round before and after, per controller measured.
2. Step 2: two at once: minutes per round, memory, load; the largest setup that fits.
3. Step 3: the timeline: hours per session, sequential against interleaved, all four controllers; idle time; flights per hour.
4. Step 4: tooling and mock-test result.
5. Step 5: the link to the decision sheet and its recommendation.
6. Open points for the user, with the decision each needs: scenario option, switch overhead, battery rule, which PC runs the loop in the lab.

## Rules

- **Resources:**
  - `nice -n 19`;
  - every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit breaks, stop that job (exact PID) and report;
  - at most 2 pools at once, only in step 2 and only after chain4 is done; otherwise one pool;
  - at most 16 CPU processes in total; one GPU job at a time, never together with a CPU pool;
  - `TORCHINDUCTOR_COMPILE_THREADS=1`.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs, no torch upgrade.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage6/` and `lab/`); new flags default to the current behaviour unless a speed-up passed the 1e-10 check;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 6");
  - the protocol draft (the new Stage 6 section only).
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md` (reading is fine), everything FBL;
  - Stage 3b–5c results (write new results under `stage6/`).
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
