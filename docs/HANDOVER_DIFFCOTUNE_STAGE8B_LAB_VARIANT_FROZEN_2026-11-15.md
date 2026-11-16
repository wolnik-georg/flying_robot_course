# DiffCoTune, Stage 8b: lab variant frozen ("ours"), safety preview of each new gain set, lab pack for tomorrow (desk only, 2026-11-15 evening)

Paste everything below the line into the tuning (main) agent.

---

**Order:**
- Do not stop or restart anything that is running. Keep the current queue: the "ours" dry runs one at a time (S0 → S2 rerun → S1 → S3, each with its round test and gradient check), and the short paper runs for S1 / S3 in the other slot, under your memory rule (stop the paper job by exact PID if MemAvailable < 3 GB).
- **Priority:** S0 and S2 "ours" (dry run, round test, gate) must be finished and reported by tomorrow morning. They are session 1. S1 and S3 are session 2 and may finish later.
- The code changes of this stage (steps 2–3) must not change any running job. Write them so the running processes are not affected (they have already imported their modules). Test them only with light single processes (`nice -n 19`, no pool) while memory allows, or after the S0 / S2 runs.

## User decisions (15 Nov, evening)

1. **The paper variant is not adopted.** It got worse in every round in the dry run (S0 18.24 → 30.81 cm, S2 14.38 → 19.11 cm), while "ours" improved within one round (S0 18.24 → 9.69, S2 14.38 → 8.09). The paper runs stay as evidence in the report.
2. **The lab variant is "ours", frozen, the same for all four controllers** (fair comparison: nothing changes between sessions):
   - DiffCoTune Split-Alternate loop; 5 tuning scenarios (A1, A8, A4 cell 1, CIRCLE, FIG8); test A4 cell 2 (held out) and A4 cell 1 (in-sample);
   - 6 rounds (l = 0 … 5), keep-best over the 6 flown sets, `--skip-last-fit` after round 5;
   - task cost on 3-D position; J_sysId = position + velocity + attitude;
   - 50 + 50 epochs, the best iterate; Adam reset; lr 0.01;
   - ±25 % gain limit per round; sensor noise in the tuning step, 2 seeds;
   - compiled forward sensitivity (`cfwd`) everywhere; RPM delay fixed at 4.0502 ms for S1 / S3.
3. **How it is described:** "DiffCoTune's co-tuning loop (Krishna et al. 2025) with DiffTune's quadrotor settings (Cheng et al. 2024)": position cost, a per-step gain limit, sensor noise in simulation, learning rate from DiffTune's compared values.
4. **Not adopted, listed only** (each would need new dry runs for S0–S3): an attitude term in the cost (DiffTune hardware), a ±10 % limit, a fixed allowed range per gain, body rates in J_sysId.
5. **Added: a safety preview of every new gain set** (step 2). It is a check, not a method change: it never changes the gains.

## Do

### 1. Finish the evidence (from the running jobs, no new pool)

- **S0 "ours" round 3 (15.96 cm):** was it affected by the overload (fit 3899 s)? The computation is deterministic, so slowness alone should not change the numbers. Check by comparing round 3's θ and β with a rerun of round 3 only (single process after the queue, or state why not needed), or explain from the logs. Report the largest gain change of that round and which scenario got worse.
- **The measured-gyro paper-exact identifiability** (still running): report it next to the two earlier rows (pvq −6.3 / −56.4 / −35.3 / +64.8 %, pvqw true +0.9 / −30.7 / +20.9 / +14.4 %).
- **The overload 20:00–21:06:** which processes and how much memory (from your logs), and which results it touched. Only "ours" S2 was stopped; confirm that the paper S0 / S2 results finished before it.

### 2. Safety preview of each new gain set (`lab/preview.py`, called by `round.py`)

- After the tuning step of a round, simulate the new θ in frsim on the 5 tuning scenarios, **with the round's fitted β**, sensor noise on (2 seeds, the evaluation seeds), and the same abort rules as the protocol (crash, floor, tilt).
- **Output:**
  - per scenario and seed: 3-D RMS, z RMS, crash / floor / tilt;
  - the same for the best flown θ so far, for comparison;
  - **"PREVIEW PASSED"**, or **"PREVIEW FAILED: <scenario, reason>. Do not fly this set; ask the user"**.
- **It never changes θ.** On a fail, `round.py` still writes the gains file but prints the fail in the summary and in `flights.json`. The decision (fly it, or fly the best set again) belongs to the user in the lab.
- `round.py` runs it by default (`--no-preview` to skip). Time it: target ≤ 2 min per round; if longer, run it in the pool workers that already exist.
- **Check:**
  - on the stored dry-run θ of every round of S0 and S2 "ours" (and paper S0 round 5): does the preview flag the bad rounds? Compare it with the dry-run "real" result per round;
  - mock test: one θ that crashes in frsim must give PREVIEW FAILED.

### 3. Lab tooling to the frozen variant

- **`round.py`:** default `--method ours` for all four controllers (it is `paper` now). Fix the stale docstring (`--n-rounds 5`, old grad modes, the 4-scenario text).
- **`preflight.py`:** checks the "ours" settings of decision 2 (method, rounds, scenarios, lr, limit, noise seeds, grad mode, RPM delay for S1 / S3, NS2 weights for S2 / S3) and fails on any other. Run it for S0, S1, S2, S3: all must print PREFLIGHT PASSED.
- **Round test:** `round.py` round 0 with the new defaults reproduces the θ₁ of the "ours" dry run (S0 and S2 at least; S1 / S3 when their runs exist).
- **Lab cards** (`S0_LAB_CARD.md`, `INTERLEAVED_LAB_CARD.md`):
  - the preview line after each desk step, with what to do on a fail (ask the user; default action: fly the best flown set again);
  - `--method ours` stated;
  - the uSD rule: empty both cards before the session, archive once in the middle (after round 2 of both controllers), never ≥ 85 files; check on the first flight that it is one file per flight.

### 4. Docs

- **Spec:** a section "Stage 8b, lab variant (user decisions 15 Nov evening)" with:
  - the decision list above;
  - a table of each part and where it comes from (DiffCoTune / DiffTune / ours), with pages from both papers;
  - the paper variant as tested and not adopted, with its numbers;
  - the four not-adopted items with the reason;
  - the preview as a safety check, not a method change.
- **Inventory and READMEs:** marked "Stage 8b".
- **Do not touch** the supervisor summary (the user's side updates it).

### 5. `dryrun/stage8/LAB_TOMORROW.md` (one page, ready at the morning check)

- Per controller (S0, S2 first; S1, S3 if done):
  - real cost per round, "ours" and paper side by side;
  - criteria a–c;
  - crashes, floor and tilt aborts per round; the largest gain change per round;
  - tuning mean, held-out A4 cell 2, in-sample A4 cell 1;
  - the preview result on each round's θ;
  - **lab-ready yes / no**, with the reason.
- Session 1 (S0 + S2 interleaved): the cards, the command sequence, the estimated hours from the Stage 6 timeline, the preflight results.
- Open items with who decides: the PC for `round.py`, flights per battery pack, the supervisor's agreement.

## Report back (short): `dryrun/stage8/report_stage8.md` (complete §5–7) plus the link to `LAB_TOMORROW.md`

1. S0 round 3 explained; the measured-gyro identifiability; the overload facts.
2. The preview: time per round, the check on the stored θ, the mock result.
3. The tooling: `round.py` default, preflight results S0–S3, round tests.
4. Lab-ready yes / no per controller.

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (pools grew to about 2.6 GB per worker; two pools broke the limits at 21:03). The short paper runs may overlap only while MemAvailable stays ≥ 3 GB;
  - every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit breaks, stop that job (exact PID) and report;
  - at most 16 CPU processes; no GPU (driver mismatch, do not touch it);
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10, torch 2.5.1) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs, no torch upgrade.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage8/` and `lab/`);
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 8b");
  - the protocol draft (lab variant and session plan only).
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine; the summaries are never edited), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md`, everything FBL;
  - Stage 3b–7 results.
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push in any repository (the commit is done after the morning check);
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. DiffTune's settings are cited as DiffTune's, never as DiffCoTune's.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
