# DiffCoTune, Stage 3c for S0: stop P2, report P1, find the cause of the drift, speed up the loop (2026-11-12)

Paste everything below the line into the tuning (main) agent.

---

The P1 results were checked against `dryrun/stage3b/results/full_P1_eval.json` and `logs/full_P1.log`. Mean 3-D RMS over 5 seeds (cm):

| Method | Tuning | Test | A1 | A8 | CIRCLE | FIG8 | A4 | OVAL | Crashes |
|---|---|---|---|---|---|---|---|---|---|
| (a) default | 15.29 | 9.04 | 37.4 | 11.0 | 6.6 | 6.1 | 11.0 | 5.2 | 0 |
| (b) tune only | 8.70 | 15.11 | 28.8 | 3.5 | 1.3 | 1.2 | 22.0 | 1.4 | 4 |
| (c) sysId then tune | 5.65 | 8.58 | 19.2 | 1.9 | 0.8 | 0.7 | 12.5 | 0.8 | 0 |
| (d) DiffCoTune (θ_best = round 1) | 7.08 | 7.11 | 14.4 | 5.7 | 4.2 | 4.0 | 9.0 | 3.4 | 0 |

DiffCoTune's real cost per round was 19.8 / 8.0 / 16.9 / 69.8 / 95.3 / 66.7 cm. β moved away from the truth after round 2: in round 5 the log errors were −0.52 / −1.14 / 0.00 / −1.09. Keep-best saved the result.

## User decisions (12 Nov)

1. **P2 is not run now.** It repeats a setup that is known to drift; the cause comes first.
   - P2 started at 19:41 after P1 (PID 2088592, children 2192659–2192663).
   - **The user OKs stopping exactly these PIDs.** Check each with `ps -o pid,ppid,args -p <PID>` first. Then stop the parent and its children, and nothing else.
   - Keep `logs/full_P2.log` as a partial record, marked "stopped by the user 12 Nov, not used".
2. **Speed-ups** as in Part C below. No method change comes with them.
3. **No new long run** (more than 1 h) without the user's OK. Report first.

## Part A: P1 report

Write `dryrun/stage3b/report_stage3b.md`, P1 only:
- B1 and B2 tables;
- B4 items 1–7, plus the Stage 3 vs Stage 3b comparison;
- the epoch of the best iterate per inner loop;
- why (c) failed in Stage 3 (63.75 cm);
- pass criteria, each yes / no.

Plots go in `dryrun/stage3b/`.

## Part B: cause of the drift (desk checks, each under 1 h)

The aim is to separate "the fit drifts away from the truth" from "even the true β does not give good real gains". Use the existing P1 flights; they are reproducible from the seeds. Use `full50best` and `pva`, learning rate 0.01.

1. **Oracle tune:**
   - tune θ from θ₀ for one inner loop in the model with the **true P1 β** (no fit);
   - evaluate in the "real drone", 5 seeds, all 6 scenarios.
   - Repeat starting from the round-2 and round-3 gains of (d).
   - If the real cost is good, the tuning step is fine and the fit is the problem. If not, the model gap (noise, unmodelled effects) is the problem.
2. **Fit from the truth:**
   - start the β fit at the true P1 β, on the round-3 and round-4 flights of (d) (the bad gains);
   - report where β goes in 50 epochs.
   - If it walks away from the truth, the fit target, not the start, pulls β away.
3. **What drives the fit:** per round 0–5 of (d), report the share of each part (position, velocity, attitude) in J_sysId and in its gradient for each β.
   - Is velocity still a flat floor (0.134 → 0.132 in B1)?
   - Does the velocity in the "real drone" trace come from the noisy estimate or the true state, and which one does the model output match?
4. **Which scenario drives the bad rounds:** per-scenario real cost of (d) in rounds 2–5, and the tilt.

Report the facts and your reading; do not change the method, the costs or the gates.

## Part C: speed-up

### C1. Memory and idle compile helpers

- Today, each worker process starts 22 torch compile helpers (92 in total). Set `TORCHINDUCTOR_COMPILE_THREADS` to the smallest value that works (try 1) for all DiffCoTune processes.
- **Report:**
  - memory per process (PSS from `/proc/<pid>/smaps_rollup`, not RSS, because RSS counts shared pages several times);
  - compile time of the first call, before and after.
- Check that the numbers do not change: one fit epoch and one tune epoch give the same cost and gradient to float64 precision, with and without the setting.

### C2. Use the idle CPU cores

- The load was about 4 of 22 during the P1 run.
- Measure one epoch (4 scenarios, 4 processes) at torch threads 2 / 4 / 8 per process. Pick the fastest that keeps load ≤ 22.
- Add a small job runner that starts independent jobs side by side (for example the comparisons (a)–(d), the plants, the final-evaluation seeds, the frsim "real flights") under these limits:
  - at most 16 CPU processes in total;
  - load ≤ 22;
  - MemAvailable ≥ 3 GB, checked before each start, and a new job waits if a limit is reached.
- Fit and tune within one round, and the rounds of one loop, stay sequential (each needs the one before).

### C3. The 4 scenarios in one GPU batch

Stage 2 measured that the GPU is launch-bound: at B = 1 it is slower than the CPU, but 16 rows cost almost the same as one. To use that:
- Put A1, A8, CIRCLE, FIG8 into one batched rollout:
  - padded to the longest scenario;
  - each scenario's cost uses only its own steps (mask);
  - the top drone and the downwash act only in A1 / A8 rows.
- Try CUDA graphs or `torch.compile` with `mode="reduce-overhead"` if launches still dominate.
- **dtype:**
  - check float64 on the GPU first;
  - use float32 only if float64 is too slow, and only if float32 passes the checks below.
- **Checks (gates, same as Stage 2 / 3):**
  - **forward:** batched GPU vs the CPU model, all 4 tuning scenarios × default and 2 other gain sets × P1 β; relative 3-D RMS difference ≤ 0.5 %;
  - **gradient:** the combined-cost gate (same sign and relative ≤ 1 % or absolute ≤ 0.1 % of the largest component), J_task and J_sysId, against the CPU model.
- **Measure:**
  - seconds per epoch;
  - minutes per round (50 + 50 epochs), against today's ~18 min.
- One GPU job at a time. The NVML warning is known; do not touch the driver.
- The CPU path stays the default until the user switches.

### Not in this stage (options only; list the expected gain and the deviation)

- **Fitting on short pieces of each flight** (2–4 s, starting from the logged state) instead of whole flights. This is a deviation from the paper.
- **A coarser time step** (2 ms). This needs a new forward check.

## Report back (short)

1. P2 stopped: the PIDs and the `ps` check before and after.
2. Part A: the report file, and the pass criteria yes / no.
3. Part B: the 4 checks (tables) and your reading of the cause.
4. Part C:
   - memory per process before / after;
   - thread timing;
   - job-runner limits tested;
   - GPU batch forward and gradient gates pass yes / no;
   - minutes per round, CPU vs GPU.
5. A proposal for the next run (what to fix, expected time), for the user to decide.

## Rules

- **Resources:**
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22 and MemAvailable ≥ 3 GB before each start;
  - no run longer than 1 h without the user's OK.
- **Edits only in:**
  - `experiments/diffcotune/`, with results in `dryrun/stage3b/` and a new `dryrun/stage3c/`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 3c".
- **Do not touch:**
  - `simulator/` (if a change is needed, stop and ask);
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis.
- **Firmware trees and `~/Desktop/crazyswarm2`:** read only. No firmware rebuild.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:**
  - no `pkill`, `killall` or kill of a PID found by pattern;
  - only the P2 PIDs above (user OK), and later only exact PIDs you started yourself.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
