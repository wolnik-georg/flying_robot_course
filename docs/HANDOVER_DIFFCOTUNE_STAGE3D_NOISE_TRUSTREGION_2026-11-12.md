# DiffCoTune, Stage 3d for S0: noise in the tuning model (A), a per-round gain limit (B), and both (A + B), short P1 comparison (2026-11-12)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 3c report was checked against the files:
- **Oracle (check 1):** 7.29 / 7.00, 5.43 / 9.40, and no step from θ₃. Confirmed.
- **C1 memory (PSS):** 7552 against 6117 MB, bit-identical. Confirmed.
- **C3 GPU gates:** float64 matches the CPU to about 1e-13, at 38 s per epoch. Confirmed.
- **Job-runner self-test:** confirmed.
- **Nothing running; no files touched outside `experiments/diffcotune/`:** confirmed.

Two corrections:
1. **No P1 rerun happened in Stage 3c.** The P1 numbers are from the Stage 3b run (14:12–19:41). Please do not write "P1 is rerun" in the reports.
2. **`report_stage3b.md`, the pass-criteria row "(d) not worse than (b) and (c)"** says that (b) is 3.45 cm "over its 31 flights that did not crash". It is **16** tuning flights (20 minus 4 crashed). Fix that number.

## What we concluded

- The β fit works; started at the truth, it stays there (check 2).
- The **noise-free tuning model is the limit.** With the true β, one tuning step gives no better gains than round 1. Later steps push the gains to where the sensor noise keeps the drone on the floor (check 5).
- θ_best was round 1 in every run so far.

## User decision (12 Nov): short comparison on P1

Three variants, each against the Stage 3b loop (d) as the reference:

| Variant | Change | Spec deviation (stated) |
|---|---|---|
| **A** | Sensor noise in the **tuning** step (J_task): frsim's existing noise model, **2 fixed noise seeds per round**, cost averaged over the 2 seeds | 10: the paper's model rollouts are deterministic and noise-free |
| **B** | **Per-round gain limit:** after each Adam step, clip every gain to within ±25 % of the flown gains θ_l, i.e. \|φ − φ_l\| ≤ log 1.25 in the log parametrisation (projected Adam) | 11: the paper has no bound besides positivity |
| **A + B** | Both | 10 and 11 |

Everything else is as in Stage 3b:
- full 50 epochs, best iterate;
- J_sysId = position + velocity + attitude;
- learning rate 0.01;
- each update starts from θ_l;
- Adam reset per inner loop;
- keep-best;
- `TORCHINDUCTOR_COMPILE_THREADS=1`.

The **β fit stays noise-free** (check 2 showed it works).

## Do

### 1. Noise in the S0 model (for A)

- Read the frsim noise model (`simulator/frsim/sensors.py` and where `episode.py` applies it: which signals, which distribution, rates, any filters or bias), read only. Write down what you find, with file : line.
- Add the same noise to the differentiable S0 model as **fixed noise arrays per seed** (additive on the same signals, generated before the rollout). The noise is not a parameter; gradients flow through the closed loop as before.
- If possible, generate the arrays with **frsim's own generator for the same seed**, so the model and frsim see the identical noise sequence.
- **Forward check with noise** (gate, our choice, stated):
  - **cells:** default gains, round-1 gains and round-3 gains of (d), × P1 β, × the 4 tuning scenarios, × 2 seeds;
  - **if the noise sequence is identical:** relative 3-D RMS difference ≤ 0.5 % per cell, and the floor failure of the round-3 gains is reproduced (z mean, as in check 5);
  - **if an identical sequence is not possible:** compare the mean over 5 seeds, gate ≤ 10 % per scenario, and the floor failure is reproduced;
  - state which case applies.
- **Gradient check with noise:** the combined-cost gate (same sign and relative ≤ 1 % or absolute ≤ 0.1 % of the largest component), J_task, 2 points (default gains, round-1 gains).
- If the forward check fails, stop and report; do not loosen the gate.

### 2. Gain limit (for B)

- A switch `--gain-limit 0.25` (default off).
- After each Adam step, project φ to [φ_l − log 1.25, φ_l + log 1.25] per gain.
- Report how often a gain hits the limit per inner loop.

### 3. Speed of A

- With noise, a round has 4 scenarios × 2 seeds = 8 rollouts per epoch.
- **Measure** one tuning epoch:
  - CPU pool (8 processes if memory allows, otherwise 4 processes with 2 rollouts each);
  - GPU float64 batch of 8 rows (extra rows were almost free in Stage 3c: 4 rows 37.9 s, 24 rows 38.6 s).
- Use the faster path for A and A + B, and say which.

### 4. The comparison runs (P1)

- **For each variant (A, B, A + B):**
  - rounds l = 0 … 3: 4 flown gain sets, 3 updates;
  - real-drone flights with noise on, the same flight seeds as the Stage 3b loop;
  - `--skip-last-fit` on.
- **The reference** is the Stage 3b (d) run, rounds 0 … 3. Do not rerun it.
- **Final evaluation** in the "real drone", 5 seeds, all 6 scenarios:
  - θ_best of each variant;
  - **also θ₃ (the last flown set)**, to see whether the loop still improves after round 1.
- **Running side by side:**
  - you may run A (or A + B) on the GPU and B on the CPU at the same time, if the job runner's limits allow;
  - otherwise run them one after the other;
  - one GPU job at a time.
- **Time limit:** the whole comparison, including the final evaluations, within about 5 h of wall time. If your estimate after the first round is longer, stop after that round and report; do not start more.

### Pass (per variant, P1)

1. The real cost of the flown gain sets does not rise after round 1. Round 2 and round 3 are ≤ round 1 + 1 cm, the margin for flight-to-flight noise; state it as our choice.
2. No flight of any flown gain set stays on the floor or crashes. Add a **floor rule** to the crash check: z mean over the scored part < 50 % of the reference z mean counts as failed. Report it for the earlier runs too.
3. θ_best and θ₃ beat (a) default on the tuning and on the test scenarios (mean over 5 seeds).
4. β after round 3: report the error per value (no gate in this short run).

Do not change the method beyond A / B. If no variant passes, report the cause.

## Report back (short): `dryrun/stage3d/report_stage3d.md`

1. **Noise model:** what frsim does (file : line), what the S0 model now does, and which case of the forward check applies, pass yes / no.
2. **Gradient check with noise:** pass yes / no.
3. **Speed:** seconds per epoch, CPU vs GPU, for 8 rollouts; minutes per round.
4. **Table per variant:**
   - real cost per round (0 … 3) next to the Stage 3b reference;
   - flights failed by the tilt or floor rule;
   - gain-limit hits;
   - β error after round 3.
5. **Final evaluation:** variant × {θ_best, θ₃} × scenario; 3-D RMS / z error; failed flights.
6. Pass criteria per variant, each yes / no.
7. **Your recommendation** for the full run (which variant; whether 6 rounds are still worth it), for the user to decide.
8. Spec deviations 10 and 11 written into `docs/diffcotune_spec.md`, marked "Stage 3d".

## Rules

- **Resources:**
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22 and MemAvailable ≥ 3 GB before each start (use `job_runner.py`);
  - `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - the 5 h limit above.
- **Edits only in:**
  - `experiments/diffcotune/`, with results in a new `dryrun/stage3d/`; Stage 3b / 3c results are not overwritten;
  - the one number fix in `report_stage3b.md`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 3d".
- **Do not touch:**
  - `simulator/`: read only this stage; if a change is needed, stop and ask;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis.
- **Firmware trees and `~/Desktop/crazyswarm2`:** read only. No firmware rebuild.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
