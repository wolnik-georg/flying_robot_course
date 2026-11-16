# DiffCoTune, Stage 4e for S0: test the paper's own stop rule and the gyro in the simulator fit, before the lab (2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 4d work was checked: the fixed pad check (own recalculation from the first point matches), the turned circle (+90°, margins, 19 pieces), `fly_file.py` refusing OVAL, nothing changed in crazyswarm2. Confirmed.

## User decisions (13 Nov evening)

1. **Lab rounds: 5** (l = 0…4, `round.py --n-rounds 5`), replacing 4. Already in the protocol draft and plan §5k. Update the lab-time table in the protocol draft to 5 rounds × 4 tuning flights.
2. **Test flights:** A4 cell 1 and 2, 3 flights each, with the default gains (round 0) and with θ_best (after round 4). Battery rule as in `docs/comparative_study_protocol.md` §4. Already in the protocol draft.
3. **Stay as close to the paper as possible.** Every deviation must be either needed for our setup (one gain set for several scenarios) or backed by a test. Two deviations were never tested on their own and are tested now.

## Why these two tests

Read `docs/papers/krishna2025diffcotune.pdf` (not only the summary) for the points below.

- **Stop rule (our deviation 8, spec §5 item 1 of Stage 3b):** the paper stops each inner loop at |J_k − J_{k−1}| < 10⁻³ or J_k > J_{k−1} + 10⁻³ and returns the last iterate (p. 4, Alg. 2 and 3). We only ever tested our own **normalised** version (Stage 3), which failed. The paper's **literal** rule on our SI costs was never run. Spec §5 item 8 (Stage 1) predicts it stops at the first epoch; that is a prediction, not a result.
- **Gyro in J_sysId (our deviation 9):** the paper uses all observable states, unweighted (Eq. 3, p. 2; p. 4). We dropped the body rates in Stage 3b **together with** the stop-rule fix, so the Stage 3 failure with body rates was mixed with the broken stop rule. Step B1 of Stage 3b compared only pos + vel + att against pos + vel. Body rates with 50 epochs were never tested. On hardware the paper filtered noisy velocity feedback with a 5th-order Savitzky-Golay filter (p. 5, Fig. 4); a filtered gyro is therefore the variant closest to the paper's practice.

Deviations 10 (noise in the tuning step) and 11 (±25 % limit) stay: the paper's sim-to-sim tests have no sensor noise, and the paper itself projects θ into a feasible set with a clipping step for the biped (p. 5). Do not test them again.

## Setup (same for every run unless stated)

- Plant P1; tuning scenarios A1, A8, CIRCLE, FIG8; test scenarios A4 (2 cells), OVAL.
- **5 rounds** (l = 0…4), keep-best over the 5 flown sets, last-round fit skipped.
- **Same seeds as Stage 3e B1:** flight seeds 100 + 10·l + i, noise seeds 6000 + 2·l (+1), evaluation seeds 2000–2004. Then rounds 0–4 compare directly with B1 (`dryrun/stage3e/results/loop_P1.json`).
- Everything else as the current method version: noise in the tuning step (2 seeds), ±25 % gain limit, learning rate 0.01, Adam reset per inner loop, each update starts from θ_l.
- **Change one thing per run.**

### Reference

B1 of Stage 3e, rounds 0–4: real cost 19.99 / 9.71 / 8.30 / 8.16 / 8.13 cm. Its final evaluation used θ_best over 6 rounds; recompute the reference final evaluation for **θ_best over rounds 0–4 of B1** (round 4 = 8.13 cm, so θ₄) with the 5 evaluation seeds, so all runs compare on 5 rounds.

## Do

### T0. Identifiability check with the gyro (fit only, about 30 min)

As Stage 3b B1: P1, noisy frsim flights of θ₀ on the 4 tuning scenarios, fit only, 50 epochs, best iterate, learning rate 0.01.
- **Variants:**
  - `pva` (current);
  - `pvqw` raw: position + velocity + attitude + body rates, unweighted, the measured gyro as logged;
  - `pvqw` filtered: the same, with the **measured** gyro smoothed by a zero-phase 5th-order Savitzky-Golay filter before the cost (the model's body rates are not filtered). Window: our choice, state it (e.g. 51 samples at 1 kHz); report one shorter and one longer window too.
- **Report:** fitted β against the truth, error per value; share of each part (p, v, q, ω) in J_sysId at start and end.
- **Real-data check (no fit):** J_sysId per part on the 23 Oct S0 flights with β₀ and the flown gains, for `pvqw` raw and filtered (the Stage 4b gyro phase gap). Does the gyro part dominate on real data?

### T1. The paper's literal stop rule (full loop)

- Fit and tune each run up to 50 epochs and stop at |J_k − J_{k−1}| < 10⁻³ or J_k > J_{k−1} + 10⁻³, in the SI units of the costs, and return the **last** iterate (Alg. 2 and 3).
- States `pva` (as now). Everything else as the setup.
- **Report** per inner loop: epochs run, stop reason, J at start / end; real cost per round; β error; final evaluation.
- If every inner loop stops within 2 epochs in round 0, stop the run after round 1 and report: the prediction holds.

### T2. Body rates in J_sysId (full loop)

- Only after T0. Use the `pvqw` variant (raw or filtered) that recovered β best in T0. If neither is within the β errors of `pva` in T0 (thrust gain first, then downwash), say so and still run the better one, for the record.
- Stop rule as now (50 epochs, best iterate). Everything else as the setup.
- Same report as T1.

### Pass and decision (our choice, stated)

Per run (T1, T2), compared with the B1 reference on rounds 0–4:
1. Rounds 2–4 ≤ round 1 + 1 cm.
2. No flown set after θ₀ fails (tilt or floor) a scenario that θ₀ or θ₁ passed.
3. θ_best beats θ₀ on the tuning and the test scenarios (mean over 5 seeds).
4. **Not worse than the reference** by more than 1 cm on the tuning mean and on the test mean (the run-to-run spread seen between Stage 3d and Stage 3e).

If a run passes 1–4, the paper's version wins and the deviation is undone (the user confirms). If it fails, the deviation stays, now backed by this test. Report the cause either way.

## Report back (short): `dryrun/stage4e/report_stage4e.md`

1. T0 table (β per variant and window, part shares) and the real-data gyro share.
2. T1: stops per inner loop, real cost per round, final evaluation, pass 1–4.
3. T2: the same.
4. Your recommendation per deviation (8 and 9): undo or keep, with the numbers.
5. An updated list of **all** deviations from the paper (spec §5, Stage 1 items 5–11 and Stage 3b–3d items), each marked "needed for several scenarios", "backed by test (which)", or "our reading of an unclear point". Put it into `docs/diffcotune_spec.md` as section "Stage 4e: deviations after the ablations"; fix the double numbering (Stage 1 items 8–11 vs Stage 3b–3d deviations 8–11) with distinct labels.
6. Lab-time table for 5 rounds in the protocol draft.

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (one pool is about 6 GB); runs strictly one after another (T0 → T1 → T2);
  - at most 16 CPU processes; one GPU job at a time;
  - load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo`, before and one minute after each start, via `dryrun/stage3c/job_runner.py`; if a limit is broken, stop that job (exact PID) and report;
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - each full loop within 3 h; the whole stage within 7 h. If the estimate after round 1 of a loop is longer, stop after that round and report.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10) for frsim and the model; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs (check that scipy's `savgol_filter` is available in the 3.10 environment; if not, implement the filter with numpy and test it against a known case).
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage4e/`); new switches default to the current behaviour so earlier results stay reproducible;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 4e");
  - the lab-time table of `docs/diffcotune_lab_protocol_S0_DRAFT.md`.
- **Do not touch:**
  - `simulator/`, `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only; no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/` (reading is fine), the meeting docs, the thesis, `docs/comparative_study_protocol.md`;
  - Stage 3b–4d results.
- **Data:** complete uSD logs only, never radio CSVs.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown; no connection to a drone or radio;
  - no reboot; do not touch the NVIDIA driver.
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open. Quote the paper with page numbers.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
