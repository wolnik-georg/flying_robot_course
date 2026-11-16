# DiffCoTune, Stage 3e + 4a for S0: two quick checks, full A + B runs (P1, then P2), and the Stage 4 desk work (2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 3d report was checked against the result files:
- **Real cost per round:** matches `loop_*.json`.
- **Final evaluations:** match `eval_*.json`.
- **β errors after the round-2 fit:** match.
- **Floor-rule table:** matches.
- **Keep-best confirmation:** 19.80 cm, the same θ₁.
- **Nothing is running.**

## User decisions (13 Nov)

1. **A + B is the method version:**
   - sensor noise in the tuning step, 2 fixed seeds per round;
   - a ±25 % per-round gain limit around θ_l;
   - full 50 epochs with the best iterate;
   - J_sysId = position + velocity + attitude;
   - learning rate 0.01;
   - `--skip-last-fit`.

   Spec deviations 10 and 11 stay as written.
2. **Gradient check with noise:** accepted as it is: 19 of 20 components pass at h = 1e-6; KRxy at the default point fails by 1.68 % in A1 floor contact. Note it in the spec; do not block on it.
3. **Order:**
   - Part A: two quick checks;
   - Part B: full A + B on P1 with new seeds, then P2;
   - Part C: the Stage 4 desk work for S0, alongside B where resources allow.

## Part A: quick checks (under 30 min together)

1. **The Inductor workaround** (`s0_model._patch_inductor_loop_split`) has not been checked against an unpatched path.
   - Compare the compiled, patched batched rollout (B = 2 rows, the 2 noise seeds, as used in the A / A + B runs) with the **eager** float64 model, same inputs.
   - **Cells:** 4 tuning scenarios × {default, θ₁ and θ₃ of the Stage 3d A + B run} × P1 β.
   - **Report:** J_task, and the gradient of J_task with respect to θ.
   - **Gate:** relative difference ≤ 1e-9 for J and ≤ 1e-6 for each gradient component. That is numerical identity up to reordering; our choice.
   - If it fails, stop and report; do not start Part B.
2. **Load gate:** show the line in `job_runner.py` (and in any chain script) that reads the load. It must read `/proc/loadavg`: that always uses a dot; the German locale prints `0,24` in `uptime`.
   - Fix the decimal-comma bug you found wherever the load is parsed.
   - Every worker sets its torch thread count (2) before it starts.
   - The runner checks the load again one minute after each start and logs it.

## Part B: full A + B runs

### B1. P1, 6 rounds, new seeds

- **Rounds:** l = 0 … 5.
- **New seeds:**
  - **flight seeds** different from all earlier runs, e.g. 100 + 10·l + i (state what you use);
  - **noise seeds for the tuning step**, likewise new.

  Rounds 0–3 are then a second sample of the Stage 3d A + B run, and rounds 4–5 show whether more rounds help.
- **Final evaluation** in the "real drone", 5 seeds (also new), all 6 scenarios:
  - θ₀ (default);
  - θ_best;
  - θ₅ (the last flown set).
- **Pass (P1):**
  1. Rounds 2–5 ≤ round 1 + 1 cm.
  2. No flown set after θ₀ fails (tilt or floor) a scenario that θ₀ or θ₁ passed.
  3. θ_best and θ₅ beat θ₀ on the tuning and on the test scenarios (mean over 5 seeds).
  4. β per round: report the error per value (no gate).
- **Run-to-run spread (no gate):** rounds 0–3 of this run next to the Stage 3d A + B run, as real cost per round, β error and θ factors.

### B2. P2, 6 rounds (information only)

- Start only after B1 is finished and its pass table is written.
- Same as B1 on P2 (P1 + state delay md10), with the same new seeds.
- Report the same items. No gate.

### Time

- About 2 h per run plus the evaluation.
- Each run within 3 h of wall time; if the estimate after round 1 is longer, stop after that round and report.
- B1 and B2 run one after the other (two pools do not fit in memory).

## Part C: Stage 4 desk work for S0 (no flights, no flashing)

Do these while B runs, as far as the resource limits allow. They are mainly code and reading, with little CPU.

### C1. Real uSD ingest

A function: round folder → per scenario
- the arrays x_t^sys (position, velocity, attitude) on the model's time grid;
- the reference p*;
- the scored window;
- a crash / floor flag;
- the real cost (spec §4).

Requirements:
- **Inputs:** the merged uSD files (`*_merged_usd.csv`, `flying_drone_stack/tools/merge_usd_logs.py`). Never radio CSVs.
- **Conventions:** degrees to radians; the legacy pitch sign; quaternion order; time zero = scenario start (`t_zero=scenario_start`).
- **Reference check:** the logged `ctrltarget` against the model's / frsim's reference for the same cell. Report the max difference. A large one means the flight file and the simulator differ.
- **Top drone:** an option to use the **logged** top-drone path as the peer input in the S0 model, instead of the planned one (in `experiments/diffcotune/`, not in `simulator/`).
- **Quality gate per flight,** reusing `check_usd_log_quality.py` where possible: rows, rate, gaps, NaNs, both drones present for A1 / A8.
- **Test** on the existing S0 flights of 23 Oct (`experiments/logs/bank_unseen_S0_2026-10-23/`):
  - A1, A8, A4: ingest, quality gate, reference check;
  - J_sysId of the S0 model with β₀ and the flown gains against these flights, per state part;
  - the noise floor per part: does velocity sit on a floor here too?

  Report it. Do not fit β on these flights yet.

### C2. One command per round

`round.py --controller S0 --round l --logs <folder>`:
- ingest + quality gate;
- fit β (skipped in the last round);
- tune θ (A + B);
- write:
  - the next gain file;
  - a round report (real cost per scenario, β, θ, keep-best state);
  - a state file for the next round (θ_l, θ_best, β, Adam settings, seeds).
- **Gain file:** written to `experiments/diffcotune/lab/S0/round_<l>/`, in the crazyswarm2 yaml format (read the format from `~/Desktop/crazyswarm2`; do not write there).
  - Name the yaml keys from the S0 firmware parameters.
  - Check the round trip: yaml → gains → the model's θ.
- **Dry test:** run the command on 4 frsim flights (written as fake merged-uSD files in the same format) for 2 rounds; it must give the same θ as `loop_lib` for the same inputs.
- **Desk time per round:** report it. A round must fit in a lab block.

### C3. Flight files for circle, figure 8, oval

- The simulator uses smooth 2 s ramps and a 1.8 m oval (`docs/SCENARIOS.md`); the existing flight CSVs differ.
- Produce flight files that reproduce **exactly the frsim reference**, in the format the lab uses (read `export_poly4d.rs` and `simple_flight.py`; propose the way, for example a new export or a sampled trajectory).
- **Output only in** `experiments/diffcotune/flightfiles/`. Nothing is copied to crazyswarm2.
- **Checks:**
  - sampled position against the frsim reference, max difference ≤ 1 mm;
  - peak speed and acceleration;
  - the flight volume (±2.0 m, height limits) with margins;
  - the start and end positions.
- A1 / A8 / A4 stay as they are; confirm that their flight files match the frsim reference the same way.

### C4. Lab protocol draft (S0)

Write `docs/diffcotune_lab_protocol_S0_DRAFT.md`, plots first, little text. The user freezes it later.
- the rounds and the order of the scenarios in a block;
- the flight card per round;
- the uSD file-count check before every block (archive at ≥ 85 files);
- `check_usd_deck.py` before flying;
- the active-controller check after any flash;
- crash / floor handling: repeat, not used for the fit, at most 2 repeats;
- keep-best;
- what is loaded when;
- battery rules;
- the desk time between rounds from C2.

Use only existing lab rules from the repository (name the source of each). Do not invent new ones; list any gap as "open".

## Report back (short)

1. **Part A:** workaround gate pass yes / no; the load-reading lines.
2. **B1 P1:**
   - real cost per round;
   - failed flights;
   - gain-limit hits;
   - β error per round;
   - final evaluation (θ₀, θ_best, θ₅);
   - pass criteria yes / no;
   - the run-to-run table.
3. **B2 P2:** the same items, as information.
4. **C1–C4:** what exists (files), the test results, and the open points.
5. **Your proposal** for the next step (the other controllers, lab readiness), for the user to decide.

Report to `dryrun/stage3e/report_stage3e.md` and `lab/README.md`.

## Rules

- **Resources:**
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22, checked from `/proc/loadavg` before **and one minute after** each start;
  - MemAvailable ≥ 3 GB;
  - torch threads set in every worker;
  - `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - each run ≤ 3 h.
- **Edits only in:**
  - `experiments/diffcotune/`, with new `dryrun/stage3e/`, `lab/` and `flightfiles/`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 3e / 4a";
  - the new `docs/diffcotune_lab_protocol_S0_DRAFT.md`.
- **Do not touch:**
  - `simulator/`: read only; if a change is needed, stop and ask;
  - `flying_drone_stack/`: read only; a new export goes to `experiments/diffcotune/`;
  - `~/Desktop/crazyswarm2` and the firmware trees: read only; no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis.
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
