# DiffCoTune, Stage 5: close the S0 lab preparation, then prepare S1, S2, S3 the same way (desk only, 2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 4e report was checked against the files:
- **Round costs:** T1 and T2 recomputed from `loop_*.json`. Confirmed.
- **Final evaluations:** T1, T2 and the B1 reference recomputed from `eval_*.json` / `ref_eval_B1_5rounds.json`. Confirmed.
- **Stop rule:** `_adam_paper` follows p. 4 literally. Confirmed.
- **Scope:** nothing outside the allowed files changed.

**Corrected by me:** spec §17, row S3b-1 said "test error 13.00 vs 6.41 cm". These are the tuning means; it now reads "tuning mean 13.00 vs 6.41 cm, test mean 8.56 vs 7.61 cm".

## User decisions (13 Nov evening)

1. **Deviations 8 and 9 stay** (stop rule: 50 epochs with the best iterate; J_sysId = position + velocity + attitude). The **method version is frozen** for all controllers:
   - DiffCoTune Split-Alternate;
   - 50 epochs per inner loop, best iterate;
   - J_sysId = p + v + q;
   - noise in the tuning step, 2 seeds per round;
   - ±25 % gain limit (projected Adam);
   - learning rate 0.01;
   - Adam reset per inner loop;
   - each update starts from θ_l;
   - 5 rounds (l = 0…4), keep-best, `--skip-last-fit`.

   Do not change it for S1–S3. If a controller fails under it, report the failure; do not tune the method.
2. **Supervisor summary** written: `docs/diffcotune_supervisor_summary_2026-11-13.md` (do not edit). The supervisor's agreement is still open; the desk work goes on.
3. **Scope now: S0, S1, S2, S3.** FBL later (do not touch the FBL copies, images or docs).

## Part A. S0: close the lab preparation (desk)

Nothing new in the method. Only what the lab day needs.

### A1. Offline preflight in one command

Write `lab/preflight.py --controller S0` (later S1–S3). It runs every offline check and prints PASS / FAIL per line:
- **Imports:** frsim bindings importable with `/usr/bin/python3`; the model compiles one short rollout.
- **Flight files:** `fly_file.py --check --pad 0.0 0.3` for CIRCLE (turned) and FIG8; OVAL refused.
- **Round state:** `round.py --init` on a temporary state dir. The `round_0/gains.yaml` holds the default gains and passes a round trip.
- **Settings:** `flights.json` template with `rotate_deg` −90 for CIRCLE and FIG8; `--n-rounds 5` is what the protocol names.
- **Resources:** free disk space for 5 rounds of logs; load and MemAvailable against the limits.
- **Lab-PC paths:** nothing that depends on this desk PC's paths without a flag (the lab PC is ROS 2 Jazzy, `~/georg/ros2_ws`; this PC has ROS 2 Humble).

### A2. ROS test without hardware, as far as possible

- Find out from the crazyswarm2 code (read only) whether `apply_gains.py readback|set` and `fly_file.py` (without `--check`) can be exercised on this PC with the crazyswarm2 `sim` backend. This must work without changing crazyswarm2, without a radio and without a drone.
- If yes, run them against `sim` and report what was covered, and what only the lab can test (the firmware readback over the radio).
- If no, say why in one line, and list the exact first-block lab steps for these two scripts.

### A3. One-page lab card

Write `lab/S0_LAB_CARD.md`, taken only from the protocol draft and the existing rules (no new rules):
- **Before the block:**
  - uSD file count on both cards, archive at ≥ 85 (memory rule; `check_usd_deck.py`);
  - active controller check after flashing (`check_active_controller.py`), not param readback alone;
  - pad check;
  - `preflight.py`.
- **Per round:** the 4 flight commands in order, `flights.json`, the `round.py` command, `apply_gains.py merge` + `readback`.
- **Rules:** abort rule (0.5 s continuous), repeat rule, battery rule (`comparative_study_protocol.md` §4), test flights.
- **Sources:** each line names its source (file and section).

## Part B. S1, S2, S3: the S0 pipeline per controller (desk)

S0 took several days because the method was being built. For S1–S3 the method and the lab tools exist. Only the controller-specific parts are new. Work strictly in the order **S2, S1, S3**, one controller at a time:
- **S2:** S0 plus NS2, with no RPM delay, so it is the closest to S0.
- **S1:** adds INDI and the RPM delay.
- **S3:** S1 plus NS2.

Report after each controller (append to the report).

### B0. Facts to read first (read only)

- `simulator/frsim/controllers.py`:
  - S1 = tree with INDI mode 3;
  - S2 = tree with NS2;
  - S3 = INDI 3 + NS2;
  - all four use the same 10 tree gains `TREE_GAIN_NAMES` and `TREE_DEFAULT_GAINS`.
- Old PyTorch copies: `experiments/tuning/diff/s1_plant_torch.py`, `s2_plant_torch.py`, `s3_plant_torch.py` and their `check_s*_equiv.py`. Inventory §1 gives their state. They are **not** on the frsim plant of the S0 model and use the rounded inertia.
- Decision 5b.2 (plan): β per controller holds only the values that act on it. The **RPM delay is fitted only for S1 and S3** and must be a smooth, fractional delay. Inventory gap 3 says that frsim truncates it to whole ms (`sensors.py:54`).
- Real logs for the real-data check: `experiments/logs/bank_unseen_S1_2026-10-23`, `bank_unseen_S3_2026-10-23` and `bank_unseen_S3S2_2026-10-23`. Read the controller of each flight from its manifest or meta; do not guess from the folder name.

### B1. Differentiable model per controller (as `s0_model.py`)

- Write `s2_model.py`, `s1_model.py` and `s3_model.py` in `experiments/diffcotune/`.
  - Use the same frsim plant, exact Omar inertia, the same β definition (log-factors: thrust gain, motor lag, downwash, plume), the same θ (10 tree gains, log-factors), full-state output, the frsim noise option, and the compiled whole-flight rollout.
  - Reuse `s0_model.py` code by import where possible. Copy from `experiments/tuning/diff/` only with a header that names the source.
  - NS2 uses `experiments/diffcotune/ns2_torch.py` with the fixed `ns2_v2` weights, the same as the firmware.
- **S1 and S3: RPM delay as a fifth β.**
  - Make it fractional (linear interpolation in the RPM history) in the model.
  - Add the same fractional delay to frsim as a **new option, off by default**. Existing frsim results must stay bit-identical; show this with `simulator/tests` (reference compare) or an equivalent check.
  - `simulator/` changes stay local; do not commit them.
- **Forward check** (as `checks/forward_check.py`, S0 gate):
  - noise off: 6 scenarios × 5 parameter rows, relative 3-D RMS difference ≤ 0.5 %;
  - with noise: the Stage 3d revised gate.
  - If a controller fails the gate, stop that controller and report; go on with the next.
- **Gradient check** (as `checks/gradient_check.py`): all θ and β, J_task and J_sysId, A8 / CIRCLE / A1. Same gate, and report A1 the same way as for S0 (marginal floor contact).
- **Runtime per epoch** with the 4-worker CPU pool, and the estimated minutes per round.

### B2. Identifiability check per controller (fit only, as Stage 3b B1 / Stage 4e T0 with `pva`)

- P1 truth: thrust gain 0.8, motor lag 1.5, downwash 1.3, plume 0.7 (factors).
- For S1 and S3, also an RPM-delay shift. Use the plan's dry-run value of 8 ms (plan §4 Stage 3). State the calibrated value it is compared with, read from `simulator/frsim/data/calibration.json`, and call the plant **P1-rd**.
- Method: noisy frsim flights of θ₀ on the 4 tuning scenarios, 50 epochs, best iterate, lr 0.01.
- Report the β error per value. For S1 / S3, say whether the RPM delay is recovered. If not (e.g. error > 50 % or wrong direction), report it. **Do not drop it yourself:** the user decides whether it stays a β.

### B3. Dry run per controller (as Stage 3e B1, but 5 rounds)

- **Plant:** P1 (S2) or P1-rd (S1, S3).
- **Seeds:** flight seeds 100 + 10·l + i, noise seeds 6000 + 2·l (+1), evaluation seeds 2000–2004. Use the same seeds for all controllers.
- **Scenarios:** tuning A1, A8, CIRCLE, FIG8; test A4 (2 cells), OVAL.
- **Pass criteria:** the Stage 3e criteria a, b and c (`dryrun/stage3e/results/pass_P1.md`) for 5 rounds:
  - a. rounds 2–4 ≤ round 1 + 1 cm;
  - b. no flown set after θ₀ fails (tilt or floor) a scenario that θ₀ or θ₁ passed;
  - c. θ_best beats θ₀ on the tuning and the test means (5 seeds).
  - Also report the β error per round.
- **If a run fails a, b or c:** report the cause, read from the tables. No method change and no reruns with other settings.
- Also run the real-data check of Stage 4b / 4e for this controller: J_sysId per part (p, v, q, and w for information) on the 23 Oct flights with β₀ and the default gains.

### B4. Lab tools per controller

- **`round.py --controller S1|S2|S3`:**
  - pick the model, β list and gain group per controller;
  - S0 behaviour must stay identical. Show this by rerunning the round-0 part of the dry test on the existing fake files (`lab/dry/`) and comparing θ₁ with `lab/results/dry_test.json` (difference 0.0), or a cheaper equivalent with the same proof.
  - Then a round-0 dry test per controller (4 frsim flights as fake merged files, against `loop_lib` with the same seeds; θ₁ identical).
- **`apply_gains.py`:** the parameter names and the strategy switches per controller, read from the firmware (read only) and the crazyswarm2 branches `study_S1`, `study_S2`, `study_S3` (read-only git commands, e.g. `git show study_S1:<path>`; no checkout).
  - The merge must keep the strategy switches of each branch unchanged.
  - The readback must check the gains **and** the switches.
- **Logs:** state per controller which uSD variables the ingest needs (e.g. RPM for S1 / S3), and check that `usd_study_config.txt` has them.
- **`preflight.py --controller S1|S2|S3`.**

### B5. Protocol

- Do not copy the S0 draft three times. Add a section "S1–S3" to `docs/diffcotune_lab_protocol_S0_DRAFT.md` with a table per controller: yaml branch, switches, β list, round time, flight card differences, known risks.
- Known risk to check against the files and state with its source: on hardware only 64–72 % of the thrust that INDI commands arrives at the A8 crossings (S0 / S2: 96–109 %), while the simulator's INDI is near-ideal (memory `project_s3_version_a_vs_b_review_2026-10-18.md`; likely cause motor headroom, not proven). Find the analysis behind it in `docs/` or `experiments/`, and say whether the real-data check of B3 shows a gap that no β covers.
- Lab time for 4 controllers: per controller 5 rounds × 4 tuning flights plus 12 test flights, with your measured desk time per round.

## Report back (short): `dryrun/stage5/report_stage5.md`, updated after each part

1. **Part A:** preflight output, the ROS sim test result (or why not), the lab card.
2. **Per controller (S2, S1, S3):**
   - forward and gradient checks;
   - identifiability (β errors; RPM delay recovered yes or no);
   - dry run: real cost per round, criteria a–c, final evaluation, β per round;
   - real-data check;
   - lab tools tested (yes / no, with what);
   - minutes per round.
3. **Open points** for the user, with the decision each needs.

Also update `lab/README.md`, `experiments/diffcotune/README.md`, and `docs/diffcotune_spec.md` / `docs/diffcotune_inventory.md` marked "Stage 5".

## Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (about 6 GB each); every job through `dryrun/stage3c/job_runner.py` (load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo` before and one minute after each start); if a limit is broken, stop that job (exact PID) and report;
  - at most 16 CPU processes; one GPU job at a time;
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - jobs strictly one after another;
  - each dry run within 3 h. If the estimate after round 1 is longer, stop after that round and report.
- **Shells:** no command that waits on standard input (no bare `cat >`); use heredocs or the edit tools.
- **Python:** `/usr/bin/python3` (3.10) for frsim and the models; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage5/` and `lab/`); new switches default to the current behaviour so earlier results stay reproducible;
  - `simulator/`: only the new RPM-delay option, off by default, bit-identical otherwise, local only;
  - `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md` (marked "Stage 5");
  - the protocol draft (new section S1–S3, P5 times; the decided S0 items stay as they are).
- **Do not touch:**
  - `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only (no checkout, no branch switch); no firmware rebuild;
  - `experiments/tuning/` (copy with a header instead), `out/`, the old docs, `docs/papers/` (reading is fine), the supervisor summary, the meeting docs, the thesis, `docs/comparative_study_protocol.md`, everything FBL;
  - Stage 3b–4e results.
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
