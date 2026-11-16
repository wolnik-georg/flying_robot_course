# DiffCoTune, Stage 4b resume for S0: finish the round-1 dry test, close the gyro and P2-stall checks, and write the report (2026-11-13, for a new agent)

Paste everything below the line into the new tuning (main) agent.

---

You are taking over from a tuning agent whose session was killed at 14:52:53 on 13 Nov. Read this whole prompt first. It gives the context, the state at the kill, and what is left. Everything was checked against the files at 15:15.

## 1. Context

**Project:** master's thesis, Crazyflie two-drone downwash study. Repository `~/Desktop/flying_robot_course`, branch `feature/auto-tuning` (stay on it).

**Direction (meeting 11 Nov):**
- DiffCoTune (Krishna et al. 2025, arXiv:2505.24068) is the only auto-tuning method.
- One gain set per controller, tuned on several scenarios together.
- 8 controllers: S0, S1, S2, S3; FBL base; FBL + downwash model (ROM); FBL + learning-based part (net); FBL + both.
- S0 is done first, end to end.

**Method version (user decisions, 12–13 Nov; in `docs/diffcotune_spec.md`, deviations 8–11):**
- Split-Alternate: fit β with J_sysId (θ fixed), then tune θ with J_task (β fixed); Adam, 50 + 50 epochs; keep-best over rounds l = 0…5; each update starts from θ_l.
- Full 50 epochs, best iterate (deviation 8).
- J_sysId = position + velocity + attitude, no body rates (deviation 9).
- Sensor noise in the tuning step, 2 fixed noise seeds per round (deviation 10).
- ±25 % per-round gain limit around θ_l, projected Adam (deviation 11).
- Learning rate 0.01; `--skip-last-fit`.
- β for S0: thrust-law gain, motor lag, downwash scale, plume scale (log factors).
- Tuning scenarios A1, A8, CIRCLE, FIG8; test scenarios A4 (2 cells), OVAL.
- Floor rule: z mean of the scored part < 50 % of the reference z mean counts as failed.

**Dry-run plants:** P1 = thrust ×0.8, lag 45 ms, downwash ×1.3, plume ×0.7. P2 = P1 + 10 ms state delay (md10).

**Where things are:**
- Plan and decisions: `docs/plan_diffcotune_2026-11-11.md` (§5d–5g).
- Spec and inventory: `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md`.
- Code: `experiments/diffcotune/` (`s0_model.py`, `dryrun/loop_lib.py`, `usd_ingest.py`, `lab/round.py`, `lab/dry_test.py`, `flightfiles/`).
- Job runner: `experiments/diffcotune/dryrun/stage3c/job_runner.py` (the only one).
- Earlier prompts: `docs/HANDOVER_DIFFCOTUNE_STAGE3E_FULLRUN_STAGE4A_2026-11-13.md`, `docs/HANDOVER_DIFFCOTUNE_STAGE4B_S0_CHECKS_2026-11-13.md` (the task you are finishing).
- Lab protocol draft: `docs/diffcotune_lab_protocol_S0_DRAFT.md`.

## 2. Stage 3e / 4a: done and verified

- B1 (P1, 6 rounds, new seeds) passes all criteria. Real cost per round 19.99 / 9.71 / 8.30 / 8.16 / 8.13 / 7.77 cm; final evaluation tuning / test 6.39 / 7.76 against default 15.36 / 9.02 cm (`dryrun/stage3e/results/pass_P1.md`).
- B2 (P2) passes too: 19.28 / 9.77 / 7.62 / 7.60 / 7.70 / 8.21 cm; keep-best round 3; final 6.29 / 7.65 against 15.01 / 9.27 cm (`pass_P2.md`).
- Ingest, `round.py`, flight files (within 0.003 mm of frsim; figure 8 turned 90°) and the protocol draft exist.
- Desk time per round about 19.4 min (CPU pool).

## 3. Why the session died

At 14:50:17 two jobs started **at the same time**, each with its own worker pool:
- `lab/dry_test.py` (round 1 of the dry test, `strict=False`);
- `lab/dry_round1_strict.py` (the same round with `strict=True`, started by `dryrun/stage4b/start_strict.sh`).

One minute later MemAvailable was 1.2 GB (`dryrun/stage4b/logs/chain.log`, 14:51:17), below the 3 GB limit. At 14:52:53 `systemd-oomd` killed the whole terminal scope (167 processes, `journalctl`), including the agent. Nothing is running now (checked 15:13, load 0.4, MemAvailable 12.6 GB).

**Rule from now on:** one worker pool at a time. One pool needs about 6 GB. If MemAvailable is below 3 GB one minute after a start, stop that job (exact PID) and report.

## 4. Stage 4b: state at the kill

### Done (checked)

1. **Corrections in `dryrun/stage3e/report_stage3e.md`:** dry test was one round only, the `t_round` crash, A4 cell 1 worse than default in P1 (9.48 vs 8.80 cm), P1 against Stage 3d A + B (6.39 / 7.76 vs 5.60 / 6.89 cm), the P2 stall, the printed Jn. All present.
2. **`loop_lib.py`:** the fit and tune log lines now print Jn at the last epoch and at the best iterate (`_jn_best`). Results unchanged.
3. **`dry_test.py`:** rewritten to run rounds 0 and 1 from start to end, `t_round` bug gone. Not yet run to the end.
4. **Dry test round 0, rerun (14:29–14:50):** identical to before (fit and tune costs, gain-limit hits [6, 0, 8, 0, 27, 0, 0, 0, 0, 18]). The round-1 flights were written (`lab/dry/round1_logs/`), and the ingest gave real cost 9.71 cm, keep-best round 1, matching B1. Then the job was killed during the round-1 fit.
   - On disk: `lab/dry/S0/round_0/` (report, state, gains), `lab/dry/S0/round_1/` (state, gains), `lab/dry/round{0,1}_logs/`. No `round_2/`.
   - `lab/results/dry_test.json` is still the **old** round-0-only file from 14:09; `dry_test_round0_reuse.json` is a copy of it.
   - `lab/dry_strict/S0/round_1/state.json` is a copy, nothing more.
5. **Gyro check (`lab/gyro_check*.py`, `lab/results/gyro_check*.json`, `dryrun/stage4b/logs/gyro_check*.log`):**
   - Logged gyro against the derivative of the logged attitude: correlation 0.96–0.997 per axis on all 10 flights. Sign, axes and units are consistent. The turned frame gives the same.
   - Power above 20 Hz: 0.00–0.02. Not vibration.
   - S0 model body rates against frsim: max difference ≤ 7e-4 rad/s; slope to the model's own attitude derivative 1.00–1.05.
   - Cause: real and model both oscillate at about 1.0–1.4 Hz with large body rates, and their phases do not line up (correlation about 0 on A1, −0.58 to 0.75 on A4 / A8; the dominant frequency is 1.27 Hz real against 1.07 Hz model on A1). So it is a dynamics gap, not an ingest error.
   - The open item 7 of the protocol draft was updated with this.
   - **Not done:** J_sysId per part (position, velocity, attitude) reported again; whether the attitude part on A1 carries the same phase problem.
6. **P2 stall raw data (`dryrun/stage4b/results_p2_stall_raw.txt`):** per-epoch J/J0, gradient norm and max step for P2 rounds 2 and 3 and P1 rounds 2 and 3.
   - P2: the cost rises at the first step (1.21 and 1.31) and never comes back below 1. The gradient norm jumps by orders of magnitude between epochs (693 → 1e6 in round 2). Most gradient signs flip between epoch 0 and 1. No gain-limit hits.
   - P1 rounds 2 and 3 also rise at the first step but come back below 1 (best 0.85 and 0.93).
   - **Not done:** the reading, the β comparison, and the report.

### Not done

- `dryrun/stage4b/report_stage4b.md` (does not exist).
- Round 1 of the dry test (θ₂ against `loop_P1.json`), the `strict=True` comparison, the yaml round trip for `round_2/gains.yaml`.
- `lab/README.md`, `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md`: no Stage 4b entries yet.

## 5. Do

Run the jobs **one after another**, through `dryrun/stage3c/job_runner.py`, never two pools at once.

### 5.1 Dry test, round 1 (about 20 min, then about 20 min for strict)

- Reuse round 0 from `lab/dry/` (it matches B1); do not redo it. Add a `--from-round 1` option (or similar) to `dry_test.py` that keeps `lab/dry/` and runs only round 1 on the existing `round1_logs/` and `round_1/state.json`.
- **Gate:** θ₂ and the round-1 β equal `dryrun/stage3e/results/loop_P1.json` (round 2 φ, round 1 `psi_after_fit`), max difference ≤ 1e-12; real cost 9.71 cm within 0.01 cm; keep-best state correct after round 1; yaml round trip for `round_2/gains.yaml`.
- Write `lab/results/dry_test.json` with round 0 (from the existing report) and round 1.
- **After it has finished**, run round 1 once with `strict=True` on a copy (`lab/dry_strict/`). Report whether θ₂ is the same. Change `dry_round1_strict.py` so it does not wait for or run alongside `dry_test.py`; delete `dryrun/stage4b/start_strict.sh` or mark it "not used".

### 5.2 Gyro and attitude (desk, little CPU)

- Report J_sysId per part (position, velocity, attitude) on the 23 Oct flights with β₀ and the flown gains, as in Stage 3e, and check whether the attitude part on A1 has the same phase problem as the gyro.
- **Real A1 tilt:** in `A1_2026-10-23_19-47-31_merged_usd.csv`, `cf5_unseen.pitch_deg` has a median |pitch| of 25° and a 90th percentile of 35°, and `gyro_y` reaches about 400 deg/s; the top drone `cf_second_unseen` stays under 1°. Confirm this per A1 flight, over the scored window only (not take-off and landing).
  - The protocol draft (from `docs/comparative_study_protocol.md` §4) aborts at |roll| or |pitch| > 25° for 0.5 s. Check whether the 23 Oct A1 flights would have hit this rule. If yes, list it in the open points of the protocol draft. Do not change the rule.
- No fix is needed in the ingest unless you find an error. If a fix needs `simulator/`, stop and ask.

### 5.3 P2 stall (desk, under 1 h, little CPU)

From `results_p2_stall_raw.txt` and the loop JSONs:
- your reading: is the J_task surface rough or nonsmooth near θ₂ in P2 (e.g. floor contact, crash edge, noise seeds), or is the first step too big? Use the gradient-norm jumps and the sign flips.
- the β of the P2 round-2 and round-3 fits against the P1 truth, and whether the 10 ms delay (not part of β) explains the gap. Compare with P1 rounds 2 and 3.
- If a fix seems needed (e.g. a smaller step after round 1), propose it for the user. Do not change the method.

### 5.4 Report

Write `dryrun/stage4b/report_stage4b.md`, and Stage 4b entries in `lab/README.md`, `docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md`:
1. **The session kill:** what happened (two pools, oomd at 14:52:53), and what was rerun.
2. **Dry test round 1:** gate yes / no, strict on / off, desk time.
3. **Gyro and attitude:** the cause, J_sysId per part, the real A1 tilt and the abort-rule check.
4. **P2 stall:** the per-epoch table and your reading.
5. **Open lab points**, as a list for the user.

Report back in the chat, short, with the same 5 items.

## 6. Decisions that stay with the user (do not decide them)

- Open points of the protocol draft: scenario order and flights per scenario, `rotate_deg` and command lines, how the yaml is applied, gain readback, file names for `simple_flight.py`, battery rule.
- Number of lab rounds (3–4 instead of 6).
- The other controllers (S1–S3, FBL variants).
- Any lab flight.

## 7. Rules

- **Resources:**
  - `nice -n 19`;
  - **one worker pool at a time** (one pool is about 6 GB);
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22 and MemAvailable ≥ 3 GB, read from `/proc/loadavg` and `/proc/meminfo`, before **and one minute after** each start, with `dryrun/stage3c/job_runner.py`; if a limit is broken, stop that job (exact PID) and report;
  - torch threads 2 per worker; `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - system `/usr/bin/python3` (3.10) for frsim and the model; `~/.pyenv/versions/flying_robots/bin/python` for plots and log analysis; no pip installs;
  - the whole stage within 2 h of wall time.
- **Edits only in:**
  - `experiments/diffcotune/` (with `dryrun/stage4b/`, `lab/`);
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 4b";
  - the open list of `docs/diffcotune_lab_protocol_S0_DRAFT.md`.
- **Do not touch:**
  - `simulator/`: read only; if a change is needed, stop and ask;
  - `flying_drone_stack/`, `~/Desktop/crazyswarm2`, `~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`: read only; read-only git commands only; no firmware rebuild;
  - `experiments/tuning/`, `out/`, the old docs, `docs/papers/`, the meeting docs, the thesis;
  - Stage 3b–3e results (do not overwrite them).
- **Data:** complete uSD logs only, never radio CSVs. Inertia and mass only from the Omar C tree: J = 16.571710e-6 / 16.655602e-6 / 29.261652e-6 kg·m², m = 0.0427 kg.
- **Git and hardware:**
  - no branch switch, checkout, stash, commit or push;
  - no Co-Authored-By;
  - nothing flashed or flown;
  - no reboot; do not touch the NVIDIA driver (the NVML warning is known).
- **Stopping processes:** no `pkill`, `killall` or kill of a PID found by pattern. Only exact PIDs you started yourself, checked with `ps -o pid,ppid,args -p <PID>`.
- **Facts:** never invent numbers or rules; write "not stated" and list it as open.
- **Wording:**
  - time stamps only from the real `date` output;
  - method names in full in docs;
  - FBL is just "FBL";
  - say "test error", never "study score".
