# DiffCoTune, Stages 0 + 1: clean break, frozen spec, inventory (2026-11-11)

Paste everything below the line into the tuning (main) agent.

---

## Context: new direction (meeting with supervisor and professor, 11 Nov 2026)

The auto-tuning changes completely. Read the plan first: `docs/plan_diffcotune_2026-11-11.md`. It is the source of truth; in short:
1. **One method: DiffCoTune** (Krishna et al. 2025, arXiv:2505.24068, `docs/papers/krishna2025diffcotune.pdf`), Split-Alternate scheme, real flights and simulator in a loop, as close to the paper as possible. The method comparison (CMA-ES, Bayesian optimisation, Sobol sampling, gradient descent) and the per-scenario picks end here; they stay on disk as the earlier record only.
2. **One gain set per controller**, tuned on 4 scenarios together (A1, A8, circle, figure 8); A4 and oval are test scenarios, never tuned on. Cost = the paper's mean squared tracking error on 3-D position (no crash penalty, no delay robustness).
3. **8 controllers, each with its own gains:** S0, S1, S2, S3, FBL base, FBL + downwash model (ROM), FBL + learning-based part (net), FBL + both. S2 / S3 no longer reuse the S0 / S1 gains.

All tuning jobs were stopped by the user on 11 Nov at 15:49 (fblv2o chain + GPU job, CPU lane 2, the old csim waiters). No files were deleted; nothing is running.

This prompt covers **Stage 0** (closing lines) and **Stage 1** (frozen spec + inventory). **Desk only, no code changes, no simulation runs.**

## Stage 0: close the old pipeline (text only)

1. Check that nothing of the old pipeline runs: `ps -eo pid,args | grep -E 'fblv2o|csim|train_|run_job|rank_'`. If something still runs, report it; do not kill anything.
2. Append one line, time from `date`, to `experiments/tuning/out/csim_status.txt` and to `docs/tuning_procedure_frozen_2026-11-01.md`:
   "Stopped 11 Nov 2026 15:49 by the user (clean break). fblv2o retune incomplete (gradient-M 15/18, gradient-V 12/18, searches seeds 3–5 partly); provisional closing_table_v3_3s.md is the last table. Superseded by DiffCoTune, see docs/plan_diffcotune_2026-11-11.md."
3. Update the next-action line of `docs/07_Thesis_Progress_Checklist.md` to point at the DiffCoTune plan (one line; do not rewrite the rest).

## Stage 1a: frozen spec `docs/diffcotune_spec.md`

Write the procedure as it will be implemented, from plan sections 1–3. Check every paper statement against the PDF yourself (page / equation / algorithm number next to each); the summary `docs/papers/summaries/krishna2025diffcotune.md` is a help, not the source. Do not edit anything under `docs/papers/`.

Content:
1. **Notation:** θ (gains per controller, names and defaults), β (thrust factor, motor lag, RPM delay, downwash scale, plume amplitude scale, start values from `simulator/frsim/data/calibration.json`), fixed values (mass 0.0427 kg, inertia J = 16.571710e-6 / 16.655602e-6 / 29.261652e-6 kg·m²).
2. **Outer loop** (paper Alg. 1): rounds l = 0 … 5; one flight per tuning scenario per round; keep-best among the 6 flown gain sets (the last update θ₆ is not used).
3. **Split-Alternate** (paper Alg. 3):
   - fit β, θ fixed: J_sysId = mean over the 4 scenarios of (1/T)·Σ‖x_t − x_t^sys‖², closed loop, same controller and reference; states = position, velocity, attitude, body rates; unweighted sum of the squared errors of each part (paper);
   - tune θ, β fixed: J_task = mean over the 4 scenarios of (1/T)·Σ‖p_t − p_t*‖² (3-D position);
   - Adam, K = 100 (50 epochs each), stop when |ΔJ| < 1e-3 or J rises by more than 1e-3 (paper);
   - gains clipped to stay positive, no other bounds;
   - tuning simulator without sensor noise (paper model rollouts are deterministic).
4. **Real-flight cost** for keep-best: mean over the 4 tuning scenarios of the 3-D position MSE from the uSD logs (never radio CSVs). Reported as 3-D RMS in cm, plus z error.
5. **Deviations from the paper:** copy plan section 3 (no official code; unavoidable 1–4; decided 5–6; our reading 7). Add any further one you find while checking the PDF, marked "found in Stage 1".
6. **Open choices** to be fixed in Stage 3 by the dry run, listed but not decided: learning rate(s) for θ and β, the time step and horizon of the rollouts, the scaling of the attitude part (quaternion error form).

## Stage 1b: inventory and gap list `docs/diffcotune_inventory.md`

Read only. For each item: what exists (file : line), what is missing, rough effort.

1. **PyTorch copies** (`experiments/tuning/diff/s0_plant_torch.py`, `s1_`, `s2_`, `s3_plant_torch.py`, `fbl_plant_torch.py`, `ns2_torch.py`, `s0_torch.py`):
   - which plant (inertia, mass) each uses; the FBL copy names J_FBL and must move to the Omar inertia and mass;
   - which of the 5 β values are inputs, and whether gradients flow through them (read the code; do not run it);
   - which states they return (position, velocity, attitude, body rates) for J_sysId;
   - the last forward check of each copy against the firmware plant (which file, which tolerance, result).
2. **The 4 FBL variants:** can ROM and net be switched on / off separately in (a) the firmware build used by the simulator package (`simulator/`, controllers.py and the firmware patches), (b) the FBL firmware on the drone, (c) the PyTorch copy. What exists, what is missing, and whether a new drone image would be needed.
3. **Scenarios:** what exists for circle, figure 8 and oval in `simulator/frsim/scenarios/` (curves.py has Circle and a Gerono lemniscate), in the PyTorch copies, and in crazyswarm2 flight files (read only). Oval is new.
4. **Real-flight data path:** which uSD log variables give position, velocity, attitude and body rates; their rate; how they are aligned with the reference (existing scripts, e.g. `copy_usd_log.py`, the metrics / report scripts). What is missing for "ingest one round of flights → J_sysId input".
5. **Simulator package `simulator/` (frsim):** what can be reused as the dry run's "real drone" (frsim with shifted β); what is missing (e.g. β overrides from the command line).
6. **Gap list** at the end: numbered items for Stage 2 (simulator work) and Stage 4 (lab preparation), each with effort (hours / days) and whether it needs hardware.

## Report back (short)

1. Stage 0: processes still running yes / no; the three lines added (show them).
2. Spec: file written; list of paper statements checked with page numbers; any new deviation found.
3. Inventory: one table per item 1–5, then the gap list.
4. Anything in the plan that is wrong or unclear after reading the code and the PDF.

## Rules

- Desk only: no simulation runs, no builds, no GPU. Reading code, PDFs and logs is fine.
- Edits only: the two status lines, the docs/07 next-action line, and the two new docs (`docs/diffcotune_spec.md`, `docs/diffcotune_inventory.md`). Nothing else in `experiments/`, `simulator/`, `docs/papers/`, the meeting docs or the thesis.
- Do not delete or move any old results.
- Firmware trees (`~/Desktop/crazyflie-firmware`, `~/Desktop/fbl_v2_firmware`) and `~/Desktop/crazyswarm2`: read only, read-only git commands only.
- No branch switch, checkout, stash, commit or push. No Co-Authored-By. Nothing flashed or flown. No reboot; do not touch the NVIDIA driver.
- No `pkill`, `killall` or kill by pattern.
- Never invent numbers or rules; if the record or the paper does not say it, write "not stated" and list it as open.
- Time stamps only from the real `date` output. Method names in full in docs; FBL is just "FBL"; say "test error", never "study score".
