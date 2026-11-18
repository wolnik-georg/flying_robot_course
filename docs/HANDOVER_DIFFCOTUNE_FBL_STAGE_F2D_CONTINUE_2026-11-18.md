# DiffCoTune for FBL, Stage F2d: decision on the 4 "both" A1 cells, continue with gateF_base and the queue (2026-11-18)

Paste everything below the line into the working agent. The F1–F2c rules still apply unless this prompt changes them.

---

You are the **working agent**. Facts only from files, with paths; "not stated" otherwise. Never invent numbers or rules. Same isolation as before. Lab guard as in F2.

## 0. F2c (commit `eae94ea3`): accepted as far as done

- The gyro low-pass is correct: firmware file:line, FBL only, S0/S2 bit-identical, A1 no longer crashes.
- The rate-PID gain correction in F2b is accepted.
- Image facts are accepted:
  - the FBL lab image is **`fbl_v2b` (`87bd963e…`)**, because the bench check needs `t_us_max`;
  - the tree binary is not proven equal, so **nothing is flashed before the S0–S3 sessions end** (E3 holds).
- frsim with the filter is calmer than the 26 Oct flights. Noted, not a blocker: those flights used the older image, the β fit uses real flights, and the lab decides. Keep it in the report as an open item.

## 1. Decision on the noisy forward check (planning agent; the user accepted it by sharing this prompt)

**F1 — known exception, gate unchanged.** The 4 failing cells are all `both` on A1, at preset 1 and gains1, both seeds. They are a gate artefact, not a model error:
- no crash in frsim or the model;
- frsim itself moves 1.7–16.8 % under a 1e-6 thrust-gain change, so the cell is sensitive, not failing;
- the absolute difference is ≤ 1.1 mm RMS on a 0.3–1.2 cm error.

The Stage 3d rule for "chaotic" cells assumes chaotic means failing, so it cannot be met here. In the report, write: "FAIL, known exception (sensitive, non-crashing cells, `both` A1)", with these numbers. **The gate rule is not changed.** Do not write a new rule to make the cells pass.

This exception covers only these 4 cells. Any other FAIL (forward or gradient, including `gateF_rom`/`gateF_both`/`gate9F_*` in sensitive A1 cells) still stops the queue, and you report it.

## 2. Tasks

### F2d.1 Finish F2c.3 and F2c.4
- `gateF_base` with the filter (heavy, outside lab sessions). Gate unchanged. A FAIL stops everything.
- If it PASSES: start `dryrun/fbl/f2.sh` exactly as F2c.4 says:
  - 3 workers, guard and memory check before every job, memory watchdog, stop on rc ≠ 0;
  - order: `dryF_base`, `rtestF_base`, `gate9F_base`, then rom, then both (`gateF_rom` / `gateF_both` at preset 1 before their dry runs, if the job list has them).
- Measure peak memory and time of the first dry run.

### F2d.2 Priming tool for the authors-aligned image (light, desk only)
`fbl_prime_controller.py` reads `ctrlFbl.z_on`, which no longer exists in `fbl_v2b`.
- Read only: say what the script does on that image (error, silent skip, wrong value), with file:line.
- Do not edit `flying_drone_stack/`. If a fix is needed, write a copy `experiments/diffcotune/lab/session_helpers/fbl_prime_v2b.py`. It sets and reads back exactly the `ctrlFbl` values the `round.py` FBL fragment writes (6 gains, `en 1`, `inner 1`, `cd 0.55`, `dw_en`, `nn_en`).
- Point `FBL_FLASH_CARD.md` and `FBL_LAB_CARD.md` to it.
- No drone connection; test it only against a parameter list from the `fbl_v2b` source.

### F2d.3 Report `experiments/diffcotune/fbl/report_F2c.md` (finish it), plus a commit on the branch
1. Everything from F2c.
2. The F1 exception with its numbers.
3. `gateF_base`.
4. The queue table (job, start, end, rc, peak memory, key result) and, per variant, the lab-ready table as in `report_stage8e.md`, as far as finished.
5. The priming tool.
6. Open items:
   - frsim calmer than the real flights;
   - tree binary equivalence not proven;
   - when to flash `fbl_v2b` (user, after the S0–S3 sessions).

Messages to the user:
- when the queue starts: "Stage F2d step done: queue started";
- when it ends: "Stage F2c/F2d done: experiments/diffcotune/fbl/report_F2c.md".

Stop and report at once if:
- a gate or regression fails (outside the 4 F1 cells);
- the memory watchdog stops a job;
- a non-finite value appears;
- an isolation rule cannot be kept.
