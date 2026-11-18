# DiffCoTune for FBL, Stage F2c: decisions after F2b, gyro filter, re-checks, queue, image preparation (2026-11-18)

Paste everything below the line into the working agent (F1/F2/F2b rules still apply unless this prompt changes them).

---

You are the **working agent**. Facts only from files, with paths; "not stated" otherwise. Never invent numbers or rules. Same isolation as before (worktree `~/Desktop/flying_robot_course_fbl`, branch `feature/diffcotune-fbl`, lab checkout untouched, no push, no Co-Authored-By). Lab guard as in F2: heavy jobs only when `~/Desktop/LAB_SESSION_ACTIVE` is absent and no `round.py` / `session.py` runs.

## 0. F2b accepted (commit `b1969eb2`)

The planning agent accepts F2b. In short:
- the A1 crash comes from frsim's gyro noise (12.08 deg/s, raw, no driver low-pass) going through the unfiltered D term of the stock rate PID;
- the real FBL A1 flights of 26 Oct (stock inner loop) were calm: tilt RMS 2.1–4.7°;
- the flown image `e3593a6f` does not match the model; `fbl_v2_authors_aligned_candidate.bin` (`2fbe6034`, 4 Nov) does.

## 1. Decisions (planning agent's recommendation; the user accepted them by sharing this prompt)

| # | question | decision |
|---|---|---|
| E1 | gyro noise | **Add the real gyro driver low-pass, do not scale the noise.** The noise level stays 12.08 deg/s (the shared S0 calibration). Add the firmware's gyro low-pass (cutoff and filter type exactly as in the drone firmware's sensor driver, file:line) to the FBL sensor path. It goes in the worktree `simulator/` copy (patch file) and in `fbl_model.py`'s noise path. **For FBL only**, switched on by controller, so S0–S3 stay bit-identical. Reason: the filter is part of the real sensor chain; the S0–S3 noise level was fitted to S0 flights as an effective value; the real data shows FBL is calm. No ×0.17 scaling (that would be a number without a source). |
| E2 | A1 and θ_0 | **A1 stays a tuning scenario, θ_0 stays preset 1.** The real 26 Oct flights show preset 1 flies A1. No change under the starting-gain rule. R3 applies only if A1 fails in the lab. |
| E3 | image | **The lab image for FBL is the authors-aligned one** (`fbl_v2` `2fbe6034`, or `fbl_v2b` if a built `.bin` exists and differs only by the timing counter: report which one). The model stays as it is (4 Nov). **Nothing is flashed now.** Flashing is the user's call, after the bench checklist, and **only after the S0–S3 tuning sessions are done**, unless F2c.5 proves the S0–S3 code in that image is identical to the image S0–S3 fly with now. |
| E4 | `gateF_base` | Stays FAIL until it is re-run with E1 (F2c.3). |

## 2. Tasks, in this order

### F2c.1 Gyro low-pass (light)
- Find the gyro low-pass of the sensor driver in the authors-aligned FBL firmware (`~/Desktop/fbl_v2_firmware`, and the same in `fbl_v2b_firmware`): driver file, cutoff (F2b says 80 Hz; confirm with file:line), filter type and order, the rate it runs at (1 kHz?), and where the controller reads the filtered gyro. Also report whether frsim's tree path already has an equivalent filter, so that the S0–S3 difference is documented.
- Implement it in the worktree `simulator/` copy (patch file `simulator_patches/0002-fbl-gyro-lpf.patch`) and in `fbl_model.py` (the same filter on the gyro noise, differentiable, the same discrete form as the firmware). Switched on for FBL only. Keep `.before_F2c` copies.
- Light checks:
  - (a) S0 frsim episode with noise (one seed): identical to before (max abs diff 0.0);
  - (b) the noise-free FBL forward check is unchanged (the filter acts on noise only; if it also acts on the true gyro, as on the drone, re-run the noise-free forward check too and report).

### F2c.2 Noisy A1 again and comparison with the real flights (light, single processes)
- Repeat the F2b.1 noisy A1 diagnosis with the filter: base / rom / both, preset 1, seeds 1000/1001, plants calibration and P1. Report crash yes/no, tilt RMS / max and 3-D RMS.
- Compare with the real 26 Oct FBL A1 flights (tilt RMS 2.1–4.7°, max 5.8–9.3°, 3-D RMS 1.9–3.8 cm; F2b.2). Report side by side, with the caveat that those flights used the older image. No pass threshold: the planning agent judges.
- **If any noisy A1 cell still crashes with the filter: stop and report.** No noise scaling, no other fix.

### F2c.3 Re-checks (noisy forward check light; gate heavy, outside lab sessions)
- Noisy forward check (`noise_forward_check_fbl.py`, Stage 3d gate unchanged) for base, rom and both with the filter.
- `gateF_base` again (gradient check, SG_R 0, QUANT 0).
- Gate unchanged. Report PASS / FAIL per row. Any FAIL: stop and report.

### F2c.4 Queue (heavy, only if F2c.2 and F2c.3 pass)
- Start `dryrun/fbl/f2.sh` as prepared in F2b: 3 workers, the guard and memory check before every job, the 5-min memory watchdog, stop on rc ≠ 0. Order: `gateF_base` (if not already done in F2c.3), `dryF_base`, `rtestF_base`, `gate9F_base`, then rom, then both. The S0/S2 regressions do not need repeating, unless the filter patch touched `loop_lib.py` or the tree path: then run them first.
- Measure peak memory and time of the first dry run, and write them into the report. If memory drops below 3 GB, the watchdog stops the job: report it.

### F2c.5 Image preparation (read only; build nothing, flash nothing)
- Which authors-aligned `.bin` exists (`2fbe6034`, and `fbl_v2b` if built): path, sha256, date, source tree.
- Does that image contain **the same S0–S3 tree code and NS2 v2 weights** as the image S0–S3 fly with now? Source diff of the tree controller files and NS2, file by file. Which image S0–S3 fly with: from the lab cards / docs, or "not stated".
- The uSD logging config in that image: does it log everything `usd_ingest.py` needs for FBL (F2.5)?
- Write `experiments/diffcotune/lab/FBL_FLASH_CARD.md`:
  - the bench checklist (from `docs/fbl/FIRST_FLIGHT_CHECKLIST.md` / `LAB_PLAN.md`);
  - the flash step;
  - `check_active_controller.py`;
  - priming (`fbl_prime_controller.py`);
  - the shakedown (hover, then A1 with a hand on the abort);
  - the crazyswarm2 branch `study_FBL_stock`;
  - how to switch back to the S0–S3 image.

### F2c.6 Report `experiments/diffcotune/fbl/report_F2c.md` plus a commit on the branch
1. The filter: source facts, patch, checks (a) and (b).
2. Noisy A1 with the filter vs the real flights.
3. Noisy forward check and `gateF_base`.
4. Queue table (job, start, end, rc, peak memory, key result), and per variant the lab-ready table as in `report_stage8e.md`, as far as finished.
5. Image facts and the flash card.
6. Open items, and who decides each.

Messages to the user:
- after F2c.1–F2c.3, and after the queue starts: "Stage F2c step done: <what>";
- at the end: "Stage F2c done: experiments/diffcotune/fbl/report_F2c.md".

Stop and report at once if:
- a noisy A1 cell still crashes with the filter;
- a gate or regression fails;
- the S0 noisy episode is not bit-identical;
- the memory watchdog stops a job;
- an isolation rule cannot be kept.
