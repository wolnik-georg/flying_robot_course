# DiffCoTune, Stage 4b for S0: the round-1 dry test, the gyro and attitude check, and the P2 stall (2026-11-13)

Paste everything below the line into the tuning (main) agent.

---

Your Stage 3e / 4a report was checked against the files:
- **Patch check:** 4.5e-15. Confirmed.
- **B1 / B2:** `pass_P1.md`, `pass_P2.md`, `loop_P*.json` and the logs match your numbers and the pass criteria.
- **Dry test:** θ₁ and β are identical to the B1 run, and so are the gain-limit hits. Confirmed.
- **Ingest reference check, flight files, protocol draft and the single job runner:** confirmed.
- **Nothing is running.**

Corrections for the report (`report_stage3e.md`):
1. **The dry test covered one round, not two.** In the chain it stopped with `NameError: t_round` after round 0 (`chain.log` 13:48:20, `dry_test exit 1`). The JSON comes from `--reuse` on that finished round. Write this plainly.
2. **Add three findings that are in the tables but not in the text:**
   - **A4 cell 1, P1:** θ_best is worse than the default (9.48 against 8.80 cm). The test mean improves only because of A4 cell 2 and the oval.
   - **P1 against Stage 3d A + B:** the final tuning / test cost is 6.39 / 7.76 here, against 5.60 / 6.89 cm there. Rounds 4–5 gained 0.4 cm in P1 and nothing in P2 (round 5 was 8.21 cm, keep-best is round 3).
   - **P2 stall:** after rounds 2 and 3 the tuning step's best iterate was epoch 0, so θ₂ = θ₃ = θ₄.
3. **Logged "Jn":** `loop_lib.py` (lines 255 and 264) prints the cost at the last epoch, not at the best iterate (e.g. P2 round 2: printed 1.534; the best iterate is epoch 0, 1.000). Print both from now on; do not change the results.

## User decision (13 Nov)

Three desk checks before the lab. The S0 lab is not started yet. The open protocol points and the number of rounds stay with the user.

## Do

### 1. Dry test, round 1

- Continue the dry run in `lab/dry/` with round 1:
  - fly the 4 tuning scenarios with θ₁ in P1, flight seeds 110–113 (as in B1 round 1);
  - write them as fake merged-uSD files;
  - run `round.py --round 1` from the state file of round 0.
- **Gate:** θ₂ and the round-1 β equal `loop_P1.json` (round 2 φ, round 1 `psi_after_fit`), max difference ≤ 1e-12; real cost equal to 9.71 cm within 0.01 cm; the keep-best state is correct after round 1.
- Also check the yaml round trip for `round_2/gains.yaml`.
- Use `strict=False` as in round 0, and state it. Round 1 has no floor flight, so also run it once with `strict=True` and report whether the result is the same.
- Fix the `t_round` bug so the script runs from start to end without `--reuse`.

### 2. Gyro and attitude check on the 23 Oct S0 flights

The gyro part of J_sysId is 1.5–17 rad²/s². The gyro noise in frsim is 12.08 deg/s, which gives about 0.04 rad²/s². `usd_ingest.py` already converts deg/s to rad/s (line 133), so units alone do not explain it. The gyro is not used in the fit, but attitude is, and both pass through the same frame handling. Find the cause:
- **Sign and axis:** compare logged `gyro_x/y/z` with the derivative of the logged attitude (roll, pitch, yaw, after the legacy pitch sign at line 119), per axis. Report the correlation and the scale per axis. A sign flip or swapped axis shows as a negative or near-zero correlation.
- **Frame:** check that the 90° frame rotation (A4, the turned figure 8) turns the body rates and the quaternion consistently. Use the same derivative test after the rotation.
- **Vibration:** compare the spectrum of the logged gyro with the model's body rates. Report the share of power above 20 Hz. Note which gyro signal the uSD log holds (raw or filtered; name the firmware source, read only).
- **Attitude part:** report J_sysId per part again (position, velocity, attitude) after any fix, against the Stage 3e values. If the attitude part changes, say by how much.
- **Model side:** check the S0 model's body-rate output against frsim for one flight (sign and axis), so the fault is placed on the right side.

Report the cause, or "not found" with what was ruled out. Fix it only in `experiments/diffcotune/` (the ingest). If the fix needs `simulator/`, stop and ask.

### 3. Why P2 stalls (desk, under 1 h)

For P2, the tuning step after rounds 2 and 3:
- the model cost J_task per epoch, and the gradient norm per gain at epoch 0;
- the learning rate and the Adam step size of the first step against the gain-limit range (log 1.25);
- whether the cost rises from the first step on (step too big) or only later;
- the β of that fit against the P1 truth, and whether the 10 ms delay (not in β) explains the gap. Compare with P1 rounds 2 and 3, which did move.

Report your reading. Do not change the method. If a fix looks needed (e.g. a smaller step), propose it for the user to decide.

## Report back (short): `dryrun/stage4b/report_stage4b.md` and `lab/README.md`

1. **Round-1 dry test:** gate yes / no, strict on / off result, desk time.
2. **Gyro and attitude:** the cause (or what was ruled out), any fix, J_sysId per part before / after.
3. **P2 stall:** the per-epoch table and your reading.
4. **The corrections** in `report_stage3e.md` (dry test, the three findings, Jn).
5. **Open lab points** still open after this, as a list for the user.

## Rules

- **Resources:**
  - `nice -n 19`;
  - at most 16 CPU processes;
  - one GPU job at a time;
  - load ≤ 22, checked from `/proc/loadavg` before **and one minute after** each start, with `dryrun/stage3c/job_runner.py`;
  - MemAvailable ≥ 3 GB;
  - torch threads set in every worker;
  - `TORCHINDUCTOR_COMPILE_THREADS=1`;
  - the whole stage within 2 h of wall time.
- **Edits only in:**
  - `experiments/diffcotune/`, with new `dryrun/stage4b/`;
  - `report_stage3e.md` (the corrections) and `lab/README.md`;
  - `docs/diffcotune_spec.md` and `docs/diffcotune_inventory.md`, marked "Stage 4b";
  - the open list of `docs/diffcotune_lab_protocol_S0_DRAFT.md`, only if the gyro check changes an item there.
- **Do not touch:**
  - `simulator/`: read only; if a change is needed, stop and ask;
  - `flying_drone_stack/`, `~/Desktop/crazyswarm2` and the firmware trees: read only; no firmware rebuild;
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
