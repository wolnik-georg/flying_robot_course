# Cursor follow-up prompt — fix the test harness, redo Analysis A properly, resolve the radio-vs-uSD contradiction (2026-10-03)

Repo: `~/Desktop/flying_robot_course`. You start fresh. Read `docs/lab_sessions/2026-10-03.md` — especially the last section **"Validation of the Cursor desk results (Claude)"** — and `docs/cursor_prompt_ns2_100hz_2026-10-03.md` (the previous prompt; same constraints apply).

## Context (verified by Claude — do not re-derive)

- Your code changes (decimation `rnn.div`, peer-velocity hold, static scratch, timing log) were reviewed and **accepted**. Builds are reproducible (`ed7e0cb7…` rebuilt identically).
- **Host tests:** baseline (HEAD) = **21/21**; your tree = **20/21**, not 19/21. The single failure `gate excludes |dvx| >= 1.5` is a **test-harness artefact**: `gate_case` reuses the same peer timestamps (`t0+100`, `t0+200`) in every case and `State` is not reset by `controllerOutOfTreeInit()`, so the previous case's stale `peer_prev` matches the new first timestamp and the new rule ("update `peer_prev` only when the timestamp advances") differences against the wrong position. Real timestamps are monotonic. **Do not weaken the firmware.**
- **Gotcha when testing:** to test a different source tree you must rebuild in this order — `RUSTFLAGS="-C panic=abort" cargo build --release --target x86_64-unknown-linux-gnu --features residual_nn` (in `firmware_app/`), `touch` the sources if you copied them with `cp -p`, then `cd ~/Desktop/crazyflie-firmware && rm -f build/_cffirmware*.so build/cffirmware_wrap.c && make bindings_python`, then `python3 host/test_residual_nn.py` (system python 3.10, not pyenv). A stale `.a`/`.so` silently gives wrong test results — the earlier "19/21 with 2 fails" was most likely that.
- **Analysis A:** your index-aligned comparison of raw arrays was invalid (different `t0`; `z` nearly constant so correlation is meaningless). Claude's correct method: decode both files, put both on a common clock by interpolating onto a shared time grid (`np.interp` on the decoded `t` column, overlap window), pair files by the **`usd.runTag` value inside each file** (decoded channel `run_tag`; it equals `usd_run_tag` in the scenario's `…meta.json`) — not by thesis index or filename order. Result: 10-03 `cf5`↔`cf_second` y corr 1.000/0.999, RMS 5/45 mm; 10-02 anti-correlated (−1.0).
- **New contradiction (the important one):** on 10-03, `cf5`'s **radio CSV** `pos_z` ≈ 0.5 m (hover, overshoot 1.06 m, landing ≈ −0.04 m) but its **uSD** `stateEstimate.z` ≈ 0.95 m, constant ±0.02 for 25 s, while the drone was on the floor (motors 0, tilt 100–180°). You established radio `pos_*` = firmware `stateEstimate` via the CS2 `{name}/state` log block. If both are the same firmware variable they must agree. They don't. Unresolved: logging artefact on `cf5`'s uSD, or something else.

## What NOT to do

- Never fly, never flash, never commit/push, no `Co-Authored-By`. Do not edit `docs/07`. Do not change firmware logic/network math/weights in this task (the only code edits allowed: the test harness `host/test_residual_nn.py` and new analysis scripts).
- Do not touch `crazyflies.yaml`, controllers 7–10, `ENABLE_Z_INTEGRAL`. Report, don't fix, anything else you find.

## What to do (in order)

**F1 — Fix the host test harness, add missing tests.** In `host/test_residual_nn.py`: give every case **monotonic, never-reused peer timestamps** (e.g. one global counter advanced by ≥100 per sample, across `gate_case` and the other cases) instead of reusing `t0=5000`. Do not change firmware to make a test pass. Add tests: (a) peer-velocity **hold**: same timestamp twice ⇒ velocity held, not zeroed; (b) **first-seen** peer ⇒ zero relative velocity; (c) **decimation**: with `g_rnn_div=10` the prediction changes only on ticks 1,10,20,… and is held in between (also `rnn.pred_*` logged continuous); (d) `g_rnn_div=0` behaves like 1. Rebuild in the order above, run, report **all** results (expect all pass). Also run the **baseline** (HEAD firmware) with the same *new* harness where meaningful, and state whether `div=1 + static scratch` predictions equal baseline predictions bit-exactly or to what tolerance (build the baseline in a separate git worktree, do not disturb the working tree).

**F2 — Redo Analysis A correctly.** Rewrite `experiments/analysis/check_estimator_identity.py` with the method above (time-aligned on a common grid, pair by the in-file `run_tag`, report y and z correlation, RMS difference, mean z per drone, overlap seconds). Run over **all** archived 2-drone uSD flights (2026-09-19 … 2026-10-03). Output a table: date, scenario, run_tag, pair status, y-corr, y-RMS, mean-z cf5/cf_second, verdict (independent / anti-correlated / **identical**). Report **when "identical" first appears** and whether it appears in any flight that used **default firmware + `cflib`** (it should not; 10-02 is the control).

**F3 — Resolve the radio-vs-uSD contradiction (highest value).** For the two 10-03 A8 flights (13:10:32 and 13:15:00; radio CSVs `experiments/logs/A8_{cf5,cf_second}_2026-10-03_*.csv`, uSD `experiments/logs/usd_raw/{cf5,cf_second}_A8_thesis0{0,1}_*.bin`):
1. Align radio and uSD **for the same drone** on a common clock using the **tilt/attitude time series** (cross-correlate radio `roll/pitch` vs uSD `roll_deg/pitch_deg`; the crash events are sharp). State the lag and the alignment quality.
2. After alignment, overlay and tabulate `x,y,z` from **radio cf5, uSD cf5, radio cf_second, uSD cf_second**. Answer: does uSD-cf5 position equal radio-cf5, radio-cf_second, or neither? Does uSD-cf_second equal radio-cf_second (the control)?
3. Establish **exactly which firmware variables** each source records: radio `state` block variables (CS2 `crazyflie_server`/`DroneLogger`, `crazyflies.yaml` `firmware_logging`, `experiments/analysis/metrics.py`) vs `tools/usd_thesis_config.txt` names and the decode mapping in `tools/decode_usd_log.py` (`RENAME`; note `usd.runTag`/`stateEstimate.*`; check the **header of the cf5 uSD file** against what the decoder assigns to `x,y,z` — a column-mapping error between cf5's 55-variable file and cf_second's 49-variable file is a candidate).
4. Check the same comparison on **10-02** (default firmware, `cflib`; cf5 `cf5_thesis70–72` + `cf_second_thesis54–57`, radio CSVs `A8_cf5_2026-10-02_19-*.csv`): do radio and uSD agree for `cf5` there? If yes ⇒ something changed on 10-03 (firmware/backend); if no ⇒ a long-standing logging mismatch.
5. Conclude (with confidence): is `cf5`'s uSD position channel unreliable on 10-03, or did `cf5`'s estimator really track the wrong drone? List what additional (cheap) evidence would settle it if still ambiguous.

**F4 — Remaining SIL work from the previous prompt (task 5b).** 2-drone A8 (or A3) SIL run, `rnn.div=1` vs `rnn.div=10`: RMS/max difference of `rnn_pred_z`, lag, and effect on the pred-vs-`a_res` agreement metric. Use existing sim entry points (`experiments/sim_validation/RESIDUAL_DRYRUN.md`, CS2 SIL); report honestly if the 100 Hz hold degrades the prediction.

**F5 — Train/serve peer-velocity mismatch, quantified (medium priority).** You found training builds `rel.v*` as differences of 500 Hz `stateEstimate.v*` while firmware differences mocap packet positions at ~100 Hz. Using archived 2-drone uSD logs, compute both ways for the same flight and report the std/RMS of the difference in `dv` (m/s) and what fraction of samples exceed the gate `|dvx| < 1.5`. Say whether it plausibly matters for the network. Report only — do not change training or firmware.

## Deliverable

- Updated `host/test_residual_nn.py` (+ result of all runs), rewritten `experiments/analysis/check_estimator_identity.py`, any new scripts under `experiments/analysis/`.
- A new section **`docs/lab_sessions/2026-10-03.md` → "Appendix B — follow-up desk results (Cursor)"** (do not edit the existing "Validation" section): per-task status table, the F2 table, the F3 overlay table + conclusion, F4/F5 numbers, reproduction commands, files touched.

## Reporting

- **Confidence level (high/medium/low) per claim**; separate "verified by running/reading code/data" from "inferred". If inconclusive, say so and say what would settle it. Put the F3 conclusion at the top of the report — it decides what the next lab session does.
