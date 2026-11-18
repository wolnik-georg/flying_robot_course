# Cursor prompt: uSD start gate (no takeoff unless both decks are writing), 2026-11-18

Written by the planning agent. Execute this, then report back to the planning agent, who validates it.
**Nothing here is flown or tested on hardware. Do not flash anything, edit any card, or change controller and gain code.**

## 0. Background (facts; do not re-investigate the desk part)
- S0 round 0 on 2026-11-18 (19:29–19:40, 8 flights: A1×3, A8×2 and A4 via `run_formation.py`; CIRCLE and FIG8 via `experiments/diffcotune/lab/fly_file.py` → `simple_flight.py`) produced **no `thesis*` file on either card, not even a 0-byte one**. See `docs/HANDOVER_USD_LOGGING_LOST_2026-11-18.md`.
- A per-drone cflib ground test at 19:57 wrote about 886 kB on both drones. So drones, cards, image fbl_v2b (sha `87bd963e…`) and the study config (`d247134c…`) all work.
- Already ruled out on the desk:
  - The radio broadcast reaches both drones: the `startTrajectory` broadcast flew both drones along A4/A8.
  - `usddeck.c` and `param_logic.c` in `~/Desktop/fbl_v2b_firmware/fw` are byte-identical to `~/Desktop/crazyflie-firmware`.
  - The crazyswarm2 logging code path is unchanged since 2026-10-25. The yaml differs from 2026-10-21 only in cf_second's uri.
- **Defect found (root of the blindness, not necessarily of the failure):** `crazyflie_py/crazyflie.py` `setParam` (both the `Crazyflie` method ~l.552 and the `CrazyflieServer` method ~l.1049) catches `KeyError` and every other exception and only logs a ROS warning. It uses `call_async` and never checks the result. `run_formation.py` (~l.993–1002) and `simple_flight.py` (~l.435–446, per-drone try/except: pass) then record "uSD started" (`usd_start_s` in the meta) without any proof.
- CS2 `getParam` returns the server's cached ROS value, not the drone's. **It must not be used as proof**; same trap as `check_active_controller.py`.
- The firmware exposes live log variables (`usddeck.c`, LOG group `usd`): `usd.runTag` (uint32, = the param), `usd.fatWrBps` (FAT write rate, > 0 only while the deck writes), `usd.spiWrBps`. These are the proof signals.

## 1. Repos and branches
- `~/Desktop/crazyswarm2`: work on **`main`** (80c54c8 at the time of writing; the laptop checkout is on `study_FBL_stock`, so switch with care and stash nothing of the user's). Then carry the same change to **every** `study_*` branch: cherry-pick, or regenerate the yaml patches as `f46bdcb` did. Push all of them.
- `~/Desktop/flying_robot_course`: branch **`feature/auto-tuning`**. Do not touch the user's uncommitted files (`git status` shows many; commit only the files you create or change for this task, by explicit path).
- Commit style: no `Co-Authored-By` trailer.

## 2. Changes

### 2a. yaml: live usd status topic (crazyswarm2, `crazyflie/config/crazyflies.yaml`)
- Add one custom log topic to the logging block that cf5 and cf_second actually use. Check which `firmware_logging` block applies: `all` or the per-type/per-robot override.
  ```yaml
  usd_status:
    frequency: 5 # Hz  # 2026-11-18: uSD start gate (runTag + write rate); see docs/cursor_prompt_usd_gate_2026-11-18.md
    vars: ["usd.runTag", "usd.fatWrBps"]
  ```
- Confirm that both variables exist in the fbl_v2b TOC (the lab agent's TOC read listed 674 vars; `usd.runTag` is also config line 103). If `fatWrBps` turns out to be a stats-rate type that CS2 logging cannot read, use `usd.spiWrBps` and say so.
- Radio budget: state the added bytes/s per drone in the report. It must stay negligible against the existing 20 Hz topics.
- **This changes the yaml sha256 values.** Recompute them for S0 (main) and S1/S2/S3/FBL (study branches). Update every place that names the old values (`bd6c05e9…` S0, `f373260c…` S2): grep both repos, including `experiments/diffcotune/lab/`, the lab cards and the handover/addendum docs. List every file you changed. `set_tree_strategy.py` S0→S2 output must still equal the study_S2 yaml byte for byte. Prove it with sha256.
- `lab_check_strategy.py` and `apply_gains.py merge` must still pass on the new yamls. Run them on scratch copies.

### 2b. Shared gate module (crazyswarm2, new `crazyflie_examples/crazyflie_examples/usd_gate.py`)
One function used by both flight scripts, e.g. `start_and_verify(node, allcfs, cfs, run_tag, timeout_s=3.0, retries=1) -> dict`:
1. Before sending anything, check that `'usd.logging'` and `'usd.runTag'` are in `allcfs.paramTypeDict` and in each `cf.paramTypeDict`. If missing → **FAIL immediately**, with a message saying the server never saw the usd params (this is the silent-KeyError case).
2. Send `usd.runTag = run_tag` and then `usd.logging = 1`, both as broadcast **and** per drone. Use the `SetParameters` futures: spin until done (bounded, ~1 s) and check `result.successful` for each. Record any failure.
3. Subscribe to `/<cf>/usd_status` (`crazyflie_interfaces/msg/LogDataGeneric`) for each drone. PASS for a drone = within `timeout_s`, at least 2 consecutive samples with `runTag == run_tag` (compare as uint32) **and** `fatWrBps > 0`.
4. If a drone fails: resend the per-drone start once (`retries`), wait again.
5. Return per-drone `{topic_seen, runTag_seen, fatWrBps_max, set_results, passed}` plus an overall `passed`.
6. A matching `stop_and_verify(...)` for the end of the flight: send `usd.logging = 0` (broadcast + per drone, checked futures), then wait until `fatWrBps` has been 0 for ≥ 1 s per drone (bounded ~3 s). Report, don't block landing. The stop must always be sent on every exit path, including exceptions and Ctrl-C; keep the existing stop calls.
- No-topic case: if `/<cf>/usd_status` never publishes (old yaml), the gate FAILs with "usd_status topic missing: yaml not updated". It must not hang.

### 2c. Use the gate in both flight scripts
- `run_formation.py`: replace the start block (~l.993–1002) with `start_and_verify`. It must run **before takeoff**, with the drones on the ground and motors idle (this is the same position as now; `usec.reset` stays where it is; **never move any broadcast of `usec.reset` mid-flight**).
- `simple_flight.py`: the same at its start block (~l.435–446), before takeoff.
- On FAIL: print one clear block (which drone, which check failed, the values seen), write `usd_gate` into the meta/sidecar, send `usd.logging = 0`, and **exit without takeoff** (non-zero exit code).
- On PASS: write `usd_gate` (full dict, `passed: true`) into the meta.json / CSV meta. Keep `usd_start_s` but set it from the gate PASS time.
- Override: `--no-usd-gate` (default off). With it, the old behaviour stays, and the meta records `usd_gate: {"skipped": true}`. The help text says such flights are **not usable** for the fit.
- Nothing else in either script changes: no trajectory, gain, apply() or timing changes. Keep the diff minimal and show it in full in the report.

### 2d. Ground test tools (flying_robot_course, `flying_drone_stack/tools/`)
- Extend `usd_broadcast_test.py` (or add `usd_gate_ground_test.py` next to it) to call the **same** `start_and_verify` / `stop_and_verify` with no motors:
  - `--mode pass`: normal start → must PASS on both drones, then stop.
  - `--mode negative`: send the start **only to cf5** (skip cf_second) → the gate must FAIL for cf_second and PASS for cf5. This proves the gate catches a missing start. Then stop both.
  - Always print `'usd.logging' in allcfs.paramTypeDict` and per drone, the set results and the topic values.
  - runTag values: 888 (pass) and 889 (negative), so the test files are recognisable.
- `experiments/diffcotune/lab/lab_run.sh prep`: after the existing `check_usd_deck.py` lines (CS2 running), run the ground test `--mode pass`. Any FAIL → `die "uSD gate failed: do not fly"`. Then print a reminder: the test files (runTag 888/889) are on the cards and must be archived before the round.
- New `flying_drone_stack/tools/usd_card_count.py` (or extend `copy_usd_log.py`): given the two mounted card paths, print the `thesis*` count and newest file size per card. Option `--expect N` → non-zero exit if the count is not N. For the "count before/after each block" check; files ≥ 85 → warn (rule: card < 85 files, never reach thesis99).

### 2e. Desk tests (laptop, no hardware)
- Unit-test `usd_gate.py` with mocked topics/futures: (1) all good → PASS; (2) KeyError case (param missing in the dict) → FAIL fast; (3) topic silent → FAIL by timeout, no hang; (4) runTag wrong → FAIL; (5) fatWrBps stays 0 → FAIL; (6) one drone fails, retry succeeds → PASS with the retry recorded.
- `python3 -m py_compile` on every changed Python file. If a ROS 2 environment exists on the laptop, `colcon build --packages-select crazyflie_examples crazyflie_py` in a scratch workspace. If not, say so; the lab PC builds from the ws root, `cpp` backend only.
- `bash -n lab_run.sh` and `DRY=1 YAML=<scratch copy> lab_run.sh prep S0 0` run through.

## 3. Lab card addition (write `experiments/diffcotune/lab/USD_GATE_LAB_STEPS.md`, short, commands only)
In this order, at the start of the next session, before any flight:
1. **Evidence from 2026-11-18 (lab PC, 1 min, read-only):**
   - `find ~/.ros/log -newermt "2026-11-18 19:00" ! -newermt "2026-11-18 20:00" -name "*.log" | xargs grep -hE "Update parameter \"usd|keyError|setParam : exception|SD-Card|Config read|Logging to|Wrote .* B to|Failed to open|usd deck write|reqParamTOC"`
   - Also the shell history and terminal scrollback of the run_formation and simple_flight runs, if still there. Save the output to `experiments/diffcotune/lab/usd_evidence_2026-11-18.txt`.
2. Pull both repos on the lab PC (crazyswarm2 main or the needed study branch; flying_robot_course feature/auto-tuning), build from the ws root, relaunch CS2.
3. yaml sha256 check (new values) on the lab PC and the laptop copy.
4. `lab_run.sh prep <C> <round>`, which now includes the gate ground test (pass).
5. Once per session: `usd_gate_ground_test.py --mode negative`. It must show FAIL for cf_second.
6. Power the drones off, take the cards out, archive the 888/889 test files (`_archive_<date>_gate_tests`), put them back, power on, `check_usd_deck.py` both, `usd_card_count.py --expect 0`.
7. Shakedown A1 → `usd_card_count.py --expect 1` on both cards (cards out, or skip until the block end if the lab agent decides the card swap is too risky; then count at the block end and compare to the flight count).
8. Then S0 round 0 again via `lab_run.sh prep S0 0` (the lab PC yaml is currently in the S0 solo state with the round 0 gains merged).
- Rules stay unchanged: uSD only for the fit, never radio CSVs; card < 85 files; no `usec.reset` in flight; sha256 of both yamls after every switch; `check_active_controller.py` after flashing.

## 4. Report back (to the planning agent; facts only)
- Commits (hash, repo, branch) and push status for every branch, including all `study_*` branches.
- Full diff of `run_formation.py`, `simple_flight.py`, the yaml, and `lab_run.sh`; the new files.
- Old → new yaml sha256 per strategy, and every file where the sha was updated.
- Desk test output (all 6 unit cases, py_compile, DRY run, build or "no ROS here").
- Radio budget added.
- Anything you could not do, and why. Do not invent results; "not tested" is a valid answer.
