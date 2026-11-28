# Chirp session (S1 loop measurement + S3 confirmation): run sheet DRAFT v2, 2026-11-28 (lab agent, desk only)

Status: DRAFT for the user's check. Nothing flown, flashed, copied to the lab PC or applied. Lab lock stays. The lab-session start is the user's call.
Source package (read-only, branch `desk/s1-simulator-gap`): `~/Desktop/frc_desk_s1gap/experiments/desk_s1_gap/proposal/` (LAB_DAY_SUMMARY, flight_card_chirp_DRAFT, chirp_flight_script_DRAFT.py, usd config v3, telemetry yaml, ring16k patch).
**v2: the user decisions of 28 Nov are applied (section 0).** The sim agent is moving its files to `proposal/lab_pkg/` (`chirp_flight.py`, `flight_card_chirp.md`); use those paths once REPORT_59 lands.
**Placeholders filled from REPORT_59 (sim worktree commit 93536bfe, 28 Nov): none left.** `<PATCHED_BIN>` = section 2.2a, `<GONOGO>` = `usd_check.py` (section 3.1). Where the sim card and this sheet differ, this sheet is the lab master (section 5).
Rules: uSD logs only; never `usec.reset` in flight; cards < 85 files; CPU only; fresh batteries; `GAINS_ROOT=$PWD/experiments/diffcotune/lab/v21`.

---------------------------------------------------------------------------------------------------
## 0. Decisions applied (user, 2026-11-28; the user may override)
1. **uSD logging:** verified start/stop path as a snippet, `chirp_session/usd_start_snippet.py` (source: crazyswarm2 `usd_gate.py`, commit 42fc430; usage in `run_formation.py` at HEAD fd1167f = post-set fix; line numbers inside the file). The sim agent puts it into `chirp_flight.py`: ground precheck, `usec.reset` on the ground only, `start_and_verify` in hover before the chirp, `stop_and_verify` after landing, `usd.logging 0` in `finally`.
2. **Flights:** F1 baseline, F2 repeat, **F3 = KR x0.9 (Kw unchanged)**, **KR x1.2 dropped** (real edge x1.0; v21 S1 round 2 at KR x1.10 oscillated), F5 repeat of F2, **F6 = one S3 chirp (S3 v21 r1 gains as flown, same amplitude)**.
3. **First flights at 0.6 m, amplitude ramp 0.1 -> 0.3 m/s2, tethered G7 before F1.**
4. **Patch:** the sim agent builds the image (REPORT_59: exact rebuild of 9e133f76 first, then the patch, with md5); the lab agent flashes it. **Rollback image = the current 9e133f76 binary:** `~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_dshot_candidate.bin`, sha256 `9e133f76dfc4dd30372aeab8fff1d59e093325f64e170ccb40646050c35657c0`, **md5 `fec0d76fb4a057de94b09b80d812ab10`** (identical sha256 also at `~/Desktop/fbl_v2b_dshot_fw/stack/firmware_app/build/cf21bl.bin`; checked 28 Nov). The patch was fully linked in the scratch build (REPORT_53); my "inconsistent" remark on REPORT_52 is withdrawn.
5. **Battery:** lab protocol value only, nothing invented. Source `experiments/diffcotune/lab/S0_LAB_CARD.md` row 6 and l.45 (CSP section 4, PD "Decided" item 2): **fresh packs, rest >= 4.1 V; swap if < 4.0 V (before a flight, repeats included)**. The protocol has NO in-flight voltage limit ("a voltage limit for a repeat: not stated (open)"), so the draft's in-flight vbat abort trigger is removed; `pm.vbat` stays in the log, the 4.1 / 4.0 V check is done on the pad before every flight F1-F6.
6. **Order:** the chirp session runs BEFORE any round 4 flights. The S0/S2/S3 round 4 sets (`v21/SUMMARY_round4.md`) are on hold; tuning restarts on the corrected simulator, INDI first. The earlier "S3 -> S0 -> S2 round 4 tomorrow" order is superseded.
7. **Logging after the session:** if G8 passes, v3 + the patched image stay as the new standard (add, never swap). Restore the 96-variable config / the rollback image only if G8 fails or the patch is rolled back.

---------------------------------------------------------------------------------------------------
## 1. Safety review: every concern (ranked)

**Blockers (must be fixed or decided before F1)**
1. **The script never starts uSD logging.** `chirp_flight_script_DRAFT.py` contains no `usd.runTag` / `usd.logging` set, no usd gate (`usd_gate.py` in crazyswarm2 is only wired into `run_formation` / `simple_flight`) and no ground `usec.reset`. As drafted every flight would produce NO uSD file (the 11-18 failure again) and the analysis is uSD-only. Needed: on the ground, before takeoff, start logging through the same verified path as `run_formation` (runtime set via CS2 post-set fix 71219f3 + gate check that BOTH decks... here only cf5 writes: `usd_gate` per-drone check), and stop (`usd.logging 0`) after landing. Never put `usec.reset` after takeoff. **RESOLVED by decision 1:** `usd_start_snippet.py`; a flight without a verified start is not counted.
2. **(RESOLVED by decision 2: F3 = KR x0.9, x1.2 dropped; concern kept for the record.) KR x1.1 / x1.2 sit beyond the real stability edge.** frsim edge is at KR x1.35 but the REAL edge is at x1.0 (LAB_DAY_SUMMARY). Real evidence: v21 S1 round 2 (KR 0.00847 = x1.10 vs round 1, Kw x0.90) oscillated at 17.05 cm (3-4 Hz), round 1 = 3.18 cm keep-best. F3 (x1.1) and F4 (x1.2) with a 0.3 m/s2 chirp may drive the real loop into the limit cycle. The card's "no oscillating gain set" rule cannot be met by design for F3/F4. Also unstated: is Kw kept at round-1 value (0.001178) while only KR is scaled? Need: the exact F3/F4 fragment (KR_x, KR_y only, KR_z unchanged?) and a decision with margin check (PREVIEW-style frsim run at the real-edge calibration, REPORT_59). Proposal: F3 only after F2 shows PM/GM from the chirp itself; F4 only if F3 Ms/PM keep the predicted trend and gyro rms in the holds stays < 30 deg/s; otherwise stop the series.
3. **No on-board 15 deg / 300 deg/s trip.** The tilt/rate trip lives in the PC script (radio telemetry, 20 Hz default, 0.3 s window + latency 50 ms, radio latency unmeasured). A radio drop-out or a Python hang = no abort; only firmware protections remain (commander timeout 0.5 s level-out / 2 s motors off, tumble check 60 deg for 1 s, emergency packet). With a limit-cycle growth (item 2) 0.3-0.5 s may be enough to reach > 25 deg. First flights must be low (item 8).
4. **Emergency stop = fall from hover height** (motors off, firmware locks, reboot needed). At 0.9 m on the chirp flights this is a ~0.9 m drop with a 41 g brushless: propeller/frame damage risk, soft floor needed. `q` (land) is the normal path, `e` last resort only.

**Serious**
5. **Keyboard abort needs a focused terminal with a TTY.** `sys.stdin` must be a terminal and the operator presses `q` + Enter (line-buffered, 1 key then Enter). Under `ros2 launch` or a backgrounded process it does not work. Run the script from the foreground terminal with `ros2 run` equivalent (`python3 chirp_flight_script.py`), operator's hand on the keyboard, a second person watching the drone. Consider a hardware abort too: the radio-less option = remote-control or pulling the battery (not instant).
6. **Monitor only runs while ROS callbacks are spun** (inside `th.sleep`). Any blocking call (takeoff/land service wait) stalls it; script uses abortable sleeps, but `cf.takeoff` / `cf.land` themselves are non-blocking requests, so OK. Mock-tested only; CS2 sim backend cannot run `emergency` (not implemented) => the real `cf.emergency()` path (cflib backend) is exercised for the first time in G4.
7. **Telemetry topics 20 Hz** unless the 100 Hz yaml fragment is applied (needs CS2 relaunch and a yaml edit in the lab-PC crazyswarm2; and is cf5 `firmware_logging.custom_topics` only). 100 Hz is a G2 acceptance (>= 95 % of nominal messages in 60 s), otherwise 50 Hz. Higher telemetry rate adds radio load; solo so OK.
8. **(RESOLVED by decision 3: 0.6 m, ramp 0.1 -> 0.3, G7 first.) First flights low.** Card says hover 0.9 m for every flight; the user's rule is low first flights. Recommendation: G7 tethered 0.3-0.4 m at 0.1 m/s2 first; then F1 at **0.5 m, amplitude 0.1 then 0.2 m/s2 for the first 20 s** is NOT in the card (card F1 = 0.3 m/s2 at 0.9 m): decision needed. Proposed: F1 = 0.6 m (the lab standard 0.5 m was used for all DiffCoTune flights; chirp reference positions at 0.9 m exceed the mocap-validated volume?: check the mocap box) with amplitude ramp 0.1 -> 0.3 over the first 20 s of the chirp.
9. **0.3 m/s2 peak roll 10.6 deg is frsim**, with the sim loop being MORE damped (Ms 10.9 dB sim vs the real unknown, real damping 0.02 vs sim 0.12). The real roll at the lightly damped 3-4 Hz mode can be much larger than 10.6 deg. The chirp passes through 3-4 Hz where the loop gain peaks. Peak roll at the mode with real damping is untested.
10. **Chirp envelope / amplitude units**: script position amplitude = amp / (2 pi f)^2 and v, a from numerical gradient on a 1 ms grid: at 1 Hz the position excursion is 0.3/39.5 = 7.6 mm, at 8 Hz 0.1 mm: tiny positions with relatively large mocap noise; the position feedback fights the feedforward acceleration. The script streams full state each 10 ms and indexes the grid by wall time (`th.time()`): any stall = jump in phase. Needs the G2 gap check (no gaps > 30 ms).
11. **Hold burst after abort sets position to `pos0`** (a 0.3 s burst at 100 Hz): if the drone has drifted or is oscillating the position loop yanks it back; untested on a real vehicle. Consider a land-from-current-position (HLC `land` already does `notifySetpointsStop` first; the burst is optional).
12. **S1 solo setup carries the S1 history**: climb overshoot to 0.74-0.80 m on A1 takeoff with these gains (S1 r0/r1 log), so `takeoff(0.9, 3 s)` may overshoot to ~1.1-1.2 m: ceiling / mocap box / downwash-free? Check the room height and the mocap volume. DShot hover check must still show diff +2.7 m/s2 after the flash.

**Process / data risks**
13. **The firmware patch is scratch-built** (fully linked in REPORT_53; no inconsistency); the RAM margin is 13.7-14.7 KB above the checked stack, main stack is only 1 KB: the new image is untested on the drone. A flash changes the image of record (87bd963e... `fbl_v2b_candidate.bin` / the DShot S1/S3 image 9e133f76; the current image sha for S1 is 9e133f76 per `lab_run.sh`). Image: `fbl_v2b_dshot_usd128_candidate.bin` (section 2.2a), 9e133f76 reproduced byte for byte + the patch; rollback image in decision 4.
14. **After the flash all S1 flown behaviour must be re-verified**: controller slot line (console) `Using OutOfTree4 (9) controller`; param readback is a cache. Check `check_dshot_rpm.py` and the DShot hover check (+2.7 m/s2).
15. **Config v3 on cf5's card** replaces `config.txt` (96-variable study config is the current one: sha d247134c...). The S0/S2/S3 round 4 flights tomorrow need the 96-variable config again + the patched image behaves the same with it (limit 128 >= 96; 16 KB ring only changes buffering, buffer line in config.txt line 2 = 8192 for the study config). Keep the study config copy and RESTORE it after the session (`restore_study_sd_config.py`): else DiffCoTune files change format. Decoder: `decode_usd_log_rename_additions.txt` needs merging into `decode_usd_log.py` names; the pipeline (`merge_usd_logs`, `round.py` ingest) reading a 115-column file is NOT validated: do not feed chirp logs into the v21 rounds.
16. **uSD write margin** (G8): 215.5 kB/s v3 vs 178.5 kB/s now; the real 96-variable logs already lose 1.1 % rows. If G8 fails, no chirp flights.
17. **Card capacity/file count**: cf5 now 29 files, cf_second 20: both < 85. Chirp uses cf5 only (solo). One log per flight + G tests (each G1/G8 writes a file; usd gate test leaves runTag 888 file): about 8 G files + 5 flights -> cf5 ~42. OK, but archive the G files (test files, not flights) before F1 as in lab_run.sh.
18. **Lab / FBL**: the lab lock exists; the chirp session is the user's call. The S1 line of v21 is PAUSED: this session must not change `v21/S1` gain files (F3/F4 sets go into `chirp_session/`, not into `v21/`), and must not use `GAINS_ROOT` default (v1 gains).
19. **Battery**: fresh pack per flight, rest >= 4.1 V, swap if < 4.0 V (S0_LAB_CARD row 6; decision 5). No in-flight limit exists in our protocol and none is invented; the draft's "battery below the lab minimum" abort trigger is removed. Loaded `pm.vbat` is only reviewed after the flight.
20. **cf_second must be OFF the radio and not powered in the room volume** (solo yaml switch; `lab_run.sh solo S1 1`), otherwise radio sharing and a second body in the mocap.
21. **Mocap**: chirp amplitude in position is mm-level; mocap dropouts (real packet statistics) change E_full results, not safety. A dropout > 0.5 s: EKF diverges: abort rule `telemetry silent` covers radio, not mocap: add a vbat/position sanity check (z < 0.3 m or |pos - pos0| > 0.3 m = abort) in the script: currently NOT in the abort list.
22. Draft script details to review: `cf.get_position()` fallback uses `initialPosition` + hover (OK only with `--auto-center` not used here), `Monitor(swarm.allcfs, cf.prefix)` passes `allcfs` as node (check it is a Node), `log` file written in the CWD, keys thread is a daemon on `sys.stdin`.

---------------------------------------------------------------------------------------------------
## 2. Lab integration plan (exact paths; crazyswarm2 NOT changed now)

Laptop = this repo `~/Desktop/flying_robot_course` (branch feature/auto-tuning); lab PC = `~/flying_robot_course`, same branch, `git pull`.

### 2.1 Files to add (all in OUR repo, none in crazyswarm2; copy, never edit the sim agent's branch)
| source (read-only) | destination in this repo | note |
|---|---|---|
| `proposal/chirp_flight_script_DRAFT.py` | `experiments/diffcotune/lab/chirp_session/chirp_flight.py` | after items 1, 5, 21 changes (below); run with `python3`, not `ros2 run` (needs stdin) |
| `proposal/usd_study_config_chirp_proposed_v3.txt` | `experiments/diffcotune/lab/chirp_session/usd_chirp_config_v3.txt` | copied to cf5's card as `config.txt` only; keep `flying_drone_stack/tools/usd_study_config.txt` untouched |
| `proposal/crazyflies_chirp_telemetry_PROPOSED.yaml` | `experiments/diffcotune/lab/chirp_session/crazyflies_chirp_telemetry.yaml` | fragment for the cf5 block; merge into the lab-PC yaml (the lab yaml is edited by `lab_run.sh prep`; `YAML.before_lab_run_2026-11-18` is the backup), see 2.4 |
| `proposal/usddeck_limit128_ring16k_AB.patch` | `experiments/diffcotune/lab/chirp_session/usddeck_limit128_ring16k_AB.patch` | applied to the firmware tree on the laptop (`~/Desktop/crazyflie-firmware` / `flying_drone_stack/firmware_app`: path of the usddeck.c tree to be confirmed) |
| `proposal/decode_usd_log_rename_additions.txt` | merged into `flying_drone_stack/tools/decode_usd_log.py` by the user's go (not before) | 115-column names |
| new | `chirp_session/v21chirp/S1/round_{1,3}/gains.yaml`, `v21chirp/S3/round_1/gains.yaml` | F1/F2/F5 = copy of `v21/S1/round_1/gains.yaml`; **F3 = KR_x, KR_y x0.9 -> 0.00693 (from 0.0077), Kw unchanged 0.001178120757958952**, **KR_z unchanged (confirmed by the user 2026-11-28)**; F6 = copy of `v21/S3/round_1/gains.yaml` as flown. **Created 2026-11-28**, see 2.1a |
| new | `chirp_session/usd_start_snippet.py` | written (decision 1), to be handed to the sim agent |
| new | `chirp_session/session_log_2026-11-28.md` | one line per step/flight |

**2.1a Gain files (created, `chirp_session/v21chirp/`; `v21/S1` and `v21/S3` untouched).** Source: `experiments/diffcotune/lab/v21/{S1,S3}/round_1/gains.yaml`, last changed in commit `dba5d986` (v2.1 round 1 desk steps), the files as flown in S1 / S3 round 1.
| chirp file | used for | content | check |
|---|---|---|---|
| `v21chirp/S1/round_1/gains.yaml` | F1, F2, F5 | byte-identical copy of `v21/S1/round_1` | `cmp` identical |
| `v21chirp/S1/round_3/gains.yaml` | F3 | the same + 1 comment line; only `KR_x` 0.0077 -> 0.00693, `KR_y` 0.0077 -> 0.00693 (x0.9); `KR_z` 0.0088, `Kw_x/y` 0.001178120757958952, `Kw_z` 0.0022 unchanged | `diff` shows only those 2 values + the header line |
| `v21chirp/S3/round_1/gains.yaml` | F6 | byte-identical copy of `v21/S3/round_1` | `cmp` identical |
`round_3` is only a directory name for `lab_run.sh prep S1 3` (chirp GAINS_ROOT); it is NOT a DiffCoTune round.

Changes needed to the script before use (list only; no edit now): (a) uSD start/stop with gate = `usd_start_snippet.py` (decision 1); (b) abort additions: position excursion > 0.3 m (**session safety limit, approved by the user 2026-11-28; not a protocol rule**), z < 0.25 m after the takeoff wait (protocol floor rule, S0_LAB_CARD l.41); NO vbat trigger (decision 5); (c) `--height` argument (0.6 m for F1-F6; raising it is the user's call after F2); (d) `--amp-ramp` 0.1 -> 0.3 m/s2 over the first 20 s of each chirp; (e) write setpoints/CSV to `chirp_session/out/` named by flight id (F1..F5, G2); (f) require TTY (exit if `not sys.stdin.isatty()`).
Changes needed to crazyswarm2 (**not made**): none mandatory. Optional: 100 Hz telemetry fragment (yaml in the lab-PC workspace); `usd_gate` import path `crazyflie_examples.usd_gate` (exists in `~/Desktop/crazyswarm2` on the laptop; on the lab PC it is in the built workspace).

### 2.2 Flash procedure (patched image) and readback
Laptop first (the planning agent / user decide; this is a firmware change):
1. `cd <firmware tree>` ; `git status` clean; `patch -p1 --dry-run < .../usddeck_limit128_ring16k_AB.patch` then apply; full build for the SAME target/config as the image of record (S1/S3 DShot image 9e133f76); record `size` output: RAM must be 116.4 KB of 131 KB, stack check passes; save `chirp_ring16k_candidate.bin` + `sha256sum` into `chirp_session/IMAGE.md`. **Stop if the link or RAM check fails.**
2. Rollback image (decision 4): `flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_dshot_candidate.bin`, sha256 `9e133f76...57c0`, md5 `fec0d76fb4a057de94b09b80d812ab10`: re-check both with `sha256sum` / `md5sum` before the flash. Check the patched image against section 2.2a as well.
3. Flash cf5 ONLY (solo session; cf_second stays on 9e133f76), CS2 stopped, drone on: `cfloader flash $BIN stm32-fw -w radio://0/80/2M/E7E7E7BB02` (command as in `docs/fbl/LAB_PLAN.md`), power-cycle.
4. Readback: `python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7BB02 9` must print "Using OutOfTree4 (9) controller" (console line; param readback is a cache); `python3 flying_drone_stack/tools/check_dshot_rpm.py radio://0/80/2M/E7E7E7BB02`; `python3 experiments/diffcotune/lab/apply_gains.py readback --uri radio://0/80/2M/E7E7E7BB02 --fragment experiments/diffcotune/lab/chirp_session/gains_F1.yaml --strategy S1` after the CS2 yaml apply (as in `lab_run.sh prep`); `python3 flying_drone_stack/tools/check_usd_deck.py radio://0/80/2M/E7E7E7BB02` after the card config is v3: console must show `malloc buffer 16384 bytes [OK]`.
5. DShot hover check after the first hover (G7 / F1): `diff = a_rpm_fz - a_imu_fz ~ +2.7 m/s2`.
6. **Rollback**: `cfloader flash ~/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_dshot_candidate.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02` (after the md5 check), power-cycle, restore the study config (`restore_study_sd_config.py`), `check_usd_deck.py`, controller 9 console check, DShot hover check.

### 2.2a Patched image (REPORT_59, filled in)
- Path (laptop, sim worktree): `~/Desktop/frc_desk_s1gap/experiments/desk_s1_gap/proposal/lab_pkg/firmware/fbl_v2b_dshot_usd128_candidate.bin`; 541,828 B; sha256 `a152c92219fc15f2d6bb12532714248eb27c293dcf07b4680b54c19eb4f86d06`; md5 `a41c349241631e4a1d4825e592085722`. **Checked by the lab agent 28 Nov: size, sha256 and md5 match REPORT_59 and the `.sha256` file next to it.** It is 9e133f76 reproduced byte for byte + `usddeck_limit128_ring16k_AB.patch` (build: `firmware/reproduce_build_ns.sh`).
- **Flash path (laptop, as on 18 Nov; copied and verified 28 Nov, user decision):** `~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_dshot_usd128_candidate.bin` (next to the rollback image `fbl_v2b_dshot_candidate.bin`) + `fbl_v2b_dshot_usd128_candidate.sha256`. After the copy: `cmp` identical to the sim worktree file, size 541,828 B, sha256 `a152c922...86d06`, md5 `a41c3492...5722` all match. The folder is git-ignored (`.gitignore` line 17): the bin and the `.sha256` are NOT committed. Re-check sha256 + md5 before `cfloader`: `cd ~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts && sha256sum -c fbl_v2b_dshot_usd128_candidate.sha256 && md5sum fbl_v2b_dshot_usd128_candidate.bin`.
- Flash command (cf5 only, CS2 stopped): `cfloader flash fbl_v2b_dshot_usd128_candidate.bin stm32-fw -w radio://0/80/2M/E7E7E7BB02`, then power-cycle.
- After the flash (cf5 only), power-cycle: console `malloc buffer 16384 bytes [OK]` (not [FAIL]), with the v3 card in `Logging to:`, `check_active_controller.py <cf5 uri> 9` console line, DShot hover check.

### 2.3 Card handling
Cards at /media/georg/THESIS1 (cf5) and THESIS2 (cf_second). Before the session: both cards < 85 files (now 29 / 20), `python3 flying_drone_stack/tools/usd_card_count.py --expect 0` after archiving test files. cf5 card: `config.txt` := `chirp_session/usd_chirp_config_v3.txt` (`cp` + `sha256sum`, copy of the old config saved as `config.txt.study96` OUTSIDE the card root). cf_second card unchanged and drone off. Copy after each flight: `python3 flying_drone_stack/tools/copy_usd_log.py /media/georg/THESIS1 cf5_chirp_F<n> --dest experiments/logs/usd_raw/2026-11-28` (byte verify; tag by flight id; match to the setpoint CSV by time). No radio CSVs.

### 2.4 Yaml / CS2
`export GAINS_ROOT=$PWD/experiments/diffcotune/lab/v21` is NOT used here (the F sets live in `chirp_session/`); `lab_run.sh prep` takes `$GAINS_ROOT/<S>/round_<l>/gains.yaml`. Plan: create `chirp_session/v21chirp/S1/round_{1,3,4}/gains.yaml` (F1/F2/F5 = copy of `v21/S1/round_1`; F3, F4 modified) so the standard wrapper works unchanged:
`export GAINS_ROOT=$PWD/experiments/diffcotune/lab/chirp_session/v21chirp` and `experiments/diffcotune/lab/lab_run.sh prep S1 1` (round dirs 1, 3, 4 = F1/F2/F5, F3, F4; names to be agreed). Then `lab_run.sh solo S1 <l>` (cf_second disabled). Note `prep` includes `check_dshot_rpm`, readback, gate test (both drones) and leaves test files to archive. For 100 Hz telemetry merge the fragment with `apply_gains.py merge --fragment chirp_session/crazyflies_chirp_telemetry.yaml --yaml $YAML --out $YAML.new` only if the merge tool supports the `firmware_logging` key (not checked) otherwise by hand; sha256 of the yaml after every switch; CS2 relaunch.

---------------------------------------------------------------------------------------------------
## 3. Lab-day run sheet

Conventions: lab PC, `cd ~/flying_robot_course && git pull`; one terminal for CS2, one for commands. "Stop rule" = stop the whole session and tell the planning agent. Log every step with a time in `chirp_session/session_log_2026-11-28.md`. After every flight: `usd_check.py` (section 3.1) on the copied log; flight is only counted if GO.

### 3.1 GO / NO-GO: `usd_check.py` (sim package `proposal/lab_pkg/`, REPORT_59 L3; < 2 min per file; exit 0 = GO, 1 = NO-GO)
Analysis-side files (all together in one directory; per PACKAGE_MANIFEST suggested `flying_drone_stack/tools/chirp_session/`): `usd_check.py chirp_lib.py cfusdlog.py decode_usd_log_v3.py v3_variables.json rebuild.py q2_loop.py q2_timing.py usd_analyse.py`. Not copied yet. Checks: 1 all 115 variables, 2 lost rows < 0.2 % and no gap > 60 ms, 3 chirp axis / peak amplitude +-15 % / sweep 1 -> 8 Hz / other axis quiet, 4 peak tilt below `--tilt-go`, 5 hold gyro rms < 30 deg/s, 6 timing pipeline (quaternion, coherence >= 0.8 in >= 4 of 5 bands), 7 KR / Kw logged as commanded (1 %), 8 INDI-correction variables alive.
Commands (run on the copied file, once per chirp axis; y chirp first, x chirp with `--axis x`):
| flight | command |
|---|---|
| G1 / G2 / G8 | `python3 usd_check.py <file> --axis y` (G1: checks 1, 2, 7, 8 must PASS; G8: check 2 + `usd.fatWrBps`) |
| F1 | `python3 usd_check.py <F1 file> --axis y --amp 0.3 --kr 0.0077 --kw 0.00117812 --tilt-go 8` |
| F2, F5 | same with `--tilt-go 10` |
| F3 | `... --kr 0.00693 --kw 0.00117812 --tilt-go 10` |
| F6 (S3) | `... --kr 0.00707035 --kw 0.00113856 --tilt-go 10` (S3 v21 r1 KR_x/y 0.007070351169510748, Kw_x/y 0.001138557308828068; the checker reads KR_x/Kw_x for the y axis) |
Caveats: (a) the check-3 peak test (0.3 +-15 %) assumes the amplitude is reached; with the 0.1 -> 0.3 ramp it is, after 20 s, but the window test (`chirp_window`) with a ramp is NOT tested by the sim agent: run it on G2 first. (b) G1 uses `usd_check` checks 1, 2, 7, 8 only.
After F2-F5: `python3 usd_analyse.py --out <dir> F2=<file>:y ...` (post-session analysis, section 4 of the sim card; not a lab GO/NO-GO).

### Step 0. Setup (user, once)
| # | Command | Expected | Stop rule |
|---|---|---|---|
| 0.1 | `git pull`; `git log --oneline -1`; `sha256sum` of chirp_session files | files present, sums as in IMAGE.md | missing / differing |
| 0.2 | mocap on, props OFF, fresh battery on cf5, cf_second OFF and removed from the room | - | - |
| 0.3 | flash the patched image on cf5 (2.2 steps 3-5) | readback lines OK, `malloc buffer 16384 bytes [OK]` | link/readback fails -> rollback 2.2.6, no flights |
| 0.4 | `python3 flying_drone_stack/tools/usd_card_count.py --expect 0` after archiving, both cards | < 85 files each | >= 85 |

### Ground tests (props OFF unless stated)
| # | Command / action | Expected | Stop rule |
|---|---|---|---|
| G1 logging chain 10 s | start logging (gate), hand-tilt the drone, stop; `python3 flying_drone_stack/tools/decode_usd_log.py <file>` | `locsrv_q*` unit norm and changing; 4 tau columns non-zero, change with tilt; `torquex` present; file closed | any column missing/zero -> NO-GO (patch/config) |
| G2 stream delivery 20 s (clamped, idle) | `python3 experiments/diffcotune/lab/chirp_session/chirp_flight.py --ground --T 20` (to be added) | `ctrltarget_ay` follows the chirp (amp 0.3, 100 Hz, no gaps > 30 ms); telemetry msgs per topic over 60 s >= 95 % of nominal | gap > 30 ms or < 95 % -> fall back to 50 Hz; still bad -> stop |
| G3 operator abort | during the stream press `q` + Enter | first hold setpoint in the log < 0.2 s after the key; HLC land accepted | >= 0.2 s -> stop |
| G4 emergency (clamped, props off) | key `e`; also `ros2 service call /all/emergency std_srvs/srv/Empty` | `motor.m*` / `rpm.m*` = 0 within one tick; reboot after | motors not 0 -> stop |
| G5 telemetry trip (hand) | tilt > 15 deg for > 0.3 s; spin > 300 deg/s for > 0.3 s | abort fires; write down delay (expected 0.30-0.35 s + radio) | no trip -> stop |
| G6 stream loss (clamped) | pull the dongle during the stream | level-out ~0.5 s, motors off ~2 s (`supervisor.info`) | times differ from code -> nothing is flown |
| G8 uSD write margin 120 s idle | 1) v3+patch, 2) control: study 96-variable config, same card, fresh format/archived | v3: rows with dt > 3 ms < 0.2 %, no gap > 60 ms, `usd.fatWrBps` ~ 215 kB/s; control ~ 1.1 % loss | not met -> lower log rate to 250 Hz (REPORT_52) or stop |
| G7 tethered low step (props ON, decision 3, BEFORE F1) | slack tether, hover 0.3-0.4 m, chirp 0.1 m/s2 for the first 20 s only, then test `q` once in the air with a hold command | no tilt > 8 deg, abort test clean | any tilt > 10 deg -> land, stop |
Order: G1, G2, G3, G4, G5, G6, G8, then G7 (G7 = first flight, last ground item).

### Flights F1-F6 (solo, hover 0.6 m, amplitude ramp 0.1 -> 0.3 m/s2 over the first 20 s of each chirp, then 0.3; each: takeoff, 10 s hold, y chirp 1->8 Hz 60 s, 5 s rest, x chirp 60 s, 10 s hold, land; ~3 min)
Before EACH flight: fresh battery (rest >= 4.1 V, swap if < 4.0 V), `check_usd_deck.py` cf5 (canLog 1, `usd.bcUSD`), `usd_card_count.py` < 85, readback of the gains of that flight, controller 9 console line after any reboot. Start: `python3 experiments/diffcotune/lab/chirp_session/chirp_flight.py --height 0.6 --amp 0.3 --flight F<n>` (foreground terminal, hand near keyboard, `q`+Enter = land, `e`+Enter = emergency). After landing: `usd.logging 0` is done by the script, power state: leave the drone on for the copy? No: copy only after power-off (card handling while powered gave canLog 0 on 11-20).
| # | Gains | Expected | Go for next | Stop rule |
|---|---|---|---|---|
| F1 | `chirp_session/.../round_1` (S1 v21 r1) | file complete (115 columns), coherence chirp->attitude > 0.8 over 1.5-8 Hz, tilt response linear | usd_check GO and tilt max < 8 deg, roll gyro rms in the holds < 30 deg/s | tilt > 15 deg, abort fired, or limit-cycle (roll gyro rms over 0.5 s > 150 deg/s) |
| F2 | same (repeat after the quick look) | measurement: gyro-vs-mocap delay +-2 ms, L(f) Ms/PM/GM (sim ref Ms 10.9 dB, PM 18 deg, GM 4.8 dB at the flown gains) | usd_check GO and tilt max < 10 deg | as F1 |
| F3 | S1, KR_x / KR_y x0.9, Kw unchanged | margin trend toward the stable side (compare with F2, usd_check) | GO and tilt max < 10 deg | any 3-4 Hz growth in the holds, roll gyro rms > 30 deg/s in the 10 s holds -> `q`, stop the S1 series |
| F4 | dropped (KR x1.2, decision 2) | - | - | - |
| F5 | same as F2 | repeatability (battery, drift): error bars | - | as F1 |
| F6 | **S3**, v21 r1 gains as flown (`lab_run.sh prep S3 1` with the chirp GAINS_ROOT; indi 3 + rnn.en 1 CS2-log / readback check, NS2 active) | one chirp, same amplitude and ramp: confirms the fix for S3 | usd_check GO | as F1; after the S3 switch: readback + hover check (S3 r0: +2.64 m/s2) |
After each flight (user): power off, card out, `copy_usd_log.py` (2.3), then `usd_check.py` (section 3.1) on the copied file; quick look: tilt max, gyro rms in the holds, `pm.vbat`. Nothing flies the next item if GO is missing.

### End of session
1. Logging (decision 7): if G8 passed, v3 + the patched image stay as the standard (cf5 card keeps `config.txt` = v3); copy `config.txt` v3 to the cf_second card only after cf_second is flashed too (not done in this session). If G8 failed or the patch is rolled back: rollback flash (2.2.6) and `restore_study_sd_config.py`.
2. Archive the test files (G-tests, usd gate runTag 888 files, G8 control) in `_archive_2026-11-28_chirp` on the cards, `usd_card_count.py --expect 0`, files < 85.
3. Session log entry, commit explicit paths; the lab lock is removed only by the user. Round 4 flights stay on hold (decision 6).

---------------------------------------------------------------------------------------------------
## 4. Open items (after the REPORT_61 review, section 6)
1. **Decision for the user: the tilt rule vs the sim's own prediction.** `usd_check` check 4 only WARNs (exit 0 = GO) when peak tilt is between `--tilt-go` and the 15 deg abort limit; the sheet's "go to next only if tilt max < 8 / 10 deg" is therefore NOT enforced by the exit code: the lab agent reads the WARN line by hand. Also the sim's expected peak roll at 0.3 m/s2 is 10.6 deg (frsim; the dry-run F2 shows 10.5 deg), above the F1 limit 8 and the F2/F3 limit 10: with the sim's own numbers F1 and F2 would end with a WARN. Choose: keep 8 / 10 (a WARN stops the series and the user decides), or raise F1/F2/F3 `--tilt-go` (e.g. 12, the sim card's old F3 value). Not changed by me.
2. Copy plan (not done): analysis files (`usd_check.py chirp_lib.py cfusdlog.py decode_usd_log_v3.py v3_variables.json rebuild.py q2_loop.py q2_timing.py usd_analyse.py`) to `flying_drone_stack/tools/chirp_session/`; `chirp_flight.py` (commit b17da76b version) to `experiments/diffcotune/lab/chirp_session/chirp_flight.py`; the sim worktree commit is local only (not pushed), the lab PC cannot see it: files must travel by our repo or by hand.
3. G2 must confirm: `cf.position` is live (the script reads `/cf5/pose` through crazyflie_py; it starts as [0, 0, 0] and is updated only inside `th.sleep`); a dead pose topic would trip the z < 0.25 m abort right after takeoff (safe direction) or leave the excursion check blind.
4. v3 config on the cf5 card: `lab_pkg/config.txt` is byte-identical to `proposal/usd_study_config_chirp_proposed_v3.txt` (checked).

---------------------------------------------------------------------------------------------------
## 5. Differences: sim card (`proposal/lab_pkg/flight_card_chirp.md`, as of 13:24, REPORT_59) vs this sheet (the sheet is the lab master)
| # | topic | sim card | this sheet |
|---|---|---|---|
| 1 | hover height | 0.9 m (title, `HOVER_Z` in the script) | **0.6 m** (decision 3) |
| 2 | amplitude | 0.3 m/s2 flat | **ramp 0.1 -> 0.3 over the first 20 s** of each chirp (decision 3) |
| 3 | flight list | F1-F5 | **F1-F3, F5, F6** (F4 dropped, S3 chirp F6 added) |
| 4 | F3 gains | KR x1.1 for x, y AND z (0.00847 / 0.00968) | **KR_x, KR_y x0.9 = 0.00693; KR_z unchanged** (decision 2, user 28 Nov) |
| 5 | F4 | KR x1.2 (0.00924 / 0.01056) | dropped (real edge x1.0; S1 r2 oscillated) |
| 6 | F6 | none | S3 v21 r1 as flown (KR 0.00707035, Kw 0.00113856) |
| 7 | `--tilt-go` | F1 8, F2 10, F3/F4 12 | F1 8, F2 10, F3 10 (no x1.1/x1.2 step), F5 10, F6 10 |
| 8 | G7 tethered | "first flight only" | **before F1**, every other flight untethered |
| 9 | where `chirp_flight.py` goes | `crazyswarm2/.../chirp_flight.py`, `ros2 run` | **`experiments/diffcotune/lab/chirp_session/chirp_flight.py`, `python3`** (no crazyswarm2 change; also needs a TTY for the keys) |
| 10 | uSD start | not in the card or the script | verified gate: `usd_start_snippet.py` (decision 1) |
| 11 | abort triggers | key q/e, tilt 15 deg / rate 300 deg/s 0.3 s, telemetry silent 0.5 s | same **plus position excursion > 0.3 m (session safety limit, user-approved 28 Nov) and z < 0.25 m (protocol floor rule)**; no in-flight battery trigger |
| 12 | battery | "fresh battery per flight" | **rest >= 4.1 V, swap if < 4.0 V** (S0_LAB_CARD row 6) |
| 13 | card state | "freshly formatted" | archive instead of format (cards 29 / 20 files, < 85); test files archived before F1; uSD logs of earlier DiffCoTune rounds stay |
| 14 | after the session | not addressed | decision 7: v3 + patch stay if G8 passes (add, never swap); rollback only if G8 fails |
| 15 | round 4 | not addressed | on hold; chirp session first (decision 6) |
| 16 | analysis tools | `flying_drone_stack/tools/chirp_session/` (manifest) | same; nothing copied yet |
| 17 | telemetry 100 Hz yaml | optional | optional; accept only if G2 shows >= 95 % messages, else 50 Hz or 20 Hz (0.35 s trip) |
| 18 | `flight_card` vs script | card says script in the card = final and mock-tested | the script is still the DRAFT logic (open item 1) |

---------------------------------------------------------------------------------------------------
## 6. Review of REPORT_61 (sim worktree commit b17da76b, read with `git show b17da76b:<path>`; lab agent, 28 Nov)
Files reviewed: `proposal/lab_pkg/chirp_flight.py`, `flight_card_chirp.md`, `PACKAGE_MANIFEST.md`, `REPORT_61.md`. Dry-run files: `results/dryrun/*.bin` and `results/dryrun_x/F2.bin` exist only as UNTRACKED files in the sim worktree (not in the commit); `usd_check.py` used for the run is the committed b17da76b version (extracted by `git archive`).
**Section 5 rows, all 18:**
| # | resolved? | where |
|---|---|---|
| 1 height 0.6 | yes | `--height` default 0.6, G7 0.35, land duration follows the height |
| 2 ramp | yes | `--amp0 0.1 --ramp 20` defaults, `chirp_triple` ramps `A(t)`; usd_check expects the ramp |
| 3 flight list | yes | card F1, F2, F3, F5, F6 |
| 4 F3 gains | yes | KR_x/y 0.00693, KR_z 0.0088 and Kw unchanged (card + `usd_analyse`) |
| 5 F4 | yes | dropped |
| 6 F6 | yes | S3 v21 r1, KR 0.00707035, Kw 0.00113856 |
| 7 tilt-go | yes in the card (8 / 10 / 10 / 10 / 10); **but not enforced**: open item 1 |
| 8 G7 first | yes | `--g7`, card order G1..G6, G8, G7 |
| 9 location / python3 / TTY | yes | script refuses without a TTY (`CHIRP_ALLOW_NO_TTY=1_MOCK_TEST_ONLY` is a test escape; never set it in the lab); no crazyswarm2 change |
| 10 uSD gate | yes | the four snippet functions are inside the script; order precheck -> `usec.reset` on the ground -> takeoff -> start_and_verify in hover -> chirp -> land -> stop_and_verify -> `usd.logging 0` in `finally`; a failed start trips an abort and lands; `usec.reset` appears only in `usd_setup` (ground) |
| 11 aborts | yes | q / e, tilt 15 deg / rate 300 deg/s 0.3 s, telemetry silent 0.5 s, excursion > 0.3 m (marked user-approved in the script docstring and the card), z < 0.25 m after the takeoff wait, uSD start failure; no battery trigger; hold burst at the CURRENT position |
| 12 battery | yes | printed at start and in the card (4.1 V / 4.0 V) |
| 13 archive not format | yes | card text |
| 14 decision 7 | yes | card section 4 |
| 15 round 4 on hold | yes | card section 4 |
| 16 analysis location | partly | manifest row updated; copies still to be made by us (open item 2) |
| 17 telemetry yaml | yes | optional; 20 Hz topics exist in `crazyflies.yaml` (attitude, gyro_acc at 20 Hz) so the 0.5 s stale rule works without it (trip 0.35 s) |
| 18 script final | yes | final logic, mock `main()` test 18/18 per REPORT_61 (not re-run by me) |
**Script review notes (read-only, no edits):** (a) on a normal end the script lands with `notifySetpointsStop` + HLC land and waits `height + 2 s` before `usd_stop`; (b) after `e` the script skips `usd_stop` but still sends `usd.logging 0` (may not arrive after the firmware lock; the file may stay unclosed: check the card after any `e`); (c) `--ground` (G2) sends full-state setpoints at ground level: motors will spin, props OFF and the drone clamped as in the sheet; (d) no change to crazyswarm2; the script needs `crazyflie_examples.usd_gate` of 42fc430 from the built lab workspace.
**`usd_check.py` on the sim agent's dry-run files (once, y axis unless noted; exit 0 = GO):** F2 y: GO (checks 1-3, 5-8 PASS, check 4 WARN: peak tilt 10.5 deg vs 10; `usd.fatWrBps` median 215,029 B/s); F2 x: GO; F3 (`--kr 0.00693`): GO; F6 (S3 values): GO. Negative files: `F2_amp2` NO-GO (check 3, peak 0.598), `F2_lostrows` NO-GO (check 2, 3.04 % rows), `F2_swapaxis` NO-GO (checks 3, 6), `F2_novar` NO-GO (checks 1, 8), `F2_wrongkr` checked against the S3 values NO-GO in the sim agent's dry run, and by me against 0.00693 / 0.00847 NO-GO (check 7), GO against its own 0.0077. These are frsim-generated files, not flights.
