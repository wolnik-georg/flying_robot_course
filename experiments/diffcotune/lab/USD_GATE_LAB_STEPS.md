# uSD gate: lab steps for the next session (2026-11-18, written by the lab agent; NOT tested on hardware)

Background: `docs/HANDOVER_USD_LOGGING_LOST_2026-11-18.md`. Flights now refuse to take off / land if the decks are not provably writing
(crazyswarm2 `crazyflie_examples/usd_gate.py`, yaml topic `usd_status`). Rules unchanged: uSD only for the fit, never radio CSVs,
cards < 85 files, no usec.reset in flight, sha256 of both yamls after every switch, `check_active_controller.py` after a flash.

Use ONLY the gate flights (no `--no-usd-gate`): flights with that flag are recorded `usd_gate skipped` and are not usable for the fit.

## 0. Evidence from 2026-11-18 (lab PC, 1 min, read-only, BEFORE pulling or relaunching)
```
cd ~/flying_robot_course
find ~/.ros/log -newermt "2026-11-18 19:00" ! -newermt "2026-11-18 20:00" -name "*.log" | xargs grep -hiE 'Update parameter "usd|KeyError|setParam : exception|SD-Card|Config read|Logging to|Wrote .* B to|Failed to open|usd deck write|reqParamTOC' > experiments/diffcotune/lab/usd_evidence_2026-11-18.txt
wc -l experiments/diffcotune/lab/usd_evidence_2026-11-18.txt
```
Also save the terminal scrollback of the `run_formation` / `fly_file.py` runs of 19:29-19:40 into the same file (append). Push it.

## 1. Pull and build (lab PC)
```
cd ~/flying_robot_course && git pull                                    # feature/auto-tuning
cd <crazyswarm2 src>                                                    # the lab PC yaml has local edits (gains, solo): keep a copy, then
cp crazyflie/config/crazyflies.yaml ~/lab_yaml_labpc_before_gate.yaml
git checkout crazyflie/config/crazyflies.yaml && git checkout main && git pull   # lab_run.sh prep re-creates the strategy + gains state
git log --oneline -1                                                    # expect 7bac495
cd <ws root> && colcon build ...                                        # from the ws root, cpp backend only (as usual); then relaunch CS2 in step 3
```
## 2. yaml sha256 (lab PC and laptop copy) — new values after the gate commit (the yaml changed by +7 lines `usd_status`)
| strategy | branch | old | new |
|---|---|---|---|
| S0 | main | bd6c05e9… | `ac37a09e…` (full `ac37a09ec00e21c53d2b2191e6de26c64921f8416d2471d66100a396c457c72d`) |
| S1 | study_S1 | 7936cf3c… | `7d5f9157…` |
| S2 | study_S2 | f373260c… | `16fb8c4d…` |
| S3 | study_S3 | 620dd678… | `c498d776…` |
| FBL | study_FBL | 0be2d0c3… | `7f24e68c…` |
`set_tree_strategy.py S1/S2/S3` applied to the new main yaml is byte-identical to the new study_S1/S2/S3 yamls (checked on the laptop).
`sha256sum` of the lab PC yaml right after `git checkout main && git pull` must be the S0 value; after `lab_run.sh prep` it differs (gains merged) — that is expected.

## 3. Prep (includes the gate ground test) and the once-per-session negative test
```
experiments/diffcotune/lab/lab_run.sh prep S0 0        # yaml switch + gains, readback, controller 9 x2, usd deck x2, then the CS2-path gate test (--mode pass)
python3 flying_drone_stack/tools/usd_gate_ground_test.py --mode negative   # CS2 running, once per session: start only cf5, the gate must FAIL cf_second
```
Both must end with `RESULT: OK`. If the pass test fails: do not fly; copy the output to the planning agent.

## 4. Remove the test files from the cards, then check the card state
Each gate test writes one file per card (runTag 888 pass, 889 negative); the 2026-11-18 ground test also left `thesis00` (runTag 777) on both cards.
1. Power the drones OFF, take both cards out, move EVERY `thesis*` test file (777/888/889) to `_archive_<date>_gate_tests`, cards back in, power on.
2. `python3 flying_drone_stack/tools/check_usd_deck.py <uri>` for both drones (CS2 stopped).
3. Cards in the laptop (or lab PC): `python3 flying_drone_stack/tools/usd_card_count.py /media/<user>/THESIS1 /media/<user>/THESIS2 --expect 0`.
(If you cannot pull the cards between tests, count at the end of the block instead and compare with the number of gate-verified flights.)

## 5. Shakedown, then round 0 again
1. Shakedown A1 (S0 prep from step 3, then the printed A1 command). Watch for `uSD logging started and VERIFIED on ['cf5', 'cf_second']`.
   A gate FAIL prints a block and the script exits non-zero (ground precheck: no takeoff; in the air: lands, trajectory not started).
2. After the flight: `usd_card_count.py ... --expect 1` on both cards (cards out), then copy with `copy_usd_log.py`.
3. Then S0 round 0 again: `lab_run.sh prep S0 0` (the lab PC yaml is currently in the solo state with the round 0 gains), flights, `lab_run.sh solo S0 0`, CIRCLE, FIG8.
   Count the cards after every block; archive at >= 85 files.

## 6. Known limits
- The gate takes about 1-4 s at the start point (hover at slot height, before the trajectory): the write-rate counter needs about 1 s to rise.
- `usd.runTag` reaches the topic as float32: it proves the param arrived (to ~128), not which flight. The write rate (`usd.fatWrBps` > 0) is the proof.
- In simulation the usd params do not exist: use `--no-usd-gate` there.
