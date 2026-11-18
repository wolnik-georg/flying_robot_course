# Handover to the planning agent: S0 round 0 (2026-11-18) produced NO uSD logs — find the cause, make it safe
Written by the lab agent, 2026-11-18 ~20:05. Facts only; "not stated" where I do not know. The lab is closed; nothing below can be tested on hardware until the next session.

## 1. What happened
- Day setup (planning agent, 17:5x): both cards emptied (old files in `_archive_2026-11-18_pre_diffcotune`), study config (`usd_study_config.txt`, 96 vars, sha256 `d247134c…`) on both. Image `fbl_v2b_candidate.bin` (sha `87bd963e…`) on both drones, first time flown.
- Preflight S0/S2 passed, `round.py --init` S0/S2 at 19:04, `lab_run.sh prep S0 0` etc. (repo `feature/auto-tuning`).
- User flew S0 round 0 by hand. Radio CSVs pushed (commit `a4e00e24`): A1 19:29:54 / 19:31:27 (~150 rows, probably aborted) / 19:32:13; A8 19:33:47 (~140 rows, probably aborted) / 19:34:57; A4 19:36:17; CIRCLE 19:39:32; FIG8 19:40:23. Which attempts crashed: from user notes, not confirmed by me.
- Both cards were read on the laptop afterwards (THESIS1 label "THESIS" mounted at /media/georg/THESIS1; THESIS2): **no `thesis*` file on either card, not even a 0-byte one.** Only config files and `_archive_*` folders; newest item = the 17:5x pre-session archive. The user states only these two cards were in the drones.
- Per the earlier lifecycle finding (memory `project_usd_logging_chain_validated_2026-09-15`) every `usd.logging` 0->1 creates a file, 0-byte if never closed. So "no file at all" = the start never reached the deck or file creation failed — it is NOT the usual "0-byte after a crash".
- Consequence: **no usable data for S0 round 0.** Radio CSVs are not allowed for the fit (rule: uSD only). S0 round 0 (and the shakedown) must be re-flown unless the planning agent decides otherwise. Nothing was fitted; `lab/S0/round_0` has only gains.yaml + state.json.

## 2. What I checked (all read-only or ground-only, no flight)
| Check | Result |
|---|---|
| Flight script started logging? | YES: per-drone CSV carries `# meta:usd_start_s=15.895596` (A1 19:32:13) and `usd_run_tag` in meta; the broadcast call raised no exception (script prints a WARN if it does). setParam broadcast is not acknowledged. |
| Card config | `THESIS2/config.txt` == `flying_drone_stack/tools/usd_study_config.txt` (sha `d247134c…`), 96 vars, format identical to the Oct 13 file. THESIS1's config.txt (dated Oct 13) has the same first lines; I did not hash it. |
| Variables in the image | Over the radio (cflib log TOC, both drones): 674 vars each, **all 96 config vars present, none missing** (script in section 5). |
| Deck state after the session | `check_usd_deck.py` on cf5 `E7E7E7BB02` and cf_second `E7E7E7E7E7`: usd group found, `usd.canLog=1`, `usd.logging=0`, `usd.bcUSD` MISSING (known, harmless). The old cf_second address `…E7E9` does not answer (obsolete). |
| **Ground test, per-drone cflib (19:57:18 cf5, 19:57:29 cf_second)** | `usd.runTag=777`, `usd.logging 1`, 5 s, `0`: console `uSD: Logging to: thesis00` / `uSD: Wrote 886351 B (2478 of 2502 events)` on cf5; `Wrote 885994 B (2477 of 2503)` on cf_second. **Logging works on both drones, both cards, with the new image and the study config.** Both started at `thesis00` => no earlier file existed on either card (consistent with the card reads). |
| Cards now | each holds one 5 s TEST file `thesis00` (runTag 777). Not a flight. Must not enter any round. |

## 3. Where the difference must be (not found yet)
Per-drone parameter set over cflib works; the flights used the CS2 **broadcast** `allcfs.setParam('usd.runTag', tag); sleep(0.05); allcfs.setParam('usd.logging', 1)` in `run_formation.py` (crazyswarm2 `main` 80c54c8, lines ~995-1002). Things that changed on 2026-11-18 vs the last day uSD logging demonstrably worked (the planning agent knows which day; I do not): image `fbl_v2b` (never flown, "binary not compared"), cf_second address fix (`E7E9` -> `E7E7E7E7E7` after reflash), both cards emptied and edited on the laptop at 17:5x, crazyswarm2 `main` + lab PC rebuild, `lab_run.sh` (does not touch logging). Candidates, none verified:
1. The CS2 broadcast `usd.logging` does not reach the decks with this image/CS2 build (different TOC/param id, broadcast address, cache of the parameter TOC in CS2/crazyflie_cpp, param type).
2. The decks were not ready at flight time (`usd.canLog` 0): cards edited on the laptop 17:5x and put in without a deck check right before the flights; I do not know whether `check_usd_deck.py` was run on the lab PC with output before round 0 (user reported OK at 17:4x only, BEFORE the 17:5x card edits).
3. Something in the new image resets/blocks logging during CS2 operation (FBL or OOT code touching the usd param, other reset) — read the firmware diff vs the last image that logged.
4. Something about the CS2 launch of today (yaml firmware_params, the solo switch relaunch) — not stated.

## 4. Open evidence the lab PC still has
- CS2 console / `~/.ros/log`: the firmware prints `uSD: Logging to: thesisNN`, `uSD: Wrote ...`, `mount SD-Card [OK/FAIL]`, `Config read [OK/FAIL]`, `Failed to open file`, `usd deck write failure`. Command: `find ~/.ros/log -mmin -600 -name "*.log" | xargs grep -hE "SD-Card|Config read|Unknown log variable|out of storage|Logging to|Wrote .* B to|Failed to open|usd deck write"`. I have NOT seen this output.
- `usd_broadcast_test.py` (new, commit `dd739a33`, tools dir): 5 s ground test of the exact CS2 broadcast call, no motors; writes `thesis01` (runTag 888) per card if it works. NOT run yet (needs CS2 on the lab PC; the laptop has no CS2 build and its crazyswarm2 is on `study_FBL_stock`, wrong yaml for the drones). It decides cause 1.

## 5. Tools added by me (pushed)
- `flying_drone_stack/tools/usd_broadcast_test.py` — CS2 broadcast ground test (above).
- `flying_drone_stack/tools/usd_ground_log_test.py` — per-drone cflib ground test with console capture (validated today).
- Log-TOC vs config check was a scratch script (not in the repo); trivial to recreate: connect with cflib, `cf.log.toc.toc`, compare `group.name` of `config.txt` lines 8..end.
- `experiments/diffcotune/lab/lab_run.sh` prep/solo/flights (commit `c814e8bd`). Repo HEAD at the time of writing: see `git log -1`.

## 6. Proposed fixes / next steps (planning agent decides; I did not change any flight code, config or card)
1. Lab, before any flight: run `usd_ground_log_test.py` (per drone) and `usd_broadcast_test.py` (CS2 path) on the ground; both drones must print `Logging to` / `Wrote`. Then ARCHIVE the test files off the cards (they are runTag 777/888, not flights) and only then fly. Do this again after every card edit or reflash.
2. Make logging verifiable inside the flight path, without changing the logic: after the broadcast start, read `usd.logging` back from every drone (fresh param get) and, if any is not 1, re-send per drone and abort before takeoff if it still is not; read `usd.canLog` before arming. `lab_run.sh prep` should run the ground smoke test automatically (it already runs `check_usd_deck.py`, which only proves deck/card presence, not that a start works — today it passed while no flight logged).
3. Diff `run_formation.py` / crazyswarm2 `main` and the firmware source/image of `fbl_v2b` against the last session with working uSD logging; look at any change to `usd.*` params, the CS2 broadcast, or the usd deck driver.
4. After a flight: check immediately that each card gained a new non-empty file (card count before/after), before the next flight; the S0 card says "check on the first flight that it is one file per flight".
5. Re-fly S0 round 0 once logging is proven (new shakedown A1 first). The lab yaml on the lab PC is currently in the S0 solo state (cf_second may be disabled) with the round 0 gains merged: run `lab_run.sh prep S0 0` again (it switches back to the two-drone yaml).
6. Rules unchanged: uSD only, never radio CSVs for the fit, card < 85 files, no usec.reset in flight, check sha256 of both yamls after every switch.
