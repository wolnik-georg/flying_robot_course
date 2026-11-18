# Lab agent addendum 2026-11-18: hardware change, new image, round 0 as the validation

Paste everything below the line into the lab agent session, together with "session starts". It adds to `docs/HANDOVER_DIFFCOTUNE_LAB_AGENT_SESSIONS_2026-11-18.md` and changes it only where it says so.

---

## 1. What changed today (record all of it in `session_log_<date>.md`, section "Hardware and image")

- **New bottom drone.** cf5 had a power fault on 26 Oct (`docs/lab_sessions/2026-10-26.md:21`). The user built a new drone. Radio addresses:
  - bottom = yaml name `cf5`, `radio://0/80/2M/E7E7E7BB02`;
  - top = yaml name `cf_second`, now **`radio://0/80/2M/E7E7E7E7E7`** (it was `…E7E9`).
  - Ask the user which physical drone is the new one and record it. This is a hardware change: the per-round β fit absorbs plant differences, but it must be on record.
- **New image on both drones:** `fbl_v2b_candidate.bin`, sha256 `87bd963e8299ff6a4bda7dd190355651df7da4e905d90c505bd4cd663b299be4` (4 Nov). Contents: S0–S3 tree + NS2 v2 + the authors-aligned FBL. It is the same firmware source as the DiffCoTune simulator. **It is the image of record for the whole campaign from now on.** `check_active_controller.py … 9` on both drones: console `Using OutOfTree4 (9) controller`. Preflight check 7a still names `study_tree_v2.bin`; that is a note only, so ignore it and record this.
  - Caveat: this binary has not flown before. The S0–S3 source is identical outside the FBL blocks (FBL worker, F2c.5); the binary was not compared.
- **crazyswarm2:** the address fix is pushed on `main` (`80c54c8`) and on every `study_*` branch. The lab PC must `git pull` crazyswarm2 `main` and rebuild before the session. The laptop copy `~/lab_yaml/crazyflies.yaml` is now identical to `main`. If the lab PC yaml differs after the pull, copy it again.
- **uSD cards** (done by the planning agent, 17:5x):
  - THESIS1 and THESIS2: the old 17 files each were backed up to `~/usd_card_backup/THESIS{1,2}_2026-11-18_pre_diffcotune/` (checksums verified) and moved on the card to `_archive_2026-11-18_pre_diffcotune/`;
  - both cards carry the study config (`usd_study_config.txt`, 96 variables, sha256 `d247134c…`); THESIS2 had the FBL config before;
  - both start at `thesis00`;
  - still to do: `check_usd_deck.py` on both drones.

## 2. Order of the day (changes section 1 of the main prompt only by adding steps 2 and 4)

1. Checklist of the main prompt, section 2: lock file, preflight S0 + S2 with `~/lab_yaml/crazyflies.yaml`, uSD deck check, pad check, batteries, `round.py --init` for S0 and S2, supervisor OK.
2. **Shakedown (new drone + new image):**
   - S0 on both drones, one short hover;
   - then one A1 with the user's hand on the abort;
   - check: the uSD file is written on both cards (one file per flight), the controller-9 console line, no tilt abort.
   - This flight is **not** a tuning flight. Log it as `shakedown`.
3. **Round 0 of S0, then round 0 of S2**, normal tuning flights with the default gains (θ_0).
4. **Round 0 is the validation of the new setup.** Before you start round 1 of a controller, compare its round 0 flights with the pilot data on the old image:
   - `docs/tree_comparison_2026-10-19.md` (A8 dip, A1);
   - `docs/lab_sessions/2026-10-20.md` (pilot A1 / A8 / A4).
   - Report the numbers side by side to the user. You set no threshold.
   - If a controller is clearly off (crash, a very different A8 dip, A1 offset far outside the pilot range, NS2 not acting on S2), **stop that controller and escalate.** The fallback image `study_tree_v2.bin` is the planning agent's call, not yours.
   - S2: check that `rnn.en 1` acts (NS2 output non-zero in the uSD log).
5. If round 0 looks like the pilot: continue rounds 1–5 as in the main prompt.
6. S1 + S3 come only after S0 + S2, or in the next session. Their round 0 is again their validation; check the DShot RPM values on the first flight.

## 3. FBL today: no

FBL is not ready for tuning: its dry runs are not finished (memory leak, Stage F2e). FBL tuning flights and FBL bench checks are **not** part of this session. The FBL hardware shakedown comes in a later session, from `experiments/diffcotune/lab/FBL_FLASH_CARD.md` on branch `feature/diffcotune-fbl`. No reflash will be needed then: the image is already `fbl_v2b`. Do not switch `ctrlFbl.en` on today.
