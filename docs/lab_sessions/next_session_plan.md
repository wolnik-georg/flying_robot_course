# Next lab session — plan (reworked 2026-10-14, date open) — PLANNED

## 0. What changed
**DECISION 2026-10-14 (author): the Pure INDI of the study (Strategy 1) is Omar C + z integral 1.5 (controller 9, `Kpos_Iz 1.5`, `deck.bcRpm 1`).** Supervisor not yet consulted. Consequences for this session:
- **Dropped:** ours with Omar's values (`ours_omar_values_*` patches), repeats of ours frozen, Tal & Karaman with Omar's values (`TK_omar_values*`), Omar Rust, the "why do ours and Omar C dip" investigation. Patches stay in the repo (parked, documented in `crazyflie/config/study/README.md`); ours stays frozen (`docs/77`) as our own implementation and a supplementary column from existing data.
- **Left from the initial plan:** the shakedown of the study scenarios per controller (card `docs/study_shakedown_card.md`), then protocol v1.0 and data collection. Nothing in this session is needed for the thesis text (no results enter the draft in this round); it is the start of the data collection.

## 1. Setup (once, ~15 min)
Pull both repos on the lab PC, CS2 stopped.
```bash
python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7BB02 6 9 6   # cf5: expect OutOfTree (6) and OutOfTree4/Omar (9)
python3 flying_drone_stack/tools/check_active_controller.py radio://0/80/2M/E7E7E7E7E9 6        # cf_second stays geometric
python3 flying_drone_stack/tools/restore_study_sd_config.py /media/georg/<cf5 card>              # cf5 card holds the TK debug config
python3 flying_drone_stack/tools/restore_study_sd_config.py /media/georg/<cf_second card>        # "already in place"
```
If the console line for a slot is missing the drone carries the other image: flash `flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk.bin` (manifest hash) and check again. Preflight per strategy: `lab_preflight.py --study S1`.

## 2. Gates before the first study flight (desk, only on your go)
- NS2 v1 weights confirmed (frozen in the protocol; supervisor confirm still open) — the shakedown needs the weights fixed.
- `make_tags.sh --execute` for `study-S0`, `study-S1`, `study-S2`, then `git push origin --tags` (crazyswarm2).

## 3. Flights — reduced shakedown (21 flights, ≈ 1.8 h at 12 flights/hour; fresh batteries)
Rows of `docs/study_shakedown_card.md`, controller-grouped, each with `--study --strategy <S> --session shakedown`:
| Block | Card rows | Scenarios |
|---|---|---|
| S0 geometric | 1–7 | A8 (0.4), A1, A2, A3, A4, A5, A7 |
| S2 NS2 v1 | 13–19 | same |
| S1 Omar C + Iz 1.5 | 21–27 | same; A1–A7 never flown with S1 and deck RPM on moving 2-drone scenarios, so this block is the point of the session |
**Dropped from the card:** extremes (rows 8–12, 20, 28), ours (29–30). **Optional:** row 31 (S0 anchor A8) as drift check at the end.
Abort rule (protocol §4): |roll| or |pitch| > 25° for 0.5 s, or z < 0.25 m; stop a block after 2 crashes; after a crash toggle `stabilizer.estimator` 1→2. Select the strategy yaml per block (`git checkout study-S<k> -- crazyflie/config/crazyflies.yaml`, relaunch CS2). One line per flight with `flight_decision.py`.

## 4. After the flights
```bash
python3 flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf_second card> cf_second --file thesisNN --tag <name>   # cf_second first
python3 flying_drone_stack/tools/copy_usd_log.py /media/georg/<cf5 card> cf5 --file thesisNN --tag <name>
python3 flying_drone_stack/tools/check_usd_log_quality.py experiments/logs/usd_raw/*<today>*
```
Then `study_metrics.py`, `aggregate.py --study-table`, `study_report.py`; archive both SD cards to `~/Desktop/sd_backups/<date>_lab/`. After a clean shakedown: freeze protocol v1.0, then tier-1 data collection (≈ 180 clean flights, `docs/study_shakedown_card.md` footer).

## 5. Can the session be skipped?
Not for the study data: A1–A7 with S1 are unflown, and the data collection (≈ 15 h of flights before the 26 Feb 2027 deadline buffer) cannot start without them. It can be skipped for everything else: the Pure INDI choice is made, the thesis draft needs no new data, NS2 and the INDI numbers for the meeting are final.
