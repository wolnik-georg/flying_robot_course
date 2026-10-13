# Shakedown checklist (2026-10-11)

Goal: fly every scenario once with every strategy, find what breaks, measure the real flight rate, check that the 92-variable uSD logging works, then freeze protocol v1.0. Flight list with validated commands: `docs/study_shakedown_card.md` (regenerate with `python3 experiments/analysis/study_flight_plan.py`).

## A. Before the lab (desk) — status
| item | status |
|---|---|
| NS2 weights decision (retrain / keep), frozen at the shakedown | **open — decision** |
| Pure-INDI choice (can go into the shakedown as S1 + ours) | open — decision (not blocking) |
| yaml tags `study-S0/S1/S2/ours`: variants as patches in crazyswarm2 `crazyflie/config/study/` (verified), tags with `crazyflie/config/study/make_tags.sh --execute`, `git push origin --tags` | patches ready; tags after the two decisions |
| `study_manifest.json`: cf5 and cf_second `firmware_sha256` of the image flashed | at the flash (step B1) |
| crazyswarm2 pulled on the lab PC (`--study`, `--duration`, manifest, strategies) | after push |
| Preflight tool: `python3 flying_drone_stack/tools/lab_preflight.py --yaml <crazyflies.yaml> --study S0 --sd-config /media/<card>/config.txt` | ready |

- Tal & Karaman INDI (controller 11/12, `docs/74` §6): image `cf21bl_study_usd96_rnn_iz_sentinel_ON_tk.bin`, NOT in the 31-flight card; optional, only if decided; the image flashed in B1 does not contain it.

## B. Lab setup (once)
1. **Flash both drones** (stop CS2 first, power-cycle after) with `flying_drone_stack/firmware_app/build_artifacts/cf21bl_study_usd96_rnn_iz_sentinel_ON.bin` (laptop only, sha256 `14575bacacccc8e50fbcab0d01aa789adaae4966033bf7397b3cff167db4be3c`, built 2026-10-11 with `RPM_FILTER_HOLD_SENTINEL=1`, NS2 weights `8a43b5b5…`):
   `cfloader flash <file> stm32-fw -w radio://0/80/2M/E7E7E7BB02` (cf5) and `… radio://0/80/2M/E7E7E7E7E9` (cf_second). `sha256sum` the file on the machine you flash from, write the hash of the flashed image into `study_manifest.json` for both drones, commit, push, pull on the lab PC.
2. **Bench timing:** `python3 flying_drone_stack/tools/read_rnn_timing.py radio://0/80/2M/E7E7E7BB02` (CS2 stopped). Old image: 504 µs mean / 533 µs max; expect the same ± few µs (only the uSD driver changed).
3. **SD cards (both):** `cp flying_drone_stack/tools/usd_study_config.txt /media/<card>/config.txt`; `lab_preflight.py … --sd-config` confirms it is identical. **Ground logging test:** `python3 flying_drone_stack/tools/check_usd_deck.py radio://0/80/2M/<uri>` for both drones (needs `usd.canLog` 1). If a card does not log at all, the 8192 B buffer allocation failed: set line 2 of the card's `config.txt` to `4096`.
4. Verify on the first flight that `a_res` ≠ 0, the deck RPM is present, and `--study` prints the config block of the strategy (both drones, before takeoff and airborne).

## C. Every session
- Fresh batteries (rest ≥ 4.1 V; swap if either drone < 4.0 V). `git pull`, build, launch (check the pacing marker), `lab_preflight.py --study <S>` per controller block, yaml of the block (`git checkout study-S<k> -- crazyflie/config/crazyflies.yaml`, relaunch).
- Pose bag per flight (`/poses /cf5/pose /cf_second/pose`). One line per flight with `crazyflie_examples/scripts/flight_decision.py --csv experiments/logs/flight_decisions.csv …` (clean / excluded / attempt-aborted, reason, vbat min).
- Abort / stop the block (protocol §4): |roll| or |pitch| > 25° for 0.5 s, z < 0.25 m, 2 crashes. After a crash toggle `stabilizer.estimator` 1→2. Human pilot ready.

## D. After each block (cards)
1. Cards: `cf_second` first, then `cf5` (`copy_usd_log.py /media/<mount> <drone>`, never `cp`); archive on the card; merge (`merge_usd_logs.py --meta … --roles bottom top`).
2. **Quality gate:** `python3 flying_drone_stack/tools/check_usd_log_quality.py <file.bin>` for every new log. Gaps > ~0.1 % of the samples (old config: 0.08 %) mean the card cannot keep up → drop channels in the order of `README_usd_thesis_logging.md` (`ctrlOmarIndi.error_*`, `locSrv*`, `ctrltarget.a*`, `stateEstimate.q*`) or fall back to the 4096 B buffer.
3. **Analysis:** `study_metrics.py --all --decisions experiments/logs/flight_decisions.csv`, then `aggregate.py … --study-table`; this is the first real test of the A2/A3/A4/A7 metric code.

## E. Exit criteria of the shakedown (then freeze protocol v1.0)
- Every scenario × strategy cell: flown / failed (reason) / needs a rule — documented in `docs/study_readiness_matrix.md`.
- A1: roll/pitch p99 per strategy recorded (the 10-08 flights oscillated ±35°; not expected, re-check).
- Omar C on the moving scenarios: no deck RPM dropout, z offset ≈ +2 cm.
- Speed 0.5 and dz 0.25 / 0.75: feasible or documented exclusion; A7 needs `--allow-extreme`.
- uSD: no clear increase of gaps with 92 variables; fallback decided.
- Flights per hour measured → session count for tier 1 / 2 / 3 → tier go/no-go with the supervisor.
- Protocol v1.0: settings table (A8 `--duration`, A2/A5 period, A3/A4 speed, A1 dz), manifest hashes, yaml tags, decisions file path.
