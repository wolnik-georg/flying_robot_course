# FBL lab card (DiffCoTune Split-Alternate, frozen lab variant "ours", rules R3 / RS, 6 rounds of 5 tuning flights): Stage F2, 18 Nov 2026 (desk, not flown)

Same method as the S0-S3 cards (`S0_LAB_CARD.md`, `INTERLEAVED_LAB_CARD.md`): items that are identical (flight commands of the 5 tuning scenarios and the test cell, merging the uSD files, `flights.json`, rules R3 / RS, abort rules, battery, never radio CSVs) are not repeated here; only what differs for FBL is listed. Variants to tune: **base, rom, both** (net is never tuned). One block of 6 rounds per variant; state folder `lab/FBL<variant>/`, one gain set per variant, θ_0 = preset 1 (kp_xy 14.0, kv_xy 9.5, ki_xy 12.0, kp_z 11.25, kv_z 5.4, ki_z 12.0; `simulator/frsim/controllers.py:36`). kr_xy 59.5 / kr_z 35.0 are not tuned and not written (unused with `inner 1`).

## Before a block (FBL)

| # | Do | Source |
|---|---|---|
| 1 | Image on both drones: `study_tree_v2_fbl_stockinner_nn4_full_candidate.bin`, sha256 `e3593a6f39a42a61789c5631784ae691d2ecb964f3a6d88bf14d02af6b673a09` (flash and bench steps S1, S2 of `docs/fbl/LAB_PLAN.md`: `fbl_bench_check.py <uri>`, `fbl_init_survival_check.py <cf5 uri>`; hash of the file is checked by `preflight.py` 8d, **which image is on the drones is not checkable offline**) | `docs/fbl/LAB_PLAN.md:10-11` |
| 2 | After a flash only: `python3 flying_drone_stack/tools/check_active_controller.py <cf5 uri> 9` (console line; a parameter read-back is a cache) | memory `feedback_verify_controller_slot_after_flash_2026-10-13`; FBL runs in controller slot 9 (`fbl_prime_controller.py`) |
| 3 | crazyswarm2 branch `study_FBL_stock` (read only here; the lab PC checks it out): `ctrlFbl en 1, inner 1, cd 0.55`, no gains or preset written | `docs/fbl/LAB_PLAN.md` step S3; `crazyswarm2` commit aa6034c |
| 4 | **Priming after EVERY power cycle (battery swap) of cf5, CS2 stopped:** `python3 flying_drone_stack/tools/fbl_prime_controller.py <cf5 uri>`. The controller init (run when `stabilizer.controller` is switched to 9) wipes the `ctrlFbl.*` yaml values of a fresh boot; with the drone already on controller 9 the CS2 launch does not wipe them. Note: that script's own pass criterion lists `z_on 0.15`, `en 0`, `cd 0.43` of the older initfix image; for the `nn4_full` image follow `fbl_bench_check.py` / `fbl_init_survival_check.py` (what the exit code of the prime script means on this image: not checked) | `flying_drone_stack/tools/fbl_prime_controller.py` header; `docs/fbl/LAB_PLAN.md` |
| 5 | `python3 experiments/diffcotune/lab/preflight.py --controller FBL --variant <base\|rom\|both> --yaml <lab crazyflies.yaml>` must print PREFLIGHT PASSED (checks 8a-8e are FBL-specific) | `lab/preflight.py` |
| 6 | Round 0 initialised (writes the `ctrlFbl` fragment of the variant, gains = preset 1): `python3 experiments/diffcotune/lab/round.py --init --controller FBL --variant <v> --state-dir experiments/diffcotune/lab/FBL<v>` | `lab/round.py` |
| 7 | Count the uSD files on BOTH cards, `check_usd_deck.py`, fresh batteries, pad check: as the S0 card items 1, 2, 4, 6. **uSD config for FBL: `flying_drone_stack/tools/usd_fbl_config.txt`** (500 Hz; contains all 16 channels `usd_ingest` reads, `preflight.py` 8e; `restore_study_sd_config.py --fbl`, `docs/fbl/LAB_PLAN.md:45`) | |

## Per round l = 0 … 5 (the variant is chosen per block: the fragment of the round sets `dw_en` / `nn_en`)

| variant | `ctrlFbl.dw_en` | `ctrlFbl.nn_en` | meaning |
|---|---|---|---|
| base | 0 | 0 | no downwash residual |
| rom | 1 | 0 | ROM only |
| both | 1 | 1 | ROM + 4-neuron net (the firmware default of the image) |

| # | Do | Source |
|---|---|---|
| 1 | `python3 experiments/diffcotune/fbl/apply_gains_fbl.py merge --fragment lab/FBL<v>/round_<l>/gains.yaml --yaml <crazyflies.yaml of study_FBL_stock>` → put the `.new` file in place. The fragment carries `en 1, inner 1, cd 0.55, dw_en, nn_en` and the 6 gains (11 values); the merge changes only these `ctrlFbl` values of cf5 and fails otherwise. `python3 experiments/diffcotune/lab/session.py switch --to FBL --variant <v> --fragment <fragment>` prints the whole sequence | `fbl/apply_gains_fbl.py`, `lab/session.py` |
| 2 | Priming (before 1 if cf5 was power cycled), launch CS2, stop CS2, read-back on a fresh connection: `python3 experiments/diffcotune/fbl/apply_gains_fbl.py readback --uri <cf5 uri> --fragment <fragment>` (the 11 values and `stabilizer.controller` 9); relaunch CS2 | `fbl/apply_gains_fbl.py` |
| 3 | Flights 1-5 (A1, A8, A4 cell 1 with cf_second; `cf_second enabled: false` for CIRCLE, FIG8), merge, `flights.json`, rule R3 item 9a, rule RS: exactly as in the S0 card, items 3-9a | `S0_LAB_CARD.md` |
| 4 | `python3 experiments/diffcotune/lab/round.py --controller FBL --variant <v> --round <l> --logs <folder>` (or `session.py desk --controller FBL --variant <v> ...`, state folder `lab/FBL<v>`). Crash limit for the flags: tilt ≥ 45° (FBL), as frsim | `lab/round.py`, `usd_ingest.py` (`CRASH_KEY`) |
| 5 | Next gains in `lab/FBL<v>/round_<l+1>/gains.yaml`, report in `round_<l>/report.md`; the last round writes `gains_theta_best.yaml`; test flights (A4 cell 2) with θ_best: as the S0 card | `lab/round.py` |

Limits (stated, not decided): `session.py status` and the interleaved timeline do not know the FBL variants (`status --pair` takes S0-S3 only); the order of FBL blocks and of the S0-S3 blocks, flights per pack and the desk minutes of FBL are not stated; with the calibration plant frsim crashes on A4 cell 1 in `rom` and `both` at θ_0 (F1 report section 4), so R3 on A4 is possible in round 0.
