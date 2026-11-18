# FBL flash card (image `fbl_v2`, authors-aligned): Stage F2c, 18 Nov 2026 (desk, nothing flashed, nothing flown)

**Not for use before the S0–S3 tuning sessions are done**, unless the user decides otherwise (decision E3 of 18 Nov: flashing is the user's call). This card is the sequence for the day it is flashed. It replaces rows 1, 4 and the image row of `FBL_LAB_CARD.md` (that card still names the 25 Oct image `e3593a6f…`, which does not match the model). Which image is on the drones now: not stated.

## Which image
| | file in `flying_drone_stack/firmware_app/build_artifacts/` | sha256 | size | written | source |
|---|---|---|---|---|---|
| `fbl_v2` | `fbl_v2_authors_aligned_candidate.bin` | `2fbe6034bf1f66670054d8244befda9ba60359b06bb44724a390373271232151` (`docs/fbl/EQUIVALENCE_CHECK_2026-11-02.md:307`; first 16 hex re-checked with `sha256sum`) | 541 524 B | 2026-11-04 18:51:47 | `~/Desktop/fbl_v2_firmware`; its FBL files are byte-identical to the simulator build used by the model (F2b, F2c) |
| **`fbl_v2b` (for the bench step)** | `fbl_v2b_candidate.bin` | `87bd963e8299ff6a4bda7dd190355651df7da4e905d90c505bd4cd663b299be4` (`EQUIVALENCE_CHECK:325`) | 541 700 B | 2026-11-04 20:57:46 | `~/Desktop/fbl_v2b_firmware` = `fbl_v2` + timing counter `t_us_max`; host outputs byte-identical to `fbl_v2` (`EQUIVALENCE_CHECK:324`) |
| S0–S3 image (switch back) | `study_tree_v2.bin` | `3ae7304a31d0f01ea1287c6e4012cfab97ea5d1126bfde462664751572215795` (`study_tree_v2.sha256`) | 532 952 B | 2026-10-18 18:17:26 | named as the S0–S3 image in `docs/tree_comparison_2026-10-19.md:13`; whether it is what the drones fly now: not stated |

`fbl_v2` and `fbl_v2b` differ only by the timing counter (74-line diff `experiments/fbl/authors_align/fbl_v2b_vs_v2.diff`, `EQUIVALENCE_CHECK:320`). The bench check of the checklist reads `ctrlFbl.t_us_max`, which exists in `fbl_v2b` (`fbl_v2b_firmware/.../controller_omar_indi.c:853`); `fbl_v2_firmware/fw/src/modules/src/controller/controller_omar_indi.c` has no `t_us_max` (0 occurrences, `grep`), **so the bench step of the checklist needs `fbl_v2b`**; with `fbl_v2` there is no timing read-out. Both hashes depend on the build path in the assert strings (a rebuild elsewhere gives another hash).

## Before flashing (desk, same day)
| # | Do | Source |
|---|---|---|
| 1 | Archive and count the uSD files on both cards (limit: archive at ≥ 85 files), `check_usd_deck.py` | `docs/fbl/LAB_PLAN.md`, memory rule on the 100-file counter |
| 2 | `sha256sum` of the chosen `.bin` equals the table above | this card |
| 3 | uSD config on the cf5 card: **`flying_drone_stack/tools/usd_fbl_v2_config.txt`** (96 variables; as `usd_fbl_config.txt` but `ctrlFbl.tripped` → `ctrlFbl.t_us_max`, `ctrlFbl.faults` → `ctrlFbl.res_aaz`). Checked F2c: all 16 channels `usd_ingest.py` reads are in it (same check as `preflight.py` 8e) | `FIRST_FLIGHT_CHECKLIST.md:38-45`; `restore_study_sd_config.py --fbl` writes the old `usd_fbl_config.txt`: which file it writes now: not checked |

## Flash (both drones)
1. Stop CS2. `BIN=~/Desktop/flying_robot_course/flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_candidate.bin` (or the `fbl_v2` file); `sha256sum $BIN`.
2. `cfloader flash $BIN stm32-fw -w radio://0/80/2M/E7E7E7BB02` (cf5), the same with cf_second's uri; power-cycle both (command and uri as in `docs/lab_sessions/fbl_session_checklist_2026-10-26.md:24`).
3. `python3 flying_drone_stack/tools/check_active_controller.py <uri> 9` on both (CS2 stopped, motors not armed): the console line must say controller 9 (param read-back is a cache).

## Bench (motors off, props off)
1. `python3 flying_drone_stack/tools/fbl_bench_check_v2.py <cf5 uri> --props-off` (60 s, feeds the neighbour, prints `dw_en`, `t_us` median / max, PASS / FAIL). **Pass: `t_us_max` < 1 000 µs** (static estimate 190 µs for one neighbour, `EQUIVALENCE_CHECK` §F10). Do not use `fbl_bench_check.py` (reads the removed `ctrlFbl.z_on`, fails on this image).
2. Image defaults to expect: `inner 1`, `nn_en 1`, `abg_on_new 0`, `dw_en 1` (`controller_omar_indi.c:91` of the simulator build). `fbl_init_survival_check.py <cf5 uri>`: its criteria were written for the older images; whether it passes on this image: not checked.

## Priming and yaml
- **Priming after every power cycle of cf5 (CS2 stopped):** `python3 flying_drone_stack/tools/fbl_prime_controller.py <cf5 uri>`. **Its own pass criterion reads `ctrlFbl.z_on`, which this image removed** (`fbl_prime_controller.py:9, :22, :25`): what the script does on this image (error or FAIL) is not checked, so its exit code cannot be used. The priming action (switching `stabilizer.controller` to 9, which runs the controller init) is what is needed; verify the values afterwards with `fbl_bench_check_v2.py` or `apply_gains_fbl.py readback` (reads en, inner, cd, dw_en, nn_en and the 6 gains; none of them was removed). Whether the controller init of this image wipes the `ctrlFbl` yaml values as before: not checked.
- **Yaml:** crazyswarm2 branch **`study_FBL_stock`** (`ctrlFbl en 1, inner 1, cd 0.55`, no gains or preset written; commit aa6034c), plus the per-variant fragment of `round.py` (`dw_en`, `nn_en`, the 6 gains) via `fbl/apply_gains_fbl.py merge`, exactly as `FBL_LAB_CARD.md` per-round row 1–2. `docs/fbl/crazyswarm2_study_FBL_authors.patch` (bottom drone `ctrlFbl.dw_en: 1`) is the checklist's yaml step; with the fragment the value is written per variant.

## Shakedown (hover, then A1 with a hand on the abort)
1. Fresh batteries, pad check, uSD log started before takeoff.
2. **Hover only** after the bench check passed (`FIRST_FLIGHT_CHECKLIST.md:38-45` step 6).
3. **A1 with a hand on the abort**, base variant (`dw_en 0`, `nn_en 0`), preset 1. This image has **no hand-over, no tilt latch, no rate limit and no NaN trip** (`EQUIVALENCE_CHECK:234, :238`): FBL flies takeoff and landing, and a NaN thrust is not caught. Watch the first takeoff and the landing.
4. Pass criteria are not stated here (the planning agent / user judge). For reference only, the last real A1 flights (26 Oct, older image, stock inner loop, preset 1): tilt RMS 2.1–4.7°, 3-D position RMS 1.9–3.8 cm (`fbl/report_F2b.md` section 2).
5. Then the 3 variant blocks as in `FBL_LAB_CARD.md`.

## Switch back to S0–S3
1. Stop CS2. Flash `study_tree_v2.bin` on both drones (same `cfloader` command, `BIN=…/study_tree_v2.bin`, sha256 `3ae7304a…`), power-cycle.
2. `check_active_controller.py <uri> 9` on both (S0–S3 also run in controller 9).
3. crazyswarm2 back to the S0–S3 branch of the session (`S0_LAB_CARD.md`, `INTERLEAVED_LAB_CARD.md`); `preflight.py --controller S0 …`.
4. Whether the S0–S3 behaviour of `fbl_v2` equals `study_tree_v2.bin` (tree code, NS2 weights): see `fbl/report_F2c.md` section 5: the source of the tree code is identical outside the FBL blocks, the binary was not compared; therefore **switch back to `study_tree_v2.bin` for S0–S3 sessions**.
