# S3 = INDI + NS2: Version A (plain sum) and Version B (INDI on what NS2 left over) — implementation and tests, 2026-10-17

> **CLOSED 2026-10-18 — history only.** Version B (`ns2_mode`) was removed from the firmware and the repo; S3 is INDI + NS2 as two plain switches. The reasoning below that A "corrects the push twice" holds only for an ideal INDI and was overstated; see `docs/s3_indi_ns2_decision_2026-10-18.md` and `docs/validation_s3_versions_2026-10-18.md`.


**Question for the supervisor:** `docs/meetings/2026-10-22.md` (Questions). **Both versions live in the same controller (controller 9), selected by `ctrlOmarIndi.ns2_mode`; default 0 = Version A = everything flown so far.**

## Code (Omar C, position loop, inside the INDI block; active only with `indi` bit 0 and NS2 on)
```c
a_indi = a_rpm_filtered - a_imu_filtered;                       // INDI measurement (30 Hz), as before
update_lowpass(filter_a_nn, a_nn_or_zero);                       // NS2 output, same 30 Hz / 500 Hz filter (always updated)
a_nn_filtered = get_lowpass(filter_a_nn);                       // logged as ctrlOmarIndi.a_nn_fz
if (ns2_mode == 1 && ns2_active()) a_indi += a_nn_filtered;      // Version B; ns2_mode = 0: unchanged
// elsewhere, as before: a_d -= a_nn (NS2);  thrust = m * dot(a_d + a_indi, z_body)
```
Files: `controller_omar_indi.{c,h}` (patches `firmware_patches/controller_omar_indi.{c,h}.patch` regenerated, README note), yaml variants `crazyswarm2/crazyflie/config/study/from_main_S3A.patch` / `from_main_S3B.patch` (apply on the current main yaml; S3B needs the new image), `ns2c_bench_check.py --readback` prints `ns2_mode` and the strategy (S3 / S3B).

## Tests
| Test | Result |
|---|---|
| Replay of 4 logged flights (A8, A1; indi 3 and 0), old build against new build with `ns2_mode = 0` (`host/test_omar_ns2_off_identical.py`) | **all byte-identical** (sha256 a300021ed3b0, 404a9d4373d3, 9466b5a18dea, 81e66789889c = the hashes of the original NS2-hook test) |
| Host bindings rebuild | `ns2_mode` exposed, builds clean |
| ARM image build (`CF_CARGO_FEATURES=--features residual_nn_flash`, v1 weights, scratch copy so nothing in the repo is overwritten) | builds clean with `-Werror`, `cf21bl.bin` 533 152 B (Flash 52 %), contains the parameter `ns2_mode`, the log variable `a_nn_fz` and the symbol `filter_a_nn` |
| Patches | apply on a clean tree and reproduce the live files |
| Closed-loop two-drone SIL, A8, indi 3, plant force 0.7 / 1.0 / 1.3 × prediction (`omar_ns2_closed_loop_sil.py --mode-matrix 0.7,1.0,1.3`, results `experiments/analysis/out/omar_ns2_closed_loop_sil/results_modes.json`) | see below |

SIL (dip / bump / offset in cm; bump = upward excursion at the crossings):
| plant scale | INDI alone | A (mode 0) | B (mode 1) |
|---|---|---|---|
| 0.7 | 0.75 / +1.21 / 0.08 | 0.50 / +2.02 / 0.19 | 0.71 / +1.23 / 0.08 |
| 1.0 | 0.74 / +1.18 / 0.08 | 0.52 / +1.52 / 0.16 | 0.76 / +1.20 / 0.08 |
| 1.3 | 0.78 / +1.24 / 0.08 | 0.51 / +1.52 / 0.21 | 0.77 / +1.22 / 0.08 |
- **B behaves like INDI alone (no double counting); A adds an upward bias** (bump +1.5 … +2.0 cm against +1.2, offset 0.16 – 0.21 cm against 0.08), the signature of the doubled correction. For `indi 0` (NS2 without INDI, S2) modes 0 and 1 are identical in every run, as designed.
- **Limits:** the simulation's INDI shows no crossing dip, so B's advantage over INDI alone (NS2's fast part, hardware dip −10.4 cm for INDI alone) cannot show up there; the real comparison is the lab: S3 and S3B on A8 and A1, 3 flights each. Single runs, perfect-prediction plant.

## Evidence from the flights of 2026-10-15 (why A double-counts)
INDI-measured push inside the gate, S1 against S3: A1 −2.89 vs −2.86 m/s², A8 −1.05 … −1.17 vs −1.22 … −1.31, with the NS2 prediction in S3 at −1.0 … −2.3: NS2 on top does not reduce what INDI measures.

## Next
Lab: relaunch with `from_main_S3A` / `from_main_S3B` on the image that contains `ns2_mode` (the planned `study_tree_v2` includes it), fly both on A8 and A1, compare dip and overshoot; the supervisor decides which one is S3 of the study (or both).
