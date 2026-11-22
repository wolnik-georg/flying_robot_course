# Omar C DShot RPM: Step 2 (image), 2026-11-22
**Build tree (separate, new):** `~/Desktop/fbl_v2b_dshot_fw` = `cp -a` of `~/Desktop/fbl_v2b_firmware` (the source and build tree of the image of record), then only this change. `~/Desktop/fbl_v2b_firmware` and `~/Desktop/crazyflie-firmware` were not touched. The tree path is exactly as long as the original (`fbl_v2b_dshot_fw` / `fbl_v2b_firmware`, 16 characters), because the image contains 46 absolute source paths: the sha only compares after replacing that name.
**Build command (from the 4 Nov build log `armbuild.log`):** `cd ~/Desktop/fbl_v2b_dshot_fw/stack/firmware_app && make all-rnn-flash CRAZYFLIE_BASE=~/Desktop/fbl_v2b_dshot_fw/fw RNN_WEIGHTS_NPZ=~/Desktop/flying_robot_course/flying_drone_stack/tools/residual/weights/ns2_v2.npz` (DRONE=bl default, no other make variable).

## 1. RPM_FILTER_HOLD_SENTINEL of the image of record 87bd963e: NOT set (= 0, legacy)
Evidence (three independent):
- `armbuild.log:1089` (4 Nov, compile line of `traj_iface.c`) has no `-DRPM_FILTER_HOLD_SENTINEL`; `Kbuild:16` adds that define only if the make variable is set, and the log shows `EXTRA_CFLAGS=""` with no such variable.
- `traj_iface.o` of the original tree (4 Nov): text 2580 B, `rpm_get_all` disassembly md5 a4573fb08c = a rebuild without the flag (same). Compiled with `-DRPM_FILTER_HOLD_SENTINEL=1` it is text 2588 B, md5 d2796cb47e (different).
- **Baseline rebuild reproduces the image of record byte for byte:** unchanged source, no flag, in the copy tree: 541 700 B, raw sha 09307d37..., and after replacing `fbl_v2b_dshot_fw` by `fbl_v2b_firmware` in the file: **sha256 87bd963e8299... = the image of record**.
The new image is built with the same setting (flag unset).

## 2. Diff to the 87bd963e source = exactly two files
`diff -rq ~/Desktop/fbl_v2b_firmware ~/Desktop/fbl_v2b_dshot_fw` (excluding build outputs): (1) `fw/src/modules/src/controller/controller_omar_indi.c` (33 changed lines, `controller_omar_indi_dshot.diff`), (2) new `fw/src/modules/interface/rpm_filter.h` (byte-identical to `flying_drone_stack/firmware_app/rpm_filter.h`, sha f1c4781d). Nothing else.

## 3. New image
`~/Desktop/fbl_v2b_dshot_fw/stack/firmware_app/build/cf21bl.bin`, copy `flying_drone_stack/firmware_app/build_artifacts/fbl_v2b_dshot_candidate.bin` (gitignored, laptop only): 541 780 B (record 541 700 B, +80 B).
- **sha256 (flash this file): 9e133f76dfc4dd30372aeab8fff1d59e093325f64e170ccb40646050c35657c0**
- path-normalized (fbl_v2b_dshot_fw -> fbl_v2b_firmware, for comparison with the record): 435bdd0f7e0e84ba04db46881c22a7fea9fafd4b3f54c01dcd58836ef0eb4b33
- No compiler warning from the controller file; link OK (flash/RAM usage as the record).

## 4. lab_run.sh prep
`prep S1|S3`: new step after the uSD deck checks: `flying_drone_stack/tools/check_dshot_rpm.py <cf5 uri>` (CS2 stopped; four log variables motor.m1_rpm..m4_rpm must exist in the log TOC = the controller's own availability test; prints one second of values, idle 0 / 65535 is normal; exit 0). `--require-spin` (Step 3 ground / hover test, motors turning): every motor 0 < rpm < 28000. There was no deck.bcRpm check inside `lab_run.sh`; the manual `check_deck_bcrpm.py` of 22 Nov (7f888376) is removed; `session.py` status text updated. **The tool is not tested on hardware** (written without a drone; compiles).
Preflight S1 and S3 with the current code: **PREFLIGHT PASSED** (both; check 3c NOTE as before: this PC's yaml is not on the strategy branch).
