# Baseline before auto-tuning (snapshot 2026-10-28)

"Before" row and fallback for the study. Nothing here is tuned by the new pipeline. Read-only copy; do not edit.

## Repos at snapshot
| Repo | State |
|---|---|
| flying_robot_course | `main` @ af85f000 (plus uncommitted work, see git status) |
| crazyswarm2 | branches `study_S1`, `study_S2`, `study_S3`, `study_FBL_stock` (heads in `crazyswarm2_branch_heads.txt`), yaml copies `crazyflies_*.yaml` |
| crazyflie-firmware | branch `stable-june20` @ ccb0606d. The Omar/FBL/NS2 sources are **untracked** there: copied to `firmware_untracked_src.tgz` (hashes in `firmware_src_sha256.txt`), tracked edits in `firmware_tracked_changes.diff` |
| Images (sha256 in `image_sha256.txt`) | S0-S3: `study_tree_v2.bin` (3ae7304a...); S4: `study_tree_v2_fbl_stockinner_nn4_full_candidate.bin` (e3593a6f...) |

## C tree (S0-S3), shared (firmware defaults, `controller_omar_indi.c`)
| Item | Value |
|---|---|
| Position P / D / I | Kpos_P 7, Kpos_D 4 (x, y, z); Kpos_I 0 with `Kpos_Iz` 1.5 (yaml, z only; Ix/Iy inert while Iz is not 0); limits P/D 100, I 2 |
| Attitude | KR 0.007 / 0.007 / 0.008; Komega 0.00115 / 0.00115 / 0.002; KI 0.03 (unclamped integral of eR) |
| INDI filters | all four Butterworth-2 (acc rpm, acc imu, tau rpm, angular acc): 30 Hz, at attitude rate |
| Mass, inertia | CF_MASS (0.0427 kg), J 16.57e-6 / 16.66e-6 / 29.26e-6 |
| NS2 | C network with flashed weights (ns2_v2); `rnn.en` per strategy |

## Strategies (yaml differences vs. main = S0)
| S | `ctrlOmarIndi.indi` | `rnn.en` | Branch |
|---|---|---|---|
| S0 | 0 | 0 | main |
| S1 | 3 | 0 | study_S1 |
| S2 | 0 | 1 | study_S2 |
| S3 | 3 | 1 | study_S3 |

## S4 FBL (preset 1, branch `study_FBL_stock` @ aa6034c)
| Item | Value |
|---|---|
| Position loop | kp_xy 14, kp_z 11.25, kv_xy 9.5, kv_z 5.4, ki_xy 12, ki_z 12 (0.25x of the authors' preset 0) |
| Rate loop | kr_xy 59.5, kr_z 35 (unused with `inner 1`: stock rate PID) |
| Settings | inner 1, nn_en 1, `cd` 0.55 (firmware default 0.43), tilt_max 180, w_max 0, z_on/z_off -10/-20 (no handover), beta 0.4, gamma 0.1 |
| Preset 0 (authors' code) | kp_xy 56, kp_z 45, kv_xy 38, kv_z 21.6, ki_xy 48, ki_z 3, kr_xy 238, kr_z 140 |

## Notes
- The 2026-10-26 flight metadata has NaN controller parameters (readback failed); the settings above come from the branch and the lab confirmation.
- FBL `ki_z` 12 vs. S0-S3 `Kpos_Iz` 1.5: not equal (8x, fairness issue before freeze).
