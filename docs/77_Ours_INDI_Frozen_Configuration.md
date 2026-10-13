# 77 — Our INDI, frozen with every parameter (2026-10-13)

**Purpose:** one place that preserves our INDI exactly as it was flown in the study shakedown, so it can always be rebuilt and compared, whatever is tried next (the "Omar's values" variant, `docs/76`). **Do not edit the frozen files; make a new variant instead.**

## What "ours" is
Controller 6 (`stabilizer.controller 6`, out-of-tree Rust `flying_drone_stack/firmware_app/src/lib.rs`) with `indi_gains.ctrl_mode 3` (full INDI: position INDI + attitude INDI), `res_sign +1`, `ki_z 0`, network off (`rnn.en 0`), DShot RPM (`rpm_source 1`). Selected by the study patch `crazyswarm2/crazyflie/config/study/ours.patch` on the clean geometric yaml (copy in `docs/frozen/ours_indi_2026-10-13/ours.patch`).

## Frozen files and identifiers
| What | Identifier |
|---|---|
| yaml as flown (resolved `all:` + cf5 overrides) | `docs/frozen/ours_indi_2026-10-13/crazyflies_ours_indi.yaml` = crazyswarm2 commit `02661f4`, sha256 of the yaml `a765120826d09e27…` |
| resolved cf5 parameters (table below) | `docs/frozen/ours_indi_2026-10-13/resolved_cf5_parameters.json` |
| study block | `crazyswarm2/crazyflie/config/study_strategies.json`, key `ours` (`controller 6`, `ctrl_mode 3`, `res_sign 1`, `rnn.en 0`, `ki_z 0`) |
| controller source | `flying_drone_stack/firmware_app/src/lib.rs`, sha256 `6ff23360d7fe1b44…`, last changed in thesis-repo commit `fbb8336b` (2026-10-13); `traj_iface.c` sha256 `11083226d6072f51…` |
| image flown today | `build_artifacts/cf21bl_study_usd96_rpmf_rnn_iz_sentinel_ON_tk.bin` (sha256 `cb38f3f4981d8fa6…`); the same controller is in `…_rpmf_…_ON.bin` (sha256 `a0ac7c6fb4f31863…`). Build artifacts are not in git; the source commit above rebuilds them |
| git tags | `ours-indi-frozen-2026-10-13` in both repos (thesis repo: the commit that adds this page; crazyswarm2: `02661f4`) |
| flight evidence | `docs/71` (10-09), `docs/lab_sessions/2026-10-13.md` §5 (A8, 2 usable flights: z dip −9.5 / −9.5 cm, steady offset +1.7 / +1.5 cm; flight 1 did not lift off), raw logs `~/Desktop/sd_backups/2026-10-13_lab/` |
| replay | `docs/76` (equal-parameter replay, ours native = the configuration below) |

## Parameters (cf5, resolved from the yaml)
| group | values |
|---|---|
| `indi_gains` law | `ctrl_mode 3`, `res_sign +1`, `rpm_source 1` (DShot), `j_scale 1.0`, `ff_free 0` |
| attitude gains | `kr 2400`, `kw 170`, `kr_z 2400`, `kw_z 170` (geometric-mode gains `kr_geo 0.01`, `kw_geo 0.0011`, `kr_z_geo 0.01`, `kw_z_geo 0.00138` unused in mode 3) |
| filters | `fc_bw 206`, `filt_dt_us 1000`, `filt_prewarp 1`, `filt_order 1` (rate filtered, then differenced), `filt_tau 1`, `res_fc 80`, `res_clamp 10`, `dt_usec 1`, notch off (`notch_en 0`, `notch_f0 6.9`, `notch_bw 3`) |
| constants | `mass 0.041`, `kt1..4 = 4.1623e-10, 4.0592e-10, 4.1116e-10, 4.0631e-10` |
| limits | `clamp_en 11`, `tau_xy_max 0.045`, `tau_z_max 0.0025`, `tilt_max_deg 30`, `thrust_max 0.8` |
| `pos_gains` | `kp_xy 64`, `kp_z 48`, `kv_xy 5`, `kv_z 7`, `ki_z 0` (never 16 with `ctrl_mode 3`: liftoff tumble), `ki_z_limit 1.5` |
| other | `rnn.en 0`, `stabilizer.estimator 2` |

## How to fly it again
```bash
cd ~/Desktop/crazyswarm2 && git checkout ours-indi-frozen-2026-10-13 -- crazyflie/config/crazyflies.yaml   # or: clean yaml + git apply crazyflie/config/study/ours.patch
ros2 launch crazyflie launch.py
ros2 run crazyflie_examples run_formation --scenario A8 --dz 0.5 --height 0.5 --passes 4 --auto-center --yes
```
Firmware: flash an image that contains the controller (any of the `rpmf` images); check the yaml against `docs/frozen/…/resolved_cf5_parameters.json` before the first flight.
