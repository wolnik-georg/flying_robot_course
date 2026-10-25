# Lab session plan — FBL as the authors flew it (stock inner loop + ROM + trained network)

Prepared 2026-10-24/25 (desk). Background: `docs/fbl/STOCK_INNER_LOOP_2026-10-24.md`. Nothing below has run on hardware yet.

## Goal
- Refly the reworked FBL on the same cells as 2026-10-24: stock rate-PID inner loop (`inner 1`), physics model (ROM) + trained 4-neuron network (`nn_en 1`) — both are now the firmware defaults.
- Fly both gain sets: ours (preset 1) and the authors' code gains (preset 0).
- Fly the authors' C_D (0.236) once against ours (0.55), to show with flight data why we use ours.
- Questions: yaw drift and slow lateral offset gone? A1 oscillation and A4 0.125 crashes still there?

## Before the lab (desk)
- [ ] `git pull` in flying_robot_course; crazyswarm2 on the lab PC: `git fetch origin && git checkout study_FBL_stock && git pull`, rebuild (the branch now also records the controller variant in every `meta.json`).
- [ ] Copy the image to the lab PC if needed: `flying_drone_stack/firmware_app/build_artifacts/study_tree_v2_fbl_stockinner_nn4_candidate.bin` (git-ignored local file), sha256 `b13bf8ecc21ed58311621a5a74671892877963316e80e4558888588ff9463125`.
- [ ] Fresh batteries; a ruler or caliper for the propeller.

## Setup (lab, CS2 stopped)
- [ ] Measure the propeller radius of cf5 (tip to hub centre, mm) and note it here: ____ mm (the physics model assumes 45 mm).
- [ ] `sha256sum <bin>` = `b13bf8ec…`; flash both drones (`cfloader flash <bin> stm32-fw -w radio://0/80/2M/E7E7E7BB02` and `…E7E7E7E7E9`), power-cycle.
- [ ] Bench both: `python3 flying_drone_stack/tools/fbl_bench_check.py <uri>` → `ALL PASS`, incl. `ctrlFbl.inner == 1`, `nn_en == 1`, `abg_on_new == 0` at boot (first run of the updated tool).
- [ ] Init check cf5: `python3 flying_drone_stack/tools/fbl_init_survival_check.py radio://0/80/2M/E7E7E7BB02` → `PASS`.
- [ ] uSD: count `thesisNN` files on both cards (archive at ≥ 85); cf5 card has `usd_fbl_config.txt`; `check_usd_deck.py`.
- [ ] `lab_check_strategy.py` → FBL (cf_second S0); launch; `fbl_bench_check.py <cf5 uri> --readback` → `preset 1 (ours), inner loop: STOCK rate PID (inner 1)`.
- [ ] Start the uSD log before takeoff (takeoff, FBL switch-on at 0.15 m, integral).
- [ ] After each launch: `run_formation` prints `controller variant: branch … inner 1 nn_en 1 preset … cd …` — check it before every block.

## Flights (commands start with `ros2 run crazyflie_examples run_formation`, end with `--auto-center --yes`)
Switch settings on the ground between flights; the value is recorded in `meta.json` (`controller_variant`):
`ros2 param set /crazyflie_server cf5.params.ctrlFbl.<name> <value>`

| Block | Setting | Cells × flights | 10-24 for comparison (inner 0) |
|---|---|---|---|
| 1 | preset 1, cd 0.55, nn_en 1 (default) | A1 × 3 (first = shakedown), A8 × 3, A4 0.075 × 3, A4 0.125 × 3 | A1 y 4–7 cm, yaw ≤ 33°, 1 Hz wobble; A8 y 15–47 cm, 1 trip; A4 0.075 y 20–33 cm, yaw ≤ 66°; A4 0.125 2 trips of 4 |
| 2 | `preset 0` (authors' code gains) | A1 × 1 first; if it holds: A1 × 2 more, A8 × 3, A4 0.075 × 3, A4 0.125 × 3 | simulation: unstable above 27 ms motor lag (ours 44 ms) — their brushed upgrade drone has even slower motors (71 ms), so the simulation may be too strict |
| 3 | `preset 1`, `nn_en 0`, `cd 0.236` (authors' value) vs `cd 0.55` | A1 × 2 each, A8 × 2 each | network off for both: it was trained relative to C_D 0.55 |
| 4 (optional) | `nn_en 0` (physics model only), preset 1 | A1 × 2, A8 × 2 | shows what the network adds |

- Cells: A1 `--scenario A1 --dz 0.45 --hold 15.0 --height 0.5`; A8 `--scenario A8 --dz 0.4 --speed 0.35 --height 0.5 --passes 4`; A4 `--scenario A4 --dz 0.4 --speed 0.35 --offset <0.075|0.125> --motion line --rotate 90 --height 0.5`.
- After each flight: resting voltage, new uSD file on BOTH cards, setting (also in `meta.json`).
- If FBL trips, the tilt latch hands over to Omar; `ctrlFbl.inner 0` = the 10-24 controller for a comparison flight (option, not a rule).
- Order if time is short: block 1, then block 2 A1, then block 3, then the rest.

## After the flights
- [ ] CS2 stopped: `fbl_bench_check.py <cf5 uri> --post-flight` (`steps > 0`, `faults 0`).
- [ ] Copy cards (`copy_usd_log.py <mount> <drone> --file thesisNN --tag fbl_nn4`), merge into a bank.
- [ ] Analysis: `fbl_hw_overview.py` (now also 3D RMSE and z_max as in the paper; `--fbl-banks <new bank>`, groups new flights by `meta.json` `controller_variant`), `fbl_replay_check.py`, `fbl_integral_windup.py --fbl-banks <new bank>`, `review_fbl_yaw_budget.py`, `review_fbl_lateral_chain.py`.
- [ ] Compare: paper hardware 3D RMSE ~5 cm, z_max ~10 cm; ours on 10-24: 8–34 cm.

## Still open
- Preset 0 on hardware (block 2 decides).
- C_D 0.236 vs 0.55 on hardware (block 3).
- Propeller radius (measure, then I check the physics model).
- A1 1 Hz wobble and A4 0.125 crashes (block 1 shows whether they are gone).
- What the network adds on the real drone (simulation cannot tell: its downwash has no sideways part; block 4).
- Only the authors know: commander interface / firmware version (structure is the same either way), their exact thrust map, their hardware gains, their network weights and data. No email decided 2026-10-22.
