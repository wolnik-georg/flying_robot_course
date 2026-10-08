# 72 — Simulation showcase (2026-10-09, validated rebuild)

Figures, animation and numbers: `experiments/analysis/out/sim_showcase_2026-10-09/` · code: `experiments/analysis/sim_showcase.py` (simulate), `sim_showcase_finalize.py` (all numbers, figures, animation), `sim_showcase_ours_indi*.py` (ours-INDI attempts). Everything below is evaluated with `a8_crossings.zero_crossings` and the same window/dip definitions as `docs/71` (steady window 6–22 s of the scenario with ±1.5 s around crossings excluded; dip = minimum of e_z within ±1 s of a crossing).

## What the SIL is
Two-drone A8 closed loop on the host (`ns2_closed_loop_sil_sim.py`): bottom drone geometric (+ optional 100 Hz network), top drone geometric, vertical downwash force from the trained bank network (`full_bank_c1_complete.npz`, 19,297 weights) made mirror-symmetric, plus a roll/pitch torque from the lateral force gradient with one scalar `c = 0.0032 m²` **fitted on the network-off cohort only**; measurement noise on position and gyro, HLC trajectory from slot-ground start, per-drone 100 Hz peer hold, `motor_tau 0.044`. The network-on cohorts are tests, not calibration (docs/62). The Omar runs use `omar_z_integral_sil.run_formation_sil(..., ns2_plant=True, cmd_gain=1.14)` (deterministic, one run each).

## Step 0 — crossing-detector audit of the SIL (done independently of the first report)
Old SIL detector (`ns2_closed_loop_sil_metrics.find_crossing_times`) vs `zero_crossings` on the bottom-drone y trace of every canonical episode: maximum difference 0.024 s (off), 0.016 s (+1), 0.019 s (−1); crossing times 8.97/14.97/20.97/26.97 s (4 s takeoff + 5/11/17/23 s). **SIL dips of docs/62 are unchanged** (the SIL trajectory has no takeoff artefact, unlike the radio logs). Hardware numbers were the ones that needed correcting (docs/68, 71).

## Step 1 — NS2 SIL vs hardware (A8, bottom drone) — PASS
Hardware = corrected zero-crossing values of docs/71 (uSD where available, radio otherwise).
| Cohort | SIL seeds (crossings) | SIL dip (cm) | HW flights (crossings) | HW dip (cm) | Δ SIL−HW | steady SIL / HW (cm) |
|---|---|---|---|---|---|---|
| network off | 5 (20) | -5.85 ± 0.07 | 4 (16) | -5.92 ± 0.63 | +0.06 | +0.54 / +0.78 |
| res_sign +1 | 4 (16) | -9.96 ± 1.37 | 3 (12) | -11.00 ± 0.87 | +1.04 | +0.91 / +1.50 |
| res_sign −1 | 5 (20) | -2.30 ± 1.19 | 4 (15) | -3.37 ± 1.19 | +1.07 | +0.12 / +0.23 |
- Criteria: |Δ| ≤ 1.5 cm per cohort — met (0.06 / 1.04 / 1.07); ordering `+1` (deepest) < off < `−1` (shallowest) — met in SIL and hardware. The SIL is 1.0–1.1 cm too shallow for both network-on cohorts.
- SIL seeds: 0–4; seed 2 of the `+1` cohort failed with the known host-binding exception (`kalmanCoreUpdateWithPose`, ≈ 1 in 10 runs) and is excluded; network off has 10 cached seeds (seeds 0–4 shown, identical result −5.85 ± 0.09 for all 10).
- Figures: `figs/sim_vs_hw_dips.png` (cohort bars), `figs/sim_vs_hw_traces.png` (e_z traces, SIL seeds and mean over the hardware flights, aligned on the first crossing).

## Step 2 — controller variants in the same SIL
| Variant | SIL steady | SIL dip rel. to steady | HW steady (docs/71) | HW dip rel. to steady |
|---|---|---|---|---|
| Geometric (= NS2 off) | +0.5 | -6.4 | +0.8 | −6.7 |
| Omar Rust exact | +17.5 | -0.0 | +21.9 | −22.3 |
| Omar Rust + Iz 1.5 | +0.7 | +0.2 | +1.5 | −9.1 |
| Omar C exact | +17.5 | -0.0 | +23.2 | −19.7 |
| Omar C + Iz 1.5 | +0.7 | +0.2 | +2.2 | −12.1 |
| Ours INDI | — (not feasible) | — | +4.2 | −11.6 |
- Figures: `figs/sim_variants_vs_hw.png`, `figs/sim_variants_table.png`.
- **Omar level:** with the thrust mismatch `cmd_gain = 1.14` the SIL gives +17.5 cm without the integral (hardware +22…+23) and +0.7 cm with `kpos_iz` 1.5 (hardware +1.5…+2.2): the integral closes the level in the SIL as on the hardware; the exact level is 4–6 cm low (a larger mismatch factor would match it; **not retuned**, calibration frozen).
- **Omar crossing dips are NOT reproduced:** in the SIL the Omar controllers show no dip at all (−0.0 / +0.2 cm relative to their level) while the hardware shows −9 to −22 cm. Rust and C give identical SIL results (as in the code replay). The SIL cannot be used to predict Omar's dips; mechanism open (the SIL INDI compensates the downwash perfectly through its RPM feedback, the hardware does not).
- The Omar SIL runs are single deterministic runs (no noise, no seeds).
- Omar SIL numbers differ slightly from the first report (+0.3 cm steady with Iz 1.5): the first report used the legacy window/detector of `omar_z_integral_sil`; the numbers above use the docs/71 definitions.

### Ours INDI in this harness — attempted, NOT feasible (4 attempts, evidence in `results/ours_indi_seed0.json` and the child script)
| Attempt | Result |
|---|---|
| Bottom drone `ctrl_mode 3` (ki_z 0, pos 64/48/5/7, fc_bw 206), per-step gain re-application, top geometric | bottom stays on the floor, tilt 67° |
| same with geometric takeoff/landing (mode 3 only inside the scenario, like the flights) | tilt 114° |
| control: both drones geometric with the same per-step re-application | crashes (tilt 165°) → the per-step mechanism itself breaks the harness (as docs/62 already warned) |
| both drones ours INDI, configured once | tilt 74°, bottom on the floor |
Host gains are process-global (`oot_select_drone` only swaps the controller state), so per-drone controller modes need the per-step re-application that destabilises the harness. Ours INDI therefore cannot be compared in this SIL; its hardware behaviour (+4.2 cm, lateral RMS 0.9 cm) stands on the flights alone. Fixing this needs a harness change (per-drone parameter blocks in the library), not tuning.

## Animation
`anim/a8_ns2_off_vs_minus1.mp4` (26 s, real time, 25 fps, 1200×720) and `.gif` (10 fps, ≈ 1 MB): A8 swap, network off (left) and `res_sign −1` (right), same seed: top view, side view (y–z, dashed commanded heights), live bottom-drone z error with minimum so far, e_z strip with the four crossings marked.

## Mismatch table
| Item | SIL | Hardware | Verdict |
|---|---|---|---|
| NS2 A8 dips | −5.85 / −9.96 / −2.30 cm (off / +1 / −1) | −5.9 / −11.0 / −3.4 | matches within ≈ 1 cm, ordering identical, SIL slightly optimistic for network-on |
| Roll p99 bottom | ≈ 17° (first report, not independently re-verified; docs/62: 13°) | 21–22° (docs/71) | SIL rolls less |
| A1 with network on | unstable 38–48° (docs/62, not re-run) | stable, 34–40° oscillation also without the network (docs/68) | not validated; SIL too pessimistic |
| Omar crossing dips | none (−0.0 … +0.2 cm) | −9 … −22 cm | NOT reproduced |
| Omar level without Iz | +17.5 cm | +22…+23 cm | 4–6 cm low (mismatch factor not retuned) |
| Omar level with Iz 1.5 | +0.7 cm | +1.5…+2.2 cm | consistent |
| Ours INDI | not feasible | +4.2 cm, dips −11.6 rel. | cannot compare |
| Timing | 1 kHz host loop, no jitter model | STM32 500 Hz log | limitation |

## What can be claimed in the meeting
1. The frozen NS2 plant (fitted on network-off only) reproduces the A8 sign effect: crossing dips −5.9 / −10.0 / −2.3 cm in the SIL against −5.9 / −11.0 / −3.4 cm on the hardware, same ordering.
2. The SIL predicted the benefit of `res_sign −1` before the lab day (−2.3 cm); the hardware gave −3.4 cm.
3. Not validated: A1 with the network on. Not predictive: Omar's crossing dips, roll amplitude.
4. The SIL reproduces that the Omar z integral closes the level (+17.5 → +0.7 cm; hardware +22 → +1.5…+2.2 cm).
5. Ours INDI could not be run in this harness (documented, with evidence); its numbers come from the flights.

## Reproduce
```bash
python3 experiments/analysis/sim_showcase.py simulate          # system python3 (SIL), cached
python3 experiments/analysis/sim_showcase_ours_indi.py 1       # ours-INDI attempt (documents the failure)
~/.pyenv/versions/flying_robots/bin/python experiments/analysis/sim_showcase_finalize.py   # numbers, figures, animation (~1 min)
```
