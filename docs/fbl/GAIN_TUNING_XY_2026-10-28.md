# FBL xy gain tuning in the plant (Step B, simulation only, 2026-10-28)

Goal: smaller xy tracking error than preset 1 with gains that are still safe on hardware. z gains (`kp_z`, `kv_z`, `ki_z`) and the rate loop are untouched. **Nothing flown.** Scripts: `experiments/fbl/sysid/tune_fbl_xy.py` (stages `grid`, `robust`, `ladder`, `ladder2`, `validate`), plant `plume_torque_sim.py`; diagnostics `docs/fbl_xy_plant_gap_diagnostic_2026-10-28.md`.

## 1. Plant used
Real firmware controller (C in the loop) in a rigid body with: stock rate PID (`inner 1`), motor lag 30 ms + 4 ms command delay, linear motor law (the quadratic law found in the rpm logs is **not** in this plant; it is in `experiments/tuning/plant_core.py` from 2026-10-29 on, and the candidate is to be re-checked there), NS2 z-force downwash, **fitted plume torque** (240 flights), gyro vibration noise, EKF-like state noise. Known gap: the hardware A8/A4 tracking lags the reference more than the plant (feedforward acts at about 40 %, cause unknown). With the feedforward scaled to 0.4 the plant reproduces the hardware numbers of preset 1: A4 8.5 / 9.8 cm (hardware 10.2 / 8.2), A8 5.1 cm (5.6); A1 stays low (0.7 vs 3.0).

## 2. Tuning
- Parameters: `kp_xy`, `kv_xy`, `ki_xy` (the xy position loop; `kr` is unused with `inner 1`).
- **Tuning cells (none is a study cell):** A1 dz 0.25 and 0.75; A4 dz 0.25 / offset 0.05 / 0.2 m/s, dz 0.5 / 0.1 / 0.3 m/s, dz 0.75 / 0.15 / 0.4 m/s. Study cells (A1 dz 0.45 / 0.5, A4 dz 0.4 offsets 0.075 / 0.125 at 0.35 m/s, A8 dz 0.4) were only used for validation.
- Objective: mean xy RMS [cm] over the cells, +100 per unstable run (tilt > 45 deg, trip or no completion).
- Search: grid over 5 x 4 x 5 factors of preset 1 (100 candidates), then robust evaluation of the best against 9 plant variants: nominal, motor lag 60 ms, gyro noise 17 deg/s, state delay 40 / 60 ms, feedforward x0.4, feedforward 0, tilt-to-acceleration gain 0.5, feedforward x0.4 + delay 40 ms.

## 3. Result
Nominal plant alone prefers ever higher gains (best at the grid edge, 6x). The robust stage removes them: gains with kp >= 35 x or kv >= 15 become unstable at 60 ms state delay; the authors' gains (preset 0) are unstable in every variant. Robust ladder (mean / worst xy [cm] over the 9 variants):

| kp_xy / kv_xy / ki_xy | mean | worst | note |
|---|---|---|---|
| 14 / 9.5 / 12 (preset 1) | 8.8 | 43.5 | unstable at noise 17 |
| 21 / 12 / 30 | 2.4 | 3.6 | |
| 28 / 9.5 / 30 | 2.2 | 3.2 | candidate B |
| **28 / 9.5 / 48** | **1.95** | **2.8** | **candidate A: best, stable in all 9 variants** |
| 35 / 9.5 / 48 and higher | 3.7-14 | 21-100 | unstable at 60 ms delay |

Candidate A = kp_xy 28 (2x), kv_xy 9.5 (1x), ki_xy 48 (4x). Validation on the **study cells** in the plant (logged top-drone trajectory, measured gyro noise, 3 seeds; ff = feedforward scale):

| Cell | preset 1, ff 1 / 0.4 | candidate A, ff 1 / 0.4 | candidate B, ff 1 / 0.4 | hardware, preset 1 |
|---|---|---|---|---|
| A4 off 0.125 | 7.2 / 8.5 | 2.7 / 3.1 | 2.6 / 3.2 | 10.2 |
| A4 off 0.075 | 7.3 / 9.8 | 2.8 / 3.2 | 3.6 / 4.3 | 8.2 |
| A8 | 0.8 / 5.1 | 0.6 / 1.6 | 0.5 / 2.2 | 5.6 |
| A1 dz 0.45 | 0.7 / 0.7 | 0.4 / 0.4 | 0.4 / 0.4 | 3.0 |
z RMS is also lower with A (A4 2.2 -> 1.7 cm), tilt peaks are not larger.

## 4. For the hardware check (your lab step)
- Set per flight, no reflash: `ctrlFbl.kp_xy 28`, `ctrlFbl.kv_xy 9.5`, `ctrlFbl.ki_xy 48` (all `PARAM_FLOAT`; preset 1 keeps `kp_z 11.25`, `kv_z 5.4`, `ki_z 12`). Do not set `preset` afterwards (it overwrites the gains).
- Cells to fly: A4 off 0.125 and 0.075, A8, A1 dz 0.5; 3 flights each; compare with the preset 1 flights of 2026-10-26 (n = 1 per cell, so fly preset 1 again 3x if the comparison is to hold).
- Pass: no trip/fault, tilt peaks and yaw no larger than preset 1, xy RMS smaller, z RMS not worse. Fallback: candidate B (28 / 9.5 / 30), then preset 1.
- Safety note: the plant says the margin to the 60 ms state delay is the limiting factor; if the first flight oscillates (tilt growing at 1-2 Hz), land and return to preset 1.

## 5. Re-check on the plant with the quadratic motor law (2026-10-29)
The tuning above ran on a plant with a linear motor law (see the note in section 1). Re-check on `experiments/tuning/plant_core.py` (quadratic thrust law, scripted top drone, 3-D RMS in cm, mean over cells; `experiments/tuning/fbl_recheck.py`):
| Gain set | tuning cells: nominal / ff x0.4 / delay 40 ms | study cells: nominal / ff x0.4 / delay 40 ms |
|---|---|---|
| preset 1 | 2.9 / 4.4 / 3.0 | 3.5 / 5.3 / 3.6 |
| **Step B candidate A (28 / 9.5 / 48)** | 1.9 / 2.1 / 1.9 | 1.9 / 2.3 / 2.2 |
| first CMA-ES result (plant without quadratic law, z gains also changed, **not** hand-checked) | 0.6 / 0.9 / 1.4 | 0.7 / 1.1 / 1.5 |
The ranking and the size of the gain are unchanged: candidate A stays the proposal for the hardware check. The CMA-ES set (kp_xy 42, kv_xy 13, ki_xy 93, kp_z 42, kv_z 16, ki_z 45) is provisional: it moves the z gains to about 4x preset 1 and was found on the old plant with 4 variants; it is re-run on the plant of record before anything is flown.
