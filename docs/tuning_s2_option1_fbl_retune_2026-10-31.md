# Auto-tuning: S2 own tuning, option 1 (no dz 0.25 cells) and FBL retune (2026-10-31)

Plant of record: lag-30 recalibration (`out/plant_calibration_joint_lag30_best_s0.json`), tree crash threshold 70 deg. Plant results only; nothing is validated on hardware. Companion to `tuning_s0_first_pass_2026-10-29.md` and `tuning_plant_p4_2026-10-29.md`.

Numbers: 3-D RMS in cm on the four hardware-validated study cells (nominal plant), best seed. "Full" = mean over the 9 full variants, crash = number of crashed episodes. Search variants: nominal, gn x1.7, md40, ffs0.4, lag 45. Job names: `s2p_*` (S2 own tuning, with dz 0.25 cells), `fbln` (FBL, with dz 0.25), option 1: `s0q_*`, `s2q_*`, `fblq` (dz 0.375 in place of 0.25, `TUNING_CELLS_V2`).

## 1. S2 own tuning with dz 0.25 cells (`s2p_*`) against S0-gain reuse

| | S2 baseline | S0 gains reused | own tuning (best seed) | own, full (crash) |
|---|---|---|---|---|
| A1 | 16.5-19.0 | 3.1 / 1.2 / 11.8 / 14.9 | s2: 0.3-0.5 | 13.4 (2) |
| A4 | 7.6 / 8.4 / 10.6 / 10.2 | 5.2 / 5.4 / 7.7 / 8.0 | s1: 4.8 / 5.3 / 4.7 / 11.4 | 26.0 (6) |
| A8 | 3.6-4.8 | 1.4 / 1.4 / 2.2 / crash | s0: 1.5-2.3 | 8.9 (1) |

Own tuning is better on A1 and A8; A4 seeds 0 and 2 crash or reach 54-100 cm on one study cell. FBL `fbln` (with dz 0.25): every CMA-ES set scores 27-51 cm on the study cells against 8.6 for preset 1, i.e. the unvalidated dz 0.25 cells dominated the search.

## 2. Option 1 (no dz 0.25 cells)

### FBL (`fblq`)

| gain set | nominal | ffs 0.4 | md 40 | full |
|---|---|---|---|---|
| preset 1 | 8.6 | 10.3 | 8.6 | 33.5 |
| Step B candidate A | 51.5 | 51.5 | 29.2 | 50.0 |
| `fblq` s0 | 8.8 | 8.2 | 7.9 | 20.1 |
| `fblq` s1 | 6.7 | 7.3 | 8.4 | 16.4 |
| `fblq` s2 | 6.8 | 7.8 | 6.9 | 18.3 |

Dropping the dz 0.25 cells fixes the FBL retune: about 20 % better than preset 1 on the study cells, about half on the full variants. Candidate A (28 / 9.5 / 48, tuned on the lag-22 plant) still crashes on this plant: do not fly it.

### S0 (`s0q`) and S2 (`s2q`), study cells, best seed

| | scenario | baseline | best seed | full (crash) |
|---|---|---|---|---|
| S0 | A1 | 19.9-21.1 | s1: 2.5 / 2.6 / 8.7 / 11.3 | 15.1 (2) |
| S0 | A4 | 11.0-12.6 | s1: 3.3 / 3.5 / 3.9 / 4.7 | 17.5 (4) |
| S0 | A8 | 5.3-6.0 | s1: 2.8 / 2.8 / 3.2 / 3.6 | 8.3 (1) |
| S2 | A1 | 16.5-19.0 | s0: 0.1 / 0.1 / 1.0 / 2.0; s1: 1.4 / 1.4 / 1.4 / 4.9 | 11.6 (2); s1 13.3 (2) |
| S2 | A4 | 7.6-10.6 | s0: 2.7 / 3.1 / 2.8 / 3.4 | 13.5 (3) |
| S2 | A8 | 3.6-4.8 | s1: 1.6 / 1.9 / 2.2 / 2.7 | 7.9 (1) |

Caveats: the seed spread is large (e.g. S0 A1 s0 crashes one study cell, S2 A4 s1/s2 crash one); tuned sets still crash 1-4 full-variant episodes at lag 60 / state delay 60 (baselines too); S2 A1 values near 0.1 cm are probably the plant over-rewarding A1 (hardware oscillates at +-27 deg); the tuning cells without dz 0.25 are validated only in so far as hardware flights exist for dz 0.375-0.75.

## 3. Proposed hardware confirmation set (not flown)

Selection: no study-cell crash, lowest full-variant crash count. The script's own ranking may differ for some seeds; this table is my reading of `out/*_rerank.log`.

| | A1 | A4 | A8 |
|---|---|---|---|
| S0 | s0q s1 | s0q s1 | s0q s1 |
| S2 | s2q s1 | s2q s0 | s2q s1 |
| FBL | `fblq` s1 (all scenarios) | | |

S2: own tuning beats S0-reuse on A1 and A4 and is similar on A8; the supervisor discussion (own tuning vs reuse) is still open. Gains (actual values, best row of each run, from `out/*_rerank.json`):

```
s0q A1 cmaes s1: {"Pxy": 4.11435, "Pz": 16.02862, "Dxy": 8.83958, "Dz": 9.56482, "Iz": 4.54121, "KRxy": 0.01216, "KRz": 0.00403, "Kwxy": 0.00127, "Kwz": 0.00124, "KIxy": 0.00478}
s0q A4 cmaes s1: {"Pxy": 9.17702, "Pz": 27.9777, "Dxy": 2.18683, "Dz": 4.51761, "Iz": 2.80595, "KRxy": 0.00789, "KRz": 0.00334, "Kwxy": 0.00083, "Kwz": 0.00057, "KIxy": 0.11871}
s0q A8 cmaes s1: {"Pxy": 5.93912, "Pz": 8.32338, "Dxy": 4.15631, "Dz": 9.51084, "Iz": 4.50072, "KRxy": 0.00758, "KRz": 0.00392, "Kwxy": 0.00133, "Kwz": 0.00057, "KIxy": 0.01223}
s2q A1 cmaes s1: {"Pxy": 1.75223, "Pz": 18.92107, "Dxy": 3.88551, "Dz": 7.99798, "Iz": 1.00806, "KRxy": 0.01254, "KRz": 0.00203, "Kwxy": 0.00095, "Kwz": 0.00129, "KIxy": 0.08142}
s2q A4 cmaes s0: {"Pxy": 3.79758, "Pz": 16.45855, "Dxy": 5.29235, "Dz": 7.18194, "Iz": 4.77563, "KRxy": 0.00887, "KRz": 0.01181, "Kwxy": 0.00091, "Kwz": 0.00193, "KIxy": 0.11931}
s2q A8 cmaes s1: {"Pxy": 1.75239, "Pz": 13.79316, "Dxy": 6.28768, "Dz": 5.41338, "Iz": 5.83684, "KRxy": 0.00877, "KRz": 0.00226, "Kwxy": 0.00147, "Kwz": 0.00101, "KIxy": 0.00671}
fblq cmaes s1: {"kp_xy": 17.92, "kv_xy": 5.55, "ki_xy": 6.864, "kp_z": 37.754, "kv_z": 7.363, "ki_z": 6.636}
```

## 4. Open
- S1 / S3 (INDI) tuning: needs the INDI plant fidelity (actuator fit R2 0.8).
- Plant gate magnitude 9/16, A8/A4 feedforward gap unexplained, unclamped KI.
- Step B hardware check: use `fblq` s1 instead of candidate A (lab decision).
- Nothing committed; files in `experiments/tuning/out/` (`s2p_*`, `s0q_*`, `s2q_*`, `fblq_*`, `*_rerank.json/.log`); `rerank_fblq.py` is `rerank_fbl.py` pointed at `fblq`.
