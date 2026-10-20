# Unified controller tree comparison S0–S3 — A8 and A1, NS2 v2 (2026-10-19)

> **UPDATE 2026-10-20:** more pilot flights (A1, A8 topped up to 4–6 flights per strategy; A4 at offsets 0.05 / 0.10 / 0.15 for all four strategies) are included in §9; the headline values of that section replace the first-session values in §1–3 (kept as the record of the first session).

**This is THE comparison of the unified Omar C controller tree** (one controller, two switches: `indi` 0/3 and `rnn.en` 0/1) and it **replaces the earlier tree validation of 2026-10-15** (NS2 v1, A8 at 0.40 m/s, one to three flights per strategy; kept as a record in `lab_sessions/2026-10-15.md`, marked superseded). Differences to that run: retrained network **NS2 v2** (all 73 flights), **new drone pair** (bottom = the drone with the RPM deck, top = new), A8 at **0.5 m/s peak**, 3–5 flights per strategy, evaluated **from uSD logs only** (complete logs of both drones; radio CSVs are not used), strategy and NS2 version verified from the logs of every flight.

| Strategy | `indi` | `rnn.en` | what runs on the bottom drone |
|---|---|---|---|
| S0 | 0 | 0 | Omar geometric controller |
| S1 | 3 | 0 | Omar full INDI |
| S2 | 0 | 1 | geometric + NS2 v2 |
| S3 | 3 | 1 | full INDI + NS2 v2 (two plain switches) |
Top drone: always S0. Shared: Kpos_P 7, Kpos_D 4, Kpos_Iz 1.5, mass 0.0427 kg, image `study_tree_v2.bin` (sha256 `3ae7304a…`), deck RPM on. Scenarios: A8 (vertical swap, dz 0.5 m, span 1 m, 4 passes, 4.375 s per pass = 0.5 m/s peak) and A1 (stacked hold, dz 0.5 m, 15 s).

## 1. Result in one picture
![A8 dip and A1 mean z error per strategy](meetings/assets/2026-10-19/overview.png)

| | S0 geometric | S1 INDI | S2 geometric + NS2 v2 | S3 INDI + NS2 v2 |
|---|---|---|---|---|
| **A8 crossing dip** (mean of the flights) | **−12.7 cm** (n=3) | **−7.1 cm** (n=3) | **−5.6 cm** (n=3) | **−4.0 cm** (n=4; −3.6 cm on the 3 fresh-battery flights) |
| **A1 mean z error in the hold** | **−17.3 cm** (n=3) | **−0.9 cm** (n=3) | **−8.7 cm** (n=5) | **+1.9 cm** (n=3) |
Reading: NS2 on the geometric controller halves the dip and the sag (A8 −12.7 → −5.6, A1 −17.3 → −8.7); INDI alone removes about 45 % of the A8 dip and holds A1 within 2 cm; INDI + NS2 is best in A8 (−4.0) and holds A1 within 2 cm (S3 starts about +13 cm high at the beginning of the hold, takeoff overshoot, gone after ~4 s). Only S1 and S3 use the full motor range (100 %), also on fresh batteries.

## 2. A8 — per flight and per strategy
Battery = loaded cell voltage logged on the bottom drone while airborne (resting voltage was not recorded). Dip = worst z error in ±0.75 s around a crossing relative to the steady offset. z mean = signed mean z error while airborne (t > 3 s).

| strat | flight | time | note | dips at the 4 crossings [cm] | mean dip | worst | z mean [cm] | z abs mean | z RMS | vbat start | mean | min [V] | motor max | tilt max |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S0 | 1 | 17-39-47 |  | -11.8, -15.1, -12.1, -11.5 | -12.6 | -15.1 | -0.7 | 3.7 | 3.95 | 3.81 | 3.76 | 3.71 | 80 % | 25° |
| S0 | 11 | 18-07-12 |  | -12.3, -13.8, -12.1, -11.4 | -12.4 | -13.8 | -1.0 | 3.6 | 3.92 | 3.85 | 3.81 | 3.74 | 79 % | 26° |
| S0 | 12 | 18-08-34 |  | -14.2, -13.9, -11.3, -12.5 | -13.0 | -14.2 | -0.7 | 3.9 | 4.06 | 3.80 | 3.76 | 3.71 | 76 % | 22° |
| S1 | 2 | 17-41-56 |  | -9.1, -7.7, -5.2, -10.4 | -8.1 | -10.4 | -0.1 | 2.5 | 2.86 | 3.64 | 3.59 | 3.49 | 100 % | 23° |
| S1 | 9 | 18-03-26 |  | -3.1, -9.7, -6.4, -8.3 | -6.9 | -9.7 | +0.4 | 2.5 | 2.70 | 3.68 | 3.64 | 3.48 | 100 % | 25° |
| S1 | 10 | 18-04-46 |  | -3.0, -3.8, -10.5, -8.1 | -6.3 | -10.5 | -0.2 | 2.1 | 2.51 | 3.67 | 3.57 | 3.44 | 100 % | 25° |
| S2 | 3 | 17-43-47 |  | -7.3, -9.3, -5.7, -4.1 | -6.6 | -9.3 | -0.6 | 2.0 | 2.21 | 3.64 | 3.60 | 3.52 | 79 % | 26° |
| S2 | 7 | 17-59-16 |  | -6.6, -4.6, -5.1, -4.9 | -5.3 | -6.6 | -0.6 | 1.7 | 1.76 | 3.68 | 3.60 | 3.47 | 81 % | 20° |
| S2 | 8 | 18-00-40 |  | -6.2, -4.5, -4.1, -4.5 | -4.8 | -6.2 | -0.5 | 1.4 | 1.41 | 3.60 | 3.53 | 3.42 | 79 % | 20° |
| S3 | 4 | 17-45-31 |  | -2.6, -7.8, -1.9, -8.6 | -5.2 | -8.6 | -0.0 | 1.8 | 2.37 | 3.56 | 3.47 | 3.35 | 100 % | 24° |
| S3 | 5 | 17-55-03 | draining battery (3 back-to-back) | -12.3, -10.8, -6.9, -9.3 | -9.8 | -12.3 | -0.2 | 3.2 | 3.99 | 3.40 | 3.31 | 3.17 | 100 % | 26° |
| S3 | 6 | 17-56-35 | draining battery (3 back-to-back) | -6.0, -14.8, -17.0, -15.5 | -13.3 | -17.0 | +0.0 | 4.5 | 6.07 | 3.35 | 3.24 | 3.13 | 100 % | 29° |
| S3 | 13 | 18-18-35 | fresh battery per flight | -3.6, -4.4, -2.1, -4.6 | -3.7 | -4.6 | +0.5 | 1.6 | 1.87 | 3.54 | 3.42 | 3.27 | 100 % | 22° |
| S3 | 15 | 18-21-50 | fresh battery per flight | -1.0, -2.5, -1.9, -6.9 | -3.1 | -6.9 | +0.6 | 1.4 | 1.89 | 3.63 | 3.50 | 3.35 | 100 % | 22° |
| S3 | 16 | 18-23-19 | fresh battery per flight | -3.7, -6.2, -1.5, -4.7 | -4.0 | -6.2 | +0.4 | 1.7 | 2.17 | 3.73 | 3.71 | 3.57 | 100 % | 27° |

## Per strategy
| strat | flights | mean dip [cm] | per-flight mean dips | worst crossing | z mean [cm] | z RMS [cm] | vbat mean [V] | vbat min [V] | motor max |
|---|---|---|---|---|---|---|---|---|---|
| S0 | 1, 11, 12 | **-12.7** | -12.6, -12.4, -13.0 | -15.1 | -0.8 | 3.98 | 3.78 | 3.71 | 76–80 % |
| S1 | 2, 9, 10 | **-7.1** | -8.1, -6.9, -6.3 | -10.5 | +0.0 | 2.69 | 3.60 | 3.44 | 100–100 % |
| S2 | 3, 7, 8 | **-5.6** | -6.6, -5.3, -4.8 | -9.3 | -0.5 | 1.79 | 3.58 | 3.42 | 79–81 % |
| S3 all 6 uSD flights | 4, 5, 6, 13, 15, 16 | **-6.5** | -5.2, -9.8, -13.3, -3.7, -3.1, -4.0 | -17.0 | +0.2 | 3.06 | 3.44 | 3.13 | 100–100 % |
| S3 without the 2 draining-battery flights (4, 13, 15, 16) | 4, 13, 15, 16 | **-4.0** | -5.2, -3.7, -3.1, -4.0 | -8.6 | +0.4 | 2.08 | 3.53 | 3.27 | 100–100 % |
| S3 fresh battery only (13, 15, 16) | 13, 15, 16 | **-3.6** | -3.7, -3.1, -4.0 | -6.9 | +0.5 | 1.98 | 3.55 | 3.27 | 100–100 % |

Time profile around the crossing: `meetings/assets/2026-10-19/A8_crossing_profile.png`; every crossing: `A8_comparison.png`.

## 3. A1 — per flight and per strategy
z error = measured z minus commanded z (bottom vehicle). Battery = loaded cell voltage logged while airborne. Flights lost to the full cards (5) are not in the bank.

| strat | flight | z mean ± std [cm] | z RMS | drift over the hold | xy RMS | max roll/pitch | loaded vbat start / mean / min [V] | motors max | top z RMS | stack gap [m] |
|---|---|---|---|---|---|---|---|---|---|---|
| S0 | 12 | -15.6 ± 3.0 | 15.9 | +5.2 | 8.5 | 41° | 3.66 / 3.65 / 3.53 | 92 % | 0.33 | 0.66 |
| S0 | 13 | -16.1 ± 3.3 | 16.4 | +6.5 | 7.8 | 40° | 3.26 / 3.28 / 3.14 | 100 % | 0.33 | 0.66 |
| S0 | 14 | -20.3 ± 3.6 | 20.6 | +0.4 | 8.4 | 42° | 3.61 / 3.53 / 3.40 | 95 % | 0.44 | 0.71 |
| S1 | 9 | -0.2 ± 2.3 | 2.3 | +1.1 | 2.8 | 22° | 3.75 / 3.71 / 3.62 | 100 % | 0.44 | 0.51 |
| S1 | 10 | -1.8 ± 1.9 | 2.7 | +1.1 | 2.5 | 19° | 3.53 / 3.41 / 3.29 | 100 % | 0.29 | 0.52 |
| S1 | 11 | -0.6 ± 2.1 | 2.2 | +1.1 | 2.6 | 19° | 3.58 / 3.54 / 3.44 | 100 % | 0.39 | 0.51 |
| S2 | 4 | -10.4 ± 4.3 | 11.2 | +4.7 | 8.5 | 42° | 3.73 / 3.76 / 3.67 | 90 % | 0.37 | 0.61 |
| S2 | 5 | -8.5 ± 3.5 | 9.2 | +0.4 | 8.9 | 41° | 3.84 / 3.77 / 3.69 | 91 % | 0.56 | 0.59 |
| S2 | 6 | -8.5 ± 5.0 | 9.9 | +6.8 | 9.2 | 41° | 3.75 / 3.71 / 3.61 | 92 % | 0.38 | 0.59 |
| S2 | 7 | -6.3 ± 8.8 | 10.8 | +18.4 | 8.5 | 40° | 3.59 / 3.51 / 3.35 | 98 % | 0.33 | 0.57 |
| S2 | 8 | -9.6 ± 3.7 | 10.3 | +3.6 | 8.3 | 39° | 3.57 / 3.46 / 3.36 | 100 % | 0.33 | 0.60 |
| S3 | 1 | +2.9 ± 4.7 | 5.5 | -7.4 | 2.3 | 18° | 3.47 / 3.44 / 3.35 | 100 % | 0.34 | 0.47 |
| S3 | 2 | +3.1 ± 3.7 | 4.8 | -8.5 | 2.0 | 21° | 3.42 / 3.40 / 3.33 | 100 % | 0.44 | 0.47 |
| S3 | 3 | -0.3 ± 1.8 | 1.8 | -1.3 | 2.9 | 14° | 3.35 / 3.32 / 3.20 | 100 % | 0.52 | 0.50 |

## Per strategy
| strat | flights | z mean [cm] | spread of the flight means | z RMS [cm] | xy RMS [cm] | max roll/pitch | vbat mean / min [V] | motors max |
|---|---|---|---|---|---|---|---|---|
| S0 geometric | 3 (12, 13, 14) | **-17.3** | -20.3 … -15.6 | 17.6 | 8.2 | 42° | 3.49 / 3.14 | 92–100 % |
| S1 INDI | 3 (9, 10, 11) | **-0.9** | -1.8 … -0.2 | 2.4 | 2.6 | 22° | 3.56 / 3.29 | 100–100 % |
| S2 geometric + NS2 | 5 (4, 5, 6, 7, 8) | **-8.7** | -10.4 … -6.3 | 10.3 | 8.7 | 42° | 3.64 / 3.35 | 90–100 % |
| S3 INDI + NS2 | 3 (1, 2, 3) | **+1.9** | -0.3 … +3.1 | 4.1 | 2.4 | 21° | 3.39 / 3.20 | 100–100 % |

Time profile of the hold: `meetings/assets/2026-10-19/A1_comparison.png`.

## 4. Coverage and data quality
| | S0 geometric | S1 INDI | S2 geometric + NS2 v2 | S3 INDI + NS2 v2 |
|---|---|---|---|---|
| A8 (0.5 m/s, 4 crossings) | 3 | 3 | 3 | 4 (+2 on a draining battery, excluded) |
| A1 (hold 15 s) | 3 | 3 | 5 | 3 |
- 15 A8 and 14 A1 flights with complete uSD logs of both drones; lost: A8 flights 14 (bottom battery collapsed in flight), 17, 18 and 5 A1 flights (**uSD file counter at 100**: the drone cannot open a log file and resets in the air when logging stops — fixed by archiving the cards, rule: count the `thesisNN` files before every block, archive at ≥ 85; `lab_sessions/2026-10-19.md` §12).
- Strategy per flight and NS2 version were checked from the logs: full thrust model (`experiments/analysis/ns2_applied_check.py`; coefficient on the NS2 prediction 0.00 without NS2, −1.00 with NS2, validated on all flights) and the logged prediction against the v1 / v2 weights (v2 RMS 0.02–0.18, v1 0.2–0.55).
- Batteries: loaded cell voltage; INDI flights run at 3.2–3.7 V, the three S3 flights on a draining battery (3.13 V minimum) were excluded from the A8 mean (they degrade monotonically −5.2 → −9.8 → −13.3 cm).
- Trend only: n = 3–5 per cell, one session, two drones, one dz. Higher-order claims (significance, other dz / speeds) belong to the comparative study.
- uSD write gaps: mean 0.6 % of samples per file, worst 2.9 %.

## 5. Reproduce
`merge_training_bank.py --date 2026-10-19 --out experiments/logs/bank_v2val_2026-10-19 --only-tag-contains v2val --rpm deck` (A8), `… bank_a1clean_2026-10-19 --scenarios A1 --only-tag-contains a1clean` (A1); then `experiments/analysis/v2val_summary.py`, `v2val_overview.py` (A8), `v2val_a1_summary.py`, `v2val_a1_overview.py` (A1), `v2val_plots.py`, `v2val_overview_plot.py` (figures). Raw uSD logs: `experiments/logs/usd_raw/*v2val*`, `*a1clean*`; summaries: `experiments/results/v2val_2026-10-19/`, `experiments/results/a1_2026-10-19/`.

## 9. Extension 2026-10-20 (all pilot flights of 2026-10-19 and 2026-10-20)
### A8 swap (dz 0.5, 0.5 m/s, 4 crossings per flight)
| strategy | flights | mean dip [cm] | std of the flight means | worst crossing | z RMS [cm] | motors max |
|---|---|---|---|---|---|---|
| S0 geometric | 4 | **-12.5** | 0.4 | -15.1 | 3.8 | 76–80 % |
| S1 INDI | 4 | **-8.4** | 2.8 | -13.9 | 3.1 | 100–100 % |
| S2 geometric + NS2 | 4 | **-5.6** | 0.7 | -9.3 | 1.8 | 79–82 % |
| S3 INDI + NS2 | 5 | **-3.8** | 0.9 | -8.6 | 2.1 | 100–100 % |

### A1 stacked hold (dz 0.5, hold 15 s, window 2–15 s)
| strategy | flights | z error mean [cm] | std of the flight means | z RMS [cm] | max roll/pitch | motors max |
|---|---|---|---|---|---|---|
| S0 geometric | 4 | **-17.9** | 2.4 | 18.2 | 42° | 92–100 % |
| S1 INDI | 4 | **-2.0** | 2.2 | 3.1 | 22° | 100–100 % |
| S2 geometric + NS2 | 6 | **-9.0** | 1.6 | 10.5 | 42° | 90–100 % |
| S3 INDI + NS2 | 4 | **+2.6** | 2.1 | 5.1 | 24° | 100–100 % |

### A4 offset line (dz 0.5, 0.5 m/s): z RMS [cm], mean ± std over flights (n)
| offset | S0 geometric | S1 INDI | S2 geometric + NS2 | S3 INDI + NS2 |
|---|---|---|---|---|
| 0.05 m | 17.2 ± 1.2 (n=3) | 5.0 ± 0.5 (n=3) | 11.7 ± 1.7 (n=3) | 3.8 ± 0.8 (n=3) |
| 0.10 m | 10.5 ± 1.1 (n=3) | 2.8 ± 0.1 (n=3) | 5.0 ± 1.5 (n=3) | 3.2 ± 0.5 (n=3) |
| 0.15 m | 6.2 ± 1.5 (n=3) | 2.4 ± 0.6 (n=3) | 2.3 ± 0.9 (n=3) | 2.7 ± 1.1 (n=3) |

![A8, A1 and A4 per strategy](meetings/assets/2026-10-19/overview.png)

![A4: z error over time, every flight](meetings/assets/2026-10-19/A4_offsets_pilot.png)

Details and per-flight tables: `lab_sessions/2026-10-20.md` §1–11. Remaining before the plan target: S1 A4 at offset 0.15 (2 flights).
