# NS2 sign test — Block 1 result (2026-10-08)

**Config (from meta/yaml):** cf5 `controller 6, ctrl_mode 0, res_sign -1, rnn.en 1`, 100 Hz network, A8 `--dz 0.5 --height 0.5 --passes 4`; cf_second geometric. No reflash (10-05 firmware). Commit 35958ad.

## Flights
| Stamp | Verdict | cf5 vbat min (loaded) | Dips (cm) | Mean |
|---|---|---|---|---|
| 17-16-27 | battery sag on cf5 (2.52 V), flight completed | 2.52 | | −3.02 |
| 17-18-40 | clean; cf_second radio log ended 8 s early (telemetry drop) | 3.18 | −2.5/−2.8/+0.2/−2.9 (uSD) | −2.71 (radio) |
| 17-22-13 | clean | 2.79 | | −3.16 |
| 17-24-54 | clean | 2.68 | | −3.72 |
3 further attempts aborted at start (empty radio logs, no uSD).

## Result (cf5 crossing dip, same definition as docs/52, negative = below setpoint, shallower is better)
| Cohort | n | mean | sd | range |
|---|---|---|---|---|
| network off (10-05) | 8 | −5.5 (uSD ref −5.9) | 1.0 | −7.2…−4.2 |
| `res_sign=+1` (10-05) | 12 | −10.4 (uSD ref −10.9) | 1.6 | −12.2…−7.1 |
| **`res_sign=−1` (10-08)** | 16 | **−3.15** | 0.69 | −4.5…−2.1 |

- Monotone in the sign: +1 −10.4 → off −5.5 → −1 −3.2 cm. Welch p = 1e-4 vs network off, 4e-10 vs +1.
- SIL prediction (docs/62): −2.3 ± 0.1; hardware −3.15. Direction and size confirmed; the SIL is ~0.9 cm too optimistic.
- Strict pack criterion "ranges not overlapping" is missed by 0.3 cm (−4.5 vs −4.2); the means differ by 2.3 cm.
- Method check: radio-CSV dips (20 Hz, z_sp = 0.5) reproduce the 10-05 uSD cohorts within 0.5 cm.
- uSD cross-check on 17-18-40 (only cf5 uSD flight): −2.5/−2.8/+0.2/−2.9; the +0.2 is one crossing, probably mis-detected (t=28.8).
- Safety: cf5 roll peak 24–28°, pitch ≤ 11°, min z ≥ 0.30 m, cf_second roll ≤ 5° — same as network-off A8.

## Caveats
- The network-off baseline comes from 10-05, not this session (Block 2 `rnn.en=0` ×2 still to fly).
- cf5 loaded vbat 2.5–2.8 V in all flights (battery limit, not a crash).
- cf5 uSD: only thesis01 (17-18-40) has data; thesis00/02/03 are 0 bytes although the flights completed → merged CSV only for 17-18-40. Cause open (card/sync on the cf5 SD).
- uSD merge cross-check "offset 305 ms, corr 0.38" is the documented unreliable z-correlation; the authoritative trajectory alignment gives 20 ms.

## Files
`experiments/analysis/ns2_signtest_2026_10_08_radio.py`, `out/ns2_signtest_2026-10-08/radio_dips.json`, `out/post_flight_check_2026-10-08.{md,json}`, merged `experiments/logs/merged_A8_2026-10-08_17-18-40.csv`.

## Block 2 — same-session network-off baseline (A8, `rnn.en=0`, `res_sign=-1` unused, commit 4906e77)
| Stamp | uSD dips (cm) | uSD mean | radio mean | cf5 vbat min | cf5 roll/pitch peak |
|---|---|---|---|---|---|
| 17-37-04 | −6.6/−6.0/−5.3/−5.8 | −5.91 | −5.96 | 3.65 | 24°/9° |
| 17-38-50 | −7.0/−5.1/−6.2/−5.5 | −5.95 | −6.23 | 3.58 | 28°/14° |

- Network off today: radio mean −6.10 (n=8, sd 0.57, −7.0…−5.2), uSD mean −5.93 — reproduces 10-05 (−5.9).
- `res_sign=−1` network on −3.15 vs network off −6.10: Δ = 2.9 cm shallower, Welch p = 4e-9, **ranges do not overlap** (−4.5…−2.1 vs −7.0…−5.2). Pack criterion met against the same-session baseline.
- Both cards complete this time (cf5 and cf_second, 2 files each, run tags match); merged CSVs for both flights (alignment RMS ≤ 2.6 cm, clock agreement 20 ms). The network still runs with `rnn.en=0` (logged `rnn_pred_z` ≈ −0.4…−0.6 m/s² at crossings) but its output is not applied.
- Files: `experiments/logs/merged_A8_2026-10-08_17-{37-04,38-50}.csv`, bags `ns2_pose_a8_en0_{1,2}`.

## Blocks 3/4 — A1 (stacked hover), network off vs on (`res_sign=-1`), cf5 uSD, steady window (t0+6 s … end−4 s)
Flights: off 17-45-15, 17-46-51 · on 17-51-55, 17-53-48, 17-55-30 (all `ctrl_mode 0`, `res_sign −1` in meta; `rnn.en` per yaml 4906e77 / 91e6067). cf5 uSD complete (5/5); cf_second uSD 4/5 (17-51-55 lost, cf_second battery 2.60 V).

| Flight | net | roll sd | pitch sd | roll p99 | gyro_x sd | xy rms | z err | z sd | f_osc | rnn_pred_z | a_res_z |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 17-45-15 | off | 19.1° | 23.4° | 34° | 154 | 7.3 cm | −0.85 | 1.64 | 1.55 Hz | −1.44 | −1.69 |
| 17-46-51 | off | 20.9° | 23.5° | 33° | 162 | 7.9 cm | −0.51 | 1.13 | 1.35 Hz | −1.31 | −1.49 |
| 17-51-55 | on | 21.1° | 24.0° | 33° | 173 | 7.6 cm | −0.50 | 0.97 | 1.55 Hz | −1.31 | −1.51 |
| 17-53-48 | on | 21.0° | 24.1° | 33° | 163 | 7.8 cm | −0.09 | 0.73 | 1.48 Hz | −1.35 | −1.48 |
| 17-55-30 | on | 21.7° | 23.3° | 36° | 176 | 7.6 cm | −0.06 | 0.84 | 1.52 Hz | −1.32 | −1.46 |

- **The A1 oscillation is NOT caused by the network.** cf5 swings ±35–40° (roll/pitch sd 19–24°, ~1.5 Hz, xy rms 7–8 cm) with the network off and the same with it on. Network on adds at most ~1° roll sd and ~10 deg/s gyro sd, within flight-to-flight spread (n=2 vs 3). Attribution: geometric controller under the stacked downwash (cf5 solo hover is calm).
- **No instability with the network on and `res_sign=−1`** (3/3 flights completed, tilt peak 34–40° like the baseline). The SIL predicted 38–48° tilt / instability for A1 with network on; it was too pessimistic (known: its A1 stack force is ~1.6× too high vs real).
- Network prediction in the stack: `rnn_pred_z` −1.3…−1.4 m/s² vs measured `a_res_z` −1.5 (≈ 88 %); never clamped.
- z in the stack: network on is marginally better (z err −0.06…−0.5 vs −0.5…−0.85 cm, z sd 0.7–1.0 vs 1.1–1.6 cm); the z integral (ki_z 16) holds steady height in both cases, so A1 steady z is not a discriminating metric for the sign.
- Merge caveat: A1 merged CSVs `merged_A1_2026-10-08_*.csv` exist for 4 flights; the trajectory alignment is weak for a stationary scenario (cross-drone clock agreement 980 ms and 1040 ms on 17-45-15 / 17-55-30, 0–80 ms on the others) — use per-drone uSD for the numbers above, not cross-drone timing.
- Files: `experiments/analysis/ns2_a1_2026_10_08.py`, `out/ns2_signtest_2026-10-08/a1_usd_summary.json`, bags `ns2_pose_a1_en{0_1,0_2,1_1,1_2,1_3}`.
