# Battery vs tracking error — desk analysis of the pilot flights (2026-10-20)

Status: side analysis, not in the meeting doc or the thesis. Question: does the tracking error depend on the battery, and is the S0–S3 ranking a battery artefact?

## Data and method
- 76 pilot flights, bottom vehicle, complete uSD logs only: A8 (17, two drain flights excluded), A1 (23), A4 (36, offsets 0.05/0.10/0.15). Same flight sets as the unified overview.
- Battery = mean loaded cell voltage of the bottom drone while airborne (z > 0.2 m), from the merged uSD csv. (No resting voltage is logged before takeoff.)
- Error: A8 mean crossing dip per flight, A1 mean z error per flight, A4 z RMS per flight (offset effect removed within each strategy).
- Per strategy: Pearson/Spearman r, slope [cm per 0.1 V], bootstrap 95 % interval over flights. Pooled by INDI on (S1+S3) / off (S0+S2) with strategy means removed. Strategy means adjusted to the common battery with the pooled slope of their group.
- Script `experiments/analysis/battery_vs_error.py`; outputs `experiments/results/battery_analysis_2026-10-20/` (`per_flight.csv`, `results.md` = all tables, `results.json`, `battery_vs_error.png`).

## Results (slope = cm per 0.1 V, [95 % interval])
| | A8 dip | A1 mean z error | A4 z RMS |
|---|---|---|---|
| INDI on (S1+S3) | +1.1 [−0.2, +2.0] (n=9) | **+1.4 [+0.6, +2.4]** (n=10) | −0.3 [−0.3, +0.1] (n=18) |
| INDI off (S0+S2) | −0.3 [−1.5, +0.3] (n=8) | 0.0 [−0.6, +0.7] (n=13) | 0.0 [−0.4, +0.4] (n=18) |

- **A1: INDI strategies depend on the battery, geometric ones do not.** Per strategy S1 +1.2 [+0.4, +2.2], S3 +2.4 [−0.1, +3.6]; S0 and S2 flat. Direction: the lower the battery, the lower the INDI drone holds (the sag grows), the same direction as the lost thrust seen on hardware.
- **A8: same direction, weaker.** S1 is driven by one low-battery flight (3.34 V, −12.4 cm); S3 shows nothing (+0.1); pooled interval touches 0.
- **A4: no usable signal.** S1's flights sit in a narrow range (3.58–3.72 V). The negative S3 slope is driven by one flight at 3.04 V mean that still flew at 4 cm RMS, so INDI+NS2 tolerated a nearly empty pack there.
- **The ranking is not a battery artefact.** Adjusted to the common battery: A8 dip S0 −12.1, S1 −7.9, S2 −5.7, S3 −3.3 (raw −12.5, −8.4, −5.6, −3.8); A1 S0 −17.7, S1 −1.6, S2 −8.9, S3 +4.0 (raw −17.7, −1.8, −8.9, +2.7); A4 RMS S0 11.3, S1 3.6, S2 6.3, S3 2.9 (raw 11.3, 3.4, 6.3, 3.2). Order unchanged everywhere. Mean battery per strategy differs by at most 0.1–0.3 V (A8 S0 was flown on fuller packs, 3.74 V; S3 on the emptiest, 3.43–3.55 V).

## Caveats
- Small n per strategy (4–9), narrow voltage range (≈3.3–3.8 V, one flight at 3.0 V); intervals are wide, only the A1 pooled INDI result excludes zero.
- Loaded voltage is partly an effect of the flight (INDI and a sagging drone draw more current), so the arrow of causation is not proven. Battery order is tied to flight order and which pack was used.
- A1 includes the 20 s video flights (window 2–15 s).

## Decision
Keep on the side. For the tier-1 study: the uSD logs already carry the battery voltage; add a resting voltage at takeoff to the flight record and flag flights below a fixed start voltage, then repeat this analysis on the larger set. No dedicated battery-sweep flights planned (a cheap check if needed: S1 and S3 on A1, fresh packs near 4.1 V against half-used, about 3 flights each).
